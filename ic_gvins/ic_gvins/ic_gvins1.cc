
#include "ic_gvins.h"
#include "misc.h"

#include "common/earth.h"
#include "common/logging.h"
#include "preintegration/preintegration.h"

#include <yaml-cpp/yaml.h>

#include <CGNSSConfig.h>

GVINS::GVINS(const string &configfile, const string &outputpath, Drawer::Ptr drawer) {
    gvinsstate_ = GVINS_ERROR;

    // 加载配置
    // Load configuration
    YAML::Node config;
    std::vector<double> vecdata;
    try {
        config = YAML::LoadFile(configfile);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return;
    }

    // 文件IO
    // Output files
    navfilesaver_    = FileSaver::create(outputpath + "/gvins.nav", 11);
    ptsfilesaver_    = FileSaver::create(outputpath + "/mappoint.txt", 3);
    statfilesaver_   = FileSaver::create(outputpath + "/statistics.txt", 3);
    extfilesaver_    = FileSaver::create(outputpath + "/extrinsic.txt", 3);
    imuerrfilesaver_ = FileSaver::create(outputpath + "/IMU_ERR.bin", 7, FileSaver::BINARY);
    trajfilesaver_   = FileSaver::create(outputpath + "/trajectory.csv", 8);
    covfilesaver_    = FileSaver::create(outputpath + "/converiance.bin", 45, FileSaver::BINARY);

    if (!navfilesaver_->isOpen() || !ptsfilesaver_->isOpen() || !statfilesaver_->isOpen() || 
        !extfilesaver_->isOpen() || !covfilesaver_->isOpen()) {
        LOGE << "Failed to open data file";
        return;
    }

    // Make a copy of configuration file to the output directory
    std::ofstream ofconfig(outputpath + "/gvins.yaml");
    ofconfig << YAML::Dump(config);
    ofconfig.close();

    initlength_       = config["initlength"].as<int>();                
    imudatarate_      = config["imudatarate"].as<double>();
    imudatadt_        = 1.0 / imudatarate_;
    reserved_ins_num_ = 2;

    // 安装参数
    // Installation parameters
    antlever_         = Vector3d(config["antlever"].as<std::vector<double>>().data());

    // IMU噪声参数
    // IMU parameters
    integration_parameters_               = std::make_shared<IntegrationParameters>();
    integration_parameters_->gyr_arw      = Vector3d(config["imumodel"]["arw"].as<std::vector<double>>().data()) * D2R;        
    integration_parameters_->gyr_bias_std = Vector3d(config["imumodel"]["gbstd"].as<std::vector<double>>().data()) * D2R;
    integration_parameters_->acc_vrw      = Vector3d(config["imumodel"]["vrw"].as<std::vector<double>>().data());
    integration_parameters_->acc_bias_std = Vector3d(config["imumodel"]["abstd"].as<std::vector<double>>().data());
    integration_parameters_->corr_time    = config["imumodel"]["corrtime"].as<double>();
    integration_parameters_->gravity      = NORMAL_GRAVITY;

    integration_config_.iswithearth = config["iswithearth"].as<bool>();
    integration_config_.isuseodo    = false;
    integration_config_.iswithscale = false;
    integration_config_.gravity     = {0, 0, integration_parameters_->gravity};

    // 初始值, 后续根据GNSS定位实时更新
    // GNSS variables intializaiton
    integration_config_.origin.setZero();
    last_gnss_.blh.setZero();
    gnss_.blh.setZero();

    preintegration_options_ = Preintegration::getOptions(integration_config_);

    // 相机参数
    // Camera parameters
    vector<double> intrinsic  = config["cam0"]["intrinsic"].as<std::vector<double>>();
    vector<double> distortion = config["cam0"]["distortion"].as<std::vector<double>>();
    vector<int> resolution    = config["cam0"]["resolution"].as<std::vector<int>>();

    camera_ = Camera::createCamera(intrinsic, distortion, resolution);          // 内参，畸变，分辨率

    // GNSS参数
    // Camera parameters
    CGNSSConfig gnssconfig;
    gnssconfig.LoadYAML(configfile.c_str());
    strcpy(gnssconfig.filopt.outdir, outputpath.c_str()); 

    // IMU和Camera外参
    // Extrinsic parameters
    vecdata           = config["cam0"]["q_b_c"].as<std::vector<double>>();
    Quaterniond q_b_c = Eigen::Quaterniond(vecdata.data());
    vecdata           = config["cam0"]["t_b_c"].as<std::vector<double>>();
    Vector3d t_b_c    = Eigen::Vector3d(vecdata.data());
    td_b_c_           = config["cam0"]["td_b_c"].as<double>();

    pose_b_c_.R = q_b_c.toRotationMatrix();
    pose_b_c_.t = t_b_c;

    // 优化参数
    // Optimization parameters
    reprojection_error_std_      = config["reprojection_error_std"].as<double>();
    optimize_estimate_extrinsic_ = config["optimize_estimate_extrinsic"].as<bool>();
    optimize_estimate_td_        = config["optimize_estimate_td"].as<bool>();    
    optimize_num_iterations_     = config["optimize_num_iterations"].as<int>();
    optimize_windows_size_       = config["optimize_windows_size"].as<size_t>();

    // 归一化相机坐标系下
    // Reprojection std
    optimize_reprojection_error_std_ = reprojection_error_std_ / camera_->focalLength();    // 焦距

    // 可视化
    is_use_visualization_ = config["is_use_visualization"].as<bool>();

    // Initialize the containers
    preintegrationlist_.clear();      
    statedatalist_.clear();
    gnsslist_.clear();
    timelist_.clear();

    // GVINS fusion objects
    map_    = std::make_shared<Map>(optimize_windows_size_);            
    drawer_ = std::move(drawer);
    drawer_->setMap(map_);
    if (is_use_visualization_) {
        drawer_thread_ = std::thread(&Drawer::run, drawer_);
    }
    tracking_ = std::make_shared<Tracking>(camera_, map_, drawer_, configfile, outputpath); 
    obss_buffer_ = std::make_shared<ObssManager>();
    positioning_ = std::make_shared<CGNSSManage>();
    positioning_->Init(gnssconfig.prcopt, gnssconfig.solopt, gnssconfig.filopt, gnssconfig.sta);
    positioning_->TraceOpen(gnssconfig.filopt.trace);             

    // Process threads
    fusion_thread_       = std::thread(&GVINS::runFusion, this);                          // 优化期间也在IMU机械编排     
    tracking_thread_     = std::thread(&GVINS::runTracking, this);                        // 优化期间也在跟踪
    positioning_thread_  = std::thread(&GVINS::runPositioning, this);                     // 优化期间也在定位   
    optimization_thread_ = std::thread(&GVINS::runOptimization, this);                                                      

    gvinsstate_ = GVINS_INITIALIZING;
}

bool GVINS::addNewImu(const IMU &imu) {
    if (imu_buffer_mutex_.try_lock()) {                
        if (imu.dt > (imudatadt_ * 1.5)) {                  //丢失数据时，进行必要的补充（操作合理）
            LOGE << absl::StrFormat("Lost IMU data with at %0.3lf dt %0.3lf", imu.time, imu.dt);

            long cnts = lround(imu.dt / imudatadt_) - 1;

            IMU imudata  = imu;
            imudata.time = imu.time - imu.dt;
            while (cnts--) {
                imudata.time += imudatadt_;
                imudata.dt = imudatadt_;
                imu_buffer_.push(imudata);
                LOGE << "Append extra IMU data at " << Logging::doubleData(imudata.time);
            }
        } else {
            imu_buffer_.push(imu);
        }

        // 释放信号量
        // Release fusion semaphore
        fusion_sem_.notify_one();              

        imu_buffer_mutex_.unlock();
        return true;
    }

    return false;
}

bool GVINS::addNewObss(GOBSS &obss) { 
    if (obss_buffer_->addNewObss(obss)) {
        positioning_sem_.notify_one();
        return true;
    }
    return false;
}

bool GVINS::addNewFrame(const Frame::Ptr &frame) { 
    if (gvinsstate_ == GVINS_GINS_NORMAL && !isframeexist_) {
        isframeexist_ = true;
    }    
    if (gvinsstate_ >= GVINS_INITIALIZING_VIO) {     
        if (frame_buffer_mutex_.try_lock()) {  

            frame_buffer_.push(frame);
            tracking_sem_.notify_one();

            frame_buffer_mutex_.unlock();
            return true;
        }
        return false;
    }
    return true;
}

void GVINS::runFusion() {
    IMU imu_pre, imu_cur;
    IntegrationState state;
    Frame::Ptr frame;

    LOGI << "Fusion thread is started";
    while (!isfinished_) { // While
        Lock lock(fusion_mutex_);     
        fusion_sem_.wait(lock);        

        // 获取所有有效数据
        // Process all IMU data
        while (!imu_buffer_.empty()) { // IMU BUFFER 
            // 读取IMU缓存
            // Load an IMU sample
            {
                Lock lock2(imu_buffer_mutex_);                     
                imu_pre = imu_cur;
                imu_cur = imu_buffer_.front();
                imu_buffer_.pop();     
            }

            // INS机械编排及INS处理
            // INS mechanization
            { // INS
                Lock lock3(ins_mutex_);                                
                if (!ins_window_.empty()) {         
                    // 上一时刻的状态
                    // The INS state in last time for mechanization
                    state = ins_window_.back().second;
                }
                ins_window_.emplace_back(imu_cur, IntegrationState());      

                // 初始化完成后开始积分输出
                if (gvinsstate_ > GVINS_INITIALIZING) {
                    if (isoptimized_ && state_mutex_.try_lock()) {               
                        // 优化求解结束, 需要更新IMU误差重新积分
                        // When the optimization is finished
                        isoptimized_ = false;

                        state = Preintegration::stateFromData(*statedatalist_.back(), preintegration_options_);  
                        MISC::redoInsMechanization(integration_config_, state, reserved_ins_num_, ins_window_); 
                        
                        state_mutex_.unlock();
                    } else {
                        // 单次机械编排
                        // Do a single INS mechanization
                        MISC::insMechanization(integration_config_, imu_pre, imu_cur, state);

                        ins_window_.back().second = state;             
                    } 
                } else {
                    // Only reserve certain INS in the window during initialization
                    if (ins_window_.size() > MAXIMUM_INS_NUMBER) {      
                        ins_window_.pop_front();
                    }
                }
                
                // 用于输出
                // For output only
                state = ins_window_.back().second; 
                imutime_ = ins_window_.back().first.time;           
            } // INS

            // 融合状态
            // Fusion process
            if (gvinsstate_ == GVINS_INITIALIZING) {
                if (isgnssfinished_ && state_mutex_.try_lock()) {                       
                    // 初始化参数
                    // GVINS initialization using GNSS/INS initialization
                    if (gvinsInitialization()) {                                                                                  
                        gvinsstate_ = GVINS_INITIALIZING_INS;                  
                        // 初始化时需要重新积分
                        // Redo INS mechanization
                        isoptimized_ = true;
                    }                   

                    state_mutex_.unlock();
                    continue;
                }
            } else if (gvinsstate_ == GVINS_INITIALIZING_INS && !isoptimized_) {      
                // 新的GNSS观测到来, 进行优化
                // New GNSS, do GNSS/INS integration
                if (isgnssfinished_ && state_mutex_.try_lock()) {                     
                    // 加入新的GNSS节点 & 更新gnss位置 (同时)
                    // Add a new GNSS time node and updated gnsslist_
                    if (addNewGnssTimeNode(imutime_)) {             
                        isgnssobs_ = true;                            
                        optimization_sem_.notify_one();
                    }

                    state_mutex_.unlock();
                }
            } else if (gvinsstate_ == GVINS_GINS_NORMAL && !isoptimized_) {
                if ((isgnssready_ || isgnssfinished_ || isframeexist_) && state_mutex_.try_lock()) {
                    // 进入视觉初始化阶段
                    // Enter the initialization of the visual system
                    if (isframeexist_) {
                        gvinsstate_ = GVINS_INITIALIZING_VIO;
                        state_mutex_.unlock();
                        continue;
                    }

                    // 加入新的GNSS节点 | 更新gnss位置 (分离)
                    // Add a new GNSS time node and updated gnsslist_
                    if (addNewGnssTimeNode(imutime_)) {
                        isgnssobs_ = true;                            
                        optimization_sem_.notify_one();
                    }

                    state_mutex_.unlock();
                }
            } else if (gvinsstate_ == GVINS_INITIALIZING_VIO && !isoptimized_) {       
                // 仅加入关键帧节点, 而不进行优化
                // Add new time node during the initialization of the visual system
                if (isframeready_ && state_mutex_.try_lock()) { 
                    // New KeyFrame 
                    if (addNewKeyFrameTimeNode()) {                                 
                        isvisualobs_  = true;             
                        gvinsstate_ = GVINS_TRACKING_INITIALIZING;            
                    }       

                    state_mutex_.unlock();
                }
            } else if (gvinsstate_ >= GVINS_TRACKING_INITIALIZING && !isoptimized_) {
                if ((isframeready_ || isgnssready_ || isgnssfinished_) && state_mutex_.try_lock()) {           
                    if (isframeready_) {       
                        if (addNewKeyFrameTimeNode()) {         
                            isvisualobs_  = true;
                        }
                    }

                    // 如果有GNSS观测
                    // Add GNSS if available
                    if (isgnssready_ || isgnssfinished_) {
                        if (istracking_) {
                            if (insertNewGnssTimeNode()) {
                                isgnssobs_ = true;
                            }
                        } else {
                            if (addNewGnssTimeNode(frametime_)) {       
                                isgnssobs_ = true;
                            }
                        }
                    }

                    state_mutex_.unlock();

                    // Release the optimization semaphore
                    if (isvisualobs_ || (!istracking_ && isgnssobs_)) {                         
                        optimization_sem_.notify_one();                                  
                    }
                }
            }

            // Always output the INS results
            if (gvinsstate_ > GVINS_INITIALIZING) {             
                MISC::writeNavResult(integration_config_, state, antlever_, navfilesaver_, imuerrfilesaver_, trajfilesaver_);
            }

        } // IMU BUFFER
    }     // While
}

void GVINS::runPositioning() {
    GNSS gnss;

    LOGI << "Positioning thread is started";
    while (!isfinished_) { // While
        Lock lock(positioning_mutex_);               
        positioning_sem_.wait(lock);

        // 处理所有缓存
        // Process all the frames
        while (1) {  

            if (obss_buffer_->addObssInlist(imutime_)) {     
                isgnssready_ = true;
            }
            
            if (obss_buffer_->empty()) break;

            auto obss = obss_buffer_->getPendingObss();     

            if (obss.empty()) {                            
                usleep(100000);        // 百万分之一秒
                continue;
            }      

            double dt = 0;
            std::queue<GNSS> gnsslist;
            std::queue<Vector3d> dpos;
            while (!obss.empty()) {
                TimeCost timecost;
                auto &obs = obss.front();   
                if (obs.state == GOBSS::inList || obs.state == GOBSS::withSkip) {                
                    positioning_->Update(obs.meas.data, obs.meas.n);                             
                }
                else if (obs.state == GOBSS::withPrior) {
                    positioning_->Update(obs.meas.data, obs.meas.n, 0, obs.xyz.data(), obs.cov.data());     // 不进行反馈
                }
                positioning_->OutGPSResult();

                gnss.time   = time2gpst(*positioning_->ti, NULL);
                gnss.blh    = Vector3d(positioning_->pos_blh);          
                gnss.std    = Vector3d(positioning_->rk_blh);  
                gnsslist.push(gnss);

                dt = fabs(obs.node) < 1E-001 ? (obs.node - obs.time) : 0;
                dpos.push(obs.vel * dt);                  

                freeobs(&obs.meas); 
                obss.pop(); 
                // LOGI << "Positioning cost " << timecost.costInMillisecond() << " ms";
            }
            
            if (integration_config_.origin.isZero() && !gnsslist.empty()) {
                // 站心原点
                // The origin of the world frame
                integration_config_.origin       = gnsslist.front().blh;
                integration_parameters_->gravity = Earth::gravity(gnsslist.front().blh);
                LOGI << "Local gravity is initialized as " << 
                        Logging::doubleData(integration_parameters_->gravity);
            }

            for (size_t i = 0, endind = gnsslist.size(); i < endind; i++) {
                gnss     = gnsslist.front();    gnsslist.pop();
                gnss.blh = Earth::global2local(integration_config_.origin, gnss.blh) 
                         + dpos.front();        dpos.pop();
                gnsslist.push(gnss);
            }
            
            obss_buffer_->feedbackSolution(gnsslist);                       
            isgnssfinished_ = true;
        }
    }
}

void GVINS::runTracking() {                
    Frame::Ptr frame;
    Pose pose;

    LOGI << "Tracking thread is started";
    while (!isfinished_) {                
        Lock lock(tracking_mutex_);               
        tracking_sem_.wait(lock);

        // 处理所有缓存
        // Process all the frames
        while (!frame_buffer_.empty()) {
            TimeCost timecost;

            Pose pose_b_c;
            double td;
            {
                Lock lock3(extrinsic_mutex_);
                pose_b_c = pose_b_c_;          
                td       = td_b_c_;             
            }

            // 读取缓存
            {
                frame_buffer_mutex_.lock();   
                frame = frame_buffer_.front();

                // 保证每个图像都有先验的惯导位姿       
                // Wait until the INS is available             
                if (imutime_ <= (frame->stamp() + td)) {             
                    frame_buffer_mutex_.unlock();

                    usleep(1000);      // 百万分之一秒  
                    continue;
                }                                                                                        
                frame_buffer_.pop(); 
                frame_buffer_mutex_.unlock();     

                // 获取初始位姿
                // The prior pose from INS
                frame->setStamp(frame->stamp() + td);        
                frame->setTimeDelay(td);

                {
                    Lock lock3(ins_mutex_);
                    if (MISC::getCameraPoseFromInsWindow(ins_window_, pose_b_c, frame->stamp(), pose)) {
                        frame->setPose(pose);
                    } else {
                        LOGE << "Frame is not find pose in ins_windows";
                        continue;
                    }       
                }                                           
            }

            TrackState trackstate = tracking_->track(frame);                                      
            if (trackstate == TRACK_LOST) {
                LOGE << "Tracking lost at " << Logging::doubleData(frame->stamp());
            }

            // 标记状态
            if (trackstate == TRACK_FIRST_FRAME || trackstate == TRACK_INITIALIZING || trackstate == TRACK_LOST) {
                istracking_ = false;
                frametime_  = frame->stamp() - MISC::MINIMUM_TIME_INTERVAL;
            } else if (trackstate == TRACK_INITIALIZED) {
                istracking_ = true;
            }

            // 包括第一帧在内的所有关键帧, 跟踪失败时的当前帧也会成为新的关键帧
            // All possible keyframes
            if (tracking_->isNewKeyFrame() || (trackstate == TRACK_FIRST_FRAME) || trackstate == TRACK_LOST) {
                Lock lock3(keyframes_mutex_);   
                keyframes_.push(frame);

                isframeready_ = true;

                LOGI << "Tracking cost " << timecost.costInMillisecond() << " ms";
            }
        }
    }
}

void GVINS::runOptimization() {                     

    TimeCost timecost, timecost2;

    LOGI << "Optimization thread is started";
    while (!isfinished_) {
        Lock lock(optimization_mutex_);
        optimization_sem_.wait(lock);

        if (isgnssobs_ || isvisualobs_) {          
            timecost.restart();

            // 加锁, 保护状态量
            // Lock the state
            state_mutex_.lock();                                   

            if (gvinsstate_ == GVINS_INITIALIZING_INS) {
                // GINS优化
                // GNSS/INS optimization
                bool isinitialized = gvinsInitializationOptimization();    
                
                if (preintegrationlist_.size() >= static_cast<size_t>(initlength_)) { 

                    obss_buffer_->activateDualMode();
                    isgnssfinished_ = false;      
    
                    gvinsstate_ = GVINS_GINS_NORMAL;
                             
                    if (isinitialized) {
                        LOGI << "GINS initialization is finished";
                    } else {
                        LOGW << "GINS initialization is not convergence";
                    }
                }
            } else if (gvinsstate_ == GVINS_GINS_NORMAL) {          
                // 两次非线性优化并进行粗差剔除
                // Two-steps optimization with outlier culling
                GIOptimization();                                                          // 1.GNSS异常点剔除

                // Do marginalization
                while (preintegrationlist_.size() > static_cast<size_t>(initlength_)) {   // 2.GNSS/INS边缘化
                    GIMarginalization();            
                }
            } else if (gvinsstate_ >= GVINS_TRACKING_INITIALIZING) {

                if (map_->isMaximumKeframes()) {       
                    gvinsstate_ = GVINS_TRACKING_NORMAL;
                }

                // 两次非线性优化并进行粗差剔除
                // Two-steps optimization with outlier culling
                GVIOptimization();                                  // 1.地图点剔除  (视觉处理)
                timecost2.restart();

                // 移除所有窗口中间插入的非关键帧
                // Remove all non-keyframes time nodes
                RemoveAllSecondNewFrame();                          // 2.关键帧剔除                 

                // 关键帧数量达到窗口大小, 需要边缘化操作, 并移除最老的关键帧及相关的GNSS和预积分观测, 由于计算力的问题,
                // 可能导致多个关键帧同时加入优化, 需要进行多次边缘化操作
                // Do marginalization
                while (map_->isMaximumKeframes()) {                 // 3.边缘化    
                    // 边缘化, 移除旧的观测, 按时间对齐到保留的最后一个关键帧
                    GVIMarginalization();                                             
                }

                timecosts_[2] = timecost2.costInMillisecond();

                // 统计并输出视觉相关的参数 
                // Log the statistic parameters
                parametersStatistic();                              // 4.视觉信息管理      
            }  

            // 可视化
            // For visualization 发布轨迹和地图点
            if (is_use_visualization_) {                            
                auto state = Preintegration::stateFromData(*statedatalist_.back(), preintegration_options_);
                drawer_->updateMap(MISC::pose2Twc(MISC::stateToCameraPose(state, pose_b_c_)));          
            }

            if (isgnssobs_)   isgnssobs_   = false;
            if (isvisualobs_) isvisualobs_ = false;

            // Release the state lock
            state_mutex_.unlock();
            isoptimized_ = true;

            LOGI << "Optimization costs " << timecost.costInMillisecond() << " ms with " << timecosts_[0] << " and "
                 << timecosts_[1] << " with marginalization costs " << timecosts_[2];
        }
    }
}

void GVINS::addNewTimeNode(double time) {          

    vector<IMU> series;
    IntegrationState state;

    // 获取时段内用于预积分的IMU数据
    // Obtain the IMU samples between the two time nodes
    double start = timelist_.back();                   
    double end   = time;
    MISC::getImuSeriesFromTo(ins_window_, start, end, series);      

    state = Preintegration::stateFromData(*statedatalist_.back(), preintegration_options_);       

    // 新建立新的预积分
    // Build a new IMU preintegration       
    preintegrationlist_.emplace_back(
        Preintegration::createPreintegration(integration_parameters_, series[0], state, preintegration_options_));  

    // 预积分, 从第二个历元开始
    // Add IMU sample
    for (size_t k = 1; k < series.size(); k++) {
        preintegrationlist_.back()->addNewImu(series[k]);
    }

    // 当前状态加入到滑窗中
    // Add current state and time node to the sliding window
    state      = preintegrationlist_.back()->currentState();                  
    state.time = time;
    statedatalist_.emplace_back(Preintegration::stateToData(state, preintegration_options_));

    timelist_.push_back(time);             
}

void GVINS::insertNewTimeNode(double time) 
{
    size_t index, ind;
    vector<IMU> series;
    IntegrationState state;
    std::shared_ptr<IntegrationStateData> statedata;
    std::shared_ptr<PreintegrationBase> preintegration0, preintegration1;

    // Find time interval
    index = 0;
    for (size_t k = timelist_.size() - 1; k >= 1; k--) {
        if ((time <= timelist_[k]) && (time > timelist_[k - 1])) {
            index = k;     break;
        }
    }
    if (index == 0) return ;
    series = preintegrationlist_[index - 1]->imuBuffer();
    state  = Preintegration::stateFromData(*statedatalist_[index - 1], preintegration_options_);
    ind    = MISC::getImuWindowIndex(series, time);

    auto isneed = MISC::isNeedInterpolation(series[ind - 1], series[ind], time);
    
    if (isneed == -1) {
        ind -= 1;
    } else if (isneed == 1) {
    } else if (isneed == 2) {
        IMU imu;
        MISC::imuInterpolation(series[ind], imu, series[ind], time);
        series.insert(series.begin() + ind, imu);
    }
    
    // 预积分0
    preintegration0 = Preintegration::createPreintegration(integration_parameters_, series[0], state, preintegration_options_);
    for (size_t i = 1; i <= ind; i++) {
        preintegration0->addNewImu(series[i]);
    }
    state = preintegration0->currentState();
    state.time = time;
    statedata = Preintegration::stateToData(state, preintegration_options_);

    // 预积分1
    preintegration1 = Preintegration::createPreintegration(integration_parameters_, series[ind], state, preintegration_options_);
    for (size_t i = ind + 1; i < series.size(); i++) {
        preintegration1->addNewImu(series[i]);
    }

    // 修改预积分指向
    preintegrationlist_[index - 1] = preintegration0;                   // 测试一下

    // Insert GNSS node to sliding window    
    timelist_.insert(timelist_.begin() + index, time);                  //  begin()指向的是第一个元素前面的位置
    statedatalist_.insert(statedatalist_.begin() + index, statedata);
    preintegrationlist_.insert(preintegrationlist_.begin() + index, preintegration1);

    LOGI << "Insert GNSS node " << Logging::doubleData(timelist_[index - 1]) << "  " 
         << Logging::doubleData(timelist_[index]) << "  " << Logging::doubleData(timelist_[index + 1]) << 1;
}

bool GVINS::addNewGnssTimeNode(double time) { 

    bool isnewnode = false;

    if (isgnssfinished_) {
        isgnssfinished_ = false;

        auto gnsslist = obss_buffer_->popObssWithSolution();

        if (obss_buffer_->getDualMode()) {
            while (!gnsslist.empty()) {
                gnsslist_.push_back(gnsslist.front());
                gnsslist.pop();
            }
        } else {
            isnewnode =(!gnsslist.empty());
            while (!gnsslist.empty()) {
                auto gnss = gnsslist.front();
                addNewTimeNode(gnss.time);
                gnsslist_.push_back(gnss);
                gnsslist.pop();
            }
        }
    }

    if (isgnssready_) {
        isgnssready_ = false;

        if (obss_buffer_->getDualMode()) {
            auto node = obss_buffer_->addObssInNode(timelist_.back(), time); 
            isnewnode = (!node.empty());
            while (!node.empty()) {
                addNewTimeNode(node.front());      
                node.pop();
            }
        }
    }

    return isnewnode;
}

bool GVINS::insertNewGnssTimeNode() {    
    static double startTime = 0.0;

    bool isnewnode = false;

    if (isgnssfinished_) {
        isgnssfinished_  = false;

        auto gnsslist = obss_buffer_->popObssWithSolution();
        while (!gnsslist.empty()) {
            gnsslist_.push_back(gnsslist.front());
            gnsslist.pop();
        }
    }
    
    if (isgnssready_) {
        isgnssready_ = false;

        std::deque<bool>   flaglist;
        std::deque<double> timelist;
        auto keyframeids  = map_->orderedKeyFrames();
        
        for (size_t k = timelist_.size() - 1; k >= 0; k--) {
            if (timelist_[k] <= startTime) break;

            timelist.push_front(timelist_[k]);
            flaglist.push_front(true);

            auto isfind = false;
            for (auto node : unused_time_nodes_) {
                if (MISC::isTheSameTimeNode(node, timelist_[k], MISC::MINIMUM_TIME_INTERVAL)) {
                    flaglist.front() = false;
                    isfind = true;
                    break;
                }
            }
            if (isfind) continue;

            for (int ii = keyframeids.size() - 1; ii >= 0; ii--) {
                auto keyframes = map_->keyframes();
                if (keyframes.find(keyframeids[ii]) == keyframes.end()) {
                    continue;
                }

                auto frame = keyframes.find(keyframeids[ii])->second;
                double keyframe_time = frame->stamp();
                if (MISC::isTheSameTimeNode(keyframe_time, timelist_[k], MISC::MINIMUM_TIME_INTERVAL)) {
                    if (frame->keyFrameState() < KEYFRAME_NORMAL || frame->features().empty()) {
                        flaglist.front() = false;
                    }
                    isfind = true;
                    break;
                }
            }

            if (!isfind) {
                // LOGW << "Insert new gnss node not find time at " << timelist_[k];
                flaglist.front() = false;
            }
        }
        
        auto node = obss_buffer_->addObssInNode(timelist, flaglist); 
        isnewnode = !node.empty();  
        
        while (!node.empty()) {  
            insertNewTimeNode(node.front());            
            startTime = node.front();  
            node.pop();
        }
    }

    return isnewnode;
}

bool GVINS::addNewKeyFrameTimeNode() {
    bool stat = false;
    
    isframeready_ = false;
    keyframes_mutex_.lock();    
    while (!keyframes_.empty()) {
        // 取出一个关键帧
        // Obtain a new valid keyframe
        auto frame       = keyframes_.front();             
        double frametime = frame->stamp();
        if (frametime <= timelist_.back()) {
            keyframes_.pop();
            continue;
        }

        keyframes_.pop();

        // 添加关键帧
        // Add new keyframe time node
        LOGI << "Insert keyframe " << frame->keyFrameId() << " at " << Logging::doubleData(frame->stamp()) << " with "
             << frame->unupdatedMappoints().size() << " new mappoints";
        map_->insertKeyFrame(frame);                               

        addNewTimeNode(frametime);                                  
        LOGI << "Add new keyframe time node at " << Logging::doubleData(frametime);
        stat = true;
    }
    keyframes_mutex_.unlock();
    
    // 移除多余的预积分节点
    // Remove unused time node
    removeUnusedTimeNode();  
    return stat;                 
}

bool GVINS::removeUnusedTimeNode() {              
    if (unused_time_nodes_.empty()) {
        return false;
    }

    LOGI << "Remove " << unused_time_nodes_.size() << " unused time node "
         << Logging::doubleData(unused_time_nodes_[0]);

    for (double node : unused_time_nodes_) {
        int index = getStateDataIndex(node);

        // Exception
        if (index < 0) {
            continue;
        }

        auto first_preintegration  = preintegrationlist_[index - 1];
        auto second_preintegration = preintegrationlist_[index];
        auto imu_buffer            = second_preintegration->imuBuffer();

        // 将后一个预积分的IMU数据合并到前一个, 不包括第一个IMU数据
        // Merge the IMU preintegration
        for (size_t k = 1; k < imu_buffer.size(); k++) {
            first_preintegration->addNewImu(imu_buffer[k]);                     
        }

        // 移除时间节点, 以及后一个预积分
        // Remove the second time node
        preintegrationlist_.erase(preintegrationlist_.begin() + index);   
        timelist_.erase(timelist_.begin() + index);
        statedatalist_.erase(statedatalist_.begin() + index);          
    }
    unused_time_nodes_.clear();

    return true;
}

void GVINS::setFinished() {
    isfinished_ = true;

    // 释放信号量, 退出所有线程
    // Release all semaphores
    fusion_sem_.notify_all();
    tracking_sem_.notify_all();
    positioning_sem_.notify_all();
    optimization_sem_.notify_all();

    tracking_thread_.join();
    positioning_thread_.join();
    optimization_thread_.join();
    fusion_thread_.join();

    if (is_use_visualization_) {
        drawer_->setFinished();
        drawer_thread_.join();
    }

    Quaterniond q_b_c = Rotation::matrix2quaternion(pose_b_c_.R);
    Vector3d t_b_c    = pose_b_c_.t;

    LOGW << "GVINS has finished processing";
    LOGW << "Estimated extrinsics: "
         << absl::StrFormat("(%0.6lf, %0.6lf, %0.6lf, %0.6lf), (%0.3lf, %0.3lf, "
                            "%0.3lf), %0.4lf",
                            q_b_c.x(), q_b_c.y(), q_b_c.z(), q_b_c.w(), t_b_c.x(), t_b_c.y(), t_b_c.z(), td_b_c_);

    Logging::shutdownLogging();
}