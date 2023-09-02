
#include "ic_gvins.h"
#include "misc.h"

#include "common/earth.h"
#include "common/logging.h"

void GVINS::constructPrior(bool is_zero_velocity) {      
    double att_prior_std  = 5 * D2R;                                 // deg 
    double ba_prior_std   = 5.00000E-002;                            // m/s^2 
    double bg_prior_std   = 2.00000E-002 * D2R;                      // deg/sec     
    double vel_prior_std  = 1.00000E+000;                            // m/s
    double pos_prior_std  = 1.00000E+000;                            // m
    
    double sodo_prior_std = 0.005;                                   // 5000 PPM

    if (!is_zero_velocity) {
        bg_prior_std = 2 * D2R;                                      // 2 deg/sec
    }

    memcpy(pose_prior_, statedatalist_[0]->pose, sizeof(double) * 7);
    memcpy(mix_prior_,  statedatalist_[0]->mix,  sizeof(double) * 18);
    for (size_t k = 0; k < 3; k++) {
        pose_prior_std_[k + 0] = pos_prior_std;
        pose_prior_std_[k + 3] = att_prior_std;

        mix_prior_std_[k + 0] = vel_prior_std;
        mix_prior_std_[k + 3] = bg_prior_std;
        mix_prior_std_[k + 6] = ba_prior_std;
    }
    pose_prior_std_[5] = att_prior_std * 3; // heading
    mix_prior_std_[9]  = sodo_prior_std;
    is_use_prior_      = true;
}

bool GVINS::gvinsInitialization() {        

    isgnssfinished_ = false;
    auto gnsslist = obss_buffer_->popObssWithSolution();

    if (gnsslist.empty()) {
        return false;
    }

    while (!gnsslist.empty()) {
        auto gnss = gnsslist.front();
        last_last_gnss_ = last_gnss_;
        last_gnss_      = gnss_;
        gnss_           = GNSS(gnss.time, gnss.blh, gnss.std);
        gnsslist.pop();
    }

    if ((gnss_.time == 0) || (last_gnss_.time == 0)) {
        return false;
    }

    // 缓存数据用于零速检测
    // Buffer for zero-velocity detection
    vector<IMU> imu_buff;
    for (const auto &ins : ins_window_) {
        auto &imu = ins.first;
        if ((imu.time > last_gnss_.time) && (imu.time < gnss_.time)) {
            imu_buff.push_back(imu);
        }
    }
    if (imu_buff.size() < 20) {
        return false;
    }
    
    // 零速检测估计陀螺零偏和横滚俯仰角
    // Obtain the gyroscope biases and roll and pitch angles
    vector<double> average;
    static Vector3d bg{0, 0, 0};
    static Vector3d initatt{0, 0, 0};
    static bool is_has_zero_velocity = false;

    bool is_zero_velocity = MISC::detectZeroVelocity(imu_buff, imudatarate_, average);   
    if (is_zero_velocity) {
        // 陀螺零偏
        bg = Vector3d(average[0], average[1], average[2]);
        bg *= imudatarate_;

        // 重力调平获取横滚俯仰角
        Vector3d fb(average[3], average[4], average[5]);
        fb *= imudatarate_;

        initatt[0] = -asin(fb[1] / integration_parameters_->gravity);
        initatt[1] =  asin(fb[0] / integration_parameters_->gravity);

        LOGI << "Zero velocity get gyroscope bias " << bg.transpose() * R2D << " (deg/sec), roll " << initatt[0] * R2D
             << ", pitch " << initatt[1] * R2D << " (deg)";
        is_has_zero_velocity = true;
    }
    
    // 非零速状态
    // Initialization conditions
    if (!is_zero_velocity) {
        Vector3d vel = gnss_.blh - last_gnss_.blh;       
        if (vel.norm() < MINMUM_ALIGN_VELOCITY) {
            return false;
        }

        if (!is_has_zero_velocity) {
            initatt[0] = 0;
            initatt[1] = atan(-vel.z() / sqrt(vel.x() * vel.x() + vel.y() * vel.y()));
            LOGI << "Initialized pitch from GNSS as " << initatt[1] * R2D << " deg";
        }
        initatt[2] = atan2(vel.y(), vel.x());
        LOGI << "Initialized heading from GNSS as " << initatt[2] * R2D << " deg";
    } else {
        return false;
    }

    // 从零速开始
    Vector3d velocity = Vector3d::Zero();

    // 初始状态, 从上一秒开始
    // The initialization state
    auto state = IntegrationState{
        .time = last_gnss_.time,
        .p    = last_gnss_.blh - Rotation::euler2quaternion(initatt) * antlever_,
        .q    = Rotation::euler2quaternion(initatt),
        .v    = velocity,
        .bg   = bg,
        .ba   = {0, 0, 0},
        .sodo = 0.0,
        .sg   = {0, 0, 0},
        .sa   = {0, 0, 0},
    };
    statedatalist_.emplace_back(Preintegration::stateToData(state, preintegration_options_));    
    gnsslist_.push_back(last_gnss_);                       
    timelist_.push_back(last_gnss_.time);
    constructPrior(is_has_zero_velocity);

    // 初始化重力和地球自转参数
    // The gravity and the Earth rotation rate
    integration_config_.gravity = Vector3d(0, 0, integration_parameters_->gravity);
    if (integration_config_.iswithearth) {
        integration_config_.iewn = Earth::iewn(integration_config_.origin, state.p);
    }  

    LOGI << "Initialization at " << Logging::doubleData(gnss_.time);

    // 加入当前GNSS时间节点
    // Add current GNSS time node
    addNewTimeNode(gnss_.time);
    gnsslist_.push_back(gnss_);            

    return true;
}

void GVINS::gnssOutlierCullingByChi2(ceres::Problem &problem,
                                     vector<std::pair<ceres::ResidualBlockId, GNSS *>> &redisual_block) {
    double chi2_threshold = 7.815;
    double cost, chi2;

    int outliers_counts = 0;
    for (auto &block : redisual_block) {
        auto id    = block.first;
        GNSS *gnss = block.second;

        problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);      // 通过id可以获取
        chi2 = cost * 2;

        if (chi2 > chi2_threshold) {

            // Reweigthed GNSS
            double scale = sqrt(chi2 / chi2_threshold);
            // gnss->std *= scale;                                                 // 观测值进行改正

            LOGW << "GNSS outliers at " << Logging::doubleData(gnss->time) << "  scale=" 
                 << scale << "  " << gnss->std[0] << ", " << gnss->std[1] << ", " << gnss->std[2];

            outliers_counts++;
        }
    }

    if (outliers_counts) {
        LOGI << "Detect " << outliers_counts << " GNSS outliers at " << Logging::doubleData(timelist_.back());
    }
}

void GVINS::removeReprojectionFactorsByChi2(ceres::Problem &problem, vector<ceres::ResidualBlockId> &residual_ids,
                                           double chi2) {
    double cost;
    int outlier_features = 0, remove_num = 0;

    // 进行卡方检验, 判定粗差因子, 待全部判定完成再进行移除, 否则会导致错误
    // Judge first and remove later
    vector<ceres::ResidualBlockId> outlier_residual_ids;
    for (auto &id : residual_ids) {
        problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);

        // cost带有1/2系数
        // To chi2
        if (cost * 2.0 > chi2) {
            outlier_features++;
            outlier_residual_ids.push_back(id);
        }
    }
    
    // 从优化问题中移除所有粗差因子
    // Remove the outliers from the optimizer
    for (auto &id : outlier_residual_ids) { 
        ulong mappointId = residualIdToMappointId_[id];

        if (--mappoint_used_times_[mappointId] < 1) {
            problem.RemoveParameterBlock(&invdepthlist_[mappointId]);
            invdepthlist_.erase(mappointId);            
            remove_num++;    
            continue;
        };             
        problem.RemoveResidualBlock(id);           
    }

    LOGI << "Remove " << outlier_features << " reprojection factors  " << remove_num << "  invdepths";
}

int GVINS::getStateDataIndex(double time) {

    size_t index = MISC::getStateDataIndex(timelist_, time, MISC::MINIMUM_TIME_INTERVAL);
    if (!MISC::isTheSameTimeNode(timelist_[index], time, MISC::MINIMUM_TIME_INTERVAL)) {
        LOGE << "Wrong matching time node " << Logging::doubleData(timelist_[index]) << " to "
             << Logging::doubleData(time);
        return -1;
    }
    return static_cast<int>(index);
}

void GVINS::updateParametersFromOptimizer() {           
    if (map_->keyframes().empty()) {
        return;
    }

    // 先更新外参, 更新位姿需要外参
    // Update the extrinsic first
    {
        if (optimize_estimate_td_) {                   
            // Update the extrinsic
            Lock lock(extrinsic_mutex_);
            td_b_c_ = extrinsic_[7];
        }

        if (optimize_estimate_extrinsic_) {
            Pose ext;
            ext.t[0] = extrinsic_[0];
            ext.t[1] = extrinsic_[1];
            ext.t[2] = extrinsic_[2];

            Quaterniond qic = Quaterniond(extrinsic_[6], extrinsic_[3], extrinsic_[4], extrinsic_[5]);      
            ext.R           = Rotation::quaternion2matrix(qic.normalized());

            // 外参估计检测, 误差较大则不更新, 1m or 5deg
            double dt = (ext.t - pose_b_c_.t).norm();
            double dr = Rotation::matrix2quaternion(ext.R * pose_b_c_.R.transpose()).vec().norm() * R2D;
            if ((dt > 1.0) || (dr > 5.0)) {
                LOGE << "Estimated extrinsic is too large, t: " << ext.t.transpose()
                     << ", R: " << Rotation::matrix2euler(ext.R).transpose() * R2D;
            } else {
                // Update the extrinsic
                Lock lock(extrinsic_mutex_);       
                pose_b_c_ = ext;
            }

            vector<double> extrinsic;
            Vector3d euler = Rotation::matrix2euler(ext.R) * R2D;

            extrinsic.push_back(timelist_.back());
            extrinsic.push_back(ext.t[0]);
            extrinsic.push_back(ext.t[1]);
            extrinsic.push_back(ext.t[2]);
            extrinsic.push_back(euler[0]);
            extrinsic.push_back(euler[1]);
            extrinsic.push_back(euler[2]);
            extrinsic.push_back(td_b_c_);

            extfilesaver_->dump(extrinsic);
            extfilesaver_->flush();
        }
    }

    // 更新关键帧的位姿
    // Update the keyframe pose
    for (auto &keyframe : map_->keyframes()) {          
        auto &frame = keyframe.second;
        auto index  = getStateDataIndex(frame->stamp());
        if (index < 0) {
            continue;
        }

        IntegrationState state = Preintegration::stateFromData(*statedatalist_[index], preintegration_options_);     
        frame->setPose(MISC::stateToCameraPose(state, pose_b_c_));                
    }

    // 更新路标点的深度和位置
    // Update the mappoints
    for (const auto &landmark : map_->landmarks()) {
        const auto &mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        auto frame = mappoint->referenceFrame();
        if (!frame || !map_->isKeyFrameInMap(frame)) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        double invdepth = invdepthlist_[mappoint->id()];
        double depth    = 1.0 / invdepth;

        auto pc0      = camera_->pixel2cam(mappoint->referenceKeypoint());
        Vector3d pc00 = {pc0.x(), pc0.y(), 1.0};
        pc00 *= depth;

        mappoint->pos() = camera_->cam2world(pc00, mappoint->referenceFrame()->pose());        
        mappoint->updateDepth(depth);
    }
}

void GVINS::updateObssBufferFromOptimizer(ceres::Problem &problem) {
    bool flag = false;
    Vector3d parameter1, parameter2;
    std::queue<double> times;
    std::queue<double> nodes;
    std::queue<size_t> index;
    std::queue<Vector3d> parameters;
    std::queue<Matrix3d> covariances;

    obss_buffer_->getObssInNode(index, times, nodes);

    // Make pairs
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    while (!nodes.empty()) {
        auto node = nodes.front();
        auto dt = times.front() - node;
        auto state = statedatalist_[getStateDataIndex(node)];
        
        parameter1[0] = state->pose[0] + state->mix[0] * dt; 
        parameter1[1] = state->pose[1] + state->mix[1] * dt;
        parameter1[2] = state->pose[2] + state->mix[2] * dt;
        parameter2[0] = state->mix[0];
        parameter2[1] = state->mix[1];
        parameter2[2] = state->mix[2];
        Quaterniond q( state->pose[6], state->pose[3], state->pose[4], state->pose[5]);
        parameter1    = parameter1 + q.toRotationMatrix() * antlever_;        
        parameter1    = Earth::blh2ecef(Earth::local2global(integration_config_.origin, parameter1));
        // parameter2    = parameter2 + q.toRotationMatrix() * antlever_;   // 旋转时的速度没处理
        parameters.emplace(parameter1);
        parameters.emplace(parameter2);
        covariance_blocks.emplace_back(state->pose, state->pose);
        times.pop();
        nodes.pop();
    }
    
    // Compute covariance & Get covariance
    ceres::Covariance::Options CovOptions;
    ceres::Covariance covariance_handle(CovOptions);                
    if (!covariance_handle.Compute(covariance_blocks, &problem)) {
        flag = true;
        LOG(WARNING) << "Failed to compute covariance! ";
    } else {
        Eigen::MatrixXd Cov = Eigen::MatrixXd::Zero(7, 7);
        Matrix3d Cne = Earth::cne(integration_config_.origin);
        for (size_t i = 0; i < index.size(); i++) {
            auto block = covariance_blocks[i];
            covariance_handle.GetCovarianceBlock(block.first, block.second, Cov.data());
            covariances.emplace(Cne.transpose() * Cov.block<3, 3>(0, 0) * Cne);    
        }
    }

    obss_buffer_->updateObssInNode(flag, index, parameters, covariances);
}

void GVINS::getConveriance(ceres::Problem &problem) {

    double Cov_dt;
    vector<double> result;
    Vector3d pos, att;
    bool estimate_extrinsic = false, estimate_td = false;
    Eigen::MatrixXd Cov_ext = Eigen::MatrixXd::Zero(7, 7);
    

    // Make pairs
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    auto state = statedatalist_.back();
    covariance_blocks.emplace_back(state->pose, state->pose);
    covariance_blocks.emplace_back(state->mix, state->mix);
    if (optimize_estimate_extrinsic_ && gvinsstate_ == GVINS_TRACKING_NORMAL) {
        covariance_blocks.emplace_back(extrinsic_, extrinsic_);
        estimate_extrinsic = true;
    }
    if (optimize_estimate_td_        && gvinsstate_ == GVINS_TRACKING_NORMAL) {
        covariance_blocks.emplace_back(&extrinsic_[7], &extrinsic_[7]);
        estimate_td = true;
    }
    
    // Compute covariance & Get covariance
    ceres::Covariance::Options CovOptions;
    ceres::Covariance covariance_handle(CovOptions);                
    if (!covariance_handle.Compute(covariance_blocks, &problem)) {
        LOG(WARNING) << "Failed to compute covariance!";
    } else {
        Eigen::MatrixXd Cov_pos = Eigen::MatrixXd::Zero(7, 7);
        auto block0 = covariance_blocks[0];
        covariance_handle.GetCovarianceBlock(block0.first, block0.second, Cov_pos.data());

        auto numMix = Preintegration::numMixParameter(preintegration_options_);
        Eigen::MatrixXd Cov_vel = Eigen::MatrixXd::Zero(numMix, numMix);
        auto block1 = covariance_blocks[1];
        covariance_handle.GetCovarianceBlock(block1.first, block1.second, Cov_vel.data());
        
        if (estimate_extrinsic) {
            auto block2 = covariance_blocks[2];
            covariance_handle.GetCovarianceBlock(block2.first, block2.second, Cov_ext.data());
        }
        if (estimate_td) {
            auto block3 = covariance_blocks[3];
            covariance_handle.GetCovarianceBlock(block3.first, block3.second, &Cov_dt);
        }

        Quaterniond q{state->pose[6], state->pose[3], state->pose[4], state->pose[5]};
        att = Rotation::quaternion2euler(q);
        pos[0] = state->pose[0];
        pos[1] = state->pose[1];
        pos[2] = state->pose[2];
        pos    = pos + q.toRotationMatrix() * antlever_;
        pos    = Earth::local2global(integration_config_.origin, pos);
        auto Cov_att = Cov_pos.block<3, 3>(3, 3);

        result.clear();
        result.push_back(state->time);

        /* parameters */
        for (int i = 0; i < 3; i++) result.push_back(att[i]);
        for (int i = 0; i < 3; i++) result.push_back(state->mix[i]);
        for (int i = 0; i < 3; i++) result.push_back(pos[i]);
        for (int i = 3; i < 9; i++) result.push_back(state->mix[i]);
        if (estimate_extrinsic) {
            Quaterniond qic = Quaterniond(extrinsic_[6], extrinsic_[3], extrinsic_[4], extrinsic_[5]); 
            att = Rotation::quaternion2euler(qic);
            for (int i = 0; i < 3; i++) result.push_back(extrinsic_[i]);
            for (int i = 0; i < 3; i++) result.push_back(att[i]);
        } else {
            for (int i = 0; i < 6; i++) result.push_back(0.0);
        }
        if (estimate_td) {
            result.push_back(extrinsic_[7]);
        } else {
            result.push_back(0.0);
        }
        
        /* covariance */
        for (int i = 0; i < 3; i++) result.push_back(sqrt(Cov_att(i,i)));       
        for (int i = 0; i < 3; i++) result.push_back(sqrt(Cov_vel(i,i)));
        for (int i = 0; i < 3; i++) result.push_back(sqrt(Cov_pos(i,i)));
        for (int i = 3; i < 9; i++) result.push_back(sqrt(Cov_vel(i,i)));
        if (estimate_extrinsic) { 
            for (int i = 0; i < 6; i++) result.push_back(sqrt(Cov_ext(i,i)));
        } else {
            for (int i = 0; i < 6; i++) result.push_back(0.0);
        }
        if (estimate_td) {
            result.push_back(sqrt(Cov_dt));
        } else {
            result.push_back(0.0);
        }

        covfilesaver_->dump(result);
        covfilesaver_->flush();
    }
}

bool GVINS::mappointOutlierCulling() {                    
    if (map_->keyframes().empty()) {
        return false;
    }

    // 移除非关键帧中的路标点, 不能在遍历中直接移除, 否则破坏了遍历
    // Find outliers first and remove later
    vector<MapPoint::Ptr> mappoints;
    int num_outliers_mappoint = 0;
    int num_outliers_feature  = 0;
    int num1 = 0, num2 = 0, num3 = 0;
    for (auto &landmark : map_->landmarks()) {          
        auto mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        // 路标点在滑动窗口内的所有观测
        // All the observations for mappoint
        vector<double> errors;
        for (auto &observation : mappoint->observations()) {
            auto feat = observation.lock();         // 弱引用 >> 强引用
            if (!feat || feat->isOutlier()) {
                continue;
            }
            auto frame = feat->getFrame();
            if (!frame || !frame->isKeyFrame() || !map_->isKeyFrameInMap(frame)) {
                continue;
            }

            auto pp = feat->keyPoint();

            // 计算重投影误差
            // Calculate the reprojection error
            double error = camera_->reprojectionError(frame->pose(), mappoint->pos(), pp).norm();

            // 大于3倍阈值, 则禁用当前观测
            // Feature outlier
            if (!tracking_->isGoodToTrack(pp, frame->pose(), mappoint->pos(), 3.0)) {
                feat->setOutlier(true);                 
                mappoint->decreaseUsedTimes();                  // 减少了使用次数

                // 如果当前观测帧是路标点的参考帧, 直接设置为outlier
                // Mappoint
                if (frame->id() == mappoint->referenceFrameId()) {         
                    mappoint->setOutlier(true);
                    mappoints.push_back(mappoint);
                    num_outliers_mappoint++;
                    num1++;
                    break;
                }
                num_outliers_feature++;
            } else {
                errors.push_back(error);
            }
        }

        // 有效观测不足, 平均重投影误差较大, 则为粗差
        // Mappoint outlier
        if (errors.size() < 2) {           
            mappoint->setOutlier(true);
            mappoints.push_back(mappoint);            
            num_outliers_mappoint++;
            num2++;
        } else {
            double avg_error = std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
            if (avg_error > reprojection_error_std_) {
                mappoint->setOutlier(true);
                mappoints.push_back(mappoint);
                num_outliers_mappoint++;
                num3++;
            }
        }
    }

    // 移除outliers
    // Remove the mappoint outliers
    for (auto &mappoint : mappoints) {                          
        map_->removeMappoint(mappoint);                  
    }

    LOGI << "Culled " << num_outliers_mappoint << " mappoint with " << num_outliers_feature << " bad observed features "
         << num1 << ", " << num2 << ", " << num3;
    outliers_[0] = num_outliers_mappoint;
    outliers_[1] = num_outliers_feature;

    return true;
}

bool GVINS::RemoveAllSecondNewFrame() {                   
    vector<ulong> keyframeids = map_->orderedKeyFrames();

    for (auto id : keyframeids) {
        auto frame = map_->keyframes().find(id)->second;
        // 移除次新帧, 以及倒数第二个空关键帧
        if ((frame->keyFrameState() == KEYFRAME_REMOVE_SECOND_NEW) ||
            (frame->features().empty() && (id != keyframeids.back()))) {
            
            unused_time_nodes_.push_back(frame->stamp());                                                    
            
            // 仅需要重置关键帧标志, 从地图中移除次新关键帧即可,
            // 无需调整状态参数和路标点
            // Just remove the frame
            frame->resetKeyFrame();                        
            map_->removeKeyFrame(frame, false);                    // 仅删除地图点的使用次数 
            LOGI << "RemoveAllSecondNewFrame  " << unused_time_nodes_.back();                                                     
        }
    }

    return true;
}

void GVINS::doReintegration() {
    int cnt = 0;
    for (size_t k = 0; k < preintegrationlist_.size(); k++) {
        IntegrationState state = Preintegration::stateFromData(*statedatalist_[k], preintegration_options_);
        Vector3d dbg           = preintegrationlist_[k]->deltaState().bg - state.bg;
        Vector3d dba           = preintegrationlist_[k]->deltaState().ba - state.ba;
        if ((dbg.norm() > 6 * integration_parameters_->gyr_bias_std.norm()) ||
            (dba.norm() > 6 * integration_parameters_->acc_bias_std.norm())) {         // 需要 改！！！

            // printf("%10.2f dbg=[%9.6f, %9.6f, %9.6f] dba=[%7.4f, %7.4f, %7.4f] [%4d %4d]\n", 
            //     state.time, dbg[0]*R2D, dbg[1]*R2D, dbg[2]*R2D, dba[0], dba[1], dba[2], 
            //     dbg.norm() > 6 * integration_parameters_->gyr_bias_std,
            //     dba.norm() > 6 * integration_parameters_->acc_bias_std);
            
            preintegrationlist_[k]->reintegration(state);  
                     
            cnt++;
        }
    }
    if (cnt) {
        LOGW << "Reintegration " << cnt << " preintegration";
    }
}

void GVINS::parametersStatistic() {

    vector<double> parameters;

    // 所有关键帧
    // All keyframes
    vector<ulong> keyframeids = map_->orderedKeyFrames();
    size_t size               = keyframeids.size();
    if (size < 2) {
        return;
    }
    auto keyframes = map_->keyframes();

    // 最新的关键帧
    // The latest keyframe
    auto frame_cur = keyframes.at(keyframeids[size - 1]);
    auto frame_pre = keyframes.at(keyframeids[size - 2]);

    // 时间戳
    // Time stamp
    parameters.push_back(frame_cur->stamp());           //关键帧处的位姿
    parameters.push_back(frame_cur->stamp() - frame_pre->stamp());

    // 当前关键帧与上一个关键帧的id差, 即最新关键帧的跟踪帧数
    // Interval
    auto frame_cnt = static_cast<double>(frame_cur->id() - frame_pre->id());
    parameters.push_back(frame_cnt);

    // 特征点数量
    // Feature points
    parameters.push_back(static_cast<double>(frame_cur->numFeatures()));

    // 路标点重投影误差统计
    // Reprojection errors
    vector<double> reprojection_errors;
    for (auto &landmark : map_->landmarks()) {
        auto mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        vector<double> errors;
        for (auto &observation : mappoint->observations()) {
            auto feat = observation.lock();
            if (!feat || feat->isOutlier()) {
                continue;
            }
            auto frame = feat->getFrame();
            if (!frame || !frame->isKeyFrame() || !map_->isKeyFrameInMap(frame)) {
                continue;
            }

            double error = camera_->reprojectionError(frame->pose(), mappoint->pos(), feat->keyPoint()).norm();
            errors.push_back(error);
        }
        if (errors.empty()) {
            LOGE << "Mappoint " << mappoint->id() << " with zero observation";
            continue;
        }
        double avg_error = std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
        reprojection_errors.emplace_back(avg_error);
    }

    if (reprojection_errors.empty()) {
        reprojection_errors.push_back(0);
    }

    double min_error = *std::min_element(reprojection_errors.begin(), reprojection_errors.end());
    parameters.push_back(min_error);
    double max_error = *std::max_element(reprojection_errors.begin(), reprojection_errors.end());
    parameters.push_back(max_error);
    double avg_error = std::accumulate(reprojection_errors.begin(), reprojection_errors.end(), 0.0) /
                       static_cast<double>(reprojection_errors.size());
    parameters.push_back(avg_error);
    double sq_sum =
        std::inner_product(reprojection_errors.begin(), reprojection_errors.end(), reprojection_errors.begin(), 0.0);
    double rms_error = std::sqrt(sq_sum / static_cast<double>(reprojection_errors.size()));
    parameters.push_back(rms_error);

    // 迭代次数
    // Iterations
    parameters.push_back(iterations_[0]);
    parameters.push_back(iterations_[1]);

    // 计算耗时
    // Time cost
    parameters.push_back(timecosts_[0]);
    parameters.push_back(timecosts_[1]);
    parameters.push_back(timecosts_[2]);

    // 路标点粗差
    // Outliers
    parameters.push_back(outliers_[0]);
    parameters.push_back(outliers_[1]);

    // 保存数据
    // Dump current parameters
    statfilesaver_->dump(parameters);
    statfilesaver_->flush();
}