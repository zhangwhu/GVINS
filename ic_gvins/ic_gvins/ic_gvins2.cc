
#include "ic_gvins.h"
#include "misc.h"

#include "common/angle.h"
#include "common/earth.h"
#include "common/gpstime.h"
#include "common/logging.h"

#include "factors/gnss_factor.h"
#include "factors/marginalization_factor.h"
#include "factors/marginalization_info.h"
#include "factors/pose_parameterization.h"
#include "factors/reprojection_factor.h"
#include "factors/residual_block_info.h"
#include "preintegration/imu_error_factor.h"
#include "preintegration/imu_mix_prior_factor.h"
#include "preintegration/imu_pose_prior_factor.h"
#include "preintegration/preintegration.h"
#include "preintegration/preintegration_factor.h"

#include <ceres/ceres.h>

bool GVINS::gvinsInitializationOptimization() {
    // GNSS/INS optimization

    // 构建优化问题
    ceres::Solver solver;
    ceres::Problem problem;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations         = 50;

    // 参数块
    // Add parameter blocks
    addStateParameters(problem);            
    
    // GNSS残差
    // Add gnss factors
    addGnssFactors(problem, true);
    
    // 预积分残差
    // Add IMU preintegration factors
    addImuFactors(problem);
    
    solver.Solve(options, &problem, &summary);
    LOGI << summary.BriefReport();
    
    // 进行必要的重积分(初始化期间)
    // Reintegration during initialization
    doReintegration();  

    return summary.termination_type == ceres::CONVERGENCE;
}

bool GVINS::GIOptimization() {
    static int first_num_iterations  = optimize_num_iterations_ / 4;
    static int second_num_iterations = optimize_num_iterations_ - first_num_iterations; 

    TimeCost timecost;

    ceres::Problem::Options problem_options;
    problem_options.enable_fast_removal = true;        

    ceres::Problem problem(problem_options);
    ceres::Solver solver;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations         = first_num_iterations;
    options.num_threads                = 4;

    // 状态参数
    // State parameters
    addStateParameters(problem);        

    // 边缘化残差
    // The prior factor
    if (last_marginalization_info_ && last_marginalization_info_->isValid()) {                    
        auto factor = new MarginalizationFactor(last_marginalization_info_);                 
        problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks_);   
    }

    // GNSS残差
    // The GNSS factors
    auto gnss_redisual_block = addGnssFactors(problem, true); 

    // 预积分残差
    // Add IMU preintegration factors
    addImuFactors(problem);

    LOGI << "Add " << preintegrationlist_.size() << " preintegration, " << gnsslist_.size() << " GNSS.";

    // 第一次优化
    // The first optimization
    {
        timecost.restart();

        solver.Solve(options, &problem, &summary);
        LOGI << summary.BriefReport();

        iterations_[0] = summary.num_successful_steps;
        timecosts_[0]  = timecost.costInMillisecond();
    }

    // 粗差检测和剔除
    // Outlier detetion for GNSS
    {
        // Do GNSS outlier culling
        gnssOutlierCullingByChi2(problem, gnss_redisual_block);      

        // Remove all GNSS factors
        for (auto &block : gnss_redisual_block) {
            problem.RemoveResidualBlock(block.first);                                    
        }

        // Add GNSS Factors without loss function
        addGnssFactors(problem, false);                                  
    }

    // 第二次优化
    // The second optimization
    {
        options.max_num_iterations = second_num_iterations;

        timecost.restart();

        solver.Solve(options, &problem, &summary);
        LOGI << summary.BriefReport();

        iterations_[1] = summary.num_successful_steps;
        timecosts_[1]  = timecost.costInMillisecond();  

        // 进行必要的重积分
        // Reintegration during initialization
        doReintegration();                                       
    }

    // 更新GNSS先验信息
    // Update the prior of GNSS from the optimizer
    updateObssBufferFromOptimizer(problem);

    // 提取协方差
    getConveriance(problem);

    return true;
}

bool GVINS::GIMarginalization() {

    std::shared_ptr<MarginalizationInfo> marginalization_info = std::make_shared<MarginalizationInfo>();

    // 指定每个参数块独立的ID, 用于索引参数
    // For fixed order
    std::unordered_map<long, long> parameters_ids;         
    parameters_ids.clear();
    long parameters_id = 0;

    {
        // 边缘化参数
        // Marginalization parameters
        for (auto &last_marginalization_parameter_block : last_marginalization_parameter_blocks_) {      
            parameters_ids[reinterpret_cast<long>(last_marginalization_parameter_block)] = parameters_id++;
        }

        // 位姿参数
        // Pose parameters
        for (const auto &statedata : statedatalist_) {
            parameters_ids[reinterpret_cast<long>(statedata->pose)] = parameters_id++;
            parameters_ids[reinterpret_cast<long>(statedata->mix)]  = parameters_id++;
        }

        // 更新参数块的特定ID, 必要的
        // Update the IS for parameters
        marginalization_info->updateParamtersIds(parameters_ids);           
    }

    // 边缘化
    // marginalization
    if (last_marginalization_info_ && last_marginalization_info_->isValid()) {

        std::vector<int> marginilized_index;
        for (size_t k = 0; k < last_marginalization_parameter_blocks_.size(); k++) {
            if (last_marginalization_parameter_blocks_[k] == statedatalist_[0]->pose ||
                last_marginalization_parameter_blocks_[k] == statedatalist_[0]->mix) {
                marginilized_index.push_back(static_cast<int>(k));
            }
        }

        auto factor   = std::make_shared<MarginalizationFactor>(last_marginalization_info_);
        auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr, 
            last_marginalization_parameter_blocks_, marginilized_index);
        marginalization_info->addResidualBlockInfo(residual);
    }

    // 先验约束因子
    // The prior factor
    if (is_use_prior_) {            
        auto pose_factor   = std::make_shared<ImuPosePriorFactor>(pose_prior_, pose_prior_std_);
        auto pose_residual = std::make_shared<ResidualBlockInfo>(
            pose_factor, nullptr, std::vector<double *>{statedatalist_[0]->pose}, vector<int>{0});
        marginalization_info->addResidualBlockInfo(pose_residual);

        auto mix_factor   = std::make_shared<ImuMixPriorFactor>(preintegration_options_, mix_prior_, mix_prior_std_);
        auto mix_residual = std::make_shared<ResidualBlockInfo>(
            mix_factor, nullptr, std::vector<double *>{statedatalist_[0]->mix}, vector<int>{0});
        marginalization_info->addResidualBlockInfo(mix_residual);

        is_use_prior_ = false;
    }
    
    // IMU残差
    // preintegration factors
    {
        auto factor   = std::make_shared<PreintegrationFactor>(preintegrationlist_[0]);
        auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr,
            std::vector<double *>{statedatalist_[0]->pose, statedatalist_[0]->mix, 
                                  statedatalist_[1]->pose, statedatalist_[1]->mix},
            std::vector<int>{0, 1});
        marginalization_info->addResidualBlockInfo(residual);
    }
    
    // GNSS残差
    // GNSS factors
    {
        auto factor   = std::make_shared<GnssFactor>(gnsslist_[0], antlever_);
        auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr, 
            std::vector<double *>{statedatalist_[0]->pose}, std::vector<int>{0});
        marginalization_info->addResidualBlockInfo(residual);
    }
    
    // 边缘化处理
    // do marginalization
    marginalization_info->marginalization();
    
    // 数据指针调整
    // get new pointers
    std::unordered_map<long, double *> address;
    for (size_t k = 1; k <= preintegrationlist_.size(); k++) {
        address[parameters_ids[reinterpret_cast<long>(statedatalist_[k]->pose)]] = statedatalist_[k]->pose;             
        address[parameters_ids[reinterpret_cast<long>(statedatalist_[k]->mix)]]  = statedatalist_[k]->mix;
    }
    last_marginalization_parameter_blocks_ = marginalization_info->getParamterBlocks(address);
    last_marginalization_info_             = std::move(marginalization_info);

    // 滑窗处理
    // sliding window
    gnsslist_.pop_front();
    timelist_.pop_front();
    statedatalist_.pop_front();
    preintegrationlist_.pop_front();

    LOGI << "GNSS/INS:  pre_num=" << preintegrationlist_.size() << "  gnss=" << gnsslist_.size(); 

    return true;
}

bool GVINS::GVIOptimization() {
    static int first_num_iterations  = optimize_num_iterations_ / 4;
    static int second_num_iterations = optimize_num_iterations_ - first_num_iterations; 

    TimeCost timecost;

    ceres::Problem::Options problem_options;
    problem_options.enable_fast_removal = true;        

    ceres::Problem problem(problem_options);
    ceres::Solver solver;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type         = ceres::DENSE_SCHUR;
    options.max_num_iterations         = first_num_iterations;
    options.num_threads                = 4;

    // 状态参数
    // State parameters
    addStateParameters(problem);        

    // 重投影参数
    // Visual parameters
    addReprojectionParameters(problem);                                                    

    // 边缘化残差
    // The prior factor
    if (last_marginalization_info_ && last_marginalization_info_->isValid()) {                    
        auto factor = new MarginalizationFactor(last_marginalization_info_);                 
        problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks_);
    }

    // GNSS残差
    // The GNSS factors
    auto gnss_redisual_block = addGnssFactors(problem, true);   

    // 预积分残差
    // The IMU preintegration factors
    addImuFactors(problem);

    // 视觉重投影残差
    // The visual reprojection factors
    auto residual_ids = addReprojectionFactors(problem, true);                                                 

    LOGI << "Add " << preintegrationlist_.size() << " preintegration, " << gnsslist_.size() << " GNSS, "
         << residual_ids.size() << " reprojection";

    // 第一次优化
    // The first optimization
    {
        timecost.restart();

        solver.Solve(options, &problem, &summary);
        LOGI << summary.BriefReport();

        iterations_[0] = summary.num_successful_steps;
        timecosts_[0]  = timecost.costInMillisecond();
    }

    // 粗差检测和剔除
    // Outlier detetion for GNSS and visual
    {
        // Remove factors in the final

        // Do GNSS outlier culling
        gnssOutlierCullingByChi2(problem, gnss_redisual_block);                                  

        // Remove outlier reprojection factors 
        removeReprojectionFactorsByChi2(problem, residual_ids, 5.991);      // 深度信息删除后，是否将地图点删除         

        // Remove all GNSS factors
        for (auto &block : gnss_redisual_block) {
            problem.RemoveResidualBlock(block.first);                                    
        }

        // Add GNSS Factors without loss function
        addGnssFactors(problem, false);                                  
    }

    // 第二次优化
    // The second optimization
    {
        options.max_num_iterations = second_num_iterations;

        timecost.restart();

        solver.Solve(options, &problem, &summary);
        LOGI << summary.BriefReport();

        iterations_[1] = summary.num_successful_steps;
        timecosts_[1]  = timecost.costInMillisecond();

        if (!map_->isMaximumKeframes()) {          
            // 进行必要的重积分(初始化期间)
            // Reintegration during initialization
            doReintegration();                                             
        }
    }

    // 更新参数, 必须的
    // Update the parameters from the optimizer
    updateParametersFromOptimizer();                                 

    // 移除粗差路标点
    // Remove mappoint and feature outliers
    mappointOutlierCulling();  

    // 更新GNSS先验信息
    // Update the prior of GNSS from the optimizer
    updateObssBufferFromOptimizer(problem);

    getConveriance(problem);

    return true;
}

bool GVINS::GVIMarginalization() {

    // 按时间先后排序的关键帧
    // Ordered keyframes
    vector<ulong> keyframeids = map_->orderedKeyFrames();
    auto latest_keyframe      = map_->latestKeyFrame();

    latest_keyframe->setKeyFrameState(KEYFRAME_NORMAL);        

    // 对齐到保留的最后一个关键帧, 可能移除多个预积分对象
    // Align to the last keyframe time
    auto frame      = map_->keyframes().find(keyframeids[1])->second;               // 这里的bug要想一下怎么搞，出现的原因要分析出来
    size_t num_marg = getStateDataIndex(frame->stamp());                            

    double last_time = timelist_[num_marg];

    LOGI << "Marginalize " << num_marg << " states, last time " << Logging::doubleData(last_time);

    std::shared_ptr<MarginalizationInfo> marginalization_info = std::make_shared<MarginalizationInfo>(); 

    // 指定每个参数块独立的ID, 用于索引参数
    // For fixed order
    std::unordered_map<long, long> parameters_ids;         
    parameters_ids.clear();
    long parameters_id = 0;
    
    {
        // 边缘化参数
        // Marginalization parameters（）
        for (auto &last_marginalization_parameter_block : last_marginalization_parameter_blocks_) {      
            parameters_ids[reinterpret_cast<long>(last_marginalization_parameter_block)] = parameters_id++;
        }

        // 外参参数
        // Extrinsic parameters
        parameters_ids[reinterpret_cast<long>(extrinsic_)]     = parameters_id++;
        parameters_ids[reinterpret_cast<long>(extrinsic_ + 7)] = parameters_id++;

        // 位姿参数
        // Pose parameters
        for (const auto &statedata : statedatalist_) {
            parameters_ids[reinterpret_cast<long>(statedata->pose)] = parameters_id++;
            parameters_ids[reinterpret_cast<long>(statedata->mix)]  = parameters_id++;
        }

        // 逆深度参数
        // Inverse depth parameters
        frame         = map_->keyframes().at(keyframeids[0]);
        auto features = frame->features();
        for (auto const &feature : features) {
            auto mappoint = feature.second->getMapPoint();          
            if (feature.second->isOutlier() || !mappoint || mappoint->isOutlier()) {
                continue;
            }

            if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
                continue;
            }

            if (mappoint->referenceFrame() != frame) {
                continue;
            }

            double *invdepth                                 = &invdepthlist_[mappoint->id()];
            parameters_ids[reinterpret_cast<long>(invdepth)] = parameters_id++;    
        }

        // 更新参数块的特定ID, 必要的
        // Update the IS for parameters
        marginalization_info->updateParamtersIds(parameters_ids);           
    }

    // 边缘化因子
    // The prior factor        
    if (last_marginalization_info_ && last_marginalization_info_->isValid()) {                     

        std::vector<int> marginalized_index;
        for (size_t i = 0; i < num_marg; i++) {                         
            for (size_t k = 0; k < last_marginalization_parameter_blocks_.size(); k++) {
                if (last_marginalization_parameter_blocks_[k] == statedatalist_[i]->pose ||
                    last_marginalization_parameter_blocks_[k] == statedatalist_[i]->mix) {          
                    marginalized_index.push_back((int) k);                                  
                }
            }
        }

        auto factor   = std::make_shared<MarginalizationFactor>(last_marginalization_info_); // a.残差（大小）  b.状态（大小）
        auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr, last_marginalization_parameter_blocks_,
                                                            marginalized_index);                    
        marginalization_info->addResidualBlockInfo(residual);           
    }

    // 先验约束因子
    // The prior factor
    if (is_use_prior_) {          
        auto pose_factor   = std::make_shared<ImuPosePriorFactor>(pose_prior_, pose_prior_std_);
        auto pose_residual = std::make_shared<ResidualBlockInfo>(
            pose_factor, nullptr, std::vector<double *>{statedatalist_[0]->pose}, vector<int>{0});
        marginalization_info->addResidualBlockInfo(pose_residual);

        auto mix_factor   = std::make_shared<ImuMixPriorFactor>(preintegration_options_, mix_prior_, mix_prior_std_);
        auto mix_residual = std::make_shared<ResidualBlockInfo>(
            mix_factor, nullptr, std::vector<double *>{statedatalist_[0]->mix}, vector<int>{0});
        marginalization_info->addResidualBlockInfo(mix_residual);

        is_use_prior_ = false;
    }

    // 预积分因子
    // The IMU preintegration factors
    for (size_t k = 0; k < num_marg; k++) {
        // 由于会移除多个预积分, 会导致出现保留和移除同时出现, 判断索引以区分
        // More than one may be removed
        vector<int> marg_index;
        if (k == (num_marg - 1)) {
            marg_index = {0, 1};
        } else {
            marg_index = {0, 1, 2, 3};
        }

        auto factor   = std::make_shared<PreintegrationFactor>(preintegrationlist_[k]);    
        auto residual = std::make_shared<ResidualBlockInfo>(
            factor, nullptr,
            std::vector<double *>{statedatalist_[k]->pose, statedatalist_[k]->mix, statedatalist_[k + 1]->pose,
                                  statedatalist_[k + 1]->mix},
            marg_index);
        marginalization_info->addResidualBlockInfo(residual);                             
    }

    // GNSS因子
    // The GNSS factors
    for (auto &gnss : gnsslist_) {
        for (size_t k = 0; k < num_marg; k++) {
            if (MISC::isTheSameTimeNode(gnss.time, timelist_[k], MISC::MINIMUM_TIME_INTERVAL)) {     
                auto factor   = std::make_shared<GnssFactor>(gnss, antlever_);                      
                auto residual = std::make_shared<ResidualBlockInfo>(
                    factor, nullptr, std::vector<double *>{statedatalist_[k]->pose}, std::vector<int>{0});       
                marginalization_info->addResidualBlockInfo(residual);       
                break;
            }
        }
    }

    // 重投影因子, 最老的关键帧
    // The visual reprojection factors
    frame         = map_->keyframes().at(keyframeids[0]);         
    auto features = frame->features();

    auto loss_function = std::make_shared<ceres::HuberLoss>(1.0);
    for (auto const &feature : features) {
        auto mappoint = feature.second->getMapPoint();
        if (feature.second->isOutlier() || !mappoint || mappoint->isOutlier()) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        auto ref_frame = mappoint->referenceFrame();                  
        if (ref_frame != frame) {
            continue;
        }

        auto ref_frame_pc      = camera_->pixel2cam(mappoint->referenceKeypoint());
        size_t ref_frame_index = getStateDataIndex(ref_frame->stamp());
        if (ref_frame_index < 0) {
            continue;
        }

        double *invdepth = &invdepthlist_[mappoint->id()];

        auto ref_feature = ref_frame->features().find(mappoint->id())->second;

        auto observations = mappoint->observations();      
        for (auto &observation : observations) {
            auto obs_feature = observation.lock();
            if (!obs_feature || obs_feature->isOutlier()) {
                continue;
            }
            auto obs_frame = obs_feature->getFrame();
            if (!obs_frame || !obs_frame->isKeyFrame() || !map_->isKeyFrameInMap(obs_frame) ||
                (obs_frame == ref_frame)) {                                         
                continue;
            }

            auto obs_frame_pc      = camera_->pixel2cam(obs_feature->keyPoint());
            size_t obs_frame_index = getStateDataIndex(obs_frame->stamp());

            if ((obs_frame_index < 0) || (ref_frame_index == obs_frame_index)) {
                LOGE << "Wrong matched mapoint keyframes " << Logging::doubleData(ref_frame->stamp()) << " with "
                     << Logging::doubleData(obs_frame->stamp());
                continue;
            }

            auto factor = std::make_shared<ReprojectionFactor>(
                ref_frame_pc, obs_frame_pc, ref_feature->velocityInPixel(), obs_feature->velocityInPixel(),
                ref_frame->timeDelay(), obs_frame->timeDelay(), optimize_reprojection_error_std_);          
            auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr,
                                                                vector<double *>{statedatalist_[ref_frame_index]->pose,
                                                                                 statedatalist_[obs_frame_index]->pose,
                                                                                 extrinsic_, invdepth, &extrinsic_[7]},
                                                                vector<int>{0, 3});         
            marginalization_info->addResidualBlockInfo(residual);                                           
        }
    }

    // 边缘化处理
    // Do marginalization
    marginalization_info->marginalization();                                                        

    // Update the address
    std::unordered_map<long, double *> address;
    for (size_t k = num_marg; k < statedatalist_.size(); k++) {                     
        address[parameters_ids[reinterpret_cast<long>(statedatalist_[k]->pose)]] = statedatalist_[k]->pose;             
        address[parameters_ids[reinterpret_cast<long>(statedatalist_[k]->mix)]]  = statedatalist_[k]->mix;
    }
    address[parameters_ids[reinterpret_cast<long>(extrinsic_)]]     = extrinsic_;
    address[parameters_ids[reinterpret_cast<long>(extrinsic_ + 7)]] = &extrinsic_[7];

    last_marginalization_parameter_blocks_ = marginalization_info->getParamterBlocks(address); 
    last_marginalization_info_             = std::move(marginalization_info);       // 转移（无内存的搬迁和拷贝）              

    // 移除边缘化的数据
    // Remove the marginalized data

    // The GNSS observations
    size_t num_gnss = gnsslist_.size();
    for (size_t k = 0; k < gnsslist_.size(); k++) {
        if (gnsslist_[k].time > last_time) {
            num_gnss = k;
            break;
        }
    }
    for (size_t k = 0; k < num_gnss; k++) {     
        gnsslist_.pop_front();
    }

    // 预积分观测及时间状态
    // The IMU preintegration and time nodes
    for (size_t k = 0; k < num_marg; k++) {                     // 1.结果输出（添加)        
        timelist_.pop_front();
        statedatalist_.pop_front();
        preintegrationlist_.pop_front();
    }

    // 保存移除的路标点, 用于可视化
    // The marginalized mappoints, for visualization
    frame    = map_->keyframes().at(keyframeids[0]);            // 这里来回进行处理
    features = frame->features();
    for (const auto &feature : features) {                  
        auto mappoint = feature.second->getMapPoint();
        if (feature.second->isOutlier() || !mappoint || mappoint->isOutlier()) {
            continue;
        }
        auto &pw = mappoint->pos();

        if (is_use_visualization_) {
            drawer_->addNewFixedMappoint(pw);
        }

        // 保存路标点
        // Save these mappoints to file
        ptsfilesaver_->dump(vector<double>{pw.x(), pw.y(), pw.z()});
    }

    // 关键帧
    // The marginalized keyframe
    map_->removeKeyFrame(frame, true); 
    LOGI << "GNSS/Vision/IMU:  frame_num=" << map_->keyframes().size() << "  point=" << map_->landmarks().size();                           

    return true;
}

/* 构建Ceres::Parameters */
void GVINS::addStateParameters(ceres::Problem &problem) {
    LOGI << "Total " << statedatalist_.size() << " pose states from "
         << Logging::doubleData(statedatalist_.front()->time) << " to "
         << Logging::doubleData(statedatalist_.back()->time);

    for (auto &statedata : statedatalist_) {       
        // 位姿
        // Pose
        ceres::LocalParameterization *parameterization = new (PoseParameterization);       
        problem.AddParameterBlock(statedata->pose, Preintegration::numPoseParameter(), parameterization);       

        // IMU mix parameters
        problem.AddParameterBlock(statedata->mix, Preintegration::numMixParameter(preintegration_options_));
    }
}

void GVINS::addReprojectionParameters(ceres::Problem &problem) {
    if (map_->landmarks().empty()) {
        return;
    }

    invdepthlist_.clear();   
    mappoint_used_times_.clear();                           
    for (const auto &landmark : map_->landmarks()) {
        const auto &mappoint = landmark.second;            
        if (!mappoint || mappoint->isOutlier()) {           
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {    
            auto frame = mappoint->referenceFrame();
            if (!frame || !map_->isKeyFrameInMap(frame)) {                    
                continue;
            }

            double depth         = mappoint->depth();
            double inverse_depth = 1.0 / depth;

            // 确保深度数值有效
            // For valid mappoints
            if (std::isnan(inverse_depth)) {                                // 地图深度不可用
                mappoint->setOutlier(true);
                LOGE << "Mappoint " << mappoint->id() << " is wrong with depth " << depth << " type "
                     << mappoint->mapPointType();
                continue;
            }

            invdepthlist_[mappoint->id()] = inverse_depth;   
            mappoint_used_times_[mappoint->id()] = 0;                   
            problem.AddParameterBlock(&invdepthlist_[mappoint->id()], 1);    

            mappoint->addOptimizedTimes();
        }
    }

    // 外参
    // Extrinsic parameters （相机和IMU间的外参估计）
    extrinsic_[0] = pose_b_c_.t[0];
    extrinsic_[1] = pose_b_c_.t[1];
    extrinsic_[2] = pose_b_c_.t[2];

    Quaterniond qic = Rotation::matrix2quaternion(pose_b_c_.R);
    qic.normalize();
    extrinsic_[3] = qic.x();
    extrinsic_[4] = qic.y();
    extrinsic_[5] = qic.z();
    extrinsic_[6] = qic.w();

    ceres::LocalParameterization *parameterization = new (PoseParameterization);
    problem.AddParameterBlock(extrinsic_, 7, parameterization);            

    if (!optimize_estimate_extrinsic_ || gvinsstate_ != GVINS_TRACKING_NORMAL) {
        problem.SetParameterBlockConstant(extrinsic_);              // 设置参数不优化
    }

    // 时间延时
    // Time delay
    extrinsic_[7] = td_b_c_;                                       
    problem.AddParameterBlock(&extrinsic_[7], 1);
    if (!optimize_estimate_td_ || gvinsstate_ != GVINS_TRACKING_NORMAL) {       
        problem.SetParameterBlockConstant(&extrinsic_[7]);
    }
}

/* 构建Ceres::factor */
void GVINS::addImuFactors(ceres::Problem &problem) {
    for (size_t k = 0; k < preintegrationlist_.size(); k++) {
        // 预积分因子
        // IMU preintegration factors
        auto factor = new PreintegrationFactor(preintegrationlist_[k]);     
        problem.AddResidualBlock(factor, nullptr, statedatalist_[k]->pose, statedatalist_[k]->mix,
                                 statedatalist_[k + 1]->pose, statedatalist_[k + 1]->mix); 
    }

    // 添加IMU误差约束, 限制过大的误差估计
    // IMU error factor
    auto factor = new ImuErrorFactor(preintegration_options_);
    problem.AddResidualBlock(factor, nullptr, statedatalist_[preintegrationlist_.size()]->mix);             

    // IMU初始先验因子, 仅限于初始化
    // IMU prior factor, only for initialization
    if (is_use_prior_) {        
        auto pose_factor = new ImuPosePriorFactor(pose_prior_, pose_prior_std_);
        problem.AddResidualBlock(pose_factor, nullptr, statedatalist_[0]->pose);             

        auto mix_factor = new ImuMixPriorFactor(preintegration_options_, mix_prior_, mix_prior_std_);       
        problem.AddResidualBlock(mix_factor, nullptr, statedatalist_[0]->mix);
    }
}

vector<std::pair<ceres::ResidualBlockId, GNSS *>> GVINS::addGnssFactors(ceres::Problem &problem, bool isusekernel) {
    vector<std::pair<ceres::ResidualBlockId, GNSS *>> residual_block;

    ceres::LossFunction *loss_function = nullptr;
    if (isusekernel) {
        loss_function = new ceres::HuberLoss(1.0);
    }

    for (auto &data : gnsslist_) {
        int index = getStateDataIndex(data.time);          
        if (index >= 0) {
            auto factor = new GnssFactor(data, antlever_);
            auto id     = problem.AddResidualBlock(factor, loss_function, statedatalist_[index]->pose);     
            residual_block.push_back(std::make_pair(id, &data));       // id+gnss数据
        }
    }

    return residual_block;      // 残差块
}

vector<ceres::ResidualBlockId> GVINS::addReprojectionFactors(ceres::Problem &problem, bool isusekernel) {
    vector<ceres::ResidualBlockId> residual_ids;

    if (map_->keyframes().empty()) {
        return residual_ids;
    }

    ceres::LossFunction *loss_function = nullptr;
    if (isusekernel) {
        loss_function = new ceres::HuberLoss(1.0);
    }

    residualIdToMappointId_.clear();
    residual_ids.clear();
    for (const auto &landmark : map_->landmarks()) {                   
        const auto &mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        auto ref_frame = mappoint->referenceFrame();
        if (!map_->isKeyFrameInMap(ref_frame)) {                                        
            continue;
        }

        auto ref_frame_pc      = camera_->pixel2cam(mappoint->referenceKeypoint());  // 转到归一化平面
        size_t ref_frame_index = getStateDataIndex(ref_frame->stamp());
        if (ref_frame_index < 0) {
            continue;
        }

        double *invdepth = &invdepthlist_[mappoint->id()];      
        if (*invdepth == 0) {
            *invdepth = 1.0 / MapPoint::DEFAULT_DEPTH;          // 这里会执行 ok
        }

        auto ref_feature = ref_frame->features().find(mappoint->id())->second;

        auto observations = mappoint->observations();
        for (auto &observation : observations) {
            auto obs_feature = observation.lock();              // 可以将weak_ptr转化为shared_ptr使用(临时可用)
            if (!obs_feature || obs_feature->isOutlier()) {
                continue;
            }
            auto obs_frame = obs_feature->getFrame();           // 地图点被观测的程度包含（非关键帧，这里要注意）         
            if (!obs_frame || !obs_frame->isKeyFrame() || !map_->isKeyFrameInMap(obs_frame) ||
                (obs_frame == ref_frame)) {
                continue;
            }

            auto obs_frame_pc      = camera_->pixel2cam(obs_feature->keyPoint());
            size_t obs_frame_index = getStateDataIndex(obs_frame->stamp());

            if ((obs_frame_index < 0) || (ref_frame_index == obs_frame_index)) {
                LOGE << "Wrong matched mapoint keyframes " << Logging::doubleData(ref_frame->stamp()) << " with "
                     << Logging::doubleData(obs_frame->stamp());
                continue;
            }

            auto factor = new ReprojectionFactor(ref_frame_pc, obs_frame_pc, ref_feature->velocityInPixel(),
                                                 obs_feature->velocityInPixel(), ref_frame->timeDelay(),
                                                 obs_frame->timeDelay(), optimize_reprojection_error_std_);   
            auto residual_block_id =
                problem.AddResidualBlock(factor, loss_function, statedatalist_[ref_frame_index]->pose,
                                         statedatalist_[obs_frame_index]->pose, extrinsic_, invdepth, &extrinsic_[7]);   
            residual_ids.push_back(residual_block_id);
            mappoint_used_times_[mappoint->id()]++;
            residualIdToMappointId_[residual_block_id] = mappoint->id();   
        }
    }

    return residual_ids;
}
