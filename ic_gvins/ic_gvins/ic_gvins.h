/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef GVINS_GVINS_H
#define GVINS_GVINS_H

#include "common/angle.h"
#include "common/timecost.h"
#include "fileio/filesaver.h"
#include "tracking/drawer.h"
#include "tracking/tracking.h"
#include "gnss/obss_manager.h"

#include "factors/marginalization_info.h"
#include "factors/reprojection_factor.h"
#include "preintegration/preintegration.h"

#include <ceres/ceres.h>

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <thread>
#include <unordered_map>

class GVINS {

public:
    enum GVINSState {
        GVINS_ERROR                 = -1,
        GVINS_INITIALIZING          = 0,
        GVINS_INITIALIZING_INS      = 1,
        GVINS_GINS_NORMAL           = 2,
        GVINS_INITIALIZING_VIO      = 3,
        GVINS_TRACKING_INITIALIZING = 4,
        GVINS_TRACKING_NORMAL       = 5,
        GVINS_TRACKING_LOST         = 6,
    };

    typedef std::shared_ptr<GVINS> Ptr;
    typedef std::unique_lock<std::mutex> Lock;

    GVINS() = delete;
    explicit GVINS(const string &configfile, const string &outputpath, Drawer::Ptr drawer);

    bool addNewImu(const IMU &imu);
    bool addNewObss(GOBSS &obss);
    bool addNewFrame(const Frame::Ptr &frame);

    void setFinished();

    bool isRunning() const {
        return !isfinished_;
    }

    GVINSState gvinsState() const {
        return gvinsstate_;
    }

private:
    void parametersStatistic();

    bool gvinsInitialization();
    bool gvinsInitializationOptimization();

    void addNewTimeNode(double time); 
    void insertNewTimeNode(double time);    
    bool addNewGnssTimeNode(double time);              
    bool insertNewGnssTimeNode();
    bool addNewKeyFrameTimeNode();
    bool removeUnusedTimeNode();
    void constructPrior(bool is_zero_velocity);

    void addStateParameters(ceres::Problem &problem);
    void addReprojectionParameters(ceres::Problem &problem);

    void addImuFactors(ceres::Problem &problem);
    vector<std::pair<ceres::ResidualBlockId, GNSS *>> addGnssFactors(ceres::Problem &problem, bool isusekernel);
    vector<ceres::ResidualBlockId> addReprojectionFactors(ceres::Problem &problem, bool isusekernel);
    void doReintegration();

    void updateParametersFromOptimizer();

    void updateObssBufferFromOptimizer(ceres::Problem &problem);

    void getConveriance(ceres::Problem &problem);

    int getStateDataIndex(double time);

    bool GIOptimization();
    bool GIMarginalization();

    bool GVIOptimization();
    bool GVIMarginalization();
    bool mappointOutlierCulling();
    bool RemoveAllSecondNewFrame();

    void gnssOutlierCullingByChi2(ceres::Problem &problem,
                                  vector<std::pair<ceres::ResidualBlockId, GNSS *>> &redisual_block);
    void removeReprojectionFactorsByChi2(ceres::Problem &problem, vector<ceres::ResidualBlockId> &residual_ids,
                                               double chi2);

    // Processing thread
    void runFusion();
    void runPositioning();
    void runTracking();
    void runOptimization();

private:
    // 正常重力
    // Normal gravity
    const double NORMAL_GRAVITY = 9.80;

    // INS窗口内的最大数量, 对于200Hz, 保留5秒数据
    // Maximum INS data in the window
    const size_t MAXIMUM_INS_NUMBER = 1000;

    // 动态航向初始的最小速度
    // Minimum velocity for GNSS/INS intializaiton
    const double MINMUM_ALIGN_VELOCITY = 0.5;

    // // 先验标准差
    // // The prior STD for IMU biases
    // const double GYROSCOPE_BIAS_PRIOR_STD     = 7200 * D2R / 3600; // 7200 deg/hr
    // const double ACCELEROMETER_BIAS_PRIOR_STD = 20000 * 1.0e-5;    // 20000 mGal

    // 优化参数, 使用deque容器管理, 移除头尾不会造成数据内存移动
    // The state data in the sliding window
    std::deque<std::shared_ptr<PreintegrationBase>> preintegrationlist_;
    std::deque<std::shared_ptr<IntegrationStateData>> statedatalist_;    
    std::deque<GNSS> gnsslist_;                 
    std::deque<double> timelist_;
    std::unordered_map<ulong, double> invdepthlist_;
    std::unordered_map<ulong, int> mappoint_used_times_;
    std::unordered_map<ceres::ResidualBlockId, ulong> residualIdToMappointId_;
    double extrinsic_[8]{0};

    std::vector<double> unused_time_nodes_;

    // 边缘化
    // Marginalization variables
    std::shared_ptr<MarginalizationInfo> last_marginalization_info_{nullptr};
    std::vector<double *> last_marginalization_parameter_blocks_;

    // 先验
    // The prior
    bool is_use_prior_{false};
    double mix_prior_[18];
    double mix_prior_std_[18];
    double pose_prior_[7];
    double pose_prior_std_[6];

    // 融合对象
    // GVINS fusion objects
    CGNSSManage::Ptr positioning_;
    Tracking::Ptr tracking_;
    Map::Ptr map_;
    Camera::Ptr camera_;
    Drawer::Ptr drawer_;

    // 多线程
    // Multi-thread variables
    std::thread drawer_thread_;
    std::thread tracking_thread_;
    std::thread positioning_thread_;
    std::thread optimization_thread_;
    std::thread fusion_thread_;

    std::atomic<bool> isoptimized_{false};          // 完成优化的标志
    std::atomic<bool> isfinished_{false};           // 算法进程结束标志
    std::atomic<bool> isgnssready_{false};          // gnss-meas到达标志
    std::atomic<bool> isgnssfinished_{false};
    std::atomic<bool> isframeready_{false};
    std::atomic<bool> isgnssobs_{false};            // gnss-obs
    std::atomic<bool> isvisualobs_{false};
    std::atomic<bool> istracking_{false};
    std::atomic<bool> isframeexist_{false};
    std::atomic<double> imutime_{0};
    std::atomic<double> frametime_{0};

    // IMU处理
    // Ins process
    std::mutex imu_buffer_mutex_;
    std::mutex fusion_mutex_;
    std::condition_variable fusion_sem_;           
    std::mutex ins_mutex_;

    // GNSS处理
    // GNSS process
    std::mutex positioning_mutex_;
    std::condition_variable positioning_sem_;           

    // 跟踪处理
    // Tracking process
    std::mutex frame_buffer_mutex_;
    std::mutex tracking_mutex_;
    std::condition_variable tracking_sem_;
    std::mutex keyframes_mutex_;

    // 优化处理
    // Optimization process
    std::mutex optimization_mutex_;
    std::mutex state_mutex_;
    std::condition_variable optimization_sem_;

    // 传感器数据
    // GVINS sensor data
    std::queue<Frame::Ptr> keyframes_;
    ObssManager::Ptr obss_buffer_;
    GNSS gnss_, last_gnss_, last_last_gnss_;          

    std::queue<Frame::Ptr> frame_buffer_;

    std::queue<IMU> imu_buffer_;        
    std::deque<std::pair<IMU, IntegrationState>> ins_window_;

    // IMU参数
    // IMU parameters
    std::shared_ptr<IntegrationParameters> integration_parameters_;
    Preintegration::PreintegrationOptions preintegration_options_;
    IntegrationConfiguration integration_config_;

    double imudatarate_{200};
    double imudatadt_{0.005};
    size_t reserved_ins_num_;

    Vector3d antlever_;

    // 初始化信息
    // Initialization
    int initlength_;

    // 外参
    // Camera-IMU extrinsic
    Pose pose_b_c_;
    double td_b_c_;
    std::mutex extrinsic_mutex_;

    bool is_use_visualization_{true};

    // 优化选项
    // Optimization options
    bool optimize_estimate_extrinsic_;
    bool optimize_estimate_td_;
    double optimize_reprojection_error_std_;
    int optimize_num_iterations_;
    size_t optimize_windows_size_;

    double reprojection_error_std_;

    // 统计参数
    // Statistic variables
    int iterations_[2]{0};
    double timecosts_[3]{0};
    double outliers_[2]{0};

    // 文件IO
    // File IO
    FileSaver::Ptr navfilesaver_;
    FileSaver::Ptr imuerrfilesaver_;
    FileSaver::Ptr ptsfilesaver_;
    FileSaver::Ptr statfilesaver_;
    FileSaver::Ptr extfilesaver_;
    FileSaver::Ptr trajfilesaver_;
    FileSaver::Ptr covfilesaver_;

    // 系统状态
    // System state
    std::atomic<GVINSState> gvinsstate_{GVINS_ERROR};
};

#endif // GVINS_GVINS_H
