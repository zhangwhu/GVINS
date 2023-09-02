#ifndef GNSS_OBSS_H
#define GNSS_OBSS_H

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <thread>
#include <Eigen/Geometry>
#include <queue>
#include <deque>
#include "common/types.h"
#include <CGNSSManage.h>

using Eigen::Vector2d;
using Eigen::Matrix3d;
using Eigen::Vector3d;

typedef struct GNSS_OBSS { 
    enum GNSS_State {
        Null,           // 无信息
        inList,         // 在序列中 

        inNode,         // 插入节点
        withPrior,      // 具备先验信息
        withSolution,   // GNSS解决方案

        withSkip,       // 跳过
        skipOver,       // 跳过成功
    }; 
    
    double time;
    obs_t  meas;
    
    GNSS_State state;

    double node;
    Vector3d xyz;
    Vector3d vel;
    Matrix3d cov;
    Vector3d enu;
    Vector3d std;
    
    GNSS_OBSS() {
        time  = 0;
        node  = 0;
        state = Null;
    }
    GNSS_OBSS(double ti, obs_t obs) {
        time  = ti;
        node  = 0;
        state = Null;
        meas  = obs;
    }
} GOBSS;

class ObssManager {
    
public:
    typedef std::shared_ptr<ObssManager> Ptr;
    typedef std::unique_lock<std::mutex> Lock;

    explicit ObssManager();

    bool addNewObss(GOBSS &obss);
    bool empty();
    bool getDualMode();
    void removeObss();
    bool addObssInlist(double time);
    std::queue<GOBSS> getPendingObss();
    bool feedbackSolution(std::queue<GNSS> &gnsslist);
    void activateDualMode();
    void setOrigin(Vector3d &blh);
    std::queue<GNSS> popObssWithSolution();

    std::queue<double> addObssInNode(double statime, double endtime);
    std::queue<double> addObssInNode(std::deque<double> timelist, std::deque<bool> flaglist);
    void getObssInNode(std::queue<size_t> &index, std::queue<double> &time, std::queue<double> &node);
    void updateObssInNode(bool isError, std::queue<size_t> index, std::queue<Vector3d> parameters,
        std::queue<Matrix3d> Covariances);
    
    std::queue<GOBSS> getBuffer();
private:
    std::mutex obss_buffer_mutex_;
    std::atomic<bool> isDualMode_{false};

    std::deque<GOBSS> obss_buffer_;

    // 允许的最小同步间隔
    // Minimum synchronization interval for GNSS
    const double MINMUM_SYNC_INTERVAL = 0.025;

    // 允许的最长预积分时间
    // Maximum length for IMU preintegration
    const double MAXIMUM_PREINTEGRATION_LENGTH = 10.0;
};

#endif