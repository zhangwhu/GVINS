
#include "common/logging.h"
#include "common/earth.h"
#include "obss_manager.h"


ObssManager::ObssManager() {
    obss_buffer_.clear();
} 

void ObssManager::activateDualMode() {

    Lock lock3(obss_buffer_mutex_);
    for (auto &obs : obss_buffer_) {
        if (obs.state == GOBSS::withSolution) {
            obss_buffer_.pop_front();
        } else if (obs.state == GOBSS::skipOver) {
            obss_buffer_.pop_front();
        } else if (obs.state == GOBSS::inList) {
            obs.state = GOBSS::withSkip;
        }
    }
    isDualMode_ = true;
}

bool ObssManager::addNewObss(GOBSS &obss) {
    if (obss_buffer_mutex_.try_lock()) {
        obss_buffer_.push_back(obss);
        obss_buffer_mutex_.unlock();
        return true;
    }
    return false;
}

bool ObssManager::empty() {
    Lock lock3(obss_buffer_mutex_);
    return obss_buffer_.empty();
}

bool ObssManager::getDualMode() {
    return isDualMode_;
}

void ObssManager::removeObss() {
    Lock lock3(obss_buffer_mutex_);
    while (!obss_buffer_.empty()) {
        obss_buffer_.pop_front();
    }
}

bool ObssManager::addObssInlist(double time) {
    bool stat = false;
    Lock lock3(obss_buffer_mutex_);

    if (time < 1E-01) {
        for (auto &obs : obss_buffer_) { 
            if (obs.state == GOBSS::Null) {
                obs.state = GOBSS::inList;
            } else if (obs.state == GOBSS::skipOver) {
                obss_buffer_.pop_front();
            } else if (obs.state == GOBSS::withSolution) {
                obss_buffer_.pop_front();
            }
        }
    } else {
        for (auto &obs : obss_buffer_) {                
            if (obs.time > time) break;
            if (obs.state == GOBSS::Null) {
                obs.state = GOBSS::inList;
                stat = true;
            }
        }
    }
    return stat;
}

std::queue<GOBSS> ObssManager::getPendingObss() {
    std::queue<GOBSS> obslist;
    Lock lock3(obss_buffer_mutex_);
    for (auto &obs : obss_buffer_) {
        if ((isDualMode_ && obs.state == GOBSS::withPrior) || 
           (!isDualMode_ && obs.state == GOBSS::inList) || 
           (obs.state == GOBSS::withSkip)) {
            obslist.push(obs);
        }
    }
    return obslist;
}

bool ObssManager::feedbackSolution(std::queue<GNSS> &gnsslist) {           
    size_t ind = 0;
    Lock lock3(obss_buffer_mutex_); 

    while(!gnsslist.empty()) {
        auto gnss = gnsslist.front();
        for (; ind < obss_buffer_.size(); ind++) {
            auto &obs  = obss_buffer_[ind];
            if(fabs(obs.time - gnss.time) < 1E-01) {
                switch (obs.state) {
                    case GOBSS::inList:    
                    case GOBSS::withPrior: obs.state = GOBSS::withSolution; break;
                    case GOBSS::withSkip:  obs.state = GOBSS::skipOver;     break;
                }
                obs.enu    = gnss.blh; 
                obs.std[0] = gnss.std[0] * WGS84_RA;
                obs.std[1] = gnss.std[1] * WGS84_RA;
                obs.std[2] = gnss.std[2];
                break;
            }
        }
        if (ind >= obss_buffer_.size()) break;
        gnsslist.pop();
    }
    return gnsslist.empty();
}

std::queue<GNSS> ObssManager::popObssWithSolution() {
    std::queue<GNSS> gnsslist;

    Lock lock3(obss_buffer_mutex_);
    while(!obss_buffer_.empty()) {
        auto &obs = obss_buffer_.front();
        if (obs.state == GOBSS::withSolution) {
            if (fabs(obs.node) > 1E-001) {  // node可以使用
                gnsslist.emplace(GNSS(obs.node, obs.enu, obs.std));
            } else {
                gnsslist.emplace(GNSS(obs.time, obs.enu, obs.std));
            }
            obss_buffer_.pop_front();
        } else if (obs.state == GOBSS::skipOver) {
            obss_buffer_.pop_front();
        } else {
            break;
        }
    }
    return gnsslist;
}

std::queue<double> ObssManager::addObssInNode(double statime, double endtime) {

    std::queue<double> node;

    Lock lock3(obss_buffer_mutex_);
    for (auto &obs : obss_buffer_) {
        if (obs.time <= statime && obs.state == GOBSS::inList) { 
            LOGW << "GNSS ERROR 1 " << obs.time;           
            obs.state = GOBSS::withSkip;
        } else if (obs.time <= endtime && obs.state == GOBSS::inList) {
            node.push(obs.time); 
            obs.node  = obs.time;
            obs.state = GOBSS::inNode;
        } else if (obs.time > endtime && obs.state == GOBSS::inList) {
            obs.state = GOBSS::Null;
        }
    }
    return node;
}

std::queue<double> ObssManager::addObssInNode(std::deque<double> timelist, std::deque<bool> flaglist) {
    size_t index;
    double sta, end;
    double cutoff_time;
    std::queue<double> node;

    if (timelist.empty()) return node;
    cutoff_time = timelist.back();
    
    Lock lock3(obss_buffer_mutex_);
    
    if (!obss_buffer_.empty() && fabs(obss_buffer_.back().time - cutoff_time) > 2) {
        for (auto &obs : obss_buffer_) {
            if (obs.state == GOBSS::Null || obs.state == GOBSS::inList) {
                LOGW << "GNSS ERROR 2 " << obs.time;
                obs.state = GOBSS::withSkip;
            }
        }
        LOGW << "Zhi hou tai jiu";
        return node;
    } 

    for (auto &obs : obss_buffer_) {
        if (obs.time > cutoff_time) {
            obs.state = GOBSS::Null;   
            continue;
        }
        if (obs.state == GOBSS::inList) {
            auto ti = obs.time; 

            // Find time interval
            sta = end = index = 0;
            for (size_t k = timelist.size() - 1; k >= 1; k--) {
                if ((ti <= timelist[k]) && (ti > timelist[k - 1])) {
                    sta = timelist[k - 1];   
                    end = timelist[k];   
                    index = k;
                }
            }

            if (sta == 0) {
                obs.state = GOBSS::withSkip;
                LOGW << "Unused GNSS due to no find time-node at " << Logging::doubleData(ti) 
                     << " start=" << timelist[0] << " end=" << timelist.back();
                continue;
            } else if ((end - sta) > MAXIMUM_PREINTEGRATION_LENGTH) {
                obs.state = GOBSS::withSkip;
                LOGI << "Unused GNSS due to long-time preintegration " << Logging::doubleData(ti) << '\t' << sta << '\t' << end;
                continue;
            } else if (!flaglist[index]) {         
                obs.state = GOBSS::withSkip;
                LOGW << "Unused GNSS due to non-normal keyframe at " << Logging::doubleData(ti) << '\t' << sta << '\t' << end;
                continue;
            } else {
                obs.state = GOBSS::inNode;
            }

            // Insert the node
            if (ti - sta < MINMUM_SYNC_INTERVAL) {
                obs.node = sta;
                LOGI << "Add new GNSS " << Logging::doubleData(ti) << " align to " << Logging::doubleData(sta);
            } else if (end - ti < MINMUM_SYNC_INTERVAL) {
                obs.node = end;
                LOGI << "Add new GNSS " << Logging::doubleData(ti) << " align to " << Logging::doubleData(end);
            } else {
                obs.node = ti;
                node.push(ti);
            }
        }
    }
    return node;
}

void ObssManager::getObssInNode(std::queue<size_t> &index, std::queue<double> &time, std::queue<double> &node) {

    Lock lock3(obss_buffer_mutex_);
    for (size_t i = 0; i < obss_buffer_.size(); i++) {
        auto &obs = obss_buffer_[i];
        if (obs.state == GOBSS::inNode) {
            index.emplace(i);
            time.emplace(obs.time);
            node.emplace(obs.node);
        }
    }
}

void ObssManager::updateObssInNode(bool isError, std::queue<size_t> index, std::queue<Vector3d> parameters,
        std::queue<Matrix3d> covariances) {
    Lock lock3(obss_buffer_mutex_);
    while(!index.empty()) {
        auto &obs = obss_buffer_[index.front()]; index.pop();
        if (!isError) {
            obs.xyz = parameters.front();     parameters.pop(); 
            obs.vel = parameters.front();     parameters.pop();
            obs.cov = covariances.front();    covariances.pop();
            // printf("updateObssInNode=[%15.3f, %15.3f %15.3f]\n", obs.xyz[0], obs.xyz[1], obs.xyz[2]);
            // printf("                 [%12.5f, %12.5f %12.5f]\n", obs.cov(0, 0), obs.cov(0, 1), obs.cov(0, 2));
            // printf("                 [%12.5f, %12.5f %12.5f]\n", obs.cov(1, 0), obs.cov(1, 1), obs.cov(1, 2));
            // printf("                 [%12.5f, %12.5f %12.5f]\n", obs.cov(2, 0), obs.cov(2, 1), obs.cov(2, 2));
            obs.state = GOBSS::withPrior;
        }
        else {
            LOGW << "GNSS ERROR 3 " << obs.time;
            obs.state = GOBSS::withSkip;
        }
    }
}

std::queue<GOBSS> ObssManager::getBuffer() {
    std::queue<GOBSS> vecdata;
    Lock lock3(obss_buffer_mutex_);
    for(size_t k = 0; k < obss_buffer_.size(); k++) {
        vecdata.push(obss_buffer_[k]);
    }
    return vecdata;
}


