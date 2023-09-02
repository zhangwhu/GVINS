/**
* This file is part of gnss_comm.
*
* Copyright (C) 2021 Aerial Robotics Group, Hong Kong University of Science and Technology
* Author: CAO Shaozu (shaozu.cao@gmail.com)
*
* gnss_comm is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* gnss_comm is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with gnss_comm. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef GNSS_ROS_HPP_
#define GNSS_ROS_HPP_

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <gnss_comm/GnssTimeMsg.h>
#include <gnss_comm/GnssObsMsg.h>
#include <gnss_comm/GnssMeasMsg.h>
#include <CGNSSConfig.h>
#include <CGNSSManage.h>

namespace gnss_comm
{
    /* parse GNSS measurement from RINEX observation file  ---------------------------------------
    * args   : std::string            rinex_filepath        I   RINEX file path
    *          prcopt_t               prcopt                I   RTKLIB configure file
    *          std::vector<obs_t>&    rinex_meas            IO  GNSS measurement in time order
    * return : None
    *---------------------------------------------------------------------------------------------*/
    int rinex2obs(const std::string &rinex_filepath, const prcopt_t &prcopt,
        std::vector<std::vector<obsd_t>> &rinex_meas);
    
    /* convert GNSS measurements to ros message ----------------------------------
    * args   : std::vector<ObsPtr> & meas      I   GNSS measurements
    * return : cooresponding GNSS measurement message
    *-----------------------------------------------------------------------------*/
    GnssMeasMsg meas2msg(const std::vector<obsd_t> &meas);

    /* parse GNSS measurements from ros message ----------------------------------
    * args   : GnssMeasConstPtr & gnss_meas_msg      I   GNSS measurement message
    * return : cooresponding GNSS measurements
    *-----------------------------------------------------------------------------*/
    obs_t msg2meas(const GnssMeasMsgConstPtr &gnss_meas_msg);
     
}   // namespace gnss_comm

#endif