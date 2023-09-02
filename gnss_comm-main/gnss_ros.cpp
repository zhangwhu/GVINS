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

#include "gnss_ros.hpp"

namespace gnss_comm
{
    /* add obs data --------------------------------------------------------------*/
    static int addobsdata(obs_t *obs, const obsd_t *data)
    {
        obsd_t *obs_data;

        if (obs->nmax <= obs->n) {
            if (obs->nmax <= 0) obs->nmax = 60; else obs->nmax *= 2;                  
            if (!(obs_data = (obsd_t *)realloc(obs->data, sizeof(obsd_t)*obs->nmax))) {	
                trace(1, "addobsdata: memalloc error n=%dx%d\n", sizeof(obsd_t), obs->nmax);
                free(obs->data); obs->data = NULL; obs->n = obs->nmax = 0;
                return -1;
            }
            obs->data = obs_data;
        }
        obs->data[obs->n++] = *data;
        return 1;
    }

    /* search next observation data index ----------------------------------------*/
    static int nextobsf(const obs_t *obs, int *i, int rcv)
    {
        int n;
        double tt;

        for (; *i < obs->n; (*i)++)
            if (obs->data[*i].rcv == rcv)
                break;
        for (n = 0; *i + n < obs->n; n++)
        {
            tt = timediff(obs->data[*i + n].time, obs->data[*i].time);
            if (obs->data[*i + n].rcv != rcv || tt > DTTOL)
                break;
        }
        return n;
    }

    int rinex2obs(const std::string &rinex_filepath, const prcopt_t &prcopt,
        std::vector<std::vector<obsd_t>> &rinex_meas)
    {
        int nepoch = 0, nu = 0, iobsu = 0;
        obs_t obss = {0};
        sta_t sta = {0};
        std::vector<obsd_t> meas;

        if (!readobs(rinex_filepath.c_str(), 1, &prcopt, &obss, &sta, &nepoch)) return -1;

        while (0 <= iobsu && iobsu < obss.n) {
            if ((nu = nextobsf(&obss, &iobsu, 1)) <= 0) {
                freeobs(&obss);
                return -1;
            }     
            for (int i = 0; i < nu; i++)
                meas.emplace_back(obss.data[iobsu + i]);
            iobsu += nu;
            rinex_meas.emplace_back(meas);
        }
        freeobs(&obss);
        return 1;
    }

    GnssMeasMsg meas2msg(const std::vector<obsd_t> &meas)
    {
        GnssMeasMsg gnss_meas_msg;
        for (obsd_t obs : meas)
        {
            GnssObsMsg obs_msg;
            int week = 0;
            double tow = time2gpst(obs.time, &week);
            obs_msg.time.week = week;
            obs_msg.time.tow  = tow;
            obs_msg.sat       = obs.sat;
            obs_msg.rcv       = obs.rcv;
            for (int i = 0; i < 3; i++) {
                obs_msg.SNR.push_back(obs.SNR[i]);
                obs_msg.LLI.push_back(obs.LLI[i]);
                obs_msg.code.push_back(obs.code[i]);
                obs_msg.psr.push_back(obs.P[i]); 
                obs_msg.cp.push_back(obs.L[i]);
                obs_msg.dopp.push_back(obs.D[i]);
            }

            gnss_meas_msg.meas.push_back(obs_msg);
        }
        return gnss_meas_msg;
    }

    obs_t msg2meas(const GnssMeasMsgConstPtr &gnss_meas_msg)
    {
        obs_t meas;
        for (size_t i = 0; i < gnss_meas_msg->meas.size(); ++i)
        {
            GnssObsMsg obs_msg = gnss_meas_msg->meas[i];
            obsd_t *obs = new obsd_t();
            obs->time       = gpst2time(obs_msg.time.week, obs_msg.time.tow);
            obs->sat        = obs_msg.sat;
            obs->rcv        = obs_msg.rcv;
            for (int i = 0; i < 3; i++) {
                obs->SNR[i]        = obs_msg.SNR[i];
                obs->LLI[i]        = obs_msg.LLI[i];
                obs->code[i]       = obs_msg.code[i];
                obs->P[i]          = obs_msg.psr[i];
                obs->L[i]          = obs_msg.cp[i];
                obs->D[i]          = obs_msg.dopp[i];  
            }
            addobsdata(&meas, obs); 
        }
        return meas;
    }

    

    
}   // namespace gnss_comm