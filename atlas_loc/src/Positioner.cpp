//
//  Positioner.cpp
//  atlas loc
//
//  Created by Janis on 18.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>

#include "atlas_loc/atlas_types.h"
#include "atlas_loc/Positioner.h"

#include "ros/ros.h"

#include <atlas_msgs/Sample.h>
#include <atlas_msgs/Measurement.h>

#include <tf/transform_datatypes.h>
// PRETTY MATRIX PRINT
std::string sep = "\n----------------------------------------\n";
Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
//std::cout << sep << "ta1:" << sep << ta1.format(HeavyFmt) << sep;


uint16_t discard_counter = 0;

PositionerTDOA::PositionerTDOA ()
{

}

void PositionerTDOA::initialize(ros::NodeHandle n, bool pzs_flag)
{
    ROS_INFO("Initializing Positioner");

    XmlRpc::XmlRpcValue anchors;
    n.getParam("/atlas/anchor", anchors);
    ROS_ASSERT(anchors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = anchors.begin(); it != anchors.end(); ++it)
    {
        ROS_INFO("Found anchor %s", (it->first).c_str());
        uint64_t eui = std::stoul(it->first, nullptr, 16);

        std::vector<double> p;
        n.getParam("/atlas/anchor/" + it->first + "/pos", p);
        Eigen::Vector3d pos(p.at(0), p.at(1), p.at(2));
        m_anchorPositions.insert(std::pair<uint64_t, Eigen::Vector3d>(eui, pos));
    }

    if(pzs_flag)
    {
        // Cell Configurations
        if (n.hasParam("cellBounds"))
        {
            //get cell bounds
            std::vector<double> cellBounds;
            n.getParam("cellBounds", cellBounds);

            Eigen::VectorXd cellBoundsVec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(cellBounds.data(), cellBounds.size());
            int cellNumber = cellBoundsVec.size()/6;

            m_cellBounds.resize(cellNumber, 6);
            m_cellBounds.setZero();

            for(int i = 0; i < cellNumber; i++)
            {
                m_cellBounds.row(i) << cellBoundsVec.segment(i*6,6).transpose();
                ROS_INFO("PARAM Cell %d: %f, %f, %f, %f", 
                        i+1, cellBoundsVec(i*6), cellBoundsVec(i*6+1), cellBoundsVec(i*6+2), cellBoundsVec(i*6+3));
            }

            // assign anchors to cells
            for(int i = 0; i < cellNumber; i++)
            {
                std::vector<uint64_t> temp;

                for(auto it = m_anchorPositions.begin(); it!=m_anchorPositions.end(); it++ )
                {
                    if((it->second(0) > m_cellBounds(i,0)) && (it->second(0) < m_cellBounds(i,1)) 
                        && (it->second(1) > m_cellBounds(i,2)) && (it->second(1) < m_cellBounds(i,3))
                        && (it->second(2) > m_cellBounds(i,4)) && (it->second(2) < m_cellBounds(i,5)))
                    {
                        temp.push_back(it->first);
                    }
                }
                m_cellAnchors.push_back(temp);
            } 

            //assign cells to anchors
            for(auto it = m_anchorPositions.begin(); it!=m_anchorPositions.end(); it++ )
            {
                for(int i = 0; i < cellNumber; i++)
                {
                    if((it->second(0) > m_cellBounds(i,0)) && (it->second(0) < m_cellBounds(i,1)) 
                        && (it->second(1) > m_cellBounds(i,2)) && (it->second(1) < m_cellBounds(i,3))
                        && (it->second(2) > m_cellBounds(i,4)) && (it->second(2) < m_cellBounds(i,5)))
                    {
                        m_anchorCells.insert(std::make_pair(it->first, i));
                    }
                }
            }
        
            //TODO: ROS_ERROR("Anchor %#lx is not part of any cell", it->first);

            for ( std::vector<std::vector<uint64_t>>::size_type i = 0; i < m_cellAnchors.size(); i++ )
            {
                ROS_INFO("Anchors of Cell %d: ", (int)i+1);

                for ( std::vector<int>::size_type j = 0; j < m_cellAnchors[i].size(); j++ )
                {
                    ROS_INFO("- %#lx", m_cellAnchors[i][j]);
                }
            }

            //compute cell centers
            m_cellCenters.resize(cellNumber,3);
            m_cellCenters.setZero();

            for(int i = 0; i < cellNumber; i++)
            {
                m_cellCenters.row(i) << (m_cellBounds(i,0)+m_cellBounds(i,1))/2, 
                                        (m_cellBounds(i,2)+m_cellBounds(i,3))/2, 
                                        (m_cellBounds(i,4)+m_cellBounds(i,5))/2;
            }
        }
        else
        {
            ROS_ERROR("PARAM FAILED to get 'Cells'");
        }

        if (n.hasParam("cellsPerZone"))
        {
            n.getParam("cellsPerZone", m_cellsPerZone);
            ROS_INFO("PARAM cellsPerZone: %d", m_cellsPerZone);
        }
        else
        {
            ROS_ERROR("PARAM FAILED to get 'cellsPerZone'");
        }

        if (n.hasParam("fixedZ"))
        {
            n.getParam("fixedZ", m_fixedZ);
            for(int i = 0; i < m_fixedZ.size(); i++){
                ROS_INFO("PARAM fixedZ %d: %f", i, m_fixedZ.at(i));
            }
        }
        else
        {
            ROS_ERROR("PARAM FAILED to get 'fixedZ'");
        }

        // Assign levels to cells    
        for (int i = 0; i < m_cellBounds.rows(); i++)
        {
            int temp;
            for(int j = 0; j < m_fixedZ.size(); j++)
            {
                //check if fixed z coordinate is in z-Range of cell bounds
                if (m_cellBounds(i, 4) < m_fixedZ.at(j) && m_cellBounds(i, 5) > m_fixedZ.at(j))
                {
                    temp = j;
                    break;
                }
            }
            m_cellLevels.push_back(temp); 
        }

        //Assign levels to anchors
        int counter = 0; 
        for (auto it = m_cellAnchors.begin(); it != m_cellAnchors.end(); it++)
        {
            for(auto jt = it->begin(); jt != it->end(); jt++)
            {
                m_anchorLevels.insert(std::make_pair(*jt, m_cellLevels.at(counter)));
            }
            counter++;
        }
    }


    // EKF Configuration
    if (n.getParam("ekf/imu", m_imu))
    {
        ROS_INFO("PARAM ekf/imu: %d", m_imu);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/imu'");
    }
    if (n.getParam("ekf/constrained", m_constrained))
    {
        ROS_INFO("PARAM ekf/constrained: %d", m_constrained);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/imu'");
    }
    if (n.getParam("ekf/sqd", m_sqd))
    {
        ROS_INFO("PARAM ekf/sqd: %d", m_sqd);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/sqd'");
    }
    if (n.getParam("ekf/dimensions", m_dimensions))
    {
        ROS_INFO("PARAM ekf/dimensions: %d", m_dimensions);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/dimensions'");
    }

    if (n.getParam("ekf/maxZ", m_maxZ))
    {
        ROS_INFO("PARAM ekf/maxZ: %f", m_maxZ);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/maxZ'");
    }

    if (n.getParam("ekf/minZ", m_minZ))
    {
        ROS_INFO("PARAM ekf/minZ: %f", m_minZ);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/minZ'");
    }

    if (n.getParam("ekf/maxY", m_maxY))
    {
        ROS_INFO("PARAM ekf/maxY: %f", m_maxY);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/maxY'");
    }

    if (n.getParam("ekf/minY", m_minY))
    {
        ROS_INFO("PARAM ekf/minY: %f", m_minY);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/minY'");
    }

    if (n.getParam("ekf/maxX", m_maxX))
    {
        ROS_INFO("PARAM ekf/maxX: %f", m_maxX);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/maxX'");
    }

    if (n.getParam("ekf/minX", m_minX))
    {
        ROS_INFO("PARAM ekf/minX: %f", m_minX);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/minX'");
    }

    if (n.getParam("ekf/processNoise", m_processNoise))
    {
        ROS_INFO("PARAM ekf/processNoise: %f", m_processNoise);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/processNoise'");
    }

    if (n.getParam("ekf/measurementVariance", m_measurementNoise))
    {
        ROS_INFO("PARAM ekf/measurementVariance: %f", m_measurementNoise);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/measurementVariance'");
    }

    if (n.getParam("ekf/initialInterval", m_initialInterval))
    {
        ROS_INFO("PARAM ekf/initialInterval: %f", m_initialInterval);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/initialInterval'");
    }

    if (n.getParam("ekf/initialVariance", m_initialStateVariance))
    {
        ROS_INFO("PARAM ekf/initialVariance: %f", m_initialStateVariance);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/initialVariance'");
    }

    if (n.getParam("ekf/initialVarianceDelta", m_initialStateVarianceDelta))
    {
        ROS_INFO("PARAM ekf/initialVarianceDelta: %f", m_initialStateVarianceDelta);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/initialVarianceDelta'");
    }

    if (n.hasParam("ekf/initialPosition"))
    {
        std::vector<double> pos;
        n.getParam("ekf/initialPosition", pos);
        m_initialState.resize(6);
        m_initialState << pos.at(0), pos.at(1), pos.at(2), 0.0, 0.0, 0.0;
        ROS_INFO("PARAM ekf/initialPosition: %f, %f, %f", pos.at(0), pos.at(1), pos.at(2));
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/initialPosition'");
    }

    if (n.getParam("ekf/outlierThreshold", m_outlierThreshold))
    {
        ROS_INFO("PARAM ekf/outlierThreshold: %f", m_outlierThreshold);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/outlierThreshold'");
    }

    if (n.getParam("ekf/outlierThresholdDelta", m_outlierThresholdDelta))
    {
        ROS_INFO("PARAM ekf/outlierThresholdDelta: %f", m_outlierThresholdDelta);
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'ekf/outlierThresholdDelta'");
    }

    if (n.hasParam("orientationOffset"))
    {
        std::vector<double> orientationOffset;
        n.getParam("orientationOffset", orientationOffset);
        Eigen::Vector3d angle(orientationOffset.at(0), orientationOffset.at(1), orientationOffset.at(2));
        m_orientOffset = angle;
        ROS_INFO("PARAM orientationOffset: %f, %f, %f", angle(0), angle(1), angle(2));
    }
    else
    {
        ROS_ERROR("PARAM FAILED to get 'orientationOffset'");
    }

    //EUI Configuration
    XmlRpc::XmlRpcValue fastKeys;
    n.getParam("/atlas/fast", fastKeys);
    ROS_ASSERT(fastKeys.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = fastKeys.begin(); it != fastKeys.end(); it++)
    {
        if(n.hasParam("/atlas/fast/" + it->first + "/enable")){
            uint64_t tagEUI = std::stoul(it->first, nullptr, 16);
            m_subSample.insert(std::make_pair(tagEUI, n.subscribe("/atlas/" + it->first + "/sample", 100, &PositionerTDOA::sampleCallback, this)));
            ROS_INFO("PARAM add tag on whitelist: %#lx", (uint64_t)tagEUI);

            if(pzs_flag)
            {
                std::vector<uint64_t> temp;
                m_anchorsInZone.insert(std::pair<uint64_t, std::vector<uint64_t>>(tagEUI, temp));

                //assign level to tag
                m_tagLevel.insert(std::pair<uint64_t, int>(tagEUI, 0));
                m_prevTagLevel.insert(std::pair<uint64_t, int>(tagEUI, 0));
            }
        }
    } 
}

void PositionerTDOA::sampleCallback(const atlas_msgs::Sample& msg)
{
    sample_t s;
    s.hts = msg.hts;
    s.txeui = msg.txId;
    s.seq = msg.seq;

    for (auto it = msg.meas.begin(); it != msg.meas.end(); ++it)
    {
        measurement_t m;
        m.ts = it->ts;
        m.toa = it->toa;
        m.lde.maxNoise = it->lde.maxNoise;
        m.lde.firstPathAmp1 = it->lde.firstPathAmp1;
        m.lde.stdNoise = it->lde.stdNoise;
        m.lde.firstPathAmp2 = it->lde.firstPathAmp2;
        m.lde.firstPathAmp3 = it->lde.firstPathAmp3;
        m.lde.maxGrowthCIR = it->lde.maxGrowthCIR;
        m.lde.rxPreamCount = it->lde.rxPreamCount;
        m.lde.firstPath = it->lde.firstPath;

        s.meas.insert(std::make_pair(it->rxId, m));
    }

    m_samples.push_back(s);
}

void PositionerTDOA::extractSamples(std::vector<sample_t> *samples)
{
    while(!m_samples.empty())
    {
        samples->push_back(m_samples.front());
        m_samples.pop_front();
    }
}

void PositionerTDOA::createNewEKF(uint64_t eui, ros::Time ts, bool pzs_flag)
{
    //get init state (center of tagCell)
    Eigen::VectorXd initState(6);
    ekf_t ekf;
    ekf.lastUpdate = ts;

    if(pzs_flag)
    {
        initState.head(3) = m_cellCenters.row(m_tagCell.at(eui)).transpose();
        initState.tail(3) << 0,0,0;
        ekf.state = initState;
    }
    else
    {
        ekf.state = m_initialState;
    }

    ekf.imu = m_imu;
    ekf.sqd = m_sqd;
    ekf.constrained = m_constrained;

    Eigen::MatrixXd stateCovariance = Eigen::MatrixXd::Zero(6, 6);
    stateCovariance.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * m_initialStateVariance;
    stateCovariance.bottomRightCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * m_initialStateVarianceDelta;
    if (m_dimensions == 2)
    {
        stateCovariance(2, 2) = 0;
        stateCovariance(5, 5) = 0;
    }
    ekf.stateCovariance = stateCovariance;

    ekf.W = Eigen::MatrixXd::Identity(3, 3);
    ekf.V = Eigen::MatrixXd::Identity(3, 3);

    m_ekf[eui] = ekf;
}

// bool PositionerTDOA::calculatePositionEKFInner(const sample_t &s, position_t *p, bool pzs_flag)
// {   
//     int count = s.meas.size();
//     Eigen::MatrixXd anchorPositions(count, 3);
//     Eigen::VectorXd anchorTOAs(count);

//     // extracting the tdoa
//     int row = 0;
//     for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
//     {
//         anchorPositions.row(row) = m_anchorPositions[it->first].transpose();
//         anchorTOAs(row) = it->second.toa;
//         row++;
//     }

//     // --- get the last interval ---
//     double interval = m_initialInterval;
//     if(m_lastPosition.find(p->eui) != m_lastPosition.end())
//     {
//         // assuming the first anchor stays the same !!
//         double i = (s.hts - m_lastSample[s.txeui].hts).toSec();

//         if(i > 0.0)
//         {
//             interval = i;
//         }
//         else
//         {
//             ROS_WARN(" Interval: %.6f smaller or equals zero !!!", i);
//         }
//     }

//     // --- threshold to reinitialize ekf ---
//     if(interval > 60.0)
//     {
//         createNewEKF(s.txeui, s.hts, pzs_flag);
//         ROS_WARN(" Interval: %.6f greater threshold, reinit EKF !!!", interval);
//     }

//     // --- get ekf state for specific participant ---
//     ekf_t ekf = m_ekf[s.txeui];
//     Eigen::VectorXd xk = ekf.state;
//     Eigen::MatrixXd Pk = ekf.stateCovariance;
//     // --- create dynamic state transition matrix ---
//     Eigen::MatrixXd Fk = Eigen::MatrixXd::Identity(6, 6);
//     Fk.topRightCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * interval;
//     if (m_dimensions == 2)
//     {
//         Fk(2, 5) = 0;
//         Fk(5, 5) = 0;
//     }

//     // --- create dynamic process noise covariance matrix ---
//     Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3,3)*m_processNoise;
//     /* Checking if inverted
//     Eigen::MatrixXd Ak(6,3);
//     Ak << pow(interval,2)/2, 0, 0,
//           0, pow(interval,2)/2, 0,
//           0, 0, pow(interval,2)/2,
//           interval, 0, 0,
//           0, interval, 0,
//           0, 0, interval; */
//     Eigen::MatrixXd Ak(6,3);
//     Ak << interval, 0, 0,
//           0, interval, 0,
//           0, 0, interval,
//           pow(interval,2)/2, 0, 0,
//           0, pow(interval,2)/2, 0,
//           0, 0, pow(interval,2)/2;
//     Q = Ak * Q * Ak.transpose();

//     if (m_dimensions == 2)
//     {
//         Q(2, 2) = 0;
//         Q(5, 5) = 0;
//     }

//     // --- prediction ---
//     // estimate state
//     Eigen::VectorXd exk = Fk * xk;

//     // estimate covariance
//     Eigen::MatrixXd ePk = Fk * Pk * Fk.transpose() + Q;

//     // --- correction ---
//     Eigen::Vector3d referenceAnchor = anchorPositions.row(0);

//     // observation vector
//     Eigen::VectorXd td = anchorTOAs;
//     Eigen::VectorXd dv = td.tail(count - 1);
//     Eigen::Vector3d ep = exk.head(3);

//     Eigen::MatrixXd epmat = ep.transpose().replicate(count - 1, 1);
//     Eigen::MatrixXd refmat = referenceAnchor.transpose().replicate(count - 1, 1);
//     Eigen::MatrixXd anmat = anchorPositions.bottomRows(count - 1);

//     Eigen::MatrixXd ta1 = epmat - anmat;
//     Eigen::MatrixXd ta2 = epmat - refmat;

//     Eigen::MatrixXd distanceToAnchors = ta1.rowwise().norm();
//     Eigen::MatrixXd distanceToReference = ta2.rowwise().norm();
//     Eigen::MatrixXd ta1dr = distanceToAnchors.replicate(1, 3);
//     Eigen::MatrixXd ta2dr = distanceToReference.replicate(1, 3);
//     Eigen::MatrixXd t1 = ta1.cwiseQuotient(ta1dr);
//     Eigen::MatrixXd t2 = ta2.cwiseQuotient(ta2dr);
//     Eigen::MatrixXd j = t1 - t2;
//     Eigen::MatrixXd Hk(j.rows(), j.cols() + 3);
//     Eigen::MatrixXd pad = Eigen::MatrixXd::Zero(j.rows(), 3);
//     Hk << j, pad;

//     // innovation
//     Eigen::VectorXd expectedMeasurement = distanceToAnchors - distanceToReference;
//     Eigen::VectorXd observedMeasurement = dv;
//     Eigen::VectorXd innovation = observedMeasurement - expectedMeasurement;

//     // --- create dynamic TDOA measurement noise covariance matrix. ---
//     Eigen::MatrixXd Rk;
//     //Rk = Eigen::MatrixXd::Identity(count - 1, count - 1) * m_measurementNoise; // original Rk
//     Rk.setOnes(count - 1, count - 1);
//     Rk += Eigen::MatrixXd::Identity(count - 1, count - 1);

//     // covariance of the innovation
//     Eigen::MatrixXd Sk = Hk * ePk * Hk.transpose() + Rk;

//     // kalman gain computation
//     Eigen::MatrixXd Kk = ePk * Hk.transpose() * Sk.inverse();

//     // --- state update ---
//     // a posteriori state estimate
//     xk = exk + Kk * innovation;

//     Eigen::ArrayXd xkarray;
//     xkarray = xk.array();

//     // a posteriori state covariance
//     Pk = (Eigen::MatrixXd::Identity(6, 6) - Kk * Hk) * ePk;

//     ekf.state = xk;
//     ekf.stateCovariance = Pk;

//     // constrain min max
//     if(ekf.state[0]<m_minX) ekf.state[0]=m_minX; //return false;
//     if(ekf.state[1]<m_minY) ekf.state[1]=m_minY; //return false;
//     if(ekf.state[2]<m_minZ) ekf.state[2]=m_minZ; //return false;
//     if(ekf.state[0]>m_maxX) ekf.state[0]=m_maxX; //return false;
//     if(ekf.state[1]>m_maxY) ekf.state[1]=m_maxY; //return false;
//     if(ekf.state[2]>m_maxZ) ekf.state[2]=m_maxZ; //return false;

// /*    if(ekf.state[0]<m_minX) return false;
//     if(ekf.state[1]<m_minY) return false;
//     if(ekf.state[2]<m_minZ) return false;
//     if(ekf.state[0]>m_maxX) return false;
//     if(ekf.state[1]>m_maxY) return false;
//     if(ekf.state[2]>m_maxZ) return false;
// */

//     m_ekf[s.txeui] = ekf;

//     p->pos =  ekf.state.head(3);
//     p->dpos = ekf.state.tail(3);

//     return true;
// }

// bool PositionerTDOA::calculatePositionEKF(const sample_t &s, position_t *p, bool pzs_flag)
// {
//     if(m_ekf.find(s.txeui) == m_ekf.end())
//     {
//         createNewEKF(s.txeui, s.hts, pzs_flag);
//         m_updateCount.insert(std::make_pair(s.txeui, 0));
//         m_lastSample.insert(std::make_pair(s.txeui, s));
//         return false;
//     }

//     // reinitialize when diverging
//     double threshold = 100;
//     if(m_ekf[s.txeui].state.head(3).norm() > threshold)
//     {
//         ROS_WARN(" Norm: %.2f greater threshold, reinit EKF !!!", threshold);
//         createNewEKF(s.txeui, s.hts, pzs_flag);
//         return false;
//     }

//     // avoid uninitialized state
//     m_updateCount[s.txeui]++;
//     if(m_updateCount[s.txeui] < 2)
//     {
//         return false;
//     }

//     // set basic position result props
//     p->hts = s.hts;
//     p->eui = s.txeui;

//     p->imu.accel = s.meas.begin()->second.imu.accel;
//     p->imu.quat = s.meas.begin()->second.imu.quat;

//     bool discard = false;
//     if(s.meas.size() < m_minAnchor)
//     {
//         discard = true;
//         return false;
//     }

//     for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
//     {
//         if(m_lastSample[s.txeui].meas.find(it->first) != m_lastSample[s.txeui].meas.end())
//         {
//             //outlier detection
//             double lastTdoa = m_lastSample[s.txeui].meas[it->first].toa;
//             double tdoa = it->second.toa;
//             double diff = fabs(lastTdoa - tdoa);

//             if(diff > m_outlierThresholdDelta)
//             {
//                 //ROS_WARN(" Outlier: TDOA diff %#lx,%#lx, exceeded d: %.2f", s.txeui, it->first, diff);
//                 discard = true;
//             }

//             if(tdoa > m_outlierThreshold || tdoa < -m_outlierThreshold)
//             {
//                 //ROS_WARN(" Outlier: TDOA large %#lx,%#lx, exceeded threshold: %.2f", s.txeui, it->first, tdoa);
//                 discard = true;
//             }
//         }
//     }

//     if(!discard)
//     {
//         if(!calculatePositionEKFInner(s, p))
//         {
//             discard = true;
//             discard_counter++;

//             if (discard_counter > 32)
//             {
//                 ROS_WARN(" Discard counter exceeded, reinit EKF !!!");
//                 createNewEKF(s.txeui, s.hts);
//                 discard_counter = 0;
//                 return false;
//             }
//         }
//         else
//         {
//             discard_counter = 0;
//         }
//     }

//     m_lastSample[s.txeui] = s;

//     if(discard)
//     {
//         return false;
//     }

//     m_lastPosition[p->eui] = *p;

//     return true;
// }

bool PositionerTDOA::calculatePositionEKFInner(const sample_t &s, position_t *p, int count, bool pzs_flag)
{   
    if(!pzs_flag)
    {
        count = s.meas.size();
    }

    Eigen::MatrixXd anchorPositions(count, 3);
    Eigen::VectorXd anchorTOAs(count);

    // extracting the tdoa from selected anchors only, choose reference TOA
    int row = 0;
    double referenceTOA;

    for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
    {
        if((!pzs_flag) || (std::find(m_anchorsInZone[s.txeui].begin(), m_anchorsInZone[s.txeui].end(), it->first)!= m_anchorsInZone[s.txeui].end()))
        {
            //std::cout << std::hex << it->first << std::endl;
            if(row == 0)
            {
                referenceTOA = it->second.toa;
            }
            anchorPositions.row(row) = m_anchorPositions[it->first].transpose();
            anchorTOAs(row) = it->second.toa - referenceTOA;
            row++;
        }
    }

    // --- get the last interval ---
    double interval = m_initialInterval;
    if(m_lastPosition.find(p->eui) != m_lastPosition.end())
    {
        // assuming the first anchor stays the same !!
        double i = (s.hts - m_lastSample[s.txeui].hts).toSec();

        if(i > 0.0)
        {
            interval = i;
        }
        else
        {
            ROS_WARN(" Interval: %.6f smaller or equals zero !!!", i);
        }
    }

    // --- threshold to reinitialize ekf ---
    if(interval > 5.0)
    {
        createNewEKF(s.txeui, s.hts, pzs_flag);
        ROS_WARN(" Interval: %.6f greater threshold, reinit EKF !!!", interval);
    }

    
    // set fixed z-coordinate based on level 
    if(pzs_flag)
    {
        if(m_tagLevel[s.txeui] != m_prevTagLevel[s.txeui])
        {
            createNewEKF(s.txeui, s.hts, pzs_flag);
            std::cout << "create new EKF! Level: " << m_tagLevel[s.txeui] << " Previous: " << m_prevTagLevel[s.txeui] << std::endl; 
        }

        m_prevTagLevel[s.txeui] = m_tagLevel[s.txeui];
    }

    // --- get ekf state for specific participant ---
    ekf_t ekf = m_ekf[s.txeui];

    if (pzs_flag && m_dimensions == 2)
    {
        ekf.state[2] = m_fixedZ.at(m_tagLevel[s.txeui]);
    }

    Eigen::VectorXd xk = ekf.state;
    Eigen::MatrixXd Pk = ekf.stateCovariance;
    // --- create dynamic state transition matrix ---
    Eigen::MatrixXd Fk = Eigen::MatrixXd::Identity(6, 6);
    Fk.topRightCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * interval;
    if (m_dimensions == 2)
    {
        Fk(2, 5) = 0;
        Fk(5, 5) = 0;
    }

    // --- create dynamic process noise covariance matrix ---
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3,3)*m_processNoise;
    /* Checking if inverted
    Eigen::MatrixXd Ak(6,3);
    Ak << pow(interval,2)/2, 0, 0,
          0, pow(interval,2)/2, 0,
          0, 0, pow(interval,2)/2,
          interval, 0, 0,
          0, interval, 0,
          0, 0, interval; */
    Eigen::MatrixXd Ak(6,3);
    Ak << interval, 0, 0,
          0, interval, 0,
          0, 0, interval,
          pow(interval,2)/2, 0, 0,
          0, pow(interval,2)/2, 0,
          0, 0, pow(interval,2)/2;
    Q = Ak * Q * Ak.transpose();

    if (m_dimensions == 2)
    {
        Q(2, 2) = 0;
        Q(5, 5) = 0;
    }

    // --- prediction ---
    // estimate state
    Eigen::VectorXd exk = Fk * xk;

    // estimate covariance
    Eigen::MatrixXd ePk = Fk * Pk * Fk.transpose() + Q;

    // --- correction ---
    Eigen::Vector3d referenceAnchor = anchorPositions.row(0); 

    // observation vector
    Eigen::VectorXd td = anchorTOAs;
    Eigen::VectorXd dv = td.tail(count - 1);
    Eigen::Vector3d ep = exk.head(3);

    Eigen::MatrixXd epmat = ep.transpose().replicate(count - 1, 1);
    Eigen::MatrixXd refmat = referenceAnchor.transpose().replicate(count - 1, 1);
    Eigen::MatrixXd anmat = anchorPositions.bottomRows(count - 1);

    Eigen::MatrixXd ta1 = epmat - anmat;
    Eigen::MatrixXd ta2 = epmat - refmat;

    Eigen::MatrixXd distanceToAnchors = ta1.rowwise().norm();
    Eigen::MatrixXd distanceToReference = ta2.rowwise().norm();
    Eigen::MatrixXd ta1dr = distanceToAnchors.replicate(1, 3);
    Eigen::MatrixXd ta2dr = distanceToReference.replicate(1, 3);
    Eigen::MatrixXd t1 = ta1.cwiseQuotient(ta1dr);
    Eigen::MatrixXd t2 = ta2.cwiseQuotient(ta2dr);
    Eigen::MatrixXd j = t1 - t2;
    Eigen::MatrixXd Hk(j.rows(), j.cols() + 3);
    Eigen::MatrixXd pad = Eigen::MatrixXd::Zero(j.rows(), 3);
    Hk << j, pad;

    // innovation
    Eigen::VectorXd expectedMeasurement = distanceToAnchors - distanceToReference;
    Eigen::VectorXd observedMeasurement = dv;
    Eigen::VectorXd innovation = observedMeasurement - expectedMeasurement;

    // --- create dynamic TDOA measurement noise covariance matrix. ---
    Eigen::MatrixXd Rk;
    //Rk = Eigen::MatrixXd::Identity(count - 1, count - 1) * m_measurementNoise; // original Rk
    Rk.setOnes(count - 1, count - 1);
    Rk += Eigen::MatrixXd::Identity(count - 1, count - 1);

    // covariance of the innovation
    Eigen::MatrixXd Sk = Hk * ePk * Hk.transpose() + Rk;

    // kalman gain computation
    Eigen::MatrixXd Kk = ePk * Hk.transpose() * Sk.inverse();

    // --- state update ---
    // a posteriori state estimate
    xk = exk + Kk * innovation;

    Eigen::ArrayXd xkarray;
    xkarray = xk.array();

    // a posteriori state covariance
    Pk = (Eigen::MatrixXd::Identity(6, 6) - Kk * Hk) * ePk;

    ekf.state = xk;
    ekf.stateCovariance = Pk;

    // constrain min max
    if(ekf.state[0]<m_minX) {ekf.state[0]=m_minX; ekf.state[3]=0;}// return false;
    if(ekf.state[1]<m_minY) {ekf.state[1]=m_minY; ekf.state[4]=0;}// return false;
    if(ekf.state[2]<m_minZ) {ekf.state[2]=m_minZ; ekf.state[5]=0;}// return false;
    if(ekf.state[0]>m_maxX) {ekf.state[0]=m_maxX; ekf.state[3]=0;}// return false;
    if(ekf.state[1]>m_maxY) {ekf.state[1]=m_maxY; ekf.state[4]=0;}// return false;
    if(ekf.state[2]>m_maxZ) {ekf.state[2]=m_maxZ; ekf.state[5]=0;}// return false;

    // if(ekf.state[0]<m_minX) return false;
    // if(ekf.state[1]<m_minY) return false;
    // if(ekf.state[2]<m_minZ) return false;
    // if(ekf.state[0]>m_maxX) return false;
    // if(ekf.state[1]>m_maxY) return false;
    // if(ekf.state[2]>m_maxZ) return false;

    m_ekf[s.txeui] = ekf;

    p->pos =  ekf.state.head(3);
    p->dpos = ekf.state.tail(3);

    return true;
}

bool PositionerTDOA::calculatePositionEKF(const sample_t &s, position_t *p, bool pzs_flag)
{
    bool discard = false;
    int count = 0;

    if(pzs_flag)
    {
        //initialize level count
        std::vector<int> measPerLevel(m_fixedZ.size(), 0);
        std::vector<int> measPerCell(m_cellAnchors.size(), 0);

        for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
        {
            //how many measurements from level x
            measPerLevel.at(m_anchorLevels[it->first])++;

            //how many measurements from cell x
            measPerCell.at(m_anchorCells[it->first])++;
        }

        // estimate level with most measurements
        int level = std::max_element(measPerLevel.begin(),measPerLevel.end()) - measPerLevel.begin();
        m_tagLevel[s.txeui] = level;

        // estimate cell with most measurements
        int initCell = std::max_element(measPerCell.begin(),measPerCell.end()) - measPerCell.begin();
        m_tagCell[s.txeui] = initCell;

        std::vector<int> cellsInZone;

        //compute predicted position
        double interval = 0;

        if(m_lastPosition.find(p->eui) != m_lastPosition.end())
        {
            interval = (s.hts - m_lastSample[s.txeui].hts).toSec();
        }

        Eigen::Vector3d ppos;

        if(interval > 3.0){
            ppos << -1,-1,-1;
        }
        else
        {
            ppos = m_lastPosition[s.txeui].pos + m_lastPosition[s.txeui].dpos*interval;
        }

        //set predicted z-coordinate to fixed z in 2D mode 
        if (m_dimensions == 2)
        {
            ppos[2] = m_fixedZ.at(level);
        }
        
        int cellNumber = m_cellBounds.size()/6;
        bool isInLocArea = false;

        //predicted position is located in cell x 
        for(int i = 0; i < cellNumber; i++)
        {
            if((ppos(0) > m_cellBounds(i,0)) && (ppos(0) < m_cellBounds(i,1)) 
                && (ppos(1) > m_cellBounds(i,2)) && (ppos(1) < m_cellBounds(i,3))
                && (ppos(2) > m_cellBounds(i,4)) && (ppos(2) < m_cellBounds(i,5)))
            {
                cellsInZone.push_back(i);
                isInLocArea = true;
                break;
            }
        }

        if(!isInLocArea)
        {
            for(int i = 0; i < cellNumber; i++)
            { 
                cellsInZone.push_back(i);
            }
        }

        //sort remaining cells by closest Euclidean distance to predicted position
        std::vector<std::pair<double,int>> distanceToCell;

        if(isInLocArea)
        {
            for(int i = 0; i < cellNumber; i++)
            {
                //exclude already chosen cell and cells with no measurement
                if (i!=cellsInZone[0] && measPerCell[i]!=0)
                {
                    int dist = sqrt((ppos(0)-m_cellCenters(i,0))*(ppos(0)-m_cellCenters(i,0))+
                                    (ppos(1)-m_cellCenters(i,1))*(ppos(1)-m_cellCenters(i,1))+
                                    (ppos(2)-m_cellCenters(i,2))*(ppos(2)-m_cellCenters(i,2)));

                    distanceToCell.push_back(std::make_pair(dist, i));
                }
            }
            if(!distanceToCell.empty())
            {
                sort(distanceToCell.begin(), distanceToCell.end());

                for(int i = 0; i < m_cellsPerZone-1; i++)
                {
                    cellsInZone.push_back(distanceToCell[i].second);
                }
            }
        }
        
        //count only measurements of anchors chosen by Predictive Zone Selection
        //int count = 0;

        std::vector<uint64_t> temp;
        for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
        {
            for(int i = 0; i < cellsInZone.size(); i++)
            {
                if(std::find(m_cellAnchors[cellsInZone[i]].begin(), m_cellAnchors[cellsInZone[i]].end(), 
                            it->first)!= m_cellAnchors[cellsInZone[i]].end())                          
                {
                    count++;
                    temp.push_back(it->first);
                }
            }
        }

        m_anchorsInZone.find(s.txeui)->second = temp;

        if(temp.empty())
        {
            discard = true;
            return false;
        }
    }

    if(m_ekf.find(s.txeui) == m_ekf.end())
    {
        createNewEKF(s.txeui, s.hts, pzs_flag);
        m_updateCount.insert(std::make_pair(s.txeui, 0));
        m_lastSample.insert(std::make_pair(s.txeui, s));
        return false;
    }

    // reinitialize when diverging
    double threshold = 100;
    if(m_ekf[s.txeui].state.head(3).norm() > threshold)
    {
        ROS_WARN(" Norm: %.2f greater threshold, reinit EKF !!!", threshold);
        createNewEKF(s.txeui, s.hts, pzs_flag);
        return false;
    }

    // avoid uninitialized state
    m_updateCount[s.txeui]++;
    if(m_updateCount[s.txeui] < 2)
    {
        return false;
    }

    // set basic position result props
    p->hts = s.hts;
    p->eui = s.txeui;

    p->imu.accel = s.meas.begin()->second.imu.accel;
    p->imu.quat = s.meas.begin()->second.imu.quat;

    if(s.meas.size() < m_minAnchor)
    {
        discard = true;
        return false;
    }

    int row = 0;
    double referenceTOA = 0.0;
    double last_referenceTOA = 0.0;

    for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
    {
        if(m_lastSample[s.txeui].meas.find(it->first) != m_lastSample[s.txeui].meas.end())
        {
            if((!pzs_flag) || (std::find(m_anchorsInZone[s.txeui].begin(), m_anchorsInZone[s.txeui].end(), it->first)!= m_anchorsInZone[s.txeui].end()))
            {
                if(row == 0)
                {
                    referenceTOA = it->second.toa;
                    last_referenceTOA = m_lastSample[s.txeui].meas[it->first].toa;
                } 
                                
                //outlier detection
                double lastTdoa = m_lastSample[s.txeui].meas[it->first].toa;
                double tdoa = it->second.toa;
                double diff = fabs(lastTdoa - tdoa);

                if(diff > m_outlierThresholdDelta)
                {
                    //ROS_WARN(" Outlier: TDOA diff %#lx,%#lx, exceeded d: %.2f", s.txeui, it->first, diff);
                    discard = true;
                }

                if(tdoa > m_outlierThreshold || tdoa < -m_outlierThreshold)
                {
                    //ROS_WARN(" Outlier: TDOA large %#lx,%#lx, exceeded threshold: %.2f", s.txeui, it->first, tdoa);
                    discard = true;
                }
            }
        }
        row++;
    }

    if(!discard)
    {
        if(!calculatePositionEKFInner(s, p, count, pzs_flag))
        {
            discard = true;
            discard_counter++;

            if (discard_counter > 32)
            {
                ROS_WARN(" Discard counter exceeded, reinit EKF !!!");
                createNewEKF(s.txeui, s.hts, pzs_flag);
                discard_counter = 0;
                return false;
            }
        }
        else
        {
            discard_counter = 0;
        }
    }

    m_lastSample[s.txeui] = s;

    if(discard)
    {
        return false;
    }

    m_lastPosition[p->eui] = *p;

    return true;
}