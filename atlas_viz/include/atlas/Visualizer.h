//
//  Visualizer.h
//  atlas
//
//  Created by Janis on 18.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__visualizer__
#define __atlas__visualizer__

#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <deque>

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>

class Visualizer
{
private:
    // system parameters
    std::map<uint64_t, Eigen::Vector3d> m_anchorPositions;

    // anchor publisher
    ros::Publisher m_pubAnchors;

public:
    Visualizer ();

    void initialize(ros::NodeHandle n);
    void visualizeAnchors(void);
};

#endif /* defined(__atlas__visualizer__) */
