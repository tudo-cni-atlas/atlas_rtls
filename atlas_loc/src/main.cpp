//
//  main.cpp
//  atlas localization
//
//  Created by Janis on 26.03.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <csignal>
#include <exception>

#include "ros/ros.h"

#include "atlas_loc/atlas_types.h"
#include "atlas_loc/Positioner.h"
#include "atlas_loc/Reporter.h"


void signal_handler(int signal)
{
    std::cerr << " Quitting..." << std::endl;
    ros::shutdown();
    exit (EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);

    ros::init(argc, argv, "atlas");

    ros::NodeHandle n("~");
    ros::Rate rate(2400);

    Reporter rep;
    rep.initialize(n);

    PositionerTDOA pos;
    pos.initialize(n);

    while (ros::ok())
    {
        std::vector<sample_t> s;
        pos.extractSamples(&s);

        for (auto it = s.begin(); it != s.end(); ++it)
        {
            sample_t s = *it;
            position_t p;
            if(pos.calculatePositionEKF(s, &p))
            {
                rep.reportPosition(p);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


