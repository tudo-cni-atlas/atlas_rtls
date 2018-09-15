//
//  main.cpp
//  atlas_viz
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

#include "atlas/Visualizer.h"


void signal_handler(int signal)
{
    std::cerr << " Quitting..." << std::endl;
    ros::shutdown();
    exit (EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);

    ros::init(argc, argv, "atlas_viz");

    ros::NodeHandle n("~");
    ros::Rate rate(0.2);

    Visualizer viz;
    viz.initialize(n);

    while (ros::ok())
    {
        viz.visualizeAnchors();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


