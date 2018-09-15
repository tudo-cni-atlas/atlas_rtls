//  main.cpp
//  atlas fast
//
//  Created by Yehya on 05.07.18
//  Copyright (c) 2018 Janis. All rights reserved.
//

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <csignal>
#include <string.h>

#include <ros/ros.h>

#include <atlas_fast/Scheduler.h>

void signal_handler(int signal)
{
    std::cerr << " Quitting..." << std::endl;
    ros::shutdown();
    exit (EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);

    ros::init(argc, argv, "fast");

    std::string node_name = ros::this_node::getName();
    std::cout << node_name << std::endl;
    ros::NodeHandle n("~");

    Scheduler scheduler(n);

    ros::Rate rate(2500);

    while (ros::ok())
    {
        scheduler.process();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}



