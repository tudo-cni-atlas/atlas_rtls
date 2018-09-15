//
//  main.cpp
//  atlas core
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

#include "atlas_core/atlas_types.h"
#include "atlas_core/Assembly.h"
#include "atlas_core/ClockCorrection.h"
#include "atlas_core/Reporter.h"


void signal_handler(int signal)
{
    std::cerr << " Quitting..." << std::endl;
    ros::shutdown();
    exit (EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);

    ros::init(argc, argv, "core");

    std::string node_name = ros::this_node::getName();
    std::cout << node_name << std::endl;
    ros::NodeHandle n("~");
    ros::Rate rate(2400);

    Assembly assembly;
    assembly.initialize(n);

    ClockCorrection cor;
    cor.initialize(n);

    Reporter rep;

    while (ros::ok())
    {
        std::vector<sample_t> s;
        assembly.extractSamples(&s);
        
        std::vector<sample_t> cs;
        cor.processSamples(&s);
        cs = cor.getCorrectedSamples();

        for (auto it = cs.begin(); it != cs.end(); ++it)
        {
            sample_t s = *it;
            rep.reportSample(s);
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


