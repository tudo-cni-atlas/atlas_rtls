//
//  main.cpp
//  atlas
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
#include <string>
#include <cstdlib>

#include "ros/ros.h"

#include "parser.h"

void signal_handler(int signal)
{
    std::cerr << " Quitting..." << std::endl;
    ros::shutdown();
    exit (EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signal_handler);
    ros::init(argc, argv, "atlas_node_bridge");

    ros::NodeHandle n("~");
    auto node_name = ros::this_node::getName();
    auto node_namespace = ros::this_node::getNamespace();

    std::cout << "Name: " << node_name << std::endl;
    std::cout << "Namespace: " << node_namespace << std::endl;

    node_name.erase(0, node_namespace.size() + 1);
    uint64_t eui = std::stoull(node_name, nullptr, 16);

    std::cout << "After Erase_ " << node_name << std::endl;

    Parser parser;
    parser.initialize(&n, eui);
    ros::Rate r(2048); //Maximum frequency = 16 slots * 128Hz = 2048 Hz

    while (ros::ok())
    {
        parser.poll();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
