#include "ros/ros.h"
#include "ros/package.h"
#include <fstream>
#include <sstream>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nord_planning");
    ros::NodeHandle n;
	
	graph::Maps run;
	
	run.move_points();
	run.create_graph();
	run.print_info();
    return 0;
}