#include "ros/ros.h"
#include "ros/package.h"
#include <fstream>
#include <sstream>
#include <string>
#include "graph/graph.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nord_planning");
    ros::NodeHandle n;
	
	graph::Maps run;
	
	run.move_points();
	//create graph receives the ros::NodeHandle because at the end it needs to publish the graph....
	run.create_graph(n);
	run.print_info();
    return 0;
}
