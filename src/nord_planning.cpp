#include "ros/ros.h"
#include "ros/package.h"
#include <fstream>
#include <sstream>
#include <string>
#include "graph/graph.hpp"

//Run with 0 as input for Tobias, 1 for Lucas

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nord_planning");
    ros::NodeHandle n;
	bool dijk = true;
	graph::Maps run(dijk);
	
	run.move_points();
	run.create_graph();
	run.print_info();

	dijk = false;
	std::cout << "run 2" << std::endl;
	graph::Maps run2(dijk);
	run2.move_points();
	run2.create_graph();
	run2.print_info();
		
    return 0;
}
