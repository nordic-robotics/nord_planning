#pragma once

#include "ros/ros.h"
#include "ros/package.h"
#include <fstream>
#include <sstream>
#include <string>
#include "dijkstra/map.hpp"
#include <math.h> 

namespace graph
{
	class Maps{
	public:
		Maps() { };
		std::vector< std::vector<int> > read_map(std::string filename);
		void create_pointmap();
		void create_pot_map();
		void move_points();
		void print_info();
		void create_graph();
	private:
		void add_pot(int cx, int cy,std::vector< std::vector<long int> > pot,int n);
		void remove_pot(int cx, int cy,std::vector< std::vector<long int> > pot,int n);
		std::vector<int> next_place(int fx, int fy, int step);
	};	
	
}