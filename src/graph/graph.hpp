#pragma once

#include "ros/ros.h"
#include "ros/package.h"
#include <fstream>
#include <sstream>
#include <string>
#include "../dijkstra/map.hpp"
#include <math.h> 

namespace graph
{
	class Maps{
	public:
		int n_wall=37;int n_point=19;
		Maps(bool dijk);
		std::vector< std::vector<int> > read_map(std::string filename);
		void create_pointmap();
		void create_pot_map();
		void move_points();
		void print_info();
		dijkstra::map * create_graph();
	private:
		float min_x, min_y, max_x, max_y;
		std::vector< std::vector<long int> > pot_wall; std::vector< std::vector<long int> > pot_point; 
		std::vector< std::vector<long int> > pot_map; std::vector< std::vector<int> > map; 
		std::vector< std::vector<int> > pointmap;
		int num_points;
		dijkstra::map m;
		std::vector<dijkstra::point> graph;
		bool dijk;
		void add_pot(int cx, int cy,std::vector< std::vector<long int> > pot,int n);
		void remove_pot(int cx, int cy,std::vector< std::vector<long int> > pot,int n);
		std::vector<int> next_place(int fx, int fy, int step);
	};	
	
}
