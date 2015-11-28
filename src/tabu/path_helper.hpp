#pragma once
#include "cone.hpp"
class PathHelper(){
	public:
		std::vector<Position> path;
		
		PathHelper(int x_max,int y_max);
		void calc_time(double rotation_t, double forward_t);

		const std::vector<Position>& get_path()const{return path};
		const std::vector<std::vector<float>>& get_explored()const {return explored};
		const double& get_time()const {return total_time};
	
	private:
		std::vector<std::vector<float>> explored;
		double total_time;
		
};