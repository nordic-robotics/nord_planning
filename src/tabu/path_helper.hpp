#pragma once
#include "cone.hpp"
class PathHelper(){
	public:
		std::vector<Position> path;
		
		PathHelper(int x_max,int y_max);
		void calc_time(double rotation_t, double forward_t);

		const std::vector<Position>& get_path()const{return path};
		const std::valarray<bool>& get_explored()const {return explored};
		const double& get_time()const {return total_time};
	
	private:
		std::valarray<bool> explored;
		double total_time;
		
};