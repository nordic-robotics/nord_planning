#include "path_helper.hpp"

PathHelper::PathHelper(int x_max, int y_max){
	explored = std::valarray<bool>(false, x_max * y_max);
	path.clear();
}

void calc_time(double rotation_t, double forward_t){
	total_time = rotation_t + forward_t;
}

