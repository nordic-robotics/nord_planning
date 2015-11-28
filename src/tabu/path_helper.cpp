#include "path_helper.hpp"

PathHelper::PathHelper(int x_max, int y_max){
	explored = std::vector<std::vector<float>>(x_max, std::vector<float>y_max);
	path.clear();
}

void calc_time(double rotation_t, double forward_t){
	total_time = rotation_t + forward_t;
}

