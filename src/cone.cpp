#include "cone.hpp"

//from here ill get start position and link to all nodes
void ConeOfSight(std::vector<dijkstra::point> graph){
	cone_angle = M_PI/6.0; //30 deg
	start_direction = 0;

	//find out start pos from first node
}


void rotateCone(){

}

void moveCone(){

}

std::vector<std::vector<int>> createCone(map maze){
	std::vector<std::vecinttor<int>> seen;
	std::vector<int> x_y;

	//Calculating all the end points of the cone
	double right_angle = start_direction - cone_angle;
	double left_angle  = start_direction + cone_angle;
	int x = 80;
	int y = 0;
	//uses centimeter to make calc easier original position is assumed to be 80 x an 0 y
	
	int right_x  = starting_x*100 + floor(x*cos(right_angle) - y*sin(right_angle));
	int right_y  = starting_y*100 + floor(x*sin(right_angle) + y*cos(right_angle));
	int left_x   = starting_x*100 + floor(x*cos(left_angle) - y*sin(left_angle));
	int left_y   = starting_y*100 + floor(x*sin(left_angle) + y*cos(left_angle));
	//inclination of the outside of the cone
	//double incline_right = right_x-starting_x/right_y-starting_y;
	//double incline_left  = left_x-starting_x/right_y-starting_y;
	
	x_min = determineSmallest(starting_x, right_x, left_x);
    y_min = determineSmallest(starting_y, right_y, left_y);
    x_max = determineLargest(starting_x, right_x, left_x);
    y_max = determineLargest(starting_y, right_y, left_y);

 //    1) Calculate area of the given triangle, i.e., area of the triangle ABC in the above diagram. 

 //    Area A = [ x1(y2 - y3) + x2(y3 - y1) + x3(y1-y2)]/2

	// 2) Calculate area of the triangle PAB. We can use the same formula for this. Let this area be A1.
	// 3) Calculate area of the triangle PBC. Let this area be A2.
	// 4) Calculate area of the triangle PAC. Let this area be A3.
	// 5) If P lies inside the triangle, then A1 + A2 + A3 must be equal to A.
    double A = triangleArea(starting_x, right_x, left_x, starting_y, right_y,left_y); 

    for(int current y = y_min; current_y <= y_max; current_y++){
    	for(int current_x = x_min; current_x <= x_max; current_x++){

    	} 
	}
	return seen;
}

double triangelArea(int x1, int x2, int x3, int y1, int y2, int y3){
	return abs((x1*(y2-y3) + x2*(y3-y1)+ x3*(y1-y2))/2.0); 
}

int determineSmallest(int n1, int n2,int n3){
	if(n1 <= n2 && n1 <= n3){
		return n1;
	}
	else if(n2 <= n1 && n2 <= n3){
		return n2;
	}
	else{
		return n3;
	}		
}

int determineLargest(int n1,int n2,int n3){
 	if(n1 >= n2 && n1 >= n3){
		return n1;
	}
	else if(n2 >= n1 && n2 >= n3){
		return n2;
	}
	else{
		return n3;
	}		
}

void changeWidth(float new_width){
	cone_angle = new_width/2;
	createCone();
}

//should store all new position
void storeExplored(std::vector<std::vector<int>> positions){


}