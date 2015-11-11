#include "cone.hpp"

//from here ill get start position and link to all nodes
void ConeOfSight(std::vector<dijkstra::point> graph){
	cone_angle = M_PI/6.0; //30 deg
	start_direction = 0;
	int x_n = 200;
	int y_n = 200;
	//find out start pos from first node
	explored = std::vector<std::vector<float>>(x_n, std::vector<float>(y_n));
}

//itteratative rotation
void rotateCone(){
	

}

void moveCone(int new_x, int new_y){

	// y = kx + m;
	double k = double(new_x -current_x)/double(new_y-current_y);
	double m = current_y - k*current_x;
	int x_temp = current_x; int y_temp = current_y;

	while(x_temp != new_x && y_temp != new_y){
	
		if(y_temp != new_y){

			createCone();
		}
	
	}

	current_x = new_x;
	current_y = new_y; 
}


std::vector<std::vector<int>> createCone(){

	// will fill cones coordinates with 1s needs to be entire map
	std::vector<std::vector<int>> cone_sight = std::vector<std::vector<int>>(200,std::vector<int>(200));
	//cone length
	int x = 80;
	int y = 0;


	//Calculating all the end points of the cone
	double right_angle = start_direction - cone_angle;
	double left_angle  = start_direction + cone_angle;


	//rotating accordingly with starting pos
	int right_x  = starting_x*100 + floor(x*cos(right_angle) - y*sin(right_angle));
	int right_y  = starting_y*100 + floor(x*sin(right_angle) + y*cos(right_angle));
	int left_x   = starting_x*100 + floor(x*cos(left_angle) - y*sin(left_angle));
	int left_y   = starting_y*100 + floor(x*sin(left_angle) + y*cos(left_angle));

	
	x_min = determineSmallest(starting_x, right_x, left_x);
    y_min = determineSmallest(starting_y, right_y, left_y);
    x_max = determineLargest(starting_x, right_x, left_x);
    y_max = determineLargest(starting_y, right_y, left_y);
    //creates a box that we wil fill in with the actual cone

    double A = triangleArea(starting_x, starting_y, right_x, right_y, left_x ,left_y); 
    double A1; double A2; double A3;
    double acceptable_error = 0.001;

    //basically checking if particles in a square twice the size the size of 
    //
    for(int current_y = y_min; current_y <= y_max; current_y++){
    	for(int current_x = x_min; current_x <= x_max; current_x++){
    		A1 = triangelArea(current_x, current_y, x2, y2, x3, y3);
    		A2 = triangleArea(x1, y1, current_x, current_y, x3, y3);
    		A3 = triangelArea(x1, y1, x2, y2, current_x,current_y);
    		//checking if coordinate is within triangle
    		if(abs(A-(A1 + A2 + A3)) <= acceptable_error){
    			//if not explored, say it is explored
    			if(explored[current_x][current_y] != 0){
    				explored[current_x][current_y] = 1;
    			}
    			cone_sight[current_x][current_y] = 1; // we are seeing this 
    		}
    	} 
	}
	return cone_sight;
}

//calculates the area.
double triangelArea(int x1, int y1, int x2, int y2, int x3, int y3){
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