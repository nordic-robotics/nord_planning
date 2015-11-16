#include "cone.hpp"

//from here ill get start position and link to all nodes
ConeOfSight::ConeOfSight(const map& maze){
	cone_angle = M_PI/6.0; //30 deg
	start_direction = 0;
	x_max = 100*maze.get_max_x();
	y_max = 100*maze.get_max_y();
	//find out start pos from first node
	explored = std::vector<std::vector<float>>(x_max, std::vector<float>(y_max));
	current_x = 79;
	current_y = 20;
	first_time = true;
}


//itteratative rotation
void ConeOfSight::rotateCone(double rotation){
	while(abs(rotation) > M_PI/36.0){ //coresponds to 5 degrees
		if(rotation > 0){
			start_direction += M_PI/36.0;
			rotation -= M_PI/36.0;
			createCone(); 
		}
		else{
			start_direction -= M_PI/36.0;
			rotation += M_PI/36.0;
			createCone();
		}		 
	}
	start_direction += rotation;
	createCone();	
}

//move the cone from point 1 to point 2
void ConeOfSight::moveCone(int new_x, int new_y){

	if(current_x == new_x){
		while(current_y != new_y){
			current_y++;
			createCone();
		}
	}	
	else if(current_y == new_y){
		while(current_x != new_x){
			current_x++;
			createCone();
		}
	}

	double dx = new_x - current_x;
	double dy = new_y - current_y;
	
	//will move cone in a straigth line from node(n) -> node(n+1)
	while(current_x != new_x && current_y != new_y){

		//------------------------------------------------
		if(dx > 0 && dy > 0){
			if(dx < dy){
				current_y++;
				current_x += floor(dx/dy*current_y);
				createCone();
			}
			else if(dy < dx){
				current_x++;
				current_y += floor(dy/dx*current_x);
				createCone(); 
			}
			else{
				current_x++;
				current_y++;
				createCone();
			}
		}
		//-----------------------------------------------
		else if(dx < 0 && dy < 0){
			if(dx > dy){
				current_y--;
				current_x -= abs(floor(dx/dy*current_y));
				createCone();
			}
			else if(dy > dx){
				current_x--;
				current_y -= abs(floor(dy/dx*current_x));
				createCone(); 
			}
			else{
				current_x--;
				current_y--;
				createCone();
			}
		}
		// ----------------------------------------------
		else if(dx < 0 && dy>0){
			if(abs(dx) > dy){
				current_y++;
				current_x -= abs(floor(dy/dx*current_y));
				createCone();
			}
			else if(dy > abs(dx)){
				current_x--;
				current_y += abs(floor(dy/dx*current_x));
				createCone(); 
			}
			else{
				current_x--;
				current_y++;
				createCone();
			}
		}
		//------------------------------------------------
		else{
			if(dx < abs(dy)){
				current_y--;
				current_x += abs(floor(dy/dx*current_y));
				createCone();
			}
			else if(abs(dy) > dx){
				current_x++;
				current_y -= abs(floor(dy/dx*current_x));
				createCone(); 
			}
			else{
				current_x++;
				current_y--;
				createCone();
			}
		}
	}	 
}



void ConeOfSight::createCone(){

	// will fill cones coordinates with 1s needs to be entire map
	cone_matrix = std::vector<std::vector<int>>(x_max,std::vector<int>(y_max));
	//cone length
	int x = current_x + 80;
	int y = current_y + 0;

	//makes sure it handles different angles after first call
	if(first_time == true){
		first_time = false;
	}
	else{
		x = current_x + floor(x*cos(start_direction) - y*sin(start_direction));
		y = current_y + floor(x*sin(start_direction) + y*cos(start_direction));
	}


	//Calculating all the end points of the cone
	double right_angle = start_direction - cone_angle;
	double left_angle  = start_direction + cone_angle;


	//rotating accordingly with starting pos
	int right_x  = current_x + floor(x*cos(right_angle) - y*sin(right_angle));
	int right_y  = current_y + floor(x*sin(right_angle) + y*cos(right_angle));
	int left_x   = current_x + floor(x*cos(left_angle) - y*sin(left_angle));
	int left_y   = current_y + floor(x*sin(left_angle) + y*cos(left_angle));

	
	int x_min = determineSmallest(current_x, right_x, left_x);
    int y_min = determineSmallest(current_y, right_y, left_y);
    int x_max = determineLargest(current_x, right_x, left_x);
    int y_max = determineLargest(current_y, right_y, left_y);
    //creates a box that we wil fill in with the actual cone

    double A = triangleArea(current_x, current_y, right_x, right_y, left_x ,left_y); 
    double A1; double A2; double A3;
    double acceptable_error = 0.01; 
    //basically checking if particles in a square twice the size the size of 
    //
    for(int temp_y = y_min; temp_y <= y_max; temp_y++){
    	for(int temp_x = x_min; temp_x <= x_max; temp_x++){
    		A1 = triangleArea(temp_x, temp_y, right_x, right_y, left_x, left_y);
    		A2 = triangleArea(current_x, current_y, temp_x, temp_y, left_x, left_y);
    		A3 = triangleArea(current_x, current_y, right_x, right_y, temp_x, temp_y);
    		//checking if coordinate is within triangle
    		if(abs(A-(A1 + A2 + A3)) <= acceptable_error){
    			//if not explored, say it is explored
    			if(explored[current_x][current_y] != 0){
    				explored[current_x][current_y] = 1;
    			}
    			cone_matrix[current_x][current_y] = 1; // we are seeing this 
    		}
    	} 
	}
}

//calculates the area.
double ConeOfSight::triangleArea(int x1, int y1, int x2, int y2, int x3, int y3){
	return abs((x1*(y2-y3) + x2*(y3-y1)+ x3*(y1-y2))/2.0); 
}

int ConeOfSight::determineSmallest(int n1, int n2,int n3){
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

int ConeOfSight::determineLargest(int n1,int n2,int n3){
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

void ConeOfSight::changeWidth(float new_width){
	cone_angle = new_width/2;
	createCone();
}

