#include "cone.hpp"

//from here ill get start position and link to all nodes
ConeOfSight::ConeOfSight(map* maze,int start_x,int start_y){

	cone_angle = M_PI/6.0; //30 deg
	current_direction = 0;
	this->maze = maze;
	float temp_x = maze->get_max_x(); 
	float temp_y = maze->get_max_y();
	largest_y = temp_y*100; 	current_x = start_x;
	largest_x = temp_x*100;	    current_y = start_y;
	explored = std::vector<std::vector<float>>(largest_x, std::vector<float>(largest_y));
	time_moving = 0; time_rotating = 0;
	start_x = current_x;
	start_y = current_y;
	path.clear();
	
	Position init(start_x, start_y); 
	point_cap =false;
	add_to_path(init);
	createCone();
}

ConeOfSight::ConeOfSight(ConeOfSight const& cone){
	cone_angle = M_PI/6;
	current_direction = cone.current_direction;
	explored = cone.explored;
	time_moving = cone.time_moving; time_rotating = cone.time_rotating;
	path = cone.path;
	current_x = cone.current_x; current_y = cone.current_y;
	cone_matrix = cone.cone_matrix;
	point_cap = cone.point_cap;
	largest_x = cone.largest_x; largest_y = cone.largest_y;
	this->maze = maze;
	// std::cout << "no problem" << std::endl;
}

void ConeOfSight::add_to_path(Position pos){
	path.push_back(pos);
}


void ConeOfSight::resetExplored(){
	explored = std::vector<std::vector<float>>(largest_x, std::vector<float>(largest_y));
}

//itteratative rotation
void ConeOfSight::rotateCone(int new_x, int new_y){
	//ROS_INFO("rotation = %f", rotation);
	
	int dx = new_x - current_x; 
	int dy = new_y - current_y;
	//std::cout << dx << " " << dy << std::endl;
	
	double rotation = std::atan2(dy,dx);
	double angle_diff = rotation-current_direction;
	double real_rotation = std::atan2(std::sin(angle_diff), std::cos(angle_diff));
	//std::cout << "real_rotation = " << real_rotation*180/M_PI <<std::endl;

	time_rotating += 0.2 + real_rotation * 1/M_PI;
	//std::cout << "time_rotating = " << 0.2 + real_rotation * 1/M_PI << std::endl; 
	while(std::fabs(real_rotation) > (M_PI/36.0)){ //coresponds to 5 degrees
		if(real_rotation > 0){
			current_direction += M_PI/36.0;
			real_rotation -= M_PI/36.0;
			createCone(); 
		}
		else{
			current_direction -= M_PI/36.0;
			real_rotation += M_PI/36.0;
			createCone();
		}		 
	}
	current_direction += real_rotation;
	createCone();	
}


//move the cone from point 1 to point 2
void ConeOfSight::moveCone(int new_x, int new_y){
	// std::cout << "moving cone" << std::endl;
	if(current_x == new_x){
		// std::cout << "going straight in y direction" << std::endl;
		while(current_y != new_y){
			if(new_y > current_y){
				++current_y;
			}
			else{
				--current_y;
			}
			createCone();
		}
		
	}	
	else if(current_y == new_y){
		// std::cout << "going straigh in x direction" << std::endl;
		while(current_x != new_x){
			if(new_x > current_x){
				++current_x;
			}
			else{
				--current_x;
			}
			createCone();
		}
		// std::cout <<"made it without any problem" << std::endl;
	}

	double dx = new_x - current_x;
	double dy = new_y - current_y;
	double k = dy/dx;
	double m = current_y - k * current_x;

	time_moving += 0.2 + std::sqrt(std::pow(dy,2)+ std::pow(dx,2))*1/35;
	//std::cout << " distance moved = " <<  std::sqrt(std::pow(dy,2)+ std::pow(dx,2)) << std::endl;
	//std::cout << "time moving = " << 0.2 + std::sqrt(std::pow(dy,2)+ std::pow(dx,2))*1/35 << std::endl;
	//will move cone in a straigth line from node(n) -> node(n+1)
	while(!(current_x == new_x && current_y == new_y)){

		//------------------------------------------------
		if(dx > 0 && dy > 0){
			// std::cout << "both postive" << std::endl;
			if(dx < dy){
				++current_y;
				current_x = floor((current_y-m)/k);
				if(current_y == new_y && current_x != new_x){
					// std::cout << "Cheat amount = " << current_x - new_x << std::endl;
					// std::cout << "Cheat activated" << std::endl;
					current_x = new_x;
				} 
				createCone();
			}
			else if(dy < dx){
				++current_x;
				current_y = floor(k*current_x + m);
				if(current_x == new_x && current_y != new_y){
					// std::cout << "Cheat activated" << std::endl;
					// std::cout << "Cheat amount = " << current_y - new_y << std::endl;
					current_y = new_y;
				} 
				createCone(); 
			}
			else{
				++current_x;
				++current_y;
				createCone();
			}
			// std::cout <<"made it without any problem" << std::endl;
		}
		//-----------------------------------------------
		else if(dx < 0 && dy < 0){
			// std::cout << "both negative" << std::endl;
			if(dx > dy){
				--current_y;
				current_x = floor((current_y-m)/k);
				if(current_y == new_y && current_x != new_x){
					current_x = new_x;
				} 
				createCone();
			}
			else if(dy > dx){
				--current_x;
				current_y = floor(k*current_x + m);
				if(current_x == new_x && current_y != new_y){
					current_y = new_y;
				}  
				createCone();
			}
			else{
				--current_x;
				--current_y;
				createCone();
			}
			// std::cout <<"made it without any problem" << std::endl;
		}
		// ----------------------------------------------
		else if(dx < 0 && dy>0){
			// std::cout <<" dx< 0 && dy > 0" << std::endl;
			if(std::abs(dx) < dy){
				++current_y;
				current_x = floor((current_y-m)/k);
				createCone();
				if(current_y == new_y && current_x != new_x){
					current_x = new_x;
				} 
			}
			else if(dy < std::abs(dx)){
				
				--current_x;
				current_y = floor(k*current_x + m);

				if(current_x == new_x && current_y != new_y){
					current_y = new_y;
				} 

				createCone(); 
			}
			else{
				--current_x;
				++current_y;
				createCone();
			}
			// std::cout <<"made it without any problem" << std::endl;
		}
		//------------------------------------------------
		else if(dx > 0 && dy < 0){
			//std::cout <<"dx > 0 && dy < 0" << std::endl;
			if(dx < abs(dy)){
				//std::cout << " abs dy > dx" << std::endl;
				--current_y;
				current_x = floor((current_y-m)/k);
				if(current_y == new_y && current_x != new_x){
					current_x = new_x;
				} 
				createCone();
			}
			else if(abs(dy) < dx){
				// std::cout << " abs dy < dx" << std::endl;
				++current_x;
				current_y = floor((k*current_x + m));
				if(current_x == new_x && current_y != new_y){
					current_y = new_y;
				} 
				createCone(); 
			}
			else{
				// std::cout << " abs dy == dx" << std::endl;
				++current_x;
				--current_y;
				createCone();
			}
			// std::cout <<"made it without any problem" << std::endl;
		}
	}
	// std::cout << "move complete" << std::endl;	 
}




void ConeOfSight::createCone(){

	// will fill cones coordinates with 1s needs to be entire map
	cone_matrix = std::vector<std::vector<int>>(largest_x,std::vector<int>(largest_y,0));
	//cone length
	//std::cout << "creating a cone" << std::endl;
	int x =  80;
	int y =  0;
	// ROS_INFO("x = %d", x);
	// ROS_INFO("y = %d", y);

	//Calculating all the end points of the cone
	double right_angle = current_direction - cone_angle;
	double left_angle  = current_direction + cone_angle;
	//ROS_INFO("right_angle = %f", right_angle);
	//ROS_INFO("left_angle = %f", left_angle);

	//rotating accordingly with starting pos

	int right_x  = current_x  + floor(x*std::cos(right_angle) - y*std::sin(right_angle));
	int right_y  = current_y  + floor(x*std::sin(right_angle) + y*std::cos(right_angle));
	// ROS_INFO("right_x = %d  right_y = %d", right_x, right_y);

	int left_x   = current_x + floor(x*std::cos(left_angle) + y*std::sin(left_angle));
	int left_y   = current_y + floor(x*std::sin(left_angle) - y*std::cos(left_angle));
	// ROS_INFO("left_x = %d  left_y = %d", left_x, left_y);
	
	int x_min = determineSmallest(current_x, right_x, left_x);
    int y_min = determineSmallest(current_y, right_y, left_y);
    int x_max = determineLargest(current_x, right_x, left_x);
    int y_max = determineLargest(current_y, right_y, left_y);

   
    if(x_min < 0){
    	x_min = 0;
    }
    if(y_min < 0){
    	y_min = 0;
    }
    if(y_max > largest_y){
    	y_max = largest_y;
    }
    if(x_max > largest_x){
    	x_max = largest_x;
    }
    // std::cout << "largest_x = "  << largest_x << " xmax = " << x_max << std::endl;
    // std::cout << "largest_y = "  << largest_y << " ymax = " << y_max << std::endl;
    // std::cout << "xmin = " << x_min << std::endl;
    // std::cout << "ymin = " << y_min << std::endl;

    double A = triangleArea(current_x, current_y, right_x, right_y, left_x ,left_y); 
    double A1; double A2; double A3;
    double acceptable_error = 1; 
    //basically checking if particles in a square twice the size the size of 
    //
    
	for(int temp_x = x_min; temp_x < x_max; ++temp_x){
	 	// std::cout << "x = " << temp_x << std::endl;
		for(int temp_y= y_min; temp_y < y_max; ++temp_y){
		 	// std::cout << "y = " << temp_y << std::endl;
    		A1 = triangleArea(temp_x, temp_y, right_x, right_y, left_x, left_y);
    		A2 = triangleArea(current_x, current_y, temp_x, temp_y, left_x, left_y);
    		A3 = triangleArea(current_x, current_y, right_x, right_y, temp_x, temp_y);
    		//checking if coordinate is within triangle

    		// double test = std::fabs(A-(A1 + A2 + A3));
    		// std::cout << " 0.1 >= " << std::fabs(A-(A1 + A2 + A3)) << std::endl;
    		// std::cout << " 0.1 >= (test) " << test << std::endl;
    		if(std::fabs(A-(A1 + A2 + A3)) <= acceptable_error){
    			//if not explored, say it is explored
    			if(!(explored[temp_x][temp_y] == 1)){
    				explored[temp_x][temp_y] = 1;
    			}

    			// ROS_INFO("temp_x = %d, temp_y = %d", temp_x, temp_y);
    			// std::cout << temp_x << " " << temp_y<< std::endl;
    			cone_matrix[temp_x][temp_y] = 1; // we are seeing this 
    		}
    	} 
	}
	//std::cout << "cone created" << std::endl;
}

const Position ConeOfSight::getPosition() const{
	Position the_pos;
	the_pos.x = current_x;
	the_pos.y = current_y;
	return the_pos;
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

