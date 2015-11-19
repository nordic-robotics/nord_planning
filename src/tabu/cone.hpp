#pragma once
#include "map.hpp"
#include "ros/ros.h"
#include "ros/package.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include "map.hpp"
#include "geometry_msgs/Pose2D.h"
#include "line.hpp"
#include "point.hpp"
    


struct Position{
    public:
        int x;
        int y;
        Position(){
            x = -1;
            y = -1;
        }
};

class ConeOfSight{
    public:
        
        
        int y_n;
    	//initialise the cone
        ConeOfSight(map* maze, std::vector<std::vector<std::vector<Position>>> nodes, ros::NodeHandle* n);
	    // ConeOfSight();

	    //needed to descretize 30 deg angle
	    //tan(30) = x/y = 0.57735026919 to find a good dexcritization

	    void changeWidth(float new_width);

        void createCone();

        void rotateCone(int new_x, int new_y);

        void moveCone(int new_x, int new_y);

        // void give_necessary_info(int x, int y);

        double triangleArea(int x1, int y1, int x2, int y2, int x3, int y3);


        const std::vector<std::vector<int>>& getCone() const{ return cone_matrix;};
        const std::vector<std::vector<float>>& getExplored() const{ return explored;};
        const Position& getCurrentPosition() const;

        int getMaxX() const { return largest_x;};
        int getMaxY() const { return largest_y;}


        int determineSmallest(int n1, int n2, int n3);
        int determineLargest(int n1, int n2, int n3);

    //important to note 0 rad means it travels along the x_axis
    // All positions will be in cm not meters;
    protected:
    	double cone_angle;
    	int current_x; int current_y; double current_direction;
    	int x_next; int y_next;
        int largest_x; int largest_y;
        std::vector<std::vector<int>> cone_matrix;
        std::vector<std::vector<float>> explored;
        int corners_found = 0; int wall_found = 0;
        double time_moving; double time_rotating; 
        int start_x; int start_y;
        std::vector<std::vector<std::vector<Position>>> node_links;
        ros::NodeHandle* n;
        map *maze;


};