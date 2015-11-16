#pragma once
#include "line.hpp"
#include "point.hpp"
#include "map.hpp"
#include "ros/ros.h"
#include "ros/package.h"
#include <math.h>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>


class ConeOfSight{
    public:
        
        
         int y_n;
    	//initialise the cone
	    ConeOfSight(const map& maze);

	    //needed to descretize 30 deg angle
	    //tan(30) = x/y = 0.57735026919 to find a good dexcritization

	    void changeWidth(float new_width);

        void createCone();

        void rotateCone(double rotation);

        void moveCone(int new_x, int new_y);

        double triangleArea(int x1, int y1, int x2, int y2, int x3, int y3);

        const std::vector<std::vector<int>>& getCone() const{ return cone_matrix;};
        const std::vector<std::vector<float>>& getExplored() const{ return explored;};;
        int getMaxX() const { return x_max;};
        int getMaxY() const { return y_max;}

        int determineSmallest(int n1, int n2, int n3);
        int determineLargest(int n1, int n2, int n3);

    //important to note 0 rad means it travels along the x_axis
    // All positions will be in cm not meters;
    private:
    	double cone_angle = 30; 
    	int current_x= 79; int current_y = 23; double start_direction = 0;
    	int x_next; int y_next;
        int x_max; int y_max;
        std::vector<std::vector<int>> cone_matrix;
        std::vector<std::vector<float>> explored;
        int corners_found = 0; int wall_found = 0;
        bool first_time = true; 


};