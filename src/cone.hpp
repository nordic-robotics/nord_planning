#pragma once

#include "ros/ros.h"
#include "ros/package.h"
#include "visualization_msgs/Marker.h"
#include "tabu_search.hpp"
#include "map.hpp"
#include "geometry_msgs/Pose2D.h"
#include <math.h>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>


class ConeOfSight{
    public:

    	//initialise the cone
	    ConeOfSight(std::vector<dijkstra::point> graph);

	    //needed to descretize 30 deg angle
	    //tan(30) = x/y = 0.57735026919 to find a good dexcritization

	    void changeWidth(float new_width);

        std::vector<std::vector<double>> createCone();
	    void storeExplored(std::vector<std::std::vector<double>> position);

        void rotateCone();

        void moveCone();

        int determineSmallets(int n1, int n2, int n3);
        int determineLargest(int n1, int n2, int n3);

    //important to note 0 rad means it travels along the x_axis
    // All positions will be in cm not meters;
    private:
    	double cone_angle;
    	int x_current; int y_current; double start_direction;
    	int x_next; int y_next; double next_direction;
        int corners_found; int wall_found;

};