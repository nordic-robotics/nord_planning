#pragma once
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
#include <cstdlib>
#include <time.h>  
#include "../dijkstra/path.hpp"      
#include "../dijkstra/map.hpp" 
#include "../dijkstra/point.hpp"

struct Position{
    public:
        int x;
        int y;
        Position(){
            x = -1;
            y = -1;
        }
        Position (int x_start, int y_start){
            x = x_start;
            y = y_start;
        }
        bool operator==(const Position& other)const{
            return(x == other.x && y == other.y);
        }
        
};

class ConeOfSight{
    public:
        
        bool point_cap;
        ConeOfSight(map* maze,int start_x, int start_y);
        ConeOfSight(ConeOfSight const& cone);

        void changeWidth(float new_width);
        void createCone();
        void rotateCone(int new_x, int new_y);
        void moveCone(int new_x, int new_y);
        void resetExplored();
        void add_to_path(Position pos);

        double triangleArea(int x1, int y1, int x2, int y2, int x3, int y3);
        const double get_time()const{return(time_rotating+time_moving);};  

        const std::vector<std::vector<int>>& getCone() const{ return cone_matrix;};
        const std::vector<std::vector<float>>& getExplored() const{ return explored;};
        const Position getPosition() const;
        const std::vector<Position>& get_path()const{return path;};
        // const double& get_time()const {return total_time;};
        const int& get_x_max()const{return largest_x;};
        const int& get_y_max()const{return largest_y;};
        // const Position& get_end_pos()const{return end_position}



        bool operator==(const ConeOfSight& other)const{return path == other.get_path();}


        int determineSmallest(int n1, int n2, int n3);
        int determineLargest(int n1, int n2, int n3);

    //important to note 0 rad means it travels along the x_axis
    // All positions will be in cm not meters;
    protected:
        double cone_angle;
        int current_x; int current_y; double current_direction;
        int x_next; int y_next;
        int largest_x; int largest_y;
        std::vector<Position> path;
        std::vector<std::vector<int>> cone_matrix;
        std::vector<std::vector<float>> explored;
        int corners_found = 0; int wall_found = 0;
        double time_moving; double time_rotating;
        int start_x; int start_y;
        ros::NodeHandle* n;
        map *maze;


};