#include "cone.hpp"
#include "visualization_msgs/Marker.h"
#include "tabu_search.hpp"
// #include "path_helper.hpp"



class InitialPath:public ConeOfSight{
   
    public:
        
        ros::Publisher cone_pub     = n->advertise<visualization_msgs::Marker>("/nord/cone_of_sight",1);
        ros::Publisher explored_pub = n->advertise<visualization_msgs::Marker>("/nord/explored",1);
        ros::Publisher map_pub      = n->advertise<visualization_msgs::Marker>("/nord/map", 1);
        ros::Publisher node_pub     = n->advertise<visualization_msgs::Marker>("/nord/nodes", 1);

        visualization_msgs::Marker explored_msg;
        visualization_msgs::Marker cone_msg; 
        visualization_msgs::Marker map_msg;
        visualization_msgs::Marker node_msg; 

        using ConeOfSight::ConeOfSight;


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  ------------------------------------------ RANDOM -----------------------------------------------------
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<Position> random(){

        end_position.x = start_x;
        end_position.y = start_y;

        ros::Time time = ros::Time::now();
        ros::Duration d = ros::Duration(1,0);

        //first pick a random node
        std::vector<Position> visited; 
        int connections = node_links[start_x][start_y].size();
        int random_nr = std::rand()%connections;
        Position new_node = node_links[start_x][start_y][random_nr];
        Position temp_node;
        rotateCone(new_node.x,new_node.y);
        moveCone(new_node.x,new_node.y);
        visited.push_back(new_node);

        //then continue to pick a random node from there on until you find your way back
        int i = 0;
        bool done = false;
        found_back = false;
        while(done != true){
            std::cout << "i = " << i << std::endl;
            connections = node_links[new_node.x][new_node.y].size();
            random_nr   = std::rand()%connections;

            temp_node   = node_links[start_x][start_y][random_nr];
            new_node    = temp_node;

            rotateCone(new_node.x,new_node.y);
            publish_all();
            d.sleep();
            moveCone(new_node.x,new_node.y);
            d.sleep();

            i++;
            if(new_node.x == end_position.x && new_node.y == end_position.y){
                done = true; found_back = false;
                std::cout << "returned to start" << std::endl;
            }
            else if(i > 35){
                done = true; found_back = false;
                std::cout << "returned to start" << std::endl;
            }  
        }
        
        return visited;
    } 
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //---------------------------------------- Neighbours ----------------------------------------------------
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////   
    
    // std::vector<Position> neighbours(std::vector<Position> visited){
    //     std::vector<Position> alternative_route;


    
    // }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //---------------------------------------- fitness ----------------------------------------------------
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////// 

   float fitness(std::vector<Position> visited)
    {
        //should i restart it, maybe try to find a different solution
        float score;
        if(found_back != true){
            score = -50000;
        } 

        start_pos
                   
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //---------------------------------------- Publish all ----------------------------------------------------
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////   
    void publish_all(){
            explored_msg = explored_message(explored, largest_x, largest_y);
            //std::cout << "1" << std::endl;
            cone_msg = cone_of_sight_message(cone_matrix, largest_x, largest_y);
            //std::cout << "2" << std::endl;
            //map_msg = create_map_message(*maze);
            //std::cout << "3" << std::endl;
            node_msg = node_message(node_links, largest_x,largest_y);


            explored_pub.publish(explored_msg);
            cone_pub.publish(cone_msg);
            //map_pub.publish(map_msg);
            node_pub.publish(node_msg);
    }        

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // -------------------------------------  PUBLISHERS -----------------------------------------------------
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////    

    visualization_msgs::Marker explored_message(const std::vector<std::vector<float>>& explored_matrix,
        const int& x_max, const int& y_max){

        visualization_msgs::Marker explored;
        explored.id= 4;
        explored.header.frame_id = "/map";
        explored.ns="explore";
        explored.header.stamp = ros::Time::now();
        explored.type = visualization_msgs::Marker::CUBE_LIST;   
        explored.pose.orientation.w = 1.0f;
        explored.scale.x = 0.01f;
        explored.scale.y = 0.01f;
        explored.scale.z = 0.01f;
        explored.color.r = 1.0f;
        explored.color.g = 0;
        explored.color.b = 1.0f;
        explored.color.a = 1.0f;
        explored.action = visualization_msgs::Marker::ADD;
        for (int i = 0; i < x_max; ++i)
        {
            for(int j = 0; j < y_max; ++j){
                // ROS_INFO("cone_matrix[i][j] = %d",cone_matrix[i][j]);
                if(explored_matrix[i][j] == 1){
                    geometry_msgs::Point pos;
                    pos.x = i/100.0;
                    pos.y = j/100.0;
                    pos.z = 0.0;
                    //ROS_INFO("pos.x = %f pos.y = %f",pos.x*0.01,pos.y*0.01);
                    explored.points.push_back(pos);
                }
            }
        } 
        return  explored;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    visualization_msgs::Marker cone_of_sight_message(const std::vector<std::vector<int>>& cone_matrix,
        const int x_max, const int y_max){

        visualization_msgs::Marker cone;
        cone.header.frame_id ="cone_of_sight";
        cone.id=2;
        cone.header.frame_id = "/map";
        cone.ns="cone";
        cone.header.stamp = ros::Time::now();
        cone.type = visualization_msgs::Marker::CUBE_LIST;   
        cone.pose.orientation.w = 1.0f;
        cone.scale.x = 0.01f;
        cone.scale.y = 0.01f;
        cone.scale.z = 0.01f;
        cone.color.r = 1.0f;
        cone.color.g = 1.0f;
        cone.color.b = 0;
        cone.color.a = 1.0f;
        cone.action = visualization_msgs::Marker::ADD;
        // ROS_INFO("seeing.getMaxX() = %d",seeing.getMaxX());
        // ROS_INFO("seeing.getMaxY() = %d",seeing.getMaxY());
        for (int i = 0; i < x_max; ++i)
        {
            geometry_msgs::Point pos;
            for(int j = 0; j < y_max; ++j){
                // ROS_INFO("cone_matrix[i][j] = %d",cone_matrix[i][j]);
                if(cone_matrix[i][j] == 1){
                    
                    pos.x = i/100.0;
                    pos.y = j/100.0;
                    pos.z = 0.01f;
                    //ROS_INFO("pos.x = %f pos.y = %f",pos.x*0.01,pos.y*0.01);
                    cone.points.push_back(pos);
                }
            }
        }

        return cone;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    visualization_msgs::Marker node_message(const std::vector<std::vector<std::vector<Position>>>& node_links,
        const int& x_max, const int& y_max){
        std::cout <<"node message" << std::endl;
        visualization_msgs::Marker node_list;
        node_list.id = 3;
        node_list.header.frame_id = "/map";
        node_list.ns="node";
        node_list.header.stamp = ros::Time::now();
        node_list.type = visualization_msgs::Marker::SPHERE_LIST;
        node_list.pose.orientation.w = 1.0f;
        node_list.scale.x = 0.01f;
        node_list.scale.y = 0.01f;
        node_list.scale.z = 0.01f;
        node_list.color.a = 1.0;
        node_list.color.r = 0;
        node_list.color.g = 0;
        node_list.color.b = 1.0;
        node_list.action = visualization_msgs::Marker::ADD;
        for(int x= 0; x < x_max-1; x++){
            for(int y = 0; y < y_max-1; y++){
                //std::cout << "node_links[x][y][0].x = " << node_links[x][y][0].x << std::endl; 
                if(node_links[x][y][0].x != -1){
                    //std::cout <<"I AM IN!!!!" << std::endl;
                    geometry_msgs::Point pos;
                    pos.x = x/100.0;
                    pos.y = y/100.0;
                    pos.z = 0.02f;
                    node_list.points.push_back(pos);
                }
            }
        }
        std::cout << "klarade det" << std::endl;
        return node_list;
    }

   
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    visualization_msgs::Marker create_map_message(const map& maze)
        {
        visualization_msgs::Marker line_list;
        line_list.id = 1;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.color.a = line_list.color.r = line_list.color.g = line_list.color.b = 1.0;
        line_list.header.frame_id = "/map";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "ip_map";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.lifetime = ros::Duration();
        line_list.scale.x = 0.01;

        for (auto& wall : maze.get_walls())
        {
            geometry_msgs::Point p0, p1;
            p0.x = wall.start.x();
            p0.y = wall.start.y();
            p1.x = wall.end.x();
            p1.y = wall.end.y();
            p0.z = p1.z = 0.02f;
            line_list.points.push_back(p0);
            line_list.points.push_back(p1);
        }

        return line_list;
    }




    private:
        Position end_position;
        bool found_back;

};


//###########!!!!!!!!!!!!!!!###############!!!!!!!!!!!!#############!!!!!!!!###########!!!!!!!!!!###########!!!!!!!!!!###########
//###########!!!!!!!!!!!!!!!###############!!!!!!!!!!!!##############!!!!!!!!!!!!##########!!!!!!!!##########!!!!!!!!#########!!!!



map read_map(std::string filename)
{
    std::ifstream file(filename);
    std::string l;
    std::vector<line<2>> walls;
    float min_x = 0, min_y = 0, max_x = 0, max_y = 0;
    while (std::getline(file, l))
    {
        std::istringstream iss(l);
        if (l[0] == '#')
            continue;

        float x0, y0, x1, y1;
        iss >> x0 >> y0 >> x1 >> y1;
        min_x = std::min({min_x, x0, x1});
        min_y = std::min({min_y, y0, y1});
        max_x = std::max({max_x, x0, x1});
        max_y = std::max({max_y, y0, y1});
        walls.push_back(line<2>(point<2>(x0, y0), point<2>(x1, y1)));
        
    }
    return map(walls, min_x, min_y, max_x, max_y);
}

// ska jag spara node connections i std::vector<std::vector<std::vector<int>>> för positionen x å y 
std::vector<std::vector<std::vector<Position>>> read_nodes(std::string filename, float x_m, float y_m){
    //std::cout << filename << std::endl;
    std::ifstream file(filename);
    std::string l;
    int x_max = x_m*100;
    int y_max = y_m*100;
    // std::cout << "x max = " << x_max << "y max = " << y_max << std::endl;
    std::vector<std::vector<std::vector<Position>>> node_connections 
    = std::vector<std::vector<std::vector<Position>>>(x_max, std::vector<std::vector<Position>>(y_max, std::vector<Position>(1)));

    float node_x = 0;
    float node_y = 0;
    float con_x, con_y;
    int x =0;
    int y=0;
    char comma;
    bool new_node = false;
    // ROS_INFO("Reading from file");

    while(std::getline(file,l)){
        std::stringstream line(l);
        if(l[0] == '%'){
            new_node = true;
        }
        else if(new_node == true){
            line >> node_x >> comma >> node_y;
            x = node_x *100;
            y = node_y *100;
            //std::cout << x << " " << y << std::endl;
            new_node = false;
            node_connections[x][y].clear();
        }
        else{
            
            line >> con_x >> comma >> con_y;
            Position new_con;
            new_con.x = con_x *100;
            new_con.y = con_y *100;
            // std::cout << x << " " << y << std::endl; 
            node_connections[x][y].push_back(new_con);
            // std::cout << "\t" << new_con.x << " " << new_con.y << std::endl;
        }
    }
 return node_connections;
}







int main(int argc, char** argv)
{

	ros::init(argc, argv, "initial_path");
	ros::NodeHandle n;
	map maze = read_map(ros::package::getPath("nord_planning") + "/data/small_maze.txt");
    auto node_vector = read_nodes(ros::package::getPath("nord_planning") + "/links.txt", maze.get_max_x(), maze.get_max_y());
    InitialPath sight(&maze, node_vector, &n);
 
    sight.createCone();
  
    auto visited = sight.random();   

    

    return 0;       
};