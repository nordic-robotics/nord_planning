#include "cone.hpp"
#include "visualization_msgs/Marker.h"
#include "tabu_search.hpp"




class InitialPath{
   
    public:
        ros::NodeHandle n;
        ros::Publisher cone_pub     = n.advertise<visualization_msgs::Marker>("/nord/cone_of_sight",1);
        ros::Publisher explored_pub = n.advertise<visualization_msgs::Marker>("/nord/explored",1);
        ros::Publisher map_pub      = n.advertise<visualization_msgs::Marker>("/nord/map", 1);
        ros::Publisher node_pub     = n.advertise<visualization_msgs::Marker>("/nord/nodes", 1);
        ros::Publisher robo_pub     = n.advertise<visualization_msgs::Marker>("/nord/robot", 1);

        visualization_msgs::Marker explored_msg;
        visualization_msgs::Marker cone_msg; 
        visualization_msgs::Marker map_msg;
        visualization_msgs::Marker node_msg; 

        InitialPath(std::vector<std::vector<std::vector<Position>>>  nodes, map* maze, int x, int y,
         dijkstra::map& dijkstra_search, std::valarray<bool> walls, const std::valarray<bool>& node_exists)
            : walls(walls), node_exists(node_exists) {
            node_links = nodes;
            this->maze = maze;
            start_x = x;
            start_y = y;
        }



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //---------------------------------------- Publish all ----------------------------------------------------
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////   
    void publish_all(const ConeOfSight& node, dijkstra::map& dijkstra_search){
            explored_msg = explored_message(node.getExplored(), node.get_x_max(), node.get_y_max());
            //std::cout << "1" << std::endl;
            cone_msg = cone_of_sight_message(node.getCone(), node.get_x_max(), node.get_y_max());
            //std::cout << "2" << std::endl;
            map_msg = create_map_message(*maze);
            //std::cout << "3" << std::endl;
            node_msg = node_message(node_links, node.get_x_max(),node.get_y_max());
            auto conn_msg = create_conn_message(dijkstra_search);
            auto robo_msg = get_robo_point(node);

            explored_pub.publish(explored_msg);
            cone_pub.publish(cone_msg);
            map_pub.publish(map_msg);
            node_pub.publish(node_msg);
            node_pub.publish(conn_msg);
            robo_pub.publish(robo_msg);
            // std::cout << "leaving publish" << std::endl;
    }        

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // -------------------------------------  PUBLISHERS -----------------------------------------------------
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////    

    visualization_msgs::Marker explored_message(const std::valarray<bool>& explored_matrix,
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
        explored.color.r = 0.4f;
        explored.color.g = 0.7;
        explored.color.b = 1.0f;
        explored.color.a = 0.5f;
        explored.action = visualization_msgs::Marker::ADD;
        for (int i = 0; i < x_max; ++i)
        {
            for(int j = 0; j < y_max; ++j){
                // ROS_INFO("cone_matrix[i][j] = %d",cone_matrix[i][j]);
                if(explored_matrix[i + j * x_max]){
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

    visualization_msgs::Marker cone_of_sight_message(const std::valarray<bool>& cone_matrix,
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
        cone.color.b = 1.0;
        cone.color.a = 0.7f;
        cone.action = visualization_msgs::Marker::ADD;
        // ROS_INFO("seeing.getMaxX() = %d",seeing.getMaxX());
        // ROS_INFO("seeing.getMaxY() = %d",seeing.getMaxY());
        for (int i = 0; i < x_max; ++i)
        {
            geometry_msgs::Point pos;
            for(int j = 0; j < y_max; ++j){
                // ROS_INFO("cone_matrix[i][j] = %d",cone_matrix[i][j]);
                if(cone_matrix[i + j * x_max]){
                    
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
        for(int x= 0; x < x_max-1; ++x){
            for(int y = 0; y < y_max-1; ++y){
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
        return node_list;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------- simulation ----------------------------------------------------
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

    // void run_simulation(std::string filename, std::valarray<bool> walls,  map* maze, dijkstra::map& dijkstra_search){
            
    //         std::cout << "Reading from file" << std::endl;
    //         std::ifstream file(filename);
    //         std::string l;
    //         float x_temp = 0; float y_temp = 0;
    //         int x= 0;int y= 0;
    //         std::vector<Position> path;

    //         while (std::getline(file, l))
    //         {
    //             std::stringstream line(l);
    //             line >> x_temp >> y_temp;
    //             x = x_temp *100; y = y_temp *100;
    //             Position part_of_path(x,y);
    //             path.push_back(part_of_path);

    //         }

    //         std::cout << "starting simulaiton in 2 seconds" << std::endl;
    //         ros::Time time = ros::Time::now();
    //         ros::Duration d = ros::Duration(1); 

    //         Position start = path[0];
    //         ConeOfSight node(maze, start.x, start.y, walls, node_exists);
    //         d.sleep();
    //         d.sleep();
    //         auto temp_cone(node);
    //         node.move_ok =true;
    //         for(unsigned int i = 0; i < path.size(); ++i){
    //             d.sleep();
    //             node.move_ok = true;
    //             temp_cone = node;

    //             //testing which rotation explores most
    //             node.rotateCone(path[i].x,path[i].y);
    //             temp_cone.rotateConeOtherWay(path[i].x,path[i].y);
    //             if(temp_cone.getNumExplored() > (node.getNumExplored()+150)){
    //                 node = temp_cone;
    //             }

    //             std::cout << "x = " << path[i].x << "y = " << path[i].y << std::endl;
    //             publish_all(node, dijkstra_search);
    //             d.sleep();
    //             if(node.getExplored()[i+(j*x_max)]);
    //                 node.move_ok = false;
    //             node.moveCone(path[i].x,path[i].y);
    //             publish_all(node, dijkstra_search);
    //             node.add_to_path(path[i]);
                
    //             //std::cout << "i = " << i  << std::endl;
    //         } 
    //         std::cout<< "simulation complete" << std::endl;   
    // }

   
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

    visualization_msgs::Marker create_conn_message(dijkstra::map& maze)
        {
        visualization_msgs::Marker line_list;
        line_list.id = 1;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.color.a = 0.1;
        line_list.color.g = 1.0;
        line_list.color.r = line_list.color.b = 0.6;
        line_list.header.frame_id = "/map";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "ip_conns";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.lifetime = ros::Duration();
        line_list.scale.x = 0.01;

        for (auto& p : maze.get_graph())
        {
            for (auto link : p.get_links())
            {
                geometry_msgs::Point p0, p1;
                p0.x = p.x;
                p0.y = p.y;
                p1.x = link->x;
                p1.y = link->y;
                p0.z = p1.z = 0.02f;
                line_list.points.push_back(p0);
                line_list.points.push_back(p1);
            }
        }

        return line_list;
    }

    visualization_msgs::Marker get_robo_point(const ConeOfSight& cone)
        {
        visualization_msgs::Marker robo_point;
        robo_point.id = 6;
        robo_point.type = visualization_msgs::Marker::SPHERE;
        robo_point.color.a = 1;
        robo_point.color.g = 0;
        robo_point.color.r = robo_point.color.b = 1;
        robo_point.header.frame_id = "/map";
        robo_point.header.stamp = ros::Time::now();
        robo_point.ns = "robot";
        robo_point.action = visualization_msgs::Marker::ADD;
        robo_point.pose.orientation.w = 1.0;
        robo_point.lifetime = ros::Duration();
        robo_point.scale.x = 0.1f;
        robo_point.scale.y = 0.1f;
        robo_point.scale.z = 0.01f;

        
        robo_point.pose.position.x = cone.getPosition().x/100.0;
        robo_point.pose.position.y = cone.getPosition().y/100.0;
        robo_point.pose.position.z = 0.03f;
        

        return robo_point;
    }

    private:
    map *maze;
    std::valarray<bool> walls;
    std::valarray<bool> node_exists;
    std::vector<std::vector<std::vector<Position>>> node_links;
    int start_x;
    int start_y;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------- itterate through path -----------------------------------------
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

ConeOfSight path_itteration(const std::vector<Position>& path, map* maze, Position start,
                            const std::valarray<bool> walls, std::valarray<bool> node_exists){
    ConeOfSight node(maze, start.x, start.y, walls, node_exists);
    node.resetExplored();
    for(auto& p : path){
        node.rotateCone(p.x, p.y);
        node.moveCone(p.x, p.y);
        node.add_to_path(p);
    }
    return node;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------- Neighbours ----------------------------------------------------
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////   

// std::vector<ConeOfSight> neighbours(const ConeOfSight& nodes,
//         const std::vector<std::vector<std::vector<Position>>>& node_links,
//         map* maze, const std::valarray<bool>& walls, const Position& start, int depth = 5){

//     std::vector<ConeOfSight> the_neighbours;
//     //std::cout << "depth: " << depth << std::endl;
//     if (depth <= 1 || !ros::ok())
//     {
//         the_neighbours.push_back(nodes);
//         return the_neighbours;
//     }

//     for (auto& p : node_links[nodes.get_path().back().x][nodes.get_path().back().y])
//     {
//         //if (i++ > 10 * depth)
//         //    break;
//         ConeOfSight copy = nodes;
//         copy.rotateCone(p.x, p.y);
//         copy.moveCone(p.x, p.y);
//         copy.add_to_path(p);
//         auto deep_neighbours = neighbours(copy, node_links, maze, walls, start, depth - 1);
//         std::move(deep_neighbours.begin(), deep_neighbours.end(), std::back_inserter(the_neighbours));
//     }
//     if (depth == 5)
//         std::cout << "num neighbours at max depth: " << the_neighbours.size() << std::endl;
//     return the_neighbours;

//     int movments =  nodes.get_path().size();
//     int random_value = std::rand()% movments; 
//     Position random_part = nodes.get_path()[random_value];
//     std::vector<Position> new_path;
//     std::vector<Position> temp_vec1;


//     Position next_next_node; 
//     Position next_node; 

//     unsigned int path_size = nodes.get_path().size()-1;
//     int last_x = nodes.get_path()[path_size].x;
//     int last_y = nodes.get_path()[path_size].y;
//     // std::cout << "path_size = " << path_size << std::endl;
//     // for(unsigned int k = 0; k < path_size; ++k){
//     //     std::cout << nodes.get_path()[k].x << ", " << nodes.get_path()[k].y << std::endl;
//     // }
//     //adds all the links to the last node
//     //here is a chance to save alot of time since i pushvka last path. just walk to next node

//     Position new_position;
//     for(unsigned int k = 0; k < node_links[last_x][last_y].size(); ++k){
//         ConeOfSight alternative_route = nodes;
//         new_position = node_links[last_x][last_y][k];
//         if (new_position.x == last_x && new_position.y == last_y)
//         {
//             std::cout << "fitness is fucked yo" << std::endl;
//             exit(1);
//         }
//         alternative_route.rotateCone(new_position.x, new_position.y);
//         alternative_route.moveCone(new_position.x,new_position.y);
//         alternative_route.add_to_path(new_position);
//         the_neighbours.push_back(alternative_route);
//         // std::cout << "läggs till ="  << node_links[last_x][last_y][k].x << ", " << node_links[last_x][last_y][k].y << std::endl;
//     }
//     //std::cout << "got past first for loop" << std::endl;

//     // tries to minimize the path 
//     // std::cout << "random_value = " << random_value << std::endl;   
//     next_node = nodes.get_path()[random_value+1];
//     //std::cout << "next_node = " << next_node.x << ", " << next_node.y << std::endl;  
//     next_next_node = nodes.get_path()[random_value+2];
//     temp_vec1 = node_links[random_part.x][random_part.y];

//     for(unsigned int i = 0; i < temp_vec1.size(); ++i){
//         //std::cout << "i =" << i << std::endl;
//         new_path = nodes.get_path();
//         if(temp_vec1[i] == next_next_node){ 
//             // std::cout << "FOUND a coool granne" << std::endl;
//             new_path.erase(new_path.begin() + (random_value +1));
//             the_neighbours.push_back(path_itteration(new_path, maze, start, walls));
//         }
//         // else{
//         //     if(node_links[(temp_vec1[i].x)][(temp_vec1[i].y)] == next_next_node){
//         //         new_path.erase(new_path.begin+)
//         //     }
//         // }
//     }   
//     std::cout << "num: " << the_neighbours.size() << std::endl;
//     return the_neighbours;
// }


ConeOfSight deep_greedy_search(const ConeOfSight& nodes,
        const std::vector<std::vector<std::vector<Position>>>& node_links,
        map* maze, const std::valarray<bool>& walls, const Position& start, int depth,
         int& best_count){

    //std::cout << "depth: " << depth << std::endl;
    if (depth <= 1 || !ros::ok())
    {
        return nodes;
    }

    // std::cout << "x_max, ymax = " << x_max << "," << y_max << std::endl; 
    auto best = nodes;
    unsigned int challenger_count = 0;
    unsigned int best_score = 0;
    for (auto& p : node_links[nodes.get_path().back().x][nodes.get_path().back().y])
    {
        // std::cout << "am I in ? " << std::endl;
        ConeOfSight challenger = nodes;
        challenger.rotateCone(p.x, p.y);
        challenger.moveCone(p.x, p.y);
        challenger.add_to_path(p);
        challenger = deep_greedy_search(challenger, node_links, maze, walls, start, depth - 1, best_count);
        unsigned int challenger_count = challenger.getNumExplored();
        if (challenger_count > best_score)
        {
            best_score = challenger_count;   
            best = challenger;
        }
    }
    if (best_score > best_count){
        best_count = best_score;
        std::cout << "best_count = " << best_count << std::endl;
    }
    return best;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ------------------------------------------ RANDOM -----------------------------------------------------
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// ConeOfSight random(const Position& start, const std::vector<std::vector<std::vector<Position>>>& node_links,
//                    const std::valarray<bool>& walls, map* maze, std::valarray<bool> node_exists){
//     Position end_position = start;
//     int connections; 
//     int random_nr; 
//     Position pos;
//     Position new_pos;
//     int i = 0;
//     ConeOfSight node(maze, start.x, start.y, walls, node_exists);
//     bool ok =false;
//     std::cout << "the size  = " << node.get_path().size() << std::endl;
//     std::cout << node.get_path()[0].x << ", " << node.get_path()[0].y << std::endl;
//     return node;
//     //ros::Time time = ros::Time::now();
//     //ros::Duration d = ros::Duration(1.5,0);
//     while(ok == false){
        
//         pos = node.getPosition();
//         connections = node_links[pos.x][pos.y].size();
//         random_nr   = rand()%connections;
//         // std::cout << "random number = " << random_nr << std::endl;
//         new_pos   = node_links[pos.x][pos.y][random_nr];
//         if(!(new_pos == end_position)){


//             node.rotateCone(new_pos.x,new_pos.y);
        
//             node.moveCone(new_pos.x,new_pos.y);
//             node.add_to_path(new_pos);

           
//             //d.sleep();
//             ++i;

//             if(i >= 20){
//                 ok= true;
//                 // std::cout << "the size after = " << node.get_path().size() << std::endl;
//                 // for(unsigned int i = 0; i < node.get_path().size(); ++i){
//                 //     std::cout << node.get_path()[i].x << ", " << node.get_path()[i].y << std::endl;
//                 }
//             }
//         }   

//     // }
//     // std::cout << "OMG hittade ut" << std::endl;
//     return node;
// } 



///////////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------- fitness ----------------------------------------------------
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

    // float fitness(const ConeOfSight& the_path, const Position& start, dijkstra::map& dijkstra_search)
    // {
    //     //should i restart it, maybe try to find a different solution
    //     float score = 0;
    //     int max_time = 5 * 60;
    //     double our_time = the_path.get_time();
    //     unsigned int path_size = the_path.get_path().size()-1;
    //     Position curr_pos = the_path.get_path()[path_size];
    //     // std::cout << "curr pos = " << curr_pos.x << ", " << curr_pos.y << std::endl;
    //     float coolness = 0;
        
    //     //use the difference between explored instead, otherwise the idea behind
    //     // it is solid   
    //     if(curr_pos.x != start.x && curr_pos.y != start.y){
    //         dijkstra::point starting_node(start.x/100.0, start.y/100.0);
    //         dijkstra::point current_node(curr_pos.x/100.0, curr_pos.y/100.0);
    //         dijkstra::path shortest_path;
    //         shortest_path = dijkstra_search.find(current_node, starting_node);
    //         shortest_path.compute_length(); 
    //         coolness = shortest_path.length()*10;
    //     }
    //     else{
    //         score -=1;
    //     }
        
        

    //     for(int i = 0; i < the_path.get_x_max(); ++i){
    //         for(int j = 0; j < the_path.get_y_max();++j){
    //             if(the_path.getExplored()[i + j * the_path.get_x_max()]){
    //                 score += 1;
    //             }
    //         }
    //     }
    //     /*
    //     if(!(the_path.get_path()[path_size] == start)){
    //         score -= 5000;
    //     }

        
    //     // score -= shortest_path.length()*2;  
    //     if(our_time>max_time){
    //         score = -5000;
    //     }
    //     if(path_size <20){
    //         score -= 20000/path_size;
    //     }
    //     */
    //     // std::cout << "The score is = " << score << std::endl;
    //     // std::cout << "Our estimated time is = " << our_time << std::endl;
    //     return score;           
    // }


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

std::valarray<bool> read_walls(std::string filename, int rows, int columns){
    std::ifstream file(filename);
    std::string l;
    std::valarray<bool> walls((rows + 1) * (columns + 1));
    int row = 0;
    int value = 0;

    while(getline(file,l)){
        std::stringstream line(l);
        int col = 0;
        while(line >> value){
            // std::cout << "col = " << col << std::endl;  
            walls[col + row * columns] = value;
            col++;
        }
        // std::cout << "row = " << row << std::endl;
        row++;    
    }
    return walls;
}

// ska jag spara node connections i std::vector<std::vector<std::vector<int>>> för positionen x å y 
std::vector<std::vector<std::vector<Position>>> post_process_nodes(std::string filename, float x_m, float y_m){
    //std::cout << filename << std::endl;
    std::ifstream file(filename);
    std::string l;
    int x_max = x_m*100;
    int y_max = y_m*100;
    // std::cout << "x max = " << x_max << "  y max = " << y_max << std::endl;
    std::vector<std::vector<std::vector<Position>>> node_connections 
    = std::vector<std::vector<std::vector<Position>>>(x_max, std::vector<std::vector<Position>>(y_max, std::vector<Position>(1)));

    float node_x = 0; float node_y = 0;
    float con_x, con_y;
    int x =0; int y=0;
    char comma;
    bool new_node = false;
    // ROS_INFO("Reading from file");
    int i  = 1; int last_x = 0; int last_y = 0;
    uint countx = 0; uint county = 0; uint count = 0;
    std::vector<Position> temp;
    std::vector<Position> rejected;
    Position default_value;
    while(std::getline(file,l)){
        std::stringstream line(l);
      
        if(new_node == true){
            line >> node_x >> comma >> node_y;
            x = lround(node_x *100);
            y = lround(node_y *100);
            // std::cout << " In node_prep_con" << std::endl;
            // std::cout << x << " " << y << " " << x_max << y_max <<std::endl;
            new_node = false;
            node_connections[x][y].clear();
            last_x = x;
            last_y = y;
            countx = 0;county = 0; count = 0;

        }
        else{
            if(l[0] != '%'){
                line >> con_x >> comma >> con_y;
                Position new_con; 
                new_con.x = lround(con_x *100); new_con.y = lround(con_y *100);
                if(new_con.x == last_x){
                    countx++;
                }
                else if(new_con.y == last_y){
                    county++;
                }

                temp.push_back(new_con);
                count++;
            }    
            else{
                new_node = true;
                ++i;
                if((countx == count || county == count) && i>3 ){
                           
                    Position unwanted(x,y);
                    node_connections[x][y].push_back(default_value); 
                    rejected.push_back(unwanted);  
                    // std::cout << "removing " << x << ", " << y << std::endl;
                    // std::cout << "i = " << i << std::endl;
                    // std::cout << "default_value = " << default_value.x << ", " << default_value.y << std::endl;
                }
                else{
                    std::cout << "adding" << x << ", " << y << std::endl;
                    for(uint k = 0; k < temp.size(); ++k){
                        node_connections[x][y].push_back(temp[k]);
                    } 
                }


                temp.clear();      
            }
                
        }
    }

    //this is just a check of what i remove
    // for(int i = 0; i <rejected.size(); ++i){
    //     std::cout << "rejected[" << i << "] = " << rejected[i].x << ", " << rejected[i].y << std::endl;
    // }  
    
    // removing connections from nodes which has rejected nodes as children 
    for (int i = 0; i < x_max; ++i)
    {
        for (int j = 0; j < y_max; ++j)
        {   
            if(node_connections[i][j][0] == default_value)
                continue;


            for (uint k = 0; k < node_connections[i][j].size(); ++k)
            {
                for(uint l = 0; l < rejected.size(); ++l)
                {
                    if(node_connections[i][j][k] == rejected[l]){
                        std::cout << "removed" << std::endl;
                        node_connections[i][j].erase(std::remove(node_connections[i][j].begin(), node_connections[i][j].end(), 
                            rejected[l]), node_connections[i][j].end());
                    }
                }
            }
        }
    }


    //Printing shit to textfile
    std::ofstream file1(ros::package::getPath("nord_planning")+"/links2.txt");
   for (int i = 0; i < x_max; ++i)
    {
        for (int j = 0; j < y_max; ++j)
        {   
            if(node_connections[i][j][0] == default_value)
                continue;

            // std::cout << "x , y = " << i/100.0 << ", " << j/100.0 << std::endl;
            file1 << "%" << "\n";
            file1 << (i / 100.0) <<','<< (j/ 100.0);
            for (uint k = 0; k < node_connections[i][j].size(); ++k)
            {
                file1 <<"\n\t"<<(node_connections[i][j][k].x / 100.0)<<','<<(node_connections[i][j][k].y / 100.0);
            }
            file1 << "\n";
        } 
    }          
    return node_connections; 
}

void load_graph(std::string filename, dijkstra::map& graph)
{
    std::ifstream file(filename);
    
    std::string l;

    std::vector<dijkstra::point> nodes;

    // get all nodes without links
    while (std::getline(file, l))
    {
        if (l[0] != '%')
            continue;

        std::getline(file, l);
        std::istringstream iss(l);
        std::getline(iss, l, ',');
        float x = std::stod(l);
        std::getline(iss, l, ',');
        float y = std::stod(l);
        nodes.emplace_back(x, y);
    }
    graph = dijkstra::map(std::move(nodes));

    std::ifstream file2(filename);
    unsigned int parent = -1;
    // get all nodes without links
    while (std::getline(file2, l))
    {
        if(l.size()==0){
            std::cout << "shit is fucked yo!" << std::endl;
            exit(1);
        }
        if (l[0] == '%')
        {
            std::getline(file2, l);
            std::istringstream iss(l);
            std::getline(iss, l, ',');
            float x = std::stod(l);
            std::getline(iss, l, ',');
            float y = std::stod(l);
            // std::cout << "looking for " << x << " " << y << std::endl;

            for (size_t i = 0; i < graph.get_graph().size(); i++)
            { 
                if (graph.get_graph()[i].x == x && graph.get_graph()[i].y == y)
                {
                    parent = i;
                    break;
                }
            }
            std::getline(file2, l);
        }

        // std::cout << "line: " << l << std::endl;
        std::istringstream iss(l);
        std::getline(iss, l, ',');
        float x = std::stod(l);
        std::getline(iss, l, ',');
        float y = std::stod(l);

        unsigned int child = -1;
        for (size_t i = 0; i < graph.get_graph().size(); i++)
        {
            if (graph.get_graph()[i].x == x && graph.get_graph()[i].y == y)
            {
                child = i;
                break;
            }
        }

        graph.connect(parent, child);
    }

}






int main(int argc, char** argv)
{

    srand(time(NULL));
	ros::init(argc, argv, "initial_path");
    map maze = read_map(ros::package::getPath("nord_planning") + "/data/small_maze.txt");
    // std::vector<std::vector<std::vector<Position>>> node_vector = read_nodes(ros::package::getPath("nord_planning") + "/links.txt", maze.get_max_x(), maze.get_max_y());
    std::valarray<bool> walls = read_walls(ros::package::getPath("nord_planning") + "/Map.txt",maze.get_max_x()*100, maze.get_max_y()*100);
    dijkstra::map minimum_path;
    auto node_vector = post_process_nodes(ros::package::getPath("nord_planning") + "/Tobias_links.txt", maze.get_max_x(), maze.get_max_y());
    




    load_graph(ros::package::getPath("nord_planning") + "/Lucas_links.txt", minimum_path);
    
    

    std::cout << "starting the tabu search" << std::endl;
    int max_attempts = 35;
    std::valarray<bool> node_exists;
    node_exists = std::valarray<bool>(false, lround(maze.get_max_x()*100) * lround(maze.get_max_y()*100));
    
    Position default_value;   
    int x_max = lround(maze.get_max_x() * 100);
    int y_max = lround(maze.get_max_y() * 100);
    for (int x = 0; x < x_max; ++x)
        {
            for (int y = 0; y < y_max; ++y)
            {
                if(!(node_vector[x][y][0] == default_value)){
                    node_exists[x+y*x_max] = true;
                    std::cout << "node = " << x << "," << y << std::endl;
                }
            }
    }    
    Position start(20, 20); // find closest? not atm
    // std::cout << "testing the shit out of this = " << node_exists[0+244*x_max] << std::endl;
    InitialPath path(node_vector, &maze, start.x, start.y, minimum_path, walls, node_exists);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // THIS CAN BE COMMENTED OUT TO JUST RUN THE PLANNED PATH
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    int best_count = 0;
    std::cout << "x_max = " << x_max << ", ymax = " << y_max << std::endl; 
    auto total_area = x_max*y_max;
    ConeOfSight current(&maze, start.x, start.y, walls, node_exists);
    int attempts = 0;
    int temp = 0;
    auto temp_cone(current);
    ros::Time time = ros::Time::now();
    ros::Duration d = ros::Duration(1.5);
    while(attempts < max_attempts)
    {
        std::cout << "Attempt " << attempts << "/" << max_attempts << std::endl;
        for(int i = 0; i < x_max; ++i)
        {
            for(int j = 0; j < y_max;++j)
            {
                if(node_exists[(i+j*x_max)]){

                    if(!current.getExplored()[i+(j*x_max)]){
                        dijkstra::point go_to(i/100.0f, j/100.0f);
                        dijkstra::point starting_at(current.getPosition().x/100.0f, current.getPosition().y/100.0f);
                        dijkstra::path new_path = minimum_path.find(starting_at,go_to);
                        for(size_t k = 1; k < new_path.size(); ++k)
                        {   
                            temp_cone = current;
                            d.sleep();

                            //testing which rotation explores most
                            current.rotateCone(lround(new_path[k]->x*100),lround(new_path[k]->y*100));
                            std::cout << "ska till = " << lround(new_path[k]->x*100) << "," << lround(new_path[k]->y*100) << std::endl;
                            temp_cone.rotateConeOtherWay(lround(new_path[k]->x*100),lround(new_path[k]->y*100));
                            if(temp_cone.getNumExplored() > (current.getNumExplored()+150)){
                                current = temp_cone;
                            }


                            d.sleep();
                            path.publish_all(current, minimum_path);
                            if(current.getExplored()[i+(j*x_max)]){
                                current.move_ok = false;
                                goto quit_here;
                            }

                            std::cout << "move_ok = " <<  current.move_ok  << std::endl;    
                            
                            path.publish_all(current, minimum_path);    
                            current.moveCone(lround(new_path[k]->x*100),lround(new_path[k]->y*100));
                            std::cout << "returned from move cone" << std::endl;
                            Position pos(lround(new_path[k]->x*100), lround(new_path[k]->y*100));
                            current.add_to_path(pos);
                           
                        }

                    }
                }
            }       
        }
        quit_here:
        attempts++;
        current.move_ok=true;      
    }
    // for (size_t t = 0; t < max_attempts; t++)
    // {
    //     auto last_it(current);
    //     current = deep_greedy_search(current, node_vector,
    //                                  &maze, walls, start, 2, best_count);

    //     if((best_count - last_it.getNumExplored()) < 50000){
    //         std::cout << "HACKING THE SHIT OUT OF THIS" << std::endl;
    //         // for (int i = (x_max-1); i >= 0; --i)
    //         // {
    //         //     for(int j = (y_max-1); j >= 0; --j)
    //         //     {
    //         for(int i = 0; i < x_max; ++i)
    //         {
    //             for(int j = 0; j < y_max;++j)
    //             {    
    //                 if(node_exists[(i+j*x_max)])
    //                 {
    //                     std::cout  <<"node_exists[(i+j*x_max)] = " << node_exists[(i+j*x_max)]  << std::endl;
    //                     if(!last_it.getExplored()[i+(j*x_max)])
    //                     {
    //                         std::cout << "the bool is before" <<  last_it.getExplored()[i+j*x_max] << std::endl;
    //                         int temp= 0;

    //                         current = last_it;
    //                         std::cout << "unexplored node is = " << i << "," << j << std::endl;
    //                         dijkstra::point go_to(i/100.0f, j/100.0f);
    //                         dijkstra::point starting_at(last_it.getPosition().x/100.0f, last_it.getPosition().y/100.0f);
    //                         dijkstra::path new_path = minimum_path.find(starting_at,go_to);
    //                         for(size_t k = 1; k < new_path.size(); ++k)
    //                         {
    //                             current.rotateCone(lround(new_path[k]->x*100),lround(new_path[k]->y*100));
    //                             current.moveCone(lround(new_path[k]->x*100),lround(new_path[k]->y*100));
    //                             Position pos(lround(new_path[k]->x*100), lround(new_path[k]->y*100));
    //                             current.add_to_path(pos);
    //                             if(current.getExplored()[i+(j*x_max)]){
    //                                 goto quit1;
    //                             }
    //                             // std::cout << "new_path y" << new_path[k]->y*100 << std::endl;
    //                             // std::cout << "at position " << pos.x << ", " << pos.y << std::endl;
    //                             temp = current.getNumExplored();
    //                             std::cout << "temp = " << temp << std::endl;
    //                         }
    //                         // std::cout << "The dijsktra gives = " << new_path[new_path.size()-1]->x*100 
    //                         // << new_path[new_path.size()-1]->y*100 << std::endl;
    //                         quit1:
    //                         std::cout << "the position after is " << current.getPosition().x << ", " << current.getPosition().y
    //                         << std::endl;
    //                         std::cout << "the bool is after" << current.getExplored()[current.getPosition().x 
    //                             + current.getPosition().y*x_max] << std::endl;

    //                         if(temp != 0 && (2000 >= temp -best_count))
    //                         {
    //                             std::cout << "WARNING WARNING" << std::endl;
    //                             std::cout << "diff  = " << temp - best_count << std::endl;
    //                         }
    //                         best_count = temp;
    //                         goto quiting_time;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    //     quiting_time:
    //     std::cout << (t + 1) << " / " << max_attempts << ", " << current.getNumExplored() << " explored out of " << total_area << std::endl;
    // }


    // auto best = std::make_pair(current.getNumExplored(), current);

    // std::cout << "Done with the search" <<  std::endl;
    // std::cout << "Winning   score is = " << best.first << std::endl;
    // std::cout << "Winning alternatieve found back = " << best.second.point_cap << std::endl;
    // std::cout << "The time it took was = " << best.second.get_time() << " following was max = " << 60*5 << std::endl;
    // //exit(1);
    // unsigned int path_size = best.second.get_path().size();
    for(unsigned int i = 0; i < current.get_path().size(); ++i){
        std::cout << current.get_path()[i].x << ", " << current.get_path()[i].y << std:: endl;
    }

    {
        std::ofstream file(ros::package::getPath("nord_planning")+"/data/plan.txt");
        for (auto& p : current.get_path())
        {
            file << (p.x / 100.0) << " " << (p.y / 100.0) << std::endl;
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // COMMENT TO HERE!!!
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // path.run_simulation((ros::package::getPath("nord_planning") + "/data/plan.txt"), walls, &maze, minimum_path);


    return 0;
};