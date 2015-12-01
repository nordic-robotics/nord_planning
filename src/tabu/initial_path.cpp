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

        visualization_msgs::Marker explored_msg;
        visualization_msgs::Marker cone_msg; 
        visualization_msgs::Marker map_msg;
        visualization_msgs::Marker node_msg; 

        InitialPath(std::vector<std::vector<std::vector<Position>>>  nodes, map* maze, int x, int y, dijkstra::map& dijkstra_search, std::valarray<bool> walls)
            : walls(walls) {
            node_links = nodes;
            this->maze = maze;
            start_x = x;
            start_y = y;
        }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //---------------------------------------- simulation ----------------------------------------------------
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////// 

    void run_simulation(ConeOfSight node, Position start, std::valarray<bool> walls,
                        dijkstra::map& dijkstra_search, map* maze){
        std::cout << "starting simulaiton" << std::endl;

        ros::Time time = ros::Time::now();
        ros::Duration d = ros::Duration(1); 

        std::vector<Position> path = node.get_path();
        std::cout << node.get_path().size() << std::endl;
        node = ConeOfSight(maze, start.x, start.y, walls);
        for(unsigned int i = 0; i < path.size(); ++i){
            d.sleep();
            node.rotateCone(path[i].x,path[i].y);
            std::cout << "x = " << path[i].x << "y = " << path[i].y << std::endl;
            publish_all(node, dijkstra_search);
            d.sleep();
            node.moveCone(path[i].x,path[i].y);
            publish_all(node, dijkstra_search);
            
            //std::cout << "i = " << i  << std::endl;
        } 
        std::cout<< "simulation complete" << std::endl;   
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


            explored_pub.publish(explored_msg);
            cone_pub.publish(cone_msg);
            map_pub.publish(map_msg);
            node_pub.publish(node_msg);
            node_pub.publish(conn_msg);
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




private:
    map *maze;
    std::valarray<bool> walls;
    std::vector<std::vector<std::vector<Position>>> node_links;
    int start_x;
    int start_y;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------- itterate through path -----------------------------------------
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

ConeOfSight path_itteration(const std::vector<Position>& path, map* maze, Position start,
                            const std::valarray<bool> walls){
    ConeOfSight node(maze, start.x, start.y, walls);
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

std::vector<ConeOfSight> neighbours(const ConeOfSight& nodes,
        const std::vector<std::vector<std::vector<Position>>>& node_links,
        map* maze, const std::valarray<bool>& walls, const Position& start, int depth = 4){

    std::vector<ConeOfSight> the_neighbours;
    //std::cout << "depth: " << depth << std::endl;
    if (depth <= 1 || !ros::ok())
    {
        the_neighbours.push_back(nodes);
        return the_neighbours;
    }

    int i = 0;
    for (auto& p : node_links[nodes.get_path().back().x][nodes.get_path().back().y])
    {
        if (i++ > 10 * depth)
            break;
        ConeOfSight copy = nodes;
        copy.rotateCone(p.x, p.y);
        copy.moveCone(p.x, p.y);
        copy.add_to_path(p);
        auto deep_neighbours = neighbours(copy, node_links, maze, walls, start, depth - 1);
        std::move(deep_neighbours.begin(), deep_neighbours.end(), std::back_inserter(the_neighbours));
    }
    return the_neighbours;

    int movments =  nodes.get_path().size();
    int random_value = std::rand()% movments; 
    Position random_part = nodes.get_path()[random_value];
    std::vector<Position> new_path;
    std::vector<Position> temp_vec1;


    Position next_next_node; 
    Position next_node; 

    unsigned int path_size = nodes.get_path().size()-1;
    int last_x = nodes.get_path()[path_size].x;
    int last_y = nodes.get_path()[path_size].y;
    // std::cout << "path_size = " << path_size << std::endl;
    // for(unsigned int k = 0; k < path_size; ++k){
    //     std::cout << nodes.get_path()[k].x << ", " << nodes.get_path()[k].y << std::endl;
    // }
    //adds all the links to the last node
    //here is a chance to save alot of time since i pushvka last path. just walk to next node

    Position new_position;
    for(unsigned int k = 0; k < node_links[last_x][last_y].size(); ++k){
        ConeOfSight alternative_route = nodes;
        new_position = node_links[last_x][last_y][k];
        if (new_position.x == last_x && new_position.y == last_y)
        {
            std::cout << "fitness is fucked yo" << std::endl;
            exit(1);
        }
        alternative_route.rotateCone(new_position.x, new_position.y);
        alternative_route.moveCone(new_position.x,new_position.y);
        alternative_route.add_to_path(new_position);
        the_neighbours.push_back(alternative_route);
        // std::cout << "läggs till ="  << node_links[last_x][last_y][k].x << ", " << node_links[last_x][last_y][k].y << std::endl;
    }
    //std::cout << "got past first for loop" << std::endl;

    // tries to minimize the path 
    // std::cout << "random_value = " << random_value << std::endl;   
    next_node = nodes.get_path()[random_value+1];
    //std::cout << "next_node = " << next_node.x << ", " << next_node.y << std::endl;  
    next_next_node = nodes.get_path()[random_value+2];
    temp_vec1 = node_links[random_part.x][random_part.y];

    for(unsigned int i = 0; i < temp_vec1.size(); ++i){
        //std::cout << "i =" << i << std::endl;
        new_path = nodes.get_path();
        if(temp_vec1[i] == next_next_node){ 
            // std::cout << "FOUND a coool granne" << std::endl;
            new_path.erase(new_path.begin() + (random_value +1));
            the_neighbours.push_back(path_itteration(new_path, maze, start, walls));
        }
        // else{
        //     if(node_links[(temp_vec1[i].x)][(temp_vec1[i].y)] == next_next_node){
        //         new_path.erase(new_path.begin+)
        //     }
        // }
    }   
    std::cout << "num: " << the_neighbours.size() << std::endl;
    return the_neighbours;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ------------------------------------------ RANDOM -----------------------------------------------------
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
ConeOfSight random(const Position& start, const std::vector<std::vector<std::vector<Position>>>& node_links,
                   const std::valarray<bool>& walls, map* maze){
    Position end_position = start;
    int connections; 
    int random_nr; 
    Position pos;
    Position new_pos;
    int i = 0;
    ConeOfSight node(maze, start.x, start.y, walls);
    bool ok =false;
    std::cout << "the size  = " << node.get_path().size() << std::endl;
    std::cout << node.get_path()[0].x << ", " << node.get_path()[0].y << std::endl;
    return node;
    //ros::Time time = ros::Time::now();
    //ros::Duration d = ros::Duration(1.5,0);
    while(ok == false){
        
        pos = node.getPosition();
        connections = node_links[pos.x][pos.y].size();
        random_nr   = rand()%connections;
        // std::cout << "random number = " << random_nr << std::endl;
        new_pos   = node_links[pos.x][pos.y][random_nr];
        if(!(new_pos == end_position)){


            node.rotateCone(new_pos.x,new_pos.y);
        
            node.moveCone(new_pos.x,new_pos.y);
            node.add_to_path(new_pos);

           
            //d.sleep();
            ++i;

            if(i >= 20){
                ok= true;
                // std::cout << "the size after = " << node.get_path().size() << std::endl;
                // for(unsigned int i = 0; i < node.get_path().size(); ++i){
                //     std::cout << node.get_path()[i].x << ", " << node.get_path()[i].y << std::endl;
                }
            }
        }   

    // }
    // std::cout << "OMG hittade ut" << std::endl;
    return node;
} 

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------- fitness ----------------------------------------------------
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

float fitness(const ConeOfSight& the_path, const Position& start, dijkstra::map& dijkstra_search)
{
    //should i restart it, maybe try to find a different solution
    float score = 0;
    int max_time = 5 * 60;
    double our_time = the_path.get_time();
    unsigned int path_size = the_path.get_path().size()-1;
    Position curr_pos = the_path.get_path()[path_size];
    // std::cout << "curr pos = " << curr_pos.x << ", " << curr_pos.y << std::endl;
    float coolness = 0;
    /*
    //use the difference between explored instead, otherwise the idea behind
    // it is solid   
    if(curr_pos.x != start.x && curr_pos.y != start.y){
        dijkstra::point starting_node(start.x/100.0, start.y/100.0);
        dijkstra::point current_node(curr_pos.x/100.0, curr_pos.y/100.0);
        dijkstra::path shortest_path;
        shortest_path = dijkstra_search.find(current_node, starting_node);
        shortest_path.compute_length(); 
        coolness = shortest_path.length()*10;
    }
    else{
        score -=1;
    }*/
    
    

    for(int i = 0; i < the_path.get_x_max(); ++i){
        for(int j = 0; j < the_path.get_y_max();++j){
            if(the_path.getExplored()[i + j * the_path.get_x_max()]){
                score += 1;
            }
        }
    }
    /*
    if(!(the_path.get_path()[path_size] == start)){
        score -= 5000;
    }

    
    // score -= shortest_path.length()*2;  
    if(our_time>max_time){
        score = -5000;
    }
    if(path_size <20){
        score -= 20000/path_size;
    }
    */
    // std::cout << "The score is = " << score << std::endl;
    // std::cout << "Our estimated time is = " << our_time << std::endl;
    return score;           
}


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
std::vector<std::vector<std::vector<Position>>> read_nodes(std::string filename, float x_m, float y_m){
    //std::cout << filename << std::endl;
    std::ifstream file(filename);
    std::string l;
    int x_max = x_m*100;
    int y_max = y_m*100;
    // std::cout << "x max = " << x_max << "  y max = " << y_max << std::endl;
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
            // std::cout << x << " " << y << std::endl;
            new_node = false;
            node_connections[x][y].clear();
        }
        else{
            
            line >> con_x >> comma >> con_y;
            Position new_con;
            new_con.x = con_x *100;
            new_con.y = con_y *100;
            // std::cout << "\t" << new_con.x << " " << new_con.y << std::endl; 
            node_connections[x][y].push_back(new_con); 
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
        if (l[0] == '%')
        {
            std::getline(file2, l);
            std::istringstream iss(l);
            std::getline(iss, l, ',');
            float x = std::stod(l);
            std::getline(iss, l, ',');
            float y = std::stod(l);
            std::cout << "looking for " << x << " " << y << std::endl;

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

        std::cout << "line: " << l << std::endl;
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
    map maze = read_map(ros::package::getPath("nord_planning") + "/data/contest_rehearsal_maze.txt");
    std::vector<std::vector<std::vector<Position>>> node_vector = read_nodes(ros::package::getPath("nord_planning") + "/links.txt", maze.get_max_x(), maze.get_max_y());
    std::valarray<bool> walls = read_walls(ros::package::getPath("nord_planning") + "/Map.txt",maze.get_max_x()*100, maze.get_max_y()*100);

    dijkstra::map minimum_path;
    load_graph(ros::package::getPath("nord_planning") + "/links.txt", minimum_path);
    Position start(20, 20); // find closest?
    InitialPath path(node_vector, &maze, start.x, start.y, minimum_path, walls);
    // auto test = path.random();
    // path.run_simulation(test);

    // auto neighbours = path.neighbours(random);
    // std::cout << neighbours.size() << std::endl;
    // path.run_simulation(neighbours[4]);
    // unsigned int fuck = node_vector[50][25].size();
    // for(unsigned int i = 0; i < fuck ; ++i){
    //     std::cout << node_vector[50][25][i].x << ", " << node_vector[50][25][i].y << std:: endl;
    // } 
    // std::cout << "bnasnkdas" << node_vector[50][25][1].x << "," << node_vector[50][25][2].y << std::endl; 
    

    std::cout << "starting the tabu search" << std::endl;
    int max_attempts = 25;
    int short_memory = 1000;   

    std::pair<float, ConeOfSight> best = tabu::search<ConeOfSight>(max_attempts, short_memory, [&](const ConeOfSight& c) { return fitness(c, start, minimum_path); },
        [&]() { return random(start, node_vector, walls, &maze); }, [&](const ConeOfSight& c) { return neighbours(c, node_vector, &maze, walls, start);});
    std::cout << "Done with the search" <<  std::endl;
    std::cout << "Winning   score is = " << best.first << std::endl;
    std::cout << "Winning alternatieve found back = " << best.second.point_cap << std::endl;
    std::cout << "The time it took was = " << best.second.get_time() << " following was max = " << 60*5 << std::endl;
    //exit(1);
    unsigned int path_size = best.second.get_path().size();
    for(unsigned int i = 0; i < path_size ; ++i){
        std::cout << best.second.get_path()[i].x << ", " << best.second.get_path()[i].y << std:: endl;
    }

    {
        std::ofstream file(ros::package::getPath("nord_planning")+"/data/plan.txt");
        for (auto& p : best.second.get_path())
        {
            file << (p.x / 100.0) << " " << (p.y / 100.0) << std::endl;
        }
    }
    path.run_simulation(best.second, start, walls, minimum_path, &maze);
    return 0;
};