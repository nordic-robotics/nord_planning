#include "initial_path.hpp"


//std::vector<dijkstra::point> graph; 
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

visualization_msgs::Marker create_map_message(const map& maze)
{
    visualization_msgs::Marker line_list;
    line_list.id = 2;
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
        p0.z = p1.z = 0;
        line_list.points.push_back(p0);
        line_list.points.push_back(p1);
    }

    return line_list;
}

visualization_msgs::Marker plotting_nodes(std::vector<dijkstra::point> graph){
    visualization_msgs::Marker node_list;
    node_list.id = 1;
    node_list.type = visualization_msgs::Marker::NODE_LIST;
    node_list.color.a = 0;
    node_list.color.r = node_list.color.g = node_list.b = 1.0;
}

visualization_msgs::Marker cone_of_sight(float x, float y){
    sensor_msgs::Range cone;
    cone.id = 3;
    cone.type = visualization_msgs::Marker::CONE;
    cone.color.g = cone.color.r = 1.0;
    cone.color.a = cone.color.b = 0.0;
    cone.header.frame_id = "/map";
    cone.header.stamp = ros::Time::now();
    cone.ns = "ip_cone";
    cone.action = visualization_msgs::Marker::ADD;
    cone.lifetime = ros::Duration();
    cone.scale.x = cone.scale.y = cone.scale.z = 0.01;

    //what do i need ? i need last current node and next node
    //assume that i start by looking forwad
    // this would lead to cone = 

    
    return cone;
}

int main(int argc, char** argv)
{
	//for plotting the map
	ros::init(argc, argv, "initial_path");
	ros::NodeHandle n;
	map maze = read_map(ros::package::getPath("nord_planning") + "/data/small_maze.txt");
	auto map_msg = create_map_message(maze);
	auto map_pub = n.advertise<visualization_msgs::Marker>("/nord/map", 1);
	auto map_timer = n.createTimer(ros::Duration(1), [&](const ros::TimerEvent& e) {
        map_pub.publish(map_msg);
    });

    
    int max_attempts = 100;
    int short_memory = 1;
    std::pair<float, int> best = tabu::search<int>(max_attempts, short_memory, &BiggestY::fitness,
        &BiggestY::random, &BiggestY::neighbours);
    ROS_INFO("y = %f, x = %d", best.first, best.second);
    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    } 
    return 0;       
};