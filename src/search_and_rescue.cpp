#include "ros/ros.h"
#include "ros/package.h"
#include "visualization_msgs/Marker.h"
#include "nord_messages/Vector2.h"
#include "nord_messages/PlanSrv.h"

#include <iostream>
#include <fstream>
#include <sstream>

#include "tabu/tabu_search.hpp"
#include "dijkstra/map.hpp"
#include "dijkstra/path.hpp"
#include "dijkstra/point.hpp"

struct rviz_line
{
    float x0, y0, x1, y1;
    rviz_line(float x0, float y0, float x1, float y1)
        : x0(x0), y0(y0), x1(x1), y1(y1) { };
};

visualization_msgs::Marker
create_lines_message(const std::vector<rviz_line>& lines,
                     float r, float g, float b, float a,
                     int id, float z)
{
    visualization_msgs::Marker line_list;
    line_list.id = id;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.color.a = a;
    line_list.color.r = r;
    line_list.color.g = g;
    line_list.color.b = b;
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "rescue_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.lifetime = ros::Duration();
    line_list.scale.x = 0.01;

    for (auto& l : lines)
    {
        geometry_msgs::Point p0, p1;
        p0.x = l.x0;
        p0.y = l.y0;
        p1.x = l.x1;
        p1.y = l.y1;
        p0.z = p1.z = z;
        line_list.points.push_back(p0);
        line_list.points.push_back(p1);
    }

    return line_list;
}


visualization_msgs::Marker
create_objects_message(const std::vector<std::pair<dijkstra::point,std::string>> objects)
{
    visualization_msgs::Marker line_list;
    line_list.id = 206;
    line_list.type = visualization_msgs::Marker::CUBE_LIST;
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "houston_objects";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.lifetime = ros::Duration();
    line_list.scale.x = 0.05;
    line_list.scale.y = 0.05;
    line_list.scale.z = 0.05;
    line_list.color.r = line_list.color.g = line_list.color.b = line_list.color.a = 1.0;

    for (auto& o : objects)
    {
        geometry_msgs::Point p;
        p.x = o.first.x;
        p.y = o.first.y;
        p.z = 0;

        std_msgs::ColorRGBA c;
        c.a = 1.0;

        if (o.second == "green cube")
        {
            c.r = 0.0;
            c.g = 1.0;
            c.b = 0.0;
        }
        else if (o.second == "red cube")
        {
            c.r = 1.0;
            c.g = 0.0;
            c.b = 0.0;
        }
        else if (o.second == "patric")
        {
            c.r = 1.0;
            c.g = 0.5;
            c.b = 0.0;
        }
        line_list.points.push_back(p);
        line_list.colors.push_back(c);
    }

    return line_list;
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
            //std::cout << "looking for " << x << " " << y << std::endl;

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

        //std::cout << "line: " << l << std::endl;
        std::istringstream iss(l);
        std::getline(iss, l, ',');
        float x = std::stod(l);
        std::getline(iss, l, ',');
        float y = std::stod(l);

        //std::cout << "looking for child at " << x << " " << y << std::endl;
        unsigned int child = -1;
        for (size_t i = 0; i < graph.get_graph().size(); i++)
        {
            if (graph.get_graph()[i].x == x && graph.get_graph()[i].y == y)
            {
                child = i;
                break;
            }
        }
        //std::cout << "ping " << parent << " " << child << std::endl;
        graph.connect(parent, child);
    }

    /*
    for (auto& p : graph.get_graph())
    {
        std::cout << p.x << " " << p.y << std::endl;
        for (auto p2 : p.get_links())
        {
            std::cout << " " << p2->x << " " << p2->y << std::endl;
        }
    }
    */
}

std::vector<std::pair<dijkstra::point, std::string>> load_objects(std::string filename)
{
    std::ifstream file(filename);
    std::string l;

    std::vector<std::pair<dijkstra::point, std::string>> objects;
    while (std::getline(file, l))
    {
        std::istringstream iss(l);
        float x, y;
        iss >> x >> y;
        std::string name;
        std::getline(iss, name);
        name.erase(name.begin());
        objects.emplace_back(dijkstra::point(x, y), name);
    }

    return objects;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nord_planning");
    ros::NodeHandle n;
    srand(time(NULL));

    dijkstra::map graph;
    load_graph(ros::package::getPath("nord_planning") + "/links.txt", graph);
    auto objects = load_objects(ros::package::getPath("nord_vision") + "/data/objects.txt");

    auto find_closest = [&](const nord_messages::Vector2& p) {
        dijkstra::point const* closest = nullptr;
        // lazy inf
        float distance = 100000.0f;
        for (auto& p2 : graph.get_graph())
        {
            float new_distance = std::hypot(p.x - p2.x, p.y - p2.y);
            if (new_distance < distance)
            {
                closest = &p2;
                distance = new_distance;
            }
        }
        return closest;
    };

    const float v = 0.2;
    const float w = M_PI/4;
    const float node_pause = 0.1;
    auto path_time = [&](const dijkstra::path& path) -> float {
        float time = 0;
        float last_rot = 0;
        if (path.size() == 0)
            return 0;

        for (size_t i = 0; i < path.size() - 1; i++)
        {
            auto dx = path[i + 1]->x - path[i]->x;
            auto dy = path[i + 1]->y - path[i]->y;
            auto new_rot = std::atan2(dy, dx);
            time += std::abs((new_rot - last_rot) / w);
            time += std::hypot(dx, dy) / v;
            time += node_pause;
            last_rot = new_rot;
        }
        return time;
    };

    auto object_max_distance = 0.5f;
    auto num_objects_encountered = [&](const dijkstra::path& path,
            std::vector<std::pair<dijkstra::point, std::string>>& objects) {
        unsigned int num_objects = 0;
        for (auto p : path)
        {
            for (int i = 0; i < objects.size(); i++)
            {
                if (objects[i].first.distance(p) < object_max_distance)
                {
                    num_objects++;
                    objects.erase(objects.begin() + i);
                    i--;
                }
            }
        }

        return num_objects;
    };

    auto object_bonus = 100;
    auto fitness = [&](dijkstra::point const* end) {
        return [&](const dijkstra::path& path) {
            if (path.size() == 0)
                return -100000.0f;

            auto dijkstra_path = graph.find(*path.back(), *end);
            auto exploration_time = path_time(path);
            auto exit_time = path_time(dijkstra_path);
            auto total_time = exploration_time + exit_time;

            // lazy negative inf
            if (total_time > 3 * 60 - 15)
                return -100000.0f;
            else
            {
                auto objects_copy = objects;
                auto num_objects = num_objects_encountered(path, objects_copy);
                num_objects += num_objects_encountered(dijkstra_path, objects_copy);
                return -total_time + num_objects * object_bonus;
            }
        };
    };

    unsigned int num_random = 2;
    auto random_path = [&](dijkstra::point const* start) {
        return [&, start]() {
            dijkstra::path path;
            auto current = start;
            unsigned int counter = 0;
            while (++counter < num_random)
            {
                path.push_back(current);
                current = current->get_links()[rand() % current->get_links().size()];
            }
            return path;
        };
    };

    auto neighbours = [&](dijkstra::point const* start) {
        return [&](const dijkstra::path& path) {
            std::vector<dijkstra::path> output;

            // random
            output.push_back(random_path(start)());

            // switch & shrink
            if (path.size() > 2)
            {
                for (size_t i = 0; i < path.size() - 2; i++)
                {
                    auto it = std::find(path[i]->get_links().begin(),
                                        path[i]->get_links().end(),
                                        path[i + 2]);
                    if (it != path[i]->get_links().end())
                    {
                        auto switched_path = path;
                        switched_path.erase(switched_path.begin() + i + 1);
                        switched_path.insert(switched_path.begin() + i + 1, *it);
                        output.push_back(switched_path);

                        auto shrunk_path = path;
                        shrunk_path.erase(shrunk_path.begin() + i + 1);
                        output.push_back(shrunk_path);
                    }
                }
            }

            // grow
            auto& links = path.back()->get_links();
            for (size_t i = 0; i < links.size(); i++)
            {
                auto grown_path = path;
                grown_path.push_back(links[i]);
                output.push_back(grown_path);
            }

            return output;
        };
    };

    auto conversion = [&](dijkstra::point const* p) {
        nord_messages::Vector2 v;
        v.x = p->x;
        v.y = p->y;
        return v;
    };


    auto service = [&](nord_messages::PlanSrv::Request& req,
                       nord_messages::PlanSrv::Response& res) mutable {
        auto start = find_closest(req.start);
        auto end = find_closest(req.end);
        if (!req.direct)
        {
            auto result = tabu::search<dijkstra::path>(10000, 1000,
                                                       fitness(end),
                                                       random_path(start),
                                                       neighbours(start));
            std::transform(result.second.begin(), result.second.end(),
                           std::back_inserter(res.path), conversion);

            auto leftovers = graph.find(*result.second.back(), *end);
            std::transform(leftovers.begin(), leftovers.end(),
                           std::back_inserter(res.path), conversion);
            res.time = path_time(result.second) + path_time(leftovers);
        }
        else
        {
            auto dijkstra_path = graph.find(*start, *end);
            std::transform(dijkstra_path.begin(), dijkstra_path.end(),
                           std::back_inserter(res.path), conversion);
            res.time = path_time(dijkstra_path);
        }

        return true;
    };

    ros::ServiceServer srv(n.advertiseService("/nord_planning/plan_service",
                           &decltype(service)::operator(), &service));

    ros::spin();

    /*
    std::cout << "searching" << std::endl;
    auto result = tabu::search<dijkstra::path>(10000, 1000,
                                               fitness, random_path, neighbours);

    std::cout << "tabu part:" << std::endl;
    for (auto p : result.second)
    {
        std::cout << "\t" << p->x << " " << p->y << std::endl;
    }
    auto dijkstra_path = graph.find(*result.second.back(), *end);
    fitness(result.second, true);
    std::cout << "dijkstra part:" << std::endl;
    for (auto p : dijkstra_path)
    {
        std::cout << "\t" << p->x << " " << p->y << std::endl;
    }
    std::cout << "total time: " << (path_time(result.second)
                                  + path_time(dijkstra_path)) << std::endl;

    ros::Publisher map_pub(n.advertise<visualization_msgs::Marker>("/nord/map", 1));
    ros::Rate r(1);
    while (ros::ok())
    {
        std::vector<line> links;
        for (auto& p0 : graph.get_graph())
        {
            for (auto p1 : p0.get_links())
            {
                links.emplace_back(p0.x, p0.y, p1->x, p1->y);
            }
        }
        
        std::vector<line> dijksta_lines;
        if (dijkstra_path.size() != 0)
        {
            for (size_t i = 0; i < dijkstra_path.size() - 1; i++)
            {
                dijksta_lines.emplace_back(dijkstra_path[i]->x, dijkstra_path[i]->y,
                                           dijkstra_path[i + 1]->x, dijkstra_path[i + 1]->y);
            }
        }

        std::vector<line> dijksta_full_lines;
        if (dijkstra_test.size() != 0)
        {
            for (size_t i = 0; i < dijkstra_test.size() - 1; i++)
            {
                dijksta_full_lines.emplace_back(dijkstra_test[i]->x, dijkstra_test[i]->y,
                                                dijkstra_test[i + 1]->x, dijkstra_test[i + 1]->y);
            }
        }

        std::vector<line> planned_lines;
        if (result.second.size() > 0)
        {
            for (size_t i = 0; i < result.second.size() - 1; i++)
            {
                planned_lines.emplace_back(result.second[i]->x, result.second[i]->y,
                                           result.second[i + 1]->x, result.second[i + 1]->y);
            }
        }

        map_pub.publish(create_lines_message(links, 1, 0, 0, 0.05, 1044, 0));
        map_pub.publish(create_lines_message(dijksta_lines, 0, 1, 0, 0.6, 1045, 0.05));
        map_pub.publish(create_lines_message(planned_lines, 0, 1, 1, 0.6, 1046, 0.1));
        map_pub.publish(create_lines_message(dijksta_full_lines, 1, 1, 0, 0.6, 1047, 0.15));
        map_pub.publish(create_objects_message(objects));
        ros::spinOnce();
        r.sleep();
    }*/

    
    return 0;
}