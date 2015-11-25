#include "ros/ros.h"
#include "ros/package.h"

#include <iostream>
#include <fstream>
#include <sstream>

#include "tabu/tabu_search.hpp"
#include "dijkstra/map.hpp"
#include "dijkstra/path.hpp"
#include "dijkstra/point.hpp"

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

    file = std::ifstream(filename);
    unsigned int parent = -1;
    // get all nodes without links
    while (std::getline(file, l))
    {
        if (l[0] == '%')
        {
            std::getline(file, l);
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
            std::getline(file, l);
        }

        std::cout << "line: " << l << std::endl;
        std::istringstream iss(l);
        std::getline(iss, l, ',');
        float x = std::stod(l);
        std::getline(iss, l, ',');
        float y = std::stod(l);

        std::cout << "looking for child at " << x << " " << y << std::endl;
        unsigned int child = -1;
        for (size_t i = 0; i < graph.get_graph().size(); i++)
        {
            if (graph.get_graph()[i].x == x && graph.get_graph()[i].y == y)
            {
                child = i;
                break;
            }
        }
        std::cout << "ping " << parent << " " << child << std::endl;
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

int main(int argc, char** argv)
{
    //ros::init(argc, argv, "nord_planning");
    //ros::NodeHandle n;

    dijkstra::map graph;
    load_graph(ros::package::getPath("nord_planning") + "/data/links.txt", graph);

    auto find_closest = [&](const dijkstra::point& p) {
        dijkstra::point const* closest = nullptr;
        // lazy inf
        float distance = 100000.0f;
        for (auto& p2 : graph.get_graph())
        {
            float new_distance = std::hypot(p.x - p2.x, p.y - p2.y);
            if (new_distance < distance)
            {
                closest = &p2;
                new_distance = distance;
            }
        }
        return closest;
    };

    const float v = 0.4;
    const float w = M_PI/2;
    auto path_time = [&](const dijkstra::path& path) {
        float time = 0;
        float last_rot = 0;
        for (size_t i = 0; i < path.size() - 1; i++)
        {
            auto dx = path[i + 1]->x - path[i]->x;
            auto dy = path[i + 1]->y - path[i]->y;
            auto new_rot = std::atan2(dy, dx);
            time += (new_rot - last_rot) / w;
            time += std::hypot(dx, dy) / v;
            last_rot = new_rot;
        }
        return time;
    };

    // bullshit values
    auto start = find_closest(dijkstra::point(1, 2.1));

    auto fitness = [&](const dijkstra::path& path) {
        std::cout << "fit" << std::endl;
        auto exploration_time = path_time(path);
        auto exit_time = path_time(graph.find(*path.back(), *start));
        auto total_time = exploration_time + exit_time;
        // lazy negative inf
        if (total_time > 3 * 60)
            return -100000.0f;
        else
            return -total_time;
    };

    unsigned int num_random = 20;
    auto random_path = [&]() {
        std::cout << "rand" << std::endl;
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

    auto neighbours = [&](const dijkstra::path& path) {
        std::cout << "neigh" << std::endl;
        std::vector<dijkstra::path> output;

        // random
        output.push_back(random_path());

        // switch & shrink
        for (size_t i = 0; i < path.size() - 1; i++)
        {
            auto it = std::find(path[i]->get_links().begin(), path[i]->get_links().end(),
                                path[i + 1]);
            if (it != path[i]->get_links().end())
            {
                auto switched_path = path;
                switched_path.insert(switched_path.begin() + i, *it);
                output.push_back(switched_path);

                auto shrunk_path = path;
                shrunk_path.erase(shrunk_path.begin() + i);
                output.push_back(shrunk_path);
            }
        }

        // grow
        auto& links = path.back()->get_links();
        for (size_t i = 0; i < links.size(); i++)
        {
            auto grown_path = path;
            grown_path.push_back(links[i]);
        }

        return output;
    };

    std::cout << "searching" << std::endl;
    auto result = tabu::search<dijkstra::path>(100000, 1000,
                                               fitness, random_path, neighbours);

    std::cout << "total time: " << result.first << std::endl;
    for (auto p : result.second)
    {
        std::cout << "\t" << p->x << " " << p->y << std::endl;
    }
    return 0;
}