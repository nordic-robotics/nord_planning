#include "map.hpp"
#include <set>
#include <iostream>
 
namespace dijkstra
{
    void map::connect(unsigned int a, unsigned int b)
    {
        graph[a].connect(&graph[b]);
        graph[b].connect(&graph[a]);
    }
 
    void map::disconnect(unsigned int a, unsigned int b)
    {
        graph[a].disconnect(&graph[b]);
        graph[b].disconnect(&graph[a]);
 
        reset();
    }
 
    void map::add(dijkstra::point p)
    {
        auto cap = graph.capacity();
        graph.push_back(p);
        if (cap != graph.capacity())
        {
            std::cout << "reallocation! aborting.." << std::endl;
            exit(1);
        }
    }
 
    void map::remove(unsigned int i)
    {
        for (auto l : graph[i].get_links())
        {
            l->disconnect(&graph[i]);
			graph[i].disconnect(l);
        }
        graph.erase(graph.begin() + i);
 
        reset();
    }

    void map::unlink(unsigned int i)
    {
        for (auto l : graph[i].get_links())
        {
            l->disconnect(&graph[i]);
            graph[i].disconnect(l);
        }
        reset();
    }
 
    /*void map::precompute(const std::vector<point>& points)
    {
        for (auto& p1 : points)
        {
            auto start = closest(p1);
            std::vector<point const*> goals;
            goals.reserve(points.size());
            // find the closest for each goal
            std::transform(points.begin(), points.end(),
                           std::back_inserter(goals),
                           [&](const point& x) { return closest(x); });
 
            explore(start, goals);
        }
    }*/
 
    void map::reset()
    {
        paths.clear();
    }
 
    const std::vector<point>& map::get_graph() const { return graph; }
 
    path& map::find(const point& start_approx, const point& goal_approx)
    {
        auto start_set = closest(start_approx);
        auto goal_set = closest(goal_approx);
 
        auto s_it = start_set.begin();
        auto g_it = goal_set.begin();
        for (size_t i = 0; i < 10; i++)
        {
            // if it already exists: it's memoized, return it
            auto it = paths.find(std::make_pair(*s_it, *g_it));
            if (it != paths.end())
            {
                return it->second;
            }
     
            // otherwise do some exploring
            explore(*s_it, {*g_it});
     
            // entry should now exist
            auto path = paths[std::make_pair(*s_it, *g_it)];
            if (path.size() != 0)
                return path;
            ++s_it;
            ++g_it;
        }

        return paths[std::make_pair(start_set.front(), goal_set.front())];
    }
 
    std::set<std::pair<point const*, float>, priority_compare>
    map::closest(const point& p_approx)
    {
        std::set<std::pair<point const*, float>, priority_compare> output;
        std::transform(graph.begin(), graph.end(), std::back_inserter(output),
            [&](point const* p) -> std::pair<point const*, float>{
                return std::make_pair(p, p_approx.distance(p));
        });

        if (output.size() > 10)
        output.erase(output.begin() + 10, output.end());
 
        return output;
    }
 
    void map::explore(point const* start, std::vector<point const*> goals)
    {
        std::set<std::pair<point const*, float>, priority_compare> unvisited;
        std::unordered_map<point const*, float> visited;
        std::unordered_map<point const*, point const*> previous;
 
        unvisited.emplace(start, 0);
        visited.emplace(start, 0);
 
        while (!unvisited.empty())
        {
            // pop the smallest one
            auto current = unvisited.begin();
 
            for (auto link : current->first->get_links())
            {
                if (visited.find(link) != visited.end())
                    continue;
 
                auto tentative = current->second + current->first->distance(link);
                auto it = std::find_if(unvisited.begin(), unvisited.end(),
                    [&](std::pair<point const*, float> kvp) {
                        return kvp.first == link;
                    });
                // if the node is in the 'unvisited' map
                if (it != unvisited.end())
                {
                    // use the shorter path of the previous and the current
                    if (tentative < it->second)
                    {
                        unvisited.emplace(link, tentative);
                        previous[link] = current->first;
                    }
                }
                else
                {
                    // if it's not in the map, add it
                    unvisited.emplace(link, tentative);
                    previous[link] = current->first;
                }
 
                visited[link] = current->second;
 
                // if this is a goal node
                auto goals_it = std::find(goals.begin(), goals.end(), link);
                if (goals_it != goals.end())
                {
                    // remove it as a goal
                    goals.erase(goals_it);
 
                    // if it was the last goal
                    if (goals.size() == 0)
                    {
                        // we done yo
                        insert_all(visited, previous);
                        return;
                    }
                }
            }
            unvisited.erase(current);
        }
    }
 
    void map::insert_all(const std::unordered_map<point const*, float>& visited,
                    const std::unordered_map<point const*, point const*>& previous)
    {
        // inserts all subpaths of a path
 
        // every node in visited is known to be optimal
        for (auto& node : visited)
        {
            path p;
            p.push_back(node.first);
 
            auto it = previous.end();
            while ((it = previous.find(p.front())) != previous.end())
            {
                // add one node from the path
 
                // TODO: don't recompute full path
                // maybe incorporate into push_back and add a pop_front
                p.insert(p.begin(), it->second);
                p.compute_length();
                insert(p);
 
                path p2 = p;
                while (p2.size() > 2)
                {
                    // and remove on the other end until done
                    p2.pop_back();
                    p2.compute_length();
                    insert(p2);
                }
            }
        }
    }
 
    void map::insert(const path& p)
    {
        paths[std::make_pair(p.front(), p.back())] = p;
        paths[std::make_pair(p.back(), p.front())] = p;
    }
}
