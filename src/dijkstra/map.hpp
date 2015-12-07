#pragma once
 
#include <unordered_map>
#include <vector>
#include <set>
#include <algorithm>
#include <cmath>
#include "point.hpp"
#include "path.hpp"
 
namespace std {
    template <> struct hash<std::pair<dijkstra::point const*, dijkstra::point const*>>
    {
        size_t operator()(const std::pair<dijkstra::point const*,
                                          dijkstra::point const*>& var) const
        {
            return hash<dijkstra::point const*>()(var.first)
                ^ (hash<dijkstra::point const*>()(var.second) << 16);
        }
    };
}
 
namespace dijkstra
{
    class priority_compare
    {
    public:
        bool operator()(const std::pair<point const*, float>& a,
                        const std::pair<point const*, float>& b)
        {
            if (a.second < b.second)
                return true;
            if (b.second < a.second)
                return false;
 
            if (a.first < b.first)
                return true;
            return false;
        }
    };

    class map
    {
    public:
        map() { };
        map(std::vector<point>&& graph) : graph(graph) { graph.reserve(100000); };
 
        void connect(unsigned int a, unsigned int b);
        void disconnect(unsigned int a, unsigned int b);
        void remove(unsigned int i);
        void add(dijkstra::point p);
        void unlink(unsigned int i);
 
        //void precompute(const std::vector<point>& points);
        void reset();
 
        const std::vector<point>& get_graph() const;
 
        path& find(const point& start_approx, const point& goal_approx);

        point& operator[](size_t i) { return graph[i]; }
        const point& operator[](size_t i) const { return graph[i]; }
 
        map(const map& other) = delete;
 
    private:
        std::set<std::pair<point const*, float>, priority_compare> closest(const point& p_approx);
        void explore(point const* start, std::vector<point const*> goals);
        void insert_all(const std::unordered_map<point const*, float>& visited,
                        const std::unordered_map<point const*, point const*>& previous);
        void insert(const path& p);
 
        std::vector<point> graph;
        std::unordered_map<std::pair<point const*, point const*>, path> paths;
    };
}
