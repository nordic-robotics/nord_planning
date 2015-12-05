#include "point.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

namespace dijkstra
{
    float point::distance(point const* other) const
    {
        return std::hypot(other->x - x, other->y - y);
    }
    float point::distance(const point& other) const
    {
        return distance(&other);
    }

    const std::vector<point*>& point::get_links() const
    {
        return links;
    }

    void point::connect(point* other)
    {
        links.push_back(other);
    }

    void point::disconnect(point* other)
    {
        links.erase(std::remove(links.begin(), links.end(), other), links.end());
    }

    bool operator==(const point& lhs, const point& rhs)
    {
        return lhs.x == rhs.x
             & lhs.y == rhs.y;
    }

    std::ostream& operator<<(std::ostream& os, const point& p)
    {
        os << "(" << p.x << ", " << p.y << ")";
        return os;
    }
}
