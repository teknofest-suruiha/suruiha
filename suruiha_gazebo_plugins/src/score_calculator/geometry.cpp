//
// Created by okan on 23.07.2018.
//
// This file is taken from: https://github.com/clearpathrobotics/occupancy_grid_utils/blob/indigo-devel/src/geometry.cpp
//

#include <suruiha_gazebo_plugins/score_calculator/geometry.h>
#include <queue>
#include <boost/foreach.hpp>

namespace occupancy_grid_utils
{
    namespace gm=geometry_msgs;
    namespace nm=nav_msgs;
    using std::vector;
    typedef std::set<Cell> Cells;

    struct Line
    {
        Line (const gm::Point32& p1, const gm::Point32& p2)
        {
            const float dx = p2.x-p1.x;
            const float dy = p2.y-p1.y;
            if (fabs(dx) < 1e-3 && fabs(dy) < 1e-3)
            {
//                boost::format e("Points (%.2f, %.2f) and (%.2f, %.2f) are too close");
//                throw GridUtilsException(e % p1.x % p1.y % p2.x % p2.y);
                std::cerr << "occupancy_grid_utils: Points are very close cannot create Line" << std::endl;
            }
            a = dy;
            b = -dx;
            c = p1.y*dx - p1.x*dy;
        }

        // Line intersects a convex polygon if two of the vertices have opposite signs
        bool intersects (const gm::Polygon& poly) const
        {
            bool seen_nonnegative = false;
            bool seen_nonpositive = false;
            BOOST_FOREACH (const gm::Point32& p, poly.points)
                        {
                            const float d = a*p.x+b*p.y+c;
                            if (d>=0)
                                seen_nonnegative = true;
                            if (d<=0)
                                seen_nonpositive = true;
                            if (seen_nonnegative && seen_nonpositive)
                                return true;
                        }
            return false;
        }

        // Coefficients of line equation ax+by+c=0
        float a;
        float b;
        float c;
    };

// Visitor for the flood fill
    struct CellsInPolygon
    {
        CellsInPolygon (const nm::MapMetaData& info, const gm::Polygon& poly) :
                info(info)
        {
            const size_t n = poly.points.size();
            for (size_t i=0; i<n; i++)
            {
                const size_t j = i>0 ? i-1 : n-1;
                sides.push_back(Line(poly.points[i], poly.points[j]));
            }
        }

        // Add cell to the set of visited cells.  If cell doesn't intersect the
        // boundary of the polygon, return true (continue propagating), else false.
        bool operator() (const Cell& c)
        {
            cells.insert(c);
            const gm::Polygon cell_poly = cellPolygon(info, c);
            BOOST_FOREACH (const Line& s, sides)
                        {
                            if (s.intersects(cell_poly))
                                return false;
                        }
            return true;
        }

        Cells cells;
        const nm::MapMetaData& info;
        std::vector<Line> sides;
    };



// Generic flood fill
    template <class Visitor>
    void flood_fill (const nm::MapMetaData& info, const std::set<Cell>& start,
                     Visitor& vis)
    {
        Cells seen;
        std::queue<Cell> q;
        BOOST_FOREACH (const Cell& c, start)
                        q.push(c);
        while (!q.empty())
        {
            const Cell c = q.front();
            q.pop();
//            ROS_DEBUG_STREAM_NAMED ("flood_fill", "Cell " << c);
            if (seen.find(c)==seen.end())
            {
                seen.insert(c);
//                ROS_DEBUG_NAMED ("flood_fill", "  Visiting");
                if (vis(c))
                {
//                    ROS_DEBUG_NAMED ("flood_fill", "  Adding neighbors");
                    for (int dy=-1; dy<=1; dy++)
                    {
                        for (int dx=-1; dx<=1; dx++)
                        {
                            if (dx!=0 || dy!=0)
                            {
                                const Cell c2(c.x+dx, c.y+dy);
                                if (withinBounds(info, c2))
                                {
                                    q.push(c2);
//                                    ROS_DEBUG_STREAM_NAMED ("flood_fill", "    Added " << c2);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

// Generic flood fill
    template <class Visitor>
    void flood_fill (const nm::MapMetaData& info, const Cell& start,
                     Visitor& vis)
    {
        std::set<Cell> s;
        s.insert(start);
        flood_fill(info, s, vis);
    }


    Cell center(const nm::MapMetaData& info,
                const gm::Polygon& poly)
    {
        float sx=0;
        float sy=0;
        BOOST_FOREACH (const gm::Point32& p, poly.points)
                    {
                        sx += p.x;
                        sy += p.y;
                    }
        gm::Point p;
        p.x = sx/poly.points.size();
        p.y = sy/poly.points.size();
        return pointCell(info, p);
    }

// We do this by starting at the center and flood-filling outwards till we
// reach the boundary
    Cells cellsInConvexPolygon (const nm::MapMetaData& info,
                                const gm::Polygon& poly)
    {
        CellsInPolygon visitor(info, poly);
        flood_fill(info, center(info, poly), visitor);
        return visitor.cells;
    }


} // namespace