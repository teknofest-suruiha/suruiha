//
// Created by okan on 23.07.2018.
//
// This file is taken from: https://github.com/clearpathrobotics/occupancy_grid_utils/blob/indigo-devel/include/occupancy_grid_utils/geometry.h
//

#ifndef SURUIHA_GAZEBO_PLUGINS_GEOMETRY_H
#define SURUIHA_GAZEBO_PLUGINS_GEOMETRY_H

#include <suruiha_gazebo_plugins/score_calculator/coordinate_conversions.h>
#include <set>
#include <boost/multi_array.hpp>

namespace occupancy_grid_utils
{


/// \retval Set of cells in the region bounded by convex polygon \a p
///
/// Cells that are partly in the polygon (because they intersect the
/// boundary) are included.  Cells that intersect the polygon only along
/// an edge or corner may or may not be included.
    std::set<Cell> cellsInConvexPolygon (const nav_msgs::MapMetaData& info,
                                         const geometry_msgs::Polygon& p);
//
//
///// \retval Locally maximal set of cells which are at least \a d apart in
///// Euclidean distance, and all of which satisfy \a pred
///// \tparam pred Defines operator(), a boolean predicate on Cell objects
//    template <typename Pred>
//    std::set<Cell> tileCells (const nav_msgs::MapMetaData& info, float d,
//                              const Pred& p);
//
//
///// Return value of distanceField function, that maps cells
///// to their Euclidean distance to nearest obstacle
//    class DistanceField
//    {
//    public:
//
//        typedef boost::multi_array<float, 2> Array;
//        typedef boost::shared_ptr<Array> ArrayPtr;
//
//        DistanceField (ArrayPtr distances) : distances_(distances) {}
//
//        inline
//        float operator[] (const Cell& c) const
//        {
//            return (*distances_)[c.x][c.y];
//        };
//
//    private:
//
//        ArrayPtr distances_;
//    };
//
//
//
///// \retval Distance field d, such that d[c] is the Manhattan distance
///// (in meters) from cell c to an obstacle cell.
///// \param max_dist Distances will be thresholded at this value if
///// it's positive
//    DistanceField distanceField (const nav_msgs::OccupancyGrid& m,
//                                 float max_dist=-42);

} // namespace

#endif //SURUIHA_GAZEBO_PLUGINS_OCCUPANCY_GRID_UTILS_H
