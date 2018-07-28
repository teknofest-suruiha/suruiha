//
// Created by okan on 23.07.2018.
// Taken from: https://github.com/clearpathrobotics/occupancy_grid_utils/blob/indigo-devel/src/coordinate_conversions.cpp
//

#include <suruiha_gazebo_plugins/score_calculator/coordinate_conversions.h>
#include <gazebo/common/Console.hh>

namespace occupancy_grid_utils
{

    namespace gm=geometry_msgs;
    namespace nm=nav_msgs;

    gm::Polygon cellPolygon (const nm::MapMetaData& info, const Cell& c)
    {
        const float dx[4] = {0.0, 0.0, 1.0, 1.0};
        const float dy[4] = {0.0, 1.0, 1.0, 0.0};

//        gzdbg << "cellPolygon .x:" << c.x << " .y:" << c.y << std::endl;

//        const tf::Transform trans = mapToWorld(info);
        gm::Polygon p;
        p.points.resize(4);
        for (unsigned i=0; i<4; i++) {
//            tf::Point pt((c.x+dx[i])*info.resolution, (c.y+dy[i])*info.resolution, 0.0);
//            tf::Point transformed = trans*pt;
            p.points[i].x = (c.x + dx[i]) * info.resolution + info.origin.position.x;
            p.points[i].y = (c.y + dy[i]) * info.resolution + info.origin.position.y;
            p.points[i].z = 0;
        }

//        gzdbg << "cellPolygon polygon:" << p << std::endl;

        return p;
    }


//    gm::Polygon gridPolygon (const nm::MapMetaData& info)
//    {
//        const float x[4] = {0.0, 0.0, 1.0, 1.0};
//        const float y[4] = {0.0, 1.0, 1.0, 0.0};
//
//        const tf::Transform trans = mapToWorld(info);
//        gm::Polygon p;
//        p.points.resize(4);
//
//        for (unsigned i=0; i<4; i++) {
//            tf::Point pt(x[i]*info.resolution*info.width, y[i]*info.resolution*info.height, 0.0);
//            tf::Point transformed=trans*pt;
//            p.points[i].x = transformed.x();
//            p.points[i].y = transformed.y();
//            p.points[i].z = transformed.z();
//        }
//        return p;
//    }
//
//    void verifyDataSize (const nm::OccupancyGrid& g)
//    {
//        const size_t expected = g.info.height*g.info.width;
////        if (expected!=g.data.size())
////            throw DataSizeException(expected, g.data.size());
//    }


} // namespace