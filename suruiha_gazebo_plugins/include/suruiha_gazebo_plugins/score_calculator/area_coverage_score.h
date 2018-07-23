//
// Created by okan on 18.07.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_AREA_COVERAGE_SCORE_H
#define SURUIHA_GAZEBO_PLUGINS_AREA_COVERAGE_SCORE_H

#include <sdf/sdf.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <string>
#include <map>
#include <utility>
#include <vector>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <gazebo/common/Time.hh>
#include <CGAL/General_polygon_set_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel>  Polygon_with_holes_2;
typedef Polygon_2::Edge_const_iterator EdgeIterator;
typedef Polygon_2::Vertex_const_iterator VertexIterator;
typedef std::list<Polygon_with_holes_2> Pwh_list_2;

class AreaCoverageScore {
    public: AreaCoverageScore();
    public: ~AreaCoverageScore();

    public: void GetParameters(sdf::ElementPtr worldSDF, sdf::ElementPtr ownSDF);
    public: void SetModels(std::map<std::string, gazebo::physics::ModelPtr> _models);
    public: void UpdateStates();
    public: void SetWorld(gazebo::physics::WorldPtr _worldPtr);

    protected: std::map<std::string, gazebo::physics::ModelPtr> models;

    protected: std::map<std::string, std::pair<float, float> > modelPerceptionHeights;
    protected: std::map<std::string, ignition::math::Frustum*> modelFrustums;

    protected: std::vector<Polygon_2> perceptedPolygons;
    protected: int updateRate;
    protected: gazebo::physics::WorldPtr worldPtr;
    protected: gazebo::common::Time lastUpdateTime;

    protected: void MyJoin(Polygon_2 p1, Polygon_2 p2);
    protected: void MyJoin2(Polygon_2 p1, Polygon_2 p2);

    protected: bool IsInside(Polygon_2& polygon, const Point_2& point);
    protected: bool IsIntersection(const Segment_2& seg1, const Segment_2& seg2, Point_2& intersection);
    protected: bool IsIntersection(const Segment_2& seg, const Polygon_2& polygon, Point_2& intersection, EdgeIterator& polygonEi);

    public: double CalculateScore();

    protected: uint8_t map[5000][5000][100];

//    protected: General_polygon_set_2 perceptedPolygonSet;

//    protected: std::vector<std::vector<ignition::math::Vector2d> > perceptedRectangles;
};

#endif //SURUIHA_GAZEBO_PLUGINS_AREA_COVERAGE_SCORE_H
