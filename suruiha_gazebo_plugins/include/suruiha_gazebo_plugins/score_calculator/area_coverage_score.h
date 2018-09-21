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
#include <gazebo/common/Time.hh>
#include <nav_msgs/OccupancyGrid.h>
#include <ignition/math/Vector2.hh>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <ros/ros.h>

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

    protected: int updateRate;
    protected: gazebo::physics::WorldPtr worldPtr;
    protected: gazebo::common::Time lastUpdateTime;

    protected: void AddOccupancy(geometry_msgs::Polygon& polygon);
    protected: geometry_msgs::Point32 ToPoint32(ignition::math::Vector2d& vec);

    protected: ros::NodeHandle* rosNode;
    protected: ros::Publisher visPub;
    protected: std::string visTopicName;
    protected: bool isVisualization;
    protected: int visualizationUpdateRate;
    protected: gazebo::common::Time lastVisUpdateTime;
    protected: std::pair<float, float> GetRosParams(ros::NodeHandle* node, std::string modelName);

    protected: double sumOccupancy;

//    protected: void MyJoin(Polygon_2 p1, Polygon_2 p2);
//    protected: void MyJoin2(Polygon_2 p1, Polygon_2 p2);
//
//    protected: bool IsInside(Polygon_2& polygon, const Point_2& point);
//    protected: bool IsIntersection(const Segment_2& seg1, const Segment_2& seg2, Point_2& intersection);
//    protected: bool IsIntersection(const Segment_2& seg, const Polygon_2& polygon, Point_2& intersection, EdgeIterator& polygonEi);

//    public: nav_msgs::OccupancyGrid::ConstPtr GetOccupancyMap();
    public: double CalculateScore();

    protected: nav_msgs::OccupancyGrid occupancyGridMap;
    protected: std::vector<geometry_msgs::Polygon> perceptedPolygons;

    protected: double scoreFactor;
    public: double GetFactor();

};

#endif //SURUIHA_GAZEBO_PLUGINS_AREA_COVERAGE_SCORE_H
