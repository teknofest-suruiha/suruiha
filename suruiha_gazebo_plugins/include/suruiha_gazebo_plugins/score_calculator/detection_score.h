//
// Created by okan on 18.07.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_DETECTION_SCORE_H
#define SURUIHA_GAZEBO_PLUGINS_DETECTION_SCORE_H

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
#include <std_msgs/String.h>

class DetectionScore {
    public: DetectionScore();
    public: ~DetectionScore();

    public: void GetParameters(sdf::ElementPtr ownSDF);
    public: void UpdateStates();
    public: void SetWorld(gazebo::physics::WorldPtr _worldPtr);

    protected: gazebo::physics::WorldPtr worldPtr;

    protected: ros::NodeHandle* rosNode;
    protected: ros::Subscriber detectionSub;
    public: double GetFactor();

    protected: void OnDetection(std_msgs::String::ConstPtr msg);

    public: double CalculateScore();

    protected: double penalty;
    protected: double scoreFactor;
    protected: double falseDetectionPenalty;
    protected: double baseScore;

    protected: std::map<std::string, double> detectionStartTimes;
    protected: std::map<std::string, double> detectionEndTimes;
    protected: std::map<std::string, double> uavDetectionTimes;


};

#endif //SURUIHA_GAZEBO_PLUGINS_DETECTION_SCORE_H
