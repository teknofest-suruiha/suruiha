//
// Created by okan on 01.07.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_AIR_TRAFFIC_CONTROLLER_H
#define SURUIHA_GAZEBO_PLUGINS_AIR_TRAFFIC_CONTROLLER_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <suruiha_gazebo_plugins/AirTraffic.h>
#include <suruiha_gazebo_plugins/air_traffic_controller/runway.h>

namespace gazebo {
    class AirTrafficController : public WorldPlugin
    {
    public: AirTrafficController();
    public: ~AirTrafficController();

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    protected: void GetModelsInitialPosition(std::string baseModelName, int maxIndex);
    protected: void SetParameters(sdf::ElementPtr sdf);

    protected: bool AirTrafficService(suruiha_gazebo_plugins::AirTraffic::Request& cmd, suruiha_gazebo_plugins::AirTraffic::Response& resp);
    protected: virtual void UpdateStates();

    protected: physics::WorldPtr worldPtr;

    protected: std::map<std::string, ignition::math::Pose3d> initialPoses;
//    protected: std::vector<physics::ModelPtr> zephyr_models;
//    protected: std::vector<physics::ModelPtr> iris_models;

    private: event::ConnectionPtr updateConnection;
    private: boost::mutex updateMutex;
    private: boost::mutex updateMutex2;
    private: boost::mutex updateMutex3;

    protected: ros::NodeHandle* rosNode;
    protected: ros::ServiceServer serviceServer;

    protected: RunWay runway;

    protected: ignition::math::Pose3d takeOffPose;
    protected: ignition::math::Pose3d landingStartPose;
    protected: ignition::math::Pose3d landingEndPose;

    protected: ignition::math::Vector3d landingBottomLeft;
    protected: ignition::math::Vector3d landingUpperRight;

    protected: double takeOffHeightThreshold;
    protected: double takeOffDistanceThreshold;

    protected: double landingHeightThreshold;
    protected: double landingVelocityThreshold;

    private: transport::NodePtr node;
    private: transport::PublisherPtr uavStatusPub;
    private: transport::PublisherPtr batteryReplacePub;
    private: std::map<std::string, bool> isUAVActive;
    protected: void SetUAVStatus(std::string uavName, bool isActive);

    protected: std::set<std::string> justLanded;


    };
}

#endif //SURUIHA_GAZEBO_PLUGINS_AIR_TRAFFIC_CONTROLLER_H
