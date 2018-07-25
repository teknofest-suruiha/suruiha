//
// Created by okan on 25.07.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_BATTERY_MANAGER_H
#define SURUIHA_GAZEBO_PLUGINS_BATTERY_MANAGER_H

#include <sdf/sdf.hh>
#include <vector>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/Time.hh>
#include <ros/ros.h>

namespace gazebo {
    class BatteryManager {
    public: BatteryManager();
    public: virtual ~BatteryManager();
    public: void GetParams(sdf::ElementPtr sdf, std::string uavName);
    public: void SetJoints(const std::vector<physics::JointPtr>& joints_);
    public: void SetWorld(physics::WorldPtr world_);
    public: void UpdateStates(bool isUAVActive);
    public: void ReplaceBattery();
    public: double GetRemaining();

    protected: ros::NodeHandle* rosNode;
    protected: ros::Publisher pub;
    protected: void PublishBatteryState();

    protected: physics::WorldPtr world;
    protected: std::vector<physics::JointPtr> joints;

    protected: double capacity;
    protected: double remaining;
    protected: double forceFactor;

    protected: double publishRate;
    protected: common::Time lastPublishTime;
    };
}

#endif //SURUIHA_GAZEBO_PLUGINS_BATTERY_MANAGER_H
