//
// Created by okan on 25.07.2018.
//

#include <suruiha_gazebo_plugins/battery/battery_manager.h>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <suruiha_gazebo_plugins/util/util.h>
#include <suruiha_gazebo_plugins/UAVBattery.h>

namespace gazebo {
    BatteryManager::BatteryManager() {
    }

    BatteryManager::~BatteryManager() {

    }

    void BatteryManager::UpdateStates(bool isUAVActive) {
        // reduce battery if the uav is active
        if (isUAVActive) {
            for (unsigned int i = 0; i < joints.size(); i++) {
                remaining -= fabs(joints.at(i)->GetForce(0)) * forceFactor;
                if (remaining < 0) {
                    remaining = 0;
                }
            }
        }

        common::Time currTime = world->SimTime();
        double dt = (currTime - lastPublishTime).Double() * 1000; // miliseconds
        if (dt > publishRate) {
            PublishBatteryState();
            lastPublishTime = currTime;
        }
    }


    void BatteryManager::GetParams(sdf::ElementPtr sdf, std::string uavName) {
        // get battery capacity of the flight
        // get the force factor
        // set the remaining as the capaticy
        // note that for zephyr plane the normal flight throttle has approximately 0.7 newton and
        // if you use full power it will create approximately 1.35 newton
        capacity = sdf->Get<double>("capacity");
        forceFactor = sdf->Get<double>("force_factor");
        publishRate = sdf->Get<double>("publish_rate");
        ReplaceBattery();

        gzdbg << "just before creating ros node" << std::endl;
        // create ros node and battery state publisher
        rosNode = Util::CreateROSNodeHandle("");
        std::stringstream ss;
        ss << uavName << "_battery";
        gzdbg << "ros node created with address:" << rosNode << " topicname:" << ss.str() << std::endl;
        pub = rosNode->advertise<suruiha_gazebo_plugins::UAVBattery>(ss.str(), 1);
        gzdbg << "publisher is created with topic name:" << ss.str() << std::endl;
    }

    void BatteryManager::SetJoints(const std::vector<physics::JointPtr>& joints_) {
        joints = joints_;
    }

    void BatteryManager::SetWorld(physics::WorldPtr world_) {
        world = world_;
    }

    void BatteryManager::ReplaceBattery() {
        // restore the capacity
        remaining = capacity;
    }

    void BatteryManager::PublishBatteryState() {
        suruiha_gazebo_plugins::UAVBattery batteryMsg;
        batteryMsg.capacity = capacity;
        batteryMsg.remaining = remaining;
        pub.publish(batteryMsg);
    }

    double BatteryManager::GetRemaining() {
        return remaining;
    }
}