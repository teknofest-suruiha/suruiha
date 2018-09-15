//
// Created by okan on 16.07.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_UAV_SENSOR_H
#define SURUIHA_GAZEBO_PLUGINS_UAV_SENSOR_H
#include <ros/ros.h>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/math/Pose3.hh>
#include <set>

namespace gazebo {
    class UAVSensor {
        public: UAVSensor();
        public: ~UAVSensor();

        public: void setPublisher(ros::Publisher pub);
//        public: void setWorldPtr(physics::WorldPtr world);
//        public: void setFOV(float hfov, float vfov);
//        public: void setOperatingHeight(float min, float max);
        public: void loadParams(sdf::ElementPtr _sdf);
        public: void setModels(physics::WorldPtr worldPtr);

        public: void sense(const ignition::math::Pose3d uavPose);
        public: void getParams(ros::NodeHandle* node, std::string modelName);

        protected: ros::Publisher sensorPublisher;
//        protected: physics::WorldPtr worldPtr;

        protected: ignition::math::Frustum frustum;
        protected: float maxHeight;
        protected: float minHeight;
        protected: std::vector<physics::ModelPtr> models;

        protected: std::set<std::string> doNotSenseModelNames;

        protected: std::map<std::string, double> actorStartTimes;
        protected: std::map<std::string, double> actorEndTimes;

        protected: physics::WorldPtr worldPtr;

//        protected: int senseCount;
    };
}
#endif //SURUIHA_GAZEBO_PLUGINS_UAV_SENSOR_H
