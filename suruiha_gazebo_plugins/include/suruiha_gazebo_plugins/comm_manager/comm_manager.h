//
// Created by okan on 12.07.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_COMM_MANAGER_H
#define SURUIHA_GAZEBO_PLUGINS_COMM_MANAGER_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <suruiha_gazebo_plugins/UAVMessage.h>
#include <queue>

namespace gazebo {
    class CommManager : public WorldPlugin {
    public: CommManager();
    public: ~CommManager();

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    public: void CommRequest(const suruiha_gazebo_plugins::UAVMessage::ConstPtr& uavMessage);

    protected: void GetModels(std::string baseModelName, int maxCount);
    protected: void SetParameters(sdf::ElementPtr sdfElement);

    protected: ros::NodeHandle* rosNode;
    protected: physics::WorldPtr worldPtr;
    protected: std::map<std::string, physics::ModelPtr> models;
    protected: std::map<std::string, ros::Publisher> modelPublishers;

    protected: event::ConnectionPtr updateConnection;
    protected: boost::mutex updateMutex;
    protected: virtual void UpdateStates();

    protected: std::string commRequestTopic;
    protected: std::string commBroadcastTopic;
    protected: double commDistance; // in meters

    protected: std::queue<suruiha_gazebo_plugins::UAVMessage> messageRequestQueue;

    protected: ros::Publisher broadcastPub;
    protected: ros::Subscriber requestSub;

    };
}

#endif //SURUIHA_GAZEBO_PLUGINS_COMM_MANAGER_H
