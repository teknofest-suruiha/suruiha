//
// Created by okan on 12.07.2018.
//
#include <suruiha_gazebo_plugins/comm_manager/comm_manager.h>

using namespace suruiha_gazebo_plugins;

namespace gazebo {
    GZ_REGISTER_WORLD_PLUGIN(CommManager);

    CommManager::CommManager() {
    }

    CommManager::~CommManager() {
    }

    void CommManager::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
        worldPtr = _parent;

        // load zephyr and iris models currently active on the simulation
        GetModels("zephyr", 6);
        GetModels("iris", 6);

        SetParameters(_sdf);

        // create ROS node handle
        rosNode = new ros::NodeHandle();

        requestSub = rosNode->subscribe(commRequestTopic.c_str(), 100, &CommManager::CommRequest, this);

        // create message broadcast publishers for each model separately
        std::map<std::string, physics::ModelPtr>::iterator it;
        for (it = models.begin(); it != models.end(); it++) {
            std::stringstream ss;
            ss << commBroadcastTopic << "_" << it->first;
            gzdbg << "created broadcast topic name:" << ss.str() << std::endl;
            ros::Publisher pub = rosNode->advertise<UAVMessage>(ss.str().c_str(), 100);
            modelPublishers.insert(std::pair<std::string, ros::Publisher>(it->first, pub));
        }

        updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&CommManager::UpdateStates, this));
    }

    void CommManager::GetModels(std::string baseModelName, int maxCount)
    {
        int i = 0;
        for (i = 0; i < maxCount; i++) {
            std::stringstream ss;
            ss << baseModelName << i;
            gzdbg << "Checking model with name " << ss.str() << std::endl;
            physics::ModelPtr modelPtr = worldPtr->ModelByName(ss.str());
            if (modelPtr == NULL) {
                break;
            } else {
                models.insert(std::pair<std::string, physics::ModelPtr>(ss.str(), modelPtr));
            }
        }
        gzdbg << "We have " << i << " many " << baseModelName << " models" << std::endl;
    }

    void CommManager::SetParameters(sdf::ElementPtr sdf) {
        commRequestTopic = sdf->Get<std::string>("comm_request_topic");
        commBroadcastTopic = sdf->Get<std::string>("comm_broadcast_topic");
        commDistance = sdf->Get<double>("comm_distance");
    }

    void CommManager::CommRequest(const UAVMessage::ConstPtr& uavMessage) {
//        gzdbg << "comm request msg is taken from " << uavMessage->sender << " msg:" << uavMessage->msg << std::endl;
        messageRequestQueue.push(*uavMessage);
    }

    void CommManager::UpdateStates() {
        boost::mutex::scoped_lock lock(updateMutex);
        // process the latest message requests
        while (messageRequestQueue.size() > 0) {
            UAVMessage msg = messageRequestQueue.front();
            messageRequestQueue.pop();
            if (models.find(msg.sender) != models.end()) {
                physics::ModelPtr model = models[msg.sender];
                ignition::math::Pose3d uavPose = model->WorldPose();
                std::map<std::string, physics::ModelPtr>::iterator it;
                for (it = models.begin(); it != models.end(); it++) {
                    // do not send msg to itself
                    if (msg.sender != it->first) {
                        ignition::math::Pose3d otherPose = it->second->WorldPose();
                        otherPose -= uavPose;
                        double dist = otherPose.Pos().Length();
                        if (dist <= commDistance) {
                            modelPublishers[it->first].publish(msg);
                        }
                    }
                }
            }
        }
    }
}