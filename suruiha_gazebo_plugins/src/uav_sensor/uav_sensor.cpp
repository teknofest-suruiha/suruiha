//
// Created by okan on 16.07.2018.
//

#include <suruiha_gazebo_plugins/uav_sensor/uav_sensor.h>
#include <gazebo/common/common.hh>
#include <suruiha_gazebo_plugins/util/util.h>
#include <suruiha_gazebo_plugins/UAVSensorMessage.h>
#include <geometry_msgs/Pose.h>

namespace gazebo {
    UAVSensor::UAVSensor() {
//        senseCount = 0;
        doNotSenseModelNames.insert("ground_plane");
        doNotSenseModelNames.insert("zephyr0");
        doNotSenseModelNames.insert("zephyr1");
        doNotSenseModelNames.insert("zephyr2");
        doNotSenseModelNames.insert("zephyr3");
        doNotSenseModelNames.insert("zephyr4");
        doNotSenseModelNames.insert("zephyr5");
        doNotSenseModelNames.insert("iris0");
        doNotSenseModelNames.insert("iris1");
        doNotSenseModelNames.insert("iris2");
        doNotSenseModelNames.insert("iris3");
        doNotSenseModelNames.insert("iris4");
        doNotSenseModelNames.insert("iris5");
    }

    UAVSensor::~UAVSensor() {
//        gzdbg << "uavsensor sense_count=" << senseCount << std::endl;
    }

    void UAVSensor::setPublisher(ros::Publisher pub) {
        sensorPublisher = pub;
    }

//    void UAVSensor::setWorldPtr(physics::WorldPtr world) {
//        worldPtr = world;
//    }

    void UAVSensor::loadParams(sdf::ElementPtr _sdf) {
        frustum.SetFOV(_sdf->Get<double>("hfov"));
        frustum.SetAspectRatio(_sdf->Get<double>("aspect_ratio"));
        frustum.SetFar(_sdf->Get<double>("far"));
        frustum.SetNear(_sdf->Get<double>("near"));

        // add small margin to make sure that when the uav see the part of the object
        // the distance of the object is in view
        minHeight = _sdf->Get<float>("min_height");
        maxHeight = _sdf->Get<float>("max_height");
    }

    void UAVSensor::getParams(ros::NodeHandle* node, std::string modelName){
        XmlRpc::XmlRpcValue scenarioParam;
        node->getParam("scenario", scenarioParam);
        XmlRpc::XmlRpcValue uavs = scenarioParam["uavs"];
        bool isParamSet = false;
        for (unsigned int i = 0; i < uavs.size(); i++) {
            int uav_index = static_cast<int>(uavs[i]["index"]);
            std::string type = static_cast<std::string>(uavs[i]["type"]);
            std::stringstream ss;
            ss << type << uav_index;
            if (modelName == ss.str()) {
                minHeight = static_cast<int>(uavs[i]["sensor"]["min_height"]);
                maxHeight = static_cast<int>(uavs[i]["sensor"]["max_height"]);
                frustum.SetFar(maxHeight);
                int battery = static_cast<int>(uavs[i]["battery_capacity"]);
                gzdbg << "getParams min_height:" << minHeight << " max_height:" <<
                      maxHeight << std::endl;
                isParamSet = true;
                break;
            }
        }
        if (!isParamSet) {
            gzdbg << "ERROR cannot set minHeight and maxHeight" << std::endl;
        }
    }

    void UAVSensor::setModels(physics::WorldPtr world) {
        worldPtr = world;
        for (unsigned int i = 0; i < worldPtr->Models().size(); i++) {
            physics::ModelPtr model = worldPtr->Models()[i];
            if (doNotSenseModelNames.find(model->GetName()) == doNotSenseModelNames.end()) {
                models.push_back(model);
//                gzdbg << "model:" << model->GetName() << " is added." << std::endl;
//                gzdbg << "gazebo model.x:" << models[models.size()-1]->WorldPose().Pos().X() << " y:" << models[models.size()-1]->WorldPose().Pos().Y() <<
//                      " .z:" << models[models.size()-1]->WorldPose().Pos().Z() << std::endl;
                if (model->GetName().find("terrorist") != std::string::npos) {
                    std::pair<double, double> timing = Util::GetStartEndTimeOfActor(model->GetSDF());
                    actorStartTimes.insert(std::pair<std::string, double>(model->GetName(), timing.first));
                    actorEndTimes.insert(std::pair<std::string, double>(model->GetName(), timing.second));
                }
            }
        }
    }
//
//    void UAVSensor::setFOV(float hfov, float vfov) {
//        hFOV = hfov;
//        vFOV = vfov;
//    }
//
//    void UAVSensor::setOperatingHeight(float min, float max) {
//        minHeight = min;
//        maxHeight = max;
//    }

    void UAVSensor::sense(const ignition::math::Pose3d uavPose) {
        if (uavPose.Pos().Z() > minHeight && uavPose.Pos().Z() < maxHeight) {
            ignition::math::Pose3d expectedPose(uavPose);
            ignition::math::Vector3d rot = expectedPose.Rot().Euler();
            rot.X(0.0);
            rot.Y(1.5707);
            rot.Z(uavPose.Rot().Euler().Z() - 1.5707);
            expectedPose.Rot().Euler(rot);
            // set frustum pose
            frustum.SetPose(expectedPose);
            suruiha_gazebo_plugins::UAVSensorMessage msg;
            double currTime = worldPtr->SimTime().Double();
            for (unsigned int i = 0; i < models.size(); i++) {
                // check the timing for the actor
                if (actorStartTimes.find(models[i]->GetName()) != actorStartTimes.end()) {
                    if (currTime < actorStartTimes[models[i]->GetName()] ||
                        currTime > actorEndTimes[models[i]->GetName()]) {
                        // actor is not active do not sense the actor
                        continue;
                    }
                }

                bool modelInFrustum = false;
                // check position of terorists
                // other models will be checked by bounding box
                if (models[i]->GetName().find("terrorist") != std::string::npos) {
                    ignition::math::Pose3d pose = models[i]->WorldPose();
                    if (frustum.Contains(pose.Pos())) {
                        modelInFrustum = true;
                    }
                } else {
                    ignition::math::Box box = models[i]->CollisionBoundingBox();
                    if (frustum.Contains(box)) {
                        modelInFrustum = true;
                    }
                }
                if (modelInFrustum) {
                    msg.types.push_back(0); // for now we do not use sensed object type index
                    geometry_msgs::Pose rosPose = Util::FromIgnitionPose(models[i]->WorldPose());
                    msg.poses.push_back(rosPose);
                    msg.names.push_back(models[i]->GetName());
//                    gzdbg << "sensed model:" << models[i]->GetName() << std::endl;
                }
            }
            // send message if there is at least one sensed object
            if (msg.types.size() > 0) {
                sensorPublisher.publish(msg);
            }
        }
    }

}