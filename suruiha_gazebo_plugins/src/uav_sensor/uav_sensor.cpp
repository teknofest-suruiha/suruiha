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
        minHeight = _sdf->Get<float>("min_height") * 0.8;
        maxHeight = _sdf->Get<float>("max_height") * 1.2f;
    }

    void UAVSensor::setModels(physics::WorldPtr world) {
        for (unsigned int i = 0; i < world->Models().size(); i++) {
            physics::ModelPtr model = world->Models()[i];
            if (doNotSenseModelNames.find(model->GetName()) == doNotSenseModelNames.end()) {
                models.push_back(model);
                gzdbg << "model:" << model->GetName() << " is added." << std::endl;
                gzdbg << "gazebo model.x:" << models[models.size()-1]->WorldPose().Pos().X() << " y:" << models[models.size()-1]->WorldPose().Pos().Y() <<
                      " .z:" << models[models.size()-1]->WorldPose().Pos().Z() << std::endl;
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

    void UAVSensor::sense(const ignition::math::Pose3d& uavPose) {
//        if (uavPose.Pos().Z() > minHeight && uavPose.Pos().Z() < maxHeight) {
            // correct the frustum orientation and set facing the downward
            ignition::math::Pose3d frustumPose(uavPose);
            ignition::math::Vector3d rot = frustumPose.Rot().Euler();
            // rot.X(rot.X() + 1.57079);
            rot.Y(rot.Y() + 1.57079);
            rot.Z(rot.Z() - 1.57079);
            frustumPose.Rot().Euler(rot);
            // set frustum pose
            frustum.SetPose(frustumPose);
            suruiha_gazebo_plugins::UAVSensorMessage msg;
            for (unsigned int i = 0; i < models.size(); i++) {
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

                    // mesaure the distance of the object to UAV
                    double dist = Util::CalDist(models[i]->WorldPose(), uavPose);
                    if (dist >= minHeight && dist <= maxHeight) {
                        msg.types.push_back(0); // for now we have only human type
                        geometry_msgs::Pose rosPose = Util::FromIgnitionPose(models[i]->WorldPose());
                        msg.poses.push_back(rosPose);
                        msg.names.push_back(models[i]->GetName());
                        gzdbg << "sensed model:" << models[i]->GetName() << std::endl;
                    }
                }
            }
            // send message if there is at least one sensed object
            if (msg.types.size() > 0) {
                sensorPublisher.publish(msg);
            }
        }

}