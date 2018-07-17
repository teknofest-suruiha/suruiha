//
// Created by okan on 16.07.2018.
//

#include <suruiha_gazebo_plugins/uav_sensor/uav_sensor.h>
#include <gazebo/common/common.hh>
#include <suruiha_gazebo_plugins/util/util.h>
#include <suruiha_gazebo_plugins/UAVSensorMessage.h>
#include <geometry_msgs/Pose.h>

namespace gazebo {
    UAVSensor::UAVSensor() {}
    UAVSensor::~UAVSensor() {}

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
        minHeight = _sdf->Get<float>("min_height");
        maxHeight = _sdf->Get<float>("max_height");
    }

    void UAVSensor::setModels(physics::WorldPtr world) {
        for (unsigned int i = 0; i < world->Models().size(); i++) {
            physics::ModelPtr model = world->Models()[i];
            // every model name starting with human will be added to the sensor
//            if (model->GetName().find("human") == 0) {
                models.push_back(model);
            gzdbg << "model:" << model->GetName() << " is added." << std::endl;
//            }
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
        if (uavPose.Pos().Z() > minHeight && uavPose.Pos().Z() < maxHeight) {
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
                ignition::math::Pose3d pose = models[i]->WorldPose();
                if (frustum.Contains(pose.Pos())) {
                    //TODO: use enumeration for percepted object types
                    msg.types.push_back(0); // for now we have only human type
                    geometry_msgs::Pose rosPose = Util::fromIgnitionPose(pose);
                    msg.poses.push_back(rosPose);
                    msg.names.push_back(models[i]->GetName());
                    gzdbg << "sensed model:" << models[i]->GetName() << std::endl;
                }
            }
            sensorPublisher.publish(msg);
        }
    }

}