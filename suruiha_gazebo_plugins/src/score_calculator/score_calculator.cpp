//
// Created by okan on 11.07.2018.
//

#include <suruiha_gazebo_plugins/score_calculator/score_calculator.h>
#include <suruiha_gazebo_plugins/util/util.h>
#include <gazebo/common/common.hh>
#include <visualization_msgs/Marker.h>

namespace gazebo{

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(ScoreCalculator);

    ScoreCalculator::ScoreCalculator() {
        isVisualization = false;
        markerCounter = 0;
    }

    ScoreCalculator::~ScoreCalculator() {

    }

    void ScoreCalculator::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
        // load models in to the map
        Util::GetModels(models, 6, "zephyr", _parent);
        Util::GetModels(models, 6, "iris", _parent);

        // get sensor parameters from models itself not to duplicate parameters
        GetParameters(_parent->SDF());
        if (_sdf->GetElement("visualization")->Get<bool>("active")) {
            isVisualization = true;
        }

        if (isVisualization) {
            rosNode = new ros::NodeHandle("");
            std::string topicName = _sdf->GetElement("visualization")->Get<std::string>("topic_name");
            visPub = rosNode->advertise<visualization_msgs::MarkerArray>(topicName, 1);
        }


        // New Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&ScoreCalculator::UpdateStates, this));
    }

    void ScoreCalculator::GetParameters(sdf::ElementPtr worldSdf) {
        // we assume that every uav can have different maximum height for perception

        // iterate over all models and if we have model with name iris or zephyr get its max_height uav_sensor
        sdf::ElementPtr modelSdf = worldSdf->GetElement("model");
        while (modelSdf != NULL) {
            std::string modelName = modelSdf->GetAttribute("name")->GetAsString();
            gzdbg << "checking model:" << modelName << std::endl;
            // we would like to find models having name zephyr or iris
            if (modelName.find("zephyr") != std::string::npos || modelName.find("iris") != std::string::npos) {
                sdf::ElementPtr pluginSdf = modelSdf->GetElement("plugin");
                while (pluginSdf != NULL) {
                    std::string pluginName = pluginSdf->GetAttribute("name")->GetAsString();
                    // we would like to find zephyr_controller or iris_controller plugins
                    if (pluginName.find("zephyr") != std::string::npos || pluginName.find("iris") != std::string::npos) {
                        gzdbg << "checking plugin:" << pluginName << std::endl;
                        float maxHeight = pluginSdf->GetElement("uav_sensor")->Get<float>("max_height");
                        modelMaxPerceptionHeights.insert(std::pair<std::string, float>(modelName, maxHeight));

                        // create model frustum
                        ignition::math::Frustum* frustum = new ignition::math::Frustum();
                        frustum->SetFOV(pluginSdf->GetElement("uav_sensor")->Get<double>("hfov"));
                        frustum->SetAspectRatio(pluginSdf->GetElement("uav_sensor")->Get<double>("aspect_ratio"));
                        frustum->SetNear(pluginSdf->GetElement("uav_sensor")->Get<double>("near"));
                        frustum->SetFar(pluginSdf->GetElement("uav_sensor")->Get<double>("far"));

                        modelFrustums.insert(std::pair<std::string, ignition::math::Frustum*>(modelName, frustum));
                        gzdbg << "model:" << modelName << " max_height:" << maxHeight << std::endl;
                        break;
                    }
                    pluginSdf = pluginSdf->GetNextElement("plugin");
                }
            }
            modelSdf = modelSdf->GetNextElement("model");
        }
    }


    void ScoreCalculator::UpdateStates() {
        boost::mutex::scoped_lock lock(updateMutex);

        std::map<std::string, physics::ModelPtr>::iterator it;
        for (it = models.begin(); it != models.end(); it++) {
            float maxHeight = modelMaxPerceptionHeights[it->first];
            ignition::math::Pose3d uavPose = it->second->WorldPose();

            if (uavPose.Pos().Z() < maxHeight && uavPose.Pos().Z() > 1) {

                ignition::math::Frustum *frustum = modelFrustums[it->first];
                ignition::math::Pose3d frustumPose(uavPose);

                // correct the pose of the frustum and set it towards the bottom of the uav
                ignition::math::Vector3d rot = frustumPose.Rot().Euler();
                // rot.X(rot.X() + 1.57079);
                rot.Y(rot.Y() + 1.57079);
                rot.Z(rot.Z() - 1.57079);

                frustumPose.Rot().Euler(rot);

                frustum->SetPose(frustumPose);

                // calculate the intersection of frustum lines on the floor plane
                float farWidth = tan(frustum->FOV().Radian()/2) * frustumPose.Pos().Z();
                float farHeight = farWidth / frustum->AspectRatio();

                ignition::math::Vector2d uavPoint(uavPose.Pos().X(), uavPose.Pos().Y());

                // calculate rectangle points from frustum width and height
                std::vector<ignition::math::Vector2d> rectanglePoints(4);
                ignition::math::Vector2d point(-farWidth, -farHeight);
                Util::Rotate(point, uavPose.Rot().Euler().Z());
                point += uavPoint;
                rectanglePoints.push_back(point);

                ignition::math::Vector2d point1(-farWidth, farHeight);
                Util::Rotate(point1, uavPose.Rot().Euler().Z());
                point1 += uavPoint;
                rectanglePoints.push_back(point1);

                ignition::math::Vector2d point2(farWidth, farHeight);
                Util::Rotate(point2, uavPose.Rot().Euler().Z());
                point2 += uavPoint;
                rectanglePoints.push_back(point2);

                ignition::math::Vector2d point3(farWidth, -farHeight);
                Util::Rotate(point3, uavPose.Rot().Euler().Z());
                point3 += uavPoint;
                rectanglePoints.push_back(point3);

                perceptedRectangles.push_back(rectanglePoints);

                if (isVisualization) {
                    // create marker message
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "world";
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "area";
                    marker.id = markerCounter;
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.pose.position.x = uavPose.Pos().X();
                    marker.pose.position.y = uavPose.Pos().Y();
                    marker.pose.position.z = 0;
                    marker.pose.orientation.x = 0;
                    marker.pose.orientation.y = 0;
                    marker.pose.orientation.z = 0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 2*farWidth;
                    marker.scale.y = 2*farHeight;
                    marker.scale.z = 0.05; // 5cm height
                    marker.color.r = 0.0f;
                    marker.color.g = 1.0f;
                    marker.color.b = 0.0f;
                    marker.color.a = 1.0f;
                    marker.lifetime = ros::Duration();
                    markerCounter++;
                    if ((markerCounter%100) == 0) {
                        markersCache.markers.clear();
                        markersCache.markers.push_back(marker);
                        visPub.publish(markersCache);
                    }
                }
            }
        }

    }

}
