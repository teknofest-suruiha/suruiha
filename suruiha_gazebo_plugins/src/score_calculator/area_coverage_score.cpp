//
// Created by okan on 18.07.2018.
//

#include <suruiha_gazebo_plugins/score_calculator/area_coverage_score.h>
#include <gazebo/common/Console.hh>
#include <suruiha_gazebo_plugins/util/util.h>
#include <chrono>
#include <suruiha_gazebo_plugins/score_calculator/geometry.h>
#include <suruiha_gazebo_plugins/score_calculator/coordinate_conversions.h>
#include <geometry_msgs/Point32.h>

using namespace gazebo;
//using namespace std::chrono;

AreaCoverageScore::AreaCoverageScore() {
    sumOccupancy = 0;
}

AreaCoverageScore::~AreaCoverageScore() {
}

void AreaCoverageScore::GetParameters(sdf::ElementPtr worldSdf, sdf::ElementPtr ownSDF) {
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
                    float minHeight = pluginSdf->GetElement("uav_sensor")->Get<float>("min_height");

                    modelPerceptionHeights.insert(std::pair<std::string, std::pair<float, float> >
                                                          (modelName, std::pair<float, float>(minHeight, maxHeight)));

                    // create model frustum
                    ignition::math::Frustum* frustum = new ignition::math::Frustum();
                    frustum->SetFOV(pluginSdf->GetElement("uav_sensor")->Get<double>("hfov"));
                    frustum->SetAspectRatio(pluginSdf->GetElement("uav_sensor")->Get<double>("aspect_ratio"));
                    frustum->SetNear(pluginSdf->GetElement("uav_sensor")->Get<double>("near"));
                    frustum->SetFar(pluginSdf->GetElement("uav_sensor")->Get<double>("far"));

                    modelFrustums.insert(std::pair<std::string, ignition::math::Frustum*>(modelName, frustum));
                    gzdbg << "model:" << modelName << " min_height:" << minHeight << " max_height:" << maxHeight << std::endl;
                    break;
                }
                pluginSdf = pluginSdf->GetNextElement("plugin");
            }
        }
        modelSdf = modelSdf->GetNextElement("model");
    }

    // create occupancy map from parameters taken from simulator

    // width and height of the total area of the map in meters
    float height = ownSDF->Get<float>("height");
    float width = ownSDF->Get<float>("width");
    scoreFactor = ownSDF->Get<double>("factor");

    updateRate = ownSDF->Get<int>("update_rate");
    nav_msgs::MapMetaData metaData;
    metaData.resolution = ownSDF->Get<float>("resolution");
    metaData.height = floor(height / metaData.resolution);
    metaData.width = floor(width / metaData.resolution);
    gzdbg << "grid map width:" << metaData.width << " height:" << metaData.height << std::endl;

    // set the origin pose of the grid map.
    // We set the real position of the 0,0 cell as the geometric center of the whole area
    metaData.origin.position.x = -width/2;
    metaData.origin.position.y = -height/2;
    metaData.origin.position.z = 0;

    occupancyGridMap.info = metaData;
    // resize and initialize with 0
    occupancyGridMap.data.resize(metaData.width*metaData.height, 0);

    // visualization
    isVisualization = ownSDF->GetElement("visualization")->Get<bool>("active");
    gzdbg << "visualization active:" << isVisualization << std::endl;
    if (isVisualization) {
        std::string visTopicName = ownSDF->GetElement("visualization")->Get<std::string>("topic_name");
        gzdbg << "visualization topicname:" << visTopicName << std::endl;
        rosNode = new ros::NodeHandle("");
        visPub = rosNode->advertise<nav_msgs::OccupancyGrid>(visTopicName.c_str(), 1);
        visualizationUpdateRate = ownSDF->GetElement("visualization")->Get<int>("update_rate");
    }
}

void AreaCoverageScore::SetModels(std::map<std::string, physics::ModelPtr> _models) {
    models = _models;
}

void AreaCoverageScore::SetWorld(physics::WorldPtr _worldPtr) {
    worldPtr = _worldPtr;
}

void AreaCoverageScore::UpdateStates() {
    common::Time currTime = worldPtr->SimTime();
    double dt_ = (currTime - lastUpdateTime).Double() * 1000; // miliseconds
    if (dt_ > updateRate) {
        std::map<std::string, physics::ModelPtr>::iterator it;
        for (it = models.begin(); it != models.end(); it++) {
            float minHeight = modelPerceptionHeights[it->first].first;
            float maxHeight = modelPerceptionHeights[it->first].second;
            ignition::math::Pose3d uavPose = it->second->WorldPose();

            if (uavPose.Pos().Z() < maxHeight && uavPose.Pos().Z() > minHeight) {

                ignition::math::Frustum *frustum = modelFrustums[it->first];
                ignition::math::Pose3d frustumPose(uavPose);

                // correct the pose of the frustum and set it towards the bottom of the uav
                ignition::math::Vector3d rot = frustumPose.Rot().Euler();

                frustumPose.Rot().Euler(rot);
                frustum->SetPose(frustumPose);

                // calculate the intersection of frustum lines on the floor plane
                float farWidth = tan(frustum->FOV().Radian() / 2) * frustumPose.Pos().Z();
                float farHeight = farWidth / frustum->AspectRatio();

                ignition::math::Vector2d uavPoint(uavPose.Pos().X(), uavPose.Pos().Y());

                // calculate rectangle points from frustum width and height
                geometry_msgs::Polygon polygon;
                ignition::math::Vector2d point(-farWidth, -farHeight);
                Util::Rotate(point, uavPose.Rot().Euler().Z());
                point += uavPoint;
                polygon.points.push_back(ToPoint32(point));

                ignition::math::Vector2d point1(-farWidth, farHeight);
                Util::Rotate(point1, uavPose.Rot().Euler().Z());
                point1 += uavPoint;
                polygon.points.push_back(ToPoint32(point1));

                ignition::math::Vector2d point2(farWidth, farHeight);
                Util::Rotate(point2, uavPose.Rot().Euler().Z());
                point2 += uavPoint;
                polygon.points.push_back(ToPoint32(point2));

                ignition::math::Vector2d point3(farWidth, -farHeight);
                Util::Rotate(point3, uavPose.Rot().Euler().Z());
                point3 += uavPoint;
                polygon.points.push_back(ToPoint32(point3));

//                perceptedPolygons.push_back(polygon);
                AddOccupancy(polygon);
            }
        }
        lastUpdateTime = currTime;
    }

    if (isVisualization) {
        double diff = (currTime - lastVisUpdateTime).Double() * 1000; // miliseconds
        if (diff > visualizationUpdateRate) {
            if (visPub.getNumSubscribers() > 0) {
                visPub.publish(occupancyGridMap);
            }
            lastVisUpdateTime = currTime;
        }
    }
}

void AreaCoverageScore::AddOccupancy(geometry_msgs::Polygon& polygon) {

//    gzdbg << "started to check cells in convex polygon:" << polygon << std::endl;
    std::set<occupancy_grid_utils::Cell> cells = occupancy_grid_utils::cellsInConvexPolygon(occupancyGridMap.info, polygon);
//    gzdbg << "cellsInConvexPolygon.size:" << cells.size() << std::endl;
    std::set<occupancy_grid_utils::Cell>::iterator it;
    for (it = cells.begin(); it != cells.end(); it++) {
        // convert to linear index to mark as occupied
        uint32_t index = occupancy_grid_utils::cellIndex(occupancyGridMap.info, *it);
        // validate the index
        if (index < 0 || index > occupancyGridMap.info.width*occupancyGridMap.info.height) {
            continue;
        }
        // set as occupied
        // for rviz visualization purposes we set data as 100.
        if (occupancyGridMap.data[index] != 100) {
            occupancyGridMap.data[index] = 100;
            sumOccupancy += 100;
        }
    }
}

double AreaCoverageScore::CalculateScore() {

//    int sum = 0;
//    std::vector<int8_t>::iterator it;
//    for (it = occupancyGridMap.data.begin(); it != occupancyGridMap.data.end(); it++) {
//        sum += *it;
//    }
    // return the value in between 0 and 100
    double numCells = occupancyGridMap.info.width*occupancyGridMap.info.height;
    return sumOccupancy/numCells;
}

geometry_msgs::Point32 AreaCoverageScore::ToPoint32(ignition::math::Vector2d& vec) {
    geometry_msgs::Point32 p;
    p.x = vec.X();
    p.y = vec.Y();
    p.z = 0;
    return p;
}

double AreaCoverageScore::GetFactor() {
    return scoreFactor;
}