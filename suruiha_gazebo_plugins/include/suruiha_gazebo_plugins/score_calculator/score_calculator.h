//
// Created by okan on 11.07.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_SCORE_CALCULATOR_H
#define SURUIHA_GAZEBO_PLUGINS_SCORE_CALCULATOR_H

#include <map>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <sdf/sdf.hh>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <suruiha_gazebo_plugins/score_calculator/area_coverage_score.h>
#include <suruiha_gazebo_plugins/score_calculator/detection_score.h>
#include <suruiha_gazebo_plugins/score_calculator/tracking_score.h>
#include <suruiha_gazebo_plugins/UAVSensorMessage.h>

namespace gazebo {
    class ScoreCalculator : public WorldPlugin {
        public: ScoreCalculator();
        public: ~ScoreCalculator();

        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
        protected: void GetParameters(sdf::ElementPtr worldSdf);

        protected: std::map<std::string, physics::ModelPtr> models;

        protected: virtual void UpdateStates();
        private: event::ConnectionPtr updateConnection;
        private: boost::mutex updateMutex;

    protected: physics::WorldPtr world;

    protected: void CalculateAndPublishScore();

    protected: ros::Publisher scorePublisher;
    protected: ros::NodeHandle* rosNode;
    protected: bool isCalculateScore;
    protected: bool isThreadAlive;

    protected: double publishRate;
    protected: gazebo::common::Time lastPublishTime;

    protected: AreaCoverageScore areaScore;
    protected: DetectionScore detectionScore;
    protected: TrackingScore trackingScore;

    protected: boost::thread* scoreCalculationThread;

    };
}

#endif //SURUIHA_GAZEBO_PLUGINS_SCORE_CALCULATOR_H
