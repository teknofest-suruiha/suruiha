//
// Created by okan on 11.07.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_SCORE_MANAGER_H
#define SURUIHA_GAZEBO_PLUGINS_SCORE_MANAGER_H

#include <map>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <sdf/sdf.hh>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <suruiha_gazebo_plugins/score_calculator/area_coverage_score.h>
#include <suruiha_gazebo_plugins/Score.h>
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

//        protected: bool isVisualization;

    protected: ros::ServiceServer serviceServer;
    protected: ros::NodeHandle* rosNode;
//        protected: ros::Publisher visPub;
//        protected: visualization_msgs::MarkerArray markersCache;
//        protected: int markerCounter;

        protected: AreaCoverageScore areaScore;

        protected: bool ScoreService(suruiha_gazebo_plugins::Score::Request& request,
                                     suruiha_gazebo_plugins::Score::Response& resp);

    };
}

#endif //SURUIHA_GAZEBO_PLUGINS_SCORE_MANAGER_H
