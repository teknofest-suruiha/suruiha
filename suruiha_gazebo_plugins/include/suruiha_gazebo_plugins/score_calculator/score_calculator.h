//
// Created by okan on 11.07.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_SCORE_MANAGER_H
#define SURUIHA_GAZEBO_PLUGINS_SCORE_MANAGER_H

#include <map>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Plugin.hh>
#include <sdf/sdf.hh>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace gazebo {
    class ScoreCalculator : public WorldPlugin {
        public: ScoreCalculator();
        public: ~ScoreCalculator();

        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
        protected: void GetParameters(sdf::ElementPtr worldSdf);

        protected: std::map<std::string, physics::ModelPtr> models;

        protected: std::map<std::string, float> modelMaxPerceptionHeights;
        protected: std::map<std::string, ignition::math::Frustum*> modelFrustums;

        protected: std::vector<std::vector<ignition::math::Vector2d> > perceptedRectangles;

        protected: virtual void UpdateStates();
        private: event::ConnectionPtr updateConnection;
        private: boost::mutex updateMutex;

        protected: bool isVisualization;
        protected: ros::NodeHandle* rosNode;
        protected: ros::Publisher visPub;
        protected: visualization_msgs::MarkerArray markersCache;
        protected: int markerCounter;
    };
}

#endif //SURUIHA_GAZEBO_PLUGINS_SCORE_MANAGER_H
