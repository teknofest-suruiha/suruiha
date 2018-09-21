//
// Created by okan on 11.07.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_SCENARIO_MANAGER_H
#define SURUIHA_GAZEBO_PLUGINS_SCENARIO_MANAGER_H

#include <map>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/OccupancyGrid.h>

namespace gazebo {
    class ScenarioManager : public WorldPlugin {
        public: ScenarioManager();
        public: ~ScenarioManager();

        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

        protected: virtual void UpdateStates();
        private: event::ConnectionPtr updateConnection;
        private: boost::mutex updateMutex;
        protected: std::vector<physics::ModelPtr> terrorists;
        protected: std::vector<physics::ModelPtr> guns;
        protected: std::map<std::string, double> teroristStartTimes;
        protected: std::map<std::string, double> teroristEndTimes;

        protected: physics::WorldPtr world;
        protected: common::Time simDuration;
        protected: transport::NodePtr gazeboNode;
        protected: transport::PublisherPtr serverControlPub;
        protected: void KillAll();

//        protected: void IndexBuildings();
        protected: void GetStreetModel();
        protected: void CalculateTerroristPath();
        protected: geometry_msgs::Point32 ToPoint32(double x, double y);
        protected: void AddOccupancy(geometry_msgs::Polygon& polygon, nav_msgs::OccupancyGrid& map);
    };
}

#endif //SURUIHA_GAZEBO_PLUGINS_SCORE_CALCULATOR_H
