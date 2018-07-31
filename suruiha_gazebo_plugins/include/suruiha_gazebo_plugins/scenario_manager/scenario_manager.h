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

namespace gazebo {
    class ScenarioManager : public WorldPlugin {
        public: ScenarioManager();
        public: ~ScenarioManager();

        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

        protected: virtual void UpdateStates();
        private: event::ConnectionPtr updateConnection;
        private: boost::mutex updateMutex;

        protected: physics::WorldPtr world;
        protected: common::Time simDuration;
        protected: transport::NodePtr gazeboNode;
        protected: transport::PublisherPtr serverControlPub;
        protected: void KillAll();

    };
}

#endif //SURUIHA_GAZEBO_PLUGINS_SCORE_CALCULATOR_H
