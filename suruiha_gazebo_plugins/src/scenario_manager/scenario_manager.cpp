//
// Created by okan on 11.07.2018.
//

#include <suruiha_gazebo_plugins/scenario_manager/scenario_manager.h>
#include <suruiha_gazebo_plugins/util/util.h>
#include <gazebo/common/common.hh>
#include <gazebo/transport/Node.hh>

namespace gazebo{
    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(ScenarioManager);

    ScenarioManager::ScenarioManager() {
    }

    ScenarioManager::~ScenarioManager() {
    }

    void ScenarioManager::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

        world = _parent;
        simDuration.Set(_sdf->Get<double>("sim_duration")/1000.0);
        gzdbg << "duration of the sim:" << simDuration.Double() << std::endl;

        gazeboNode = transport::NodePtr(new transport::Node());
        gazeboNode->Init(_parent->Name());
        serverControlPub = gazeboNode->Advertise<msgs::ServerControl>("/gazebo/server/control");

        // New Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&ScenarioManager::UpdateStates, this));
    }

    void ScenarioManager::KillAll() {
        // wait some time for propagating the stop message
//        std::system("killall gzclient && killall gzserver");
    }


    void ScenarioManager::UpdateStates() {
        boost::mutex::scoped_lock lock(updateMutex);
//        gzdbg << "simtime:" << world->SimTime().Double() << std::endl;
        if (world->SimTime().Double() >= simDuration.Double()) {
            gzdbg << "Stop and Finish scenario manager" << std::endl;
            msgs::ServerControl controlMsg;
            controlMsg.set_stop(true);
            serverControlPub->Publish(controlMsg);
            KillAll();
        }
    }

}
