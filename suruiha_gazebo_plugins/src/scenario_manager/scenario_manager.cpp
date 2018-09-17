//
// Created by okan on 11.07.2018.
//

#include <suruiha_gazebo_plugins/scenario_manager/scenario_manager.h>
#include <suruiha_gazebo_plugins/util/util.h>
#include <gazebo/common/common.hh>
#include <gazebo/transport/Node.hh>
#include <locale>
#include <fstream>

namespace gazebo{
    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(ScenarioManager);

    ScenarioManager::ScenarioManager() {
        printed = false;
    }

    ScenarioManager::~ScenarioManager() {
    }

    void ScenarioManager::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

        world = _parent;
        simDuration.Set(_sdf->Get<double>("sim_duration")/1000.0);
        gzdbg << "duration of the sim:" << simDuration.Double() << std::endl;

        gazeboNode = transport::NodePtr(new transport::Node());
        gazeboNode->Init("");
        serverControlPub = gazeboNode->Advertise<msgs::ServerControl>("/gazebo/server/control");

        actor = world->ModelByName("terrorist_0");
        gun = world->ModelByName("gun_0");

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
//        boost::mutex::scoped_lock lock(updateMutex);
//        ignition::math::Pose3d actorPose = actor->WorldPose();
//        ignition::math::Vector3d rot = ignition::math::Vector3d::Zero;
////        actorPose.Rot().Euler();
//        rot.Y(3.1415);
//        rot.Z(3.1415 + actorPose.Rot().Euler().Z());
//
//        ignition::math::Vector2d moveBack(0, 0.36);
//        moveBack.X(-moveBack.Y()*sin(actorPose.Rot().Euler().Z()));
//        moveBack.Y(moveBack.Y()*cos(actorPose.Rot().Euler().Z()));
//        actorPose.Pos().X(actorPose.Pos().X()+moveBack.X());
//        actorPose.Pos().Y(actorPose.Pos().Y()+moveBack.Y());
//        actorPose.Pos().Z(actorPose.Pos().Z()-0.2);
//        actorPose.Rot().Euler(rot);
//        gun->SetWorldPose(actorPose);

        // For Testing
//        if (!printed) {
        
        std::ofstream fileStream;
        fileStream.open("/tmp/models.xml");
        std::locale loc;
        int modelIndex = 0;
            std::vector<physics::ModelPtr> models = world->Models();
            fileStream << "<models>" << std::endl;
            for (unsigned int i = 0; i < models.size(); i++) {
                ignition::math::Pose3d pose = models[i]->WorldPose();
                if (models[i]->GetName().find("asphalt_plane") != std::string::npos) {
                    fileStream << "<model name=\"asphalt_plane_" << modelIndex << "\">" << std::endl;
                    fileStream << "  <static>true</static>" << std::endl;
                    fileStream << "  <pose>" << pose.Pos().X() << " " << pose.Pos().Y() <<
                              " " << pose.Pos().Z() << "</pose>" << std::endl;
                    fileStream << "  <include><uri>model://asphalt_plane</uri></include>" << std::endl;
                    fileStream << "</model>" << std::endl;

//                    std::cout << "<model name=\"asphalt_plane_" << modelIndex << "\">" << std::endl;
//                    std::cout << "  <static>true</static>" << std::endl;
//                    std::cout << "  <pose>" << pose.Pos().X() << " " << pose.Pos().Y() <<
//                              " " << pose.Pos().Z() << "</pose>" << std::endl;
//                    std::cout << "  <include><uri>model://asphalt_plane</uri></include>" << std::endl;
//                    std::cout << "</model>"<<std::endl;
                    modelIndex++;
                } else if (models[i]->GetName().find("building_") != std::string::npos) {
//                    std::cout << "<model name=\"" << models[i]->
//                    gzdbg << "model:" << models[i]->GetName() << ":" << models[i]->GetSDF()->ToString("") << std::endl;
                    std::string linkName = models[i]->GetSDF()->GetElement("link")->GetAttribute("name")->GetAsString();
//                    gzdbg << "model-" << models[i]->GetName() << "-" << linkName << "-" << std::endl;
                    size_t index = linkName.find("::");
                    if (index != std::string::npos) {
                        linkName = linkName.substr(0, index);
                        std::stringstream ss;
//                        gzdbg << "model-" << models[i]->GetName() << "-" << linkName << "-" << std::endl;
                        for (unsigned j = 0; j < linkName.size(); j++) {
                            if (linkName[j] == ' ') {
                                ss << "_";
                            } else {
                                ss << std::tolower(linkName[j], loc);
                            }
                        }

                        fileStream << "<model name=\"building_" << modelIndex << "\">" << std::endl;
                        fileStream << "  <static>true</static>" << std::endl;
                        fileStream << "  <pose>" << pose.Pos().X() << " " << pose.Pos().Y()
                                   << " " << pose.Pos().Z() << "</pose>" << std::endl;
                        fileStream << "  <include><uri>model://" << ss.str() << "</uri></include>" << std::endl;
                        fileStream << "</model>" << std::endl;

//                        gzdbg << "model:" << models[i]->GetName() << " type:" << ss.str() << std::endl;
//                        std::cout << "<model name=\"building_" << modelIndex << "\">" << std::endl;
//                        std::cout << "  <static>true</static>" << std::endl;
//                        std::cout << "  <pose>" << pose.Pos().X() << " " << pose.Pos().Y()
//                                  << " " << pose.Pos().Z() << "</pose>" << std::endl;
//                        std::cout << "  <include><uri>model://" << ss.str() << "</uri></include>" << std::endl;
//                        std::cout << "</model>" <<std::endl;
                        modelIndex++;
                    }
                }
//                ignition::math::Vector3d min = models[i]->BoundingBox().Min();
//                ignition::math::Vector3d max = models[i]->BoundingBox().Max();
//                gzdbg << "model:" << models[i]->GetName() << " min.x:" << min.X() << " min.y:" << min.Y() <<
//                      " max.x:" << max.X() << " max.y:" << max.Y() << std::endl;
            }
            fileStream << "</models>" << std::endl;
//            printed = true;
//        }

        fileStream.flush();
        fileStream.close();

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
