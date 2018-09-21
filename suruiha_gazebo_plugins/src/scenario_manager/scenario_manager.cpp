//
// Created by okan on 11.07.2018.
//

#include <suruiha_gazebo_plugins/scenario_manager/scenario_manager.h>
#include <suruiha_gazebo_plugins/util/util.h>
#include <gazebo/common/common.hh>
#include <gazebo/transport/Node.hh>
#include <locale>
#include <fstream>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/OccupancyGrid.h>
#include <suruiha_gazebo_plugins/score_calculator/geometry.h>

namespace gazebo{
    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(ScenarioManager);

    ScenarioManager::ScenarioManager() {
    }

    ScenarioManager::~ScenarioManager() {
    }

    void ScenarioManager::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

        world = _parent;
        simDuration.Set(_sdf->Get<double>("sim_duration"));
        gzdbg << "duration of the sim:" << simDuration.Double() << std::endl;

        gazeboNode = transport::NodePtr(new transport::Node());
        gazeboNode->Init("");
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

//    void ScenarioManager::IndexBuildings() {
//        std::set<physics::ModelPtr> buildingSet;
//        std::vector<physics::ModelPtr> models = world->Models();
//        for (unsigned int i = 0; i < models.size(); i++) {
//            if (models[i]->GetName().find("building_") != std::string::npos) {
//                std::cout << "<model name=\"building_" << i << "\">" << std::endl;
//                ignition::math::Pose3d pose = models[i]->WorldPose();
//                std::cout << "<pose>" << pose.Pos().X() << " " << pose.Pos().Y() << " " << pose.Pos().Z()
//                          << " " << pose.Rot().Euler().X() << " " << pose.Rot().Euler().Y() << " "
//                          << pose.Rot().Euler().Z() << "</pose>" << std::endl;
//
//            }
//        }
//    }


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
//        IndexBuildings();
        GetStreetModel();
        CalculateTerroristPath();

//        gzdbg << "simtime:" << world->SimTime().Double() << std::endl;
        if (world->SimTime().Double() >= simDuration.Double()) {
            gzdbg << "Stop and Finish scenario manager" << std::endl;
            msgs::ServerControl controlMsg;
            controlMsg.set_stop(true);
            serverControlPub->Publish(controlMsg);
            KillAll();
        }
    }

    void ScenarioManager::GetStreetModel() {
        static bool isProcessed = false;
        if (!isProcessed) {
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
                               " " << pose.Pos().Z() << " " << pose.Rot().Euler().X() <<
                               " " << pose.Rot().Euler().Y() << " " << pose.Rot().Euler().Z() << "</pose>" << std::endl;
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
                                   << " " << pose.Pos().Z() << " " << pose.Rot().Euler().X() <<
                                   " " << pose.Rot().Euler().Y() << " " << pose.Rot().Euler().Z() << "</pose>"
                                   << std::endl;
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

            isProcessed = true;
        }

    }

    void ScenarioManager::CalculateTerroristPath() {
        static bool isRun = false;
        if (!isRun) {
            double width = 500;
            double height = 500;
            nav_msgs::MapMetaData metaData;
            metaData.resolution = 5;
            metaData.height = floor(height / metaData.resolution);
            metaData.width = floor(width / metaData.resolution);
            metaData.origin.position.x = -width/2;
            metaData.origin.position.y = -height/2;
            metaData.origin.position.z = 0;
            nav_msgs::OccupancyGrid gridMap;
            gridMap.info = metaData;
            gridMap.data.resize(width*height, 0);

            std::string startBuildingName = "building_4h";
            std::string endBuildingName = "building_0n";

            std::vector<physics::ModelPtr> models = world->Models();
            for (unsigned int i = 0; i < models.size(); i++) {
                if (models[i]->GetName().find("building") != std::string::npos) {

                    // do not add terrorist start and finish
                    if (startBuildingName == models[i]->GetName() || endBuildingName == models[i]->GetName()) {
                        continue;
                    }

                    geometry_msgs::Polygon polygon;
//                    gzdbg << "model:" << models[i]->GetName() << " min.x:" << models[i]->BoundingBox().Min().X()
//                          << " min.y:" <<  models[i]->BoundingBox().Min().Y()
//                                       << " max.x:" << models[i]->BoundingBox().Max().X()
//                                                    << " max.y:" << models[i]->BoundingBox().Max().Y() << std::endl;
                    geometry_msgs::Point32 p1 = ToPoint32(models[i]->CollisionBoundingBox().Min().X(), models[i]->CollisionBoundingBox().Min().Y());
                    geometry_msgs::Point32 p2 = ToPoint32(models[i]->CollisionBoundingBox().Min().X(), models[i]->CollisionBoundingBox().Max().Y());
                    geometry_msgs::Point32 p3 = ToPoint32(models[i]->CollisionBoundingBox().Max().X(), models[i]->CollisionBoundingBox().Max().Y());
                    geometry_msgs::Point32 p4 = ToPoint32(models[i]->CollisionBoundingBox().Max().X(), models[i]->CollisionBoundingBox().Min().Y());
//                    gzdbg << "model:" << models[i]->GetName() << " p1.x:" << p1.x << " p1.y:" << p1.y <<
//                          " p2.x:" << p2.x << " p2.y:" << p2.y << " p3.x:" << p3.x << " p3.y:" << p3.y <<
//                          " p4.x:" << p4.x << " p4.y:" << p4.y << std::endl;
//                    gzdbg << "model:" << models[i]->GetName() << " min.x:" << models[i]->CollisionBoundingBox().Min().X()
//                          << " min.y:" << models[i]->CollisionBoundingBox().Min().Y() << " max.x:" <<
//                          models[i]->CollisionBoundingBox().Max().X() << " max.y:" << models[i]->CollisionBoundingBox().Max().Y() << std::endl;
                    polygon.points.push_back(p1);
                    polygon.points.push_back(p2);
                    polygon.points.push_back(p3);
                    polygon.points.push_back(p4);
                    AddOccupancy(polygon, gridMap);
                }
            }

            //TODO: run a-star path planning algorithm


//            for (unsigned int i = 0; i < 100; i++) {
//                for (unsigned int j = 0; j < 100; j++) {
//                    std::cout << static_cast<int>(gridMap.data[i*100+j]);
//                }
//                std::cout << std::endl;
//            }

            isRun = true;
        }
    }

    geometry_msgs::Point32 ScenarioManager::ToPoint32(double x, double y) {
        geometry_msgs::Point32 p;
        p.x = x;
        p.y = y;
        p.z = 0;
        return p;
    }


    void ScenarioManager::AddOccupancy(geometry_msgs::Polygon& polygon, nav_msgs::OccupancyGrid& gridMap) {

//    gzdbg << "started to check cells in convex polygon:" << polygon << std::endl;
        std::set<occupancy_grid_utils::Cell> cells = occupancy_grid_utils::cellsInConvexPolygon(gridMap.info, polygon);
//    gzdbg << "cellsInConvexPolygon.size:" << cells.size() << std::endl;
        std::set<occupancy_grid_utils::Cell>::iterator it;
        for (it = cells.begin(); it != cells.end(); it++) {
            // convert to linear index to mark as occupied
            uint32_t index = occupancy_grid_utils::cellIndex(gridMap.info, *it);
            // validate the index
            if (index < 0 || index > gridMap.info.width*gridMap.info.height) {
                continue;
            }
            // set as occupied
            gridMap.data[index] = 1;
        }
    }
}
