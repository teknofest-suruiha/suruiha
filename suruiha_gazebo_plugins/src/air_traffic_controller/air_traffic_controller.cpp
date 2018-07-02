//
// Created by okan on 01.07.2018.
//

#include <suruiha_gazebo_plugins/air_traffic_controller/air_traffic_controller.h>
#include <suruiha_gazebo_plugins/air_traffic_controller/air_traffic_constants.h>
#include <ignition/math.hh>

using namespace suruiha_gazebo_plugins;

namespace gazebo {
    GZ_REGISTER_WORLD_PLUGIN(AirTrafficController);

    AirTrafficController::AirTrafficController() {
    }

    AirTrafficController::~AirTrafficController() {
    }

    void AirTrafficController::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
        worldPtr = _parent;
        // load zephyr and iris models currently active on the simulation
        initialPoses.clear();
        GetModelsInitialPosition("zephyr", 6);
        GetModelsInitialPosition("iris", 6);

        // read parameters from SDF
        SetParameters(_sdf);

        // create ROS node handle
        rosNode = new ros::NodeHandle();

        serviceServer = rosNode->advertiseService("air_traffic_control", &AirTrafficController::AirTrafficService, this);


        updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&AirTrafficController::UpdateStates, this));
    }

    void AirTrafficController::GetModelsInitialPosition(std::string baseModelName, int maxIndex) {
        int i = 0;
        for (i = 0; i < maxIndex; i++) {
            std::stringstream ss;
            ss << baseModelName << i;
            gzdbg << "Checking model with name " << ss.str() << std::endl;
            physics::ModelPtr modelPtr = worldPtr->ModelByName(ss.str());
            if (modelPtr == NULL) {
                break;
            } else {
                ignition::math::Pose3d modelPose = modelPtr->WorldPose();
                initialPoses.insert(std::pair<std::string, ignition::math::Pose3d>(ss.str(), modelPose));
            }
        }
        gzdbg << "We have " << i << " many " << baseModelName << " models" << std::endl;
    }

    void AirTrafficController::SetParameters(sdf::ElementPtr sdf) {
        takeOffPosition = sdf->GetElement("takeoff")->Get<ignition::math::Vector3d>("position");
        takeOffHeightThreshold = sdf->GetElement("takeoff")->Get<double>("height_threshold");
        takeOffDistanceThreshold = sdf->GetElement("takeoff")->Get<double>("distance_threshold");

        landingHeightThreshold = sdf->GetElement("landing")->Get<double>("height_threshold");
        landingVelocityThreshold = sdf->GetElement("landing")->Get<double>("velocity_threshold");
    }

    void AirTrafficController::UpdateStates() {
        boost::mutex::scoped_lock lock(updateMutex);

        if (runway.GetStatus() == air_traffic_constants::ALLOCATED_TO_TAKEOFF) {
            std::string flight = runway.GetAllocatedFlight();
            physics::ModelPtr modelPtr = worldPtr->ModelByName(flight);
            if (modelPtr != NULL) {
                ignition::math::Pose3d pose;
                pose.Pos().Set(takeOffPosition[0], takeOffPosition[1], takeOffPosition[2]);
                modelPtr->SetWorldPose(pose);
                runway.SetStatus(air_traffic_constants::READY_TO_TAKEOFF);
            } else {
                gzdbg << "Cannot find the flight by name:" << flight << std::endl;
            }
        } else if (runway.GetStatus() == air_traffic_constants::ALLOCATED_TO_LAND) {
            std::string flight = runway.GetAllocatedFlight();
            physics::ModelPtr modelPtr = worldPtr->ModelByName(flight);
            if (modelPtr != NULL) {
                ignition::math::Vector3d flightVel = modelPtr->WorldLinearVel();
                ignition::math::Pose3d flightPose = modelPtr->WorldPose();
                if (flightPose.Pos().Z() < landingHeightThreshold && flightVel.Length() < landingVelocityThreshold) {
                    runway.SetStatus(air_traffic_constants::LANDED);
                }
            }
            else {
                gzdbg << "Cannot find the flight by name:" << flight << std::endl;
            }
        } else if (runway.GetStatus() == air_traffic_constants::LANDED) {
            std::string flight = runway.GetAllocatedFlight();
            physics::ModelPtr modelPtr = worldPtr->ModelByName(flight);
            if (modelPtr != NULL) {
                ignition::math::Pose3d initialModelPose = initialPoses[flight];
                modelPtr->SetWorldPose(initialModelPose);
                runway.SetStatus(air_traffic_constants::AVAILABLE);
            }
            else {
                gzdbg << "Cannot find the flight by name:" << flight << std::endl;
            }
        } else if (runway.GetStatus() == air_traffic_constants::READY_TO_TAKEOFF) {
            std::string flight = runway.GetAllocatedFlight();
            physics::ModelPtr modelPtr = worldPtr->ModelByName(flight);
            if (modelPtr != NULL) {
                ignition::math::Pose3d flightPose = modelPtr->WorldPose();

                if (flightPose.Pos().Z() > takeOffHeightThreshold &&
                        flightPose.Pos().Length() > takeOffDistanceThreshold) {
                    runway.SetStatus(air_traffic_constants::AVAILABLE);
                }
            }
        }
    }

    bool AirTrafficController::AirTrafficService(AirTraffic::Request& req, AirTraffic::Response& resp) {
        gzdbg << "We get a service request command:" << req.command <<  " from " << req.sender << std::endl;
        resp.result = runway.ProcessCommand(req.command, req.sender);
        return true;
    }
}