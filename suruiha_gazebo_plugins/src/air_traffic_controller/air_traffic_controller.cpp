//
// Created by okan on 01.07.2018.
//

#include <suruiha_gazebo_plugins/air_traffic_controller/air_traffic_controller.h>
#include <suruiha_gazebo_plugins/air_traffic_controller/air_traffic_constants.h>
#include <gazebo/gazebo.hh>
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
        runway.SetLandingPose(landingStartPose, landingEndPose);

        // create ROS node handle
        rosNode = new ros::NodeHandle();

        serviceServer = rosNode->advertiseService("air_traffic_control", &AirTrafficController::AirTrafficService, this);

        // create inner communication node
        node = transport::NodePtr(new transport::Node());
        node->Init(_parent->Name());
        uavStatusPub = node->Advertise<msgs::Any>("/air_control");

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
                // also fill the uav status. at first all uavs are inactive until finishing takeoff operation being on the air again
                isUAVActive.insert(std::pair<std::string, bool>(ss.str(), false));
            }
        }
        gzdbg << "We have " << i << " many " << baseModelName << " models" << std::endl;
    }

    void AirTrafficController::SetParameters(sdf::ElementPtr sdf) {
        takeOffPose = sdf->GetElement("takeoff")->Get<ignition::math::Pose3d>("pose");
        takeOffHeightThreshold = sdf->GetElement("takeoff")->Get<double>("height_threshold");
        takeOffDistanceThreshold = sdf->GetElement("takeoff")->Get<double>("distance_threshold");

        landingStartPose = sdf->GetElement("landing")->Get<ignition::math::Pose3d>("start_pose");
        landingEndPose = sdf->GetElement("landing")->Get<ignition::math::Pose3d>("end_pose");
        landingHeightThreshold = sdf->GetElement("landing")->Get<double>("height_threshold");
        landingVelocityThreshold = sdf->GetElement("landing")->Get<double>("velocity_threshold");

        landingBottomLeft = sdf->GetElement("landing")->Get<ignition::math::Vector3d>("area_bottom_left");
        landingUpperRight = sdf->GetElement("landing")->Get<ignition::math::Vector3d>("area_upper_right");
    }

    void AirTrafficController::UpdateStates() {
        boost::mutex::scoped_lock lock(updateMutex);
//        physics::ModelPtr modelPtr = worldPtr->ModelByName("zephyr0");
//        if (modelPtr != NULL) {
//            gzdbg << "----" << std::endl;
//            std::vector<physics::LinkPtr> links = modelPtr->GetLinks();
//            for (physics::LinkPtr link : links) {
//                gzdbg << "linkname:" << link->GetName() << std::endl;
//            }
//        }


        if (runway.GetStatus() == air_traffic_constants::ALLOCATED_TO_TAKEOFF) {
            std::string uavName = runway.GetAllocatedUAV();
            physics::ModelPtr modelPtr = worldPtr->ModelByName(uavName);
            if (modelPtr != NULL) {
                modelPtr->SetWorldPose(takeOffPose);
//                gzdbg << "set " << uavName << " to takeoff position x:" << takeOffPose.Pos().X() << " y:" << takeOffPose.Pos().Y() << " z:" << takeOffPose.Pos().Z() << std::endl;
                runway.SetStatus(air_traffic_constants::READY_TO_TAKEOFF);
                SetUAVStatus(uavName, true);
            } else {
                gzdbg << "Cannot find the UAV by name:" << uavName << std::endl;
            }
        } else if (runway.GetStatus() == air_traffic_constants::ALLOCATED_TO_LAND) {
            std::string uavName = runway.GetAllocatedUAV();
            physics::ModelPtr modelPtr = worldPtr->ModelByName(uavName);
            if (modelPtr != NULL) {
                ignition::math::Vector3d flightVel = modelPtr->WorldLinearVel();
                ignition::math::Pose3d flightPose = modelPtr->WorldPose();
//                gzdbg << "flight velocity:" << flightVel.Length() << " pose.x:" << flightPose.Pos().X() << " pose.y:" << flightPose.Pos().Y() << std::endl;
                if (flightPose.Pos().Z() < landingHeightThreshold && flightVel.Length() < landingVelocityThreshold) {
                    // check whether the flight is in the runway zone
                    if (flightPose.Pos().X() > landingBottomLeft.X() && flightPose.Pos().X() < landingUpperRight.X()
                            && flightPose.Pos().Y() > landingBottomLeft.Y() && flightPose.Pos().Y() < landingUpperRight.Y()) {
                        runway.SetStatus(air_traffic_constants::LANDED);
                    }
                }
            }
            else {
                gzdbg << "Cannot find the uavName by name:" << uavName << std::endl;
            }
        } else if (runway.GetStatus() == air_traffic_constants::LANDED) {
            std::string uavName = runway.GetAllocatedUAV();
            physics::ModelPtr modelPtr = worldPtr->ModelByName(uavName);
            if (modelPtr != NULL) {

//                modelPtr->SetStatic(true);

//                ignition::math::Vector3d zeroVelocity(0, 0, 0);
//                modelPtr->SetLinearVel(zeroVelocity);
//                modelPtr->SetAngularVel(zeroVelocity);
//
//                physics::LinkPtr wingPtr = modelPtr->GetLink("zephyr_with_skid_pad::zephyr_fixed_wing::wing");
//                gzdbg << "wing force:" << wingPtr->WorldForce().Length() << " torque:" << wingPtr->WorldTorque().Length() << std::endl;
//
//                ignition::math::Vector3d negativeForce(-wingPtr->WorldForce().X(), -wingPtr->WorldForce().Y(), -wingPtr->WorldForce().Z());
//                wingPtr->SetForce(negativeForce);
//                ignition::math::Vector3d negativeTorque(-wingPtr->WorldTorque().X(), -wingPtr->WorldTorque().Y(), -wingPtr->WorldTorque().Z());
//                wingPtr->SetTorque(negativeTorque);

//                ignition::math::Pose3d initialModelPose = initialPoses[uavName];
//                modelPtr->SetWorldPose(initialModelPose);
                justLanded.insert(uavName);

                runway.SetStatus(air_traffic_constants::AVAILABLE);
                SetUAVStatus(uavName, false);
            }
            else {
                gzdbg << "Cannot find the uavName by name:" << uavName << std::endl;
            }
        } else if (runway.GetStatus() == air_traffic_constants::READY_TO_TAKEOFF) {
            std::string uavName = runway.GetAllocatedUAV();
            physics::ModelPtr modelPtr = worldPtr->ModelByName(uavName);
            if (modelPtr != NULL) {
                ignition::math::Pose3d uavPose = modelPtr->WorldPose();
                if (uavPose.Pos().Z() > takeOffHeightThreshold &&
                        uavPose.Pos().Length() > takeOffDistanceThreshold) {
                    runway.SetStatus(air_traffic_constants::AVAILABLE);
                }
            }
        }
        // BEGIN: test
//        static bool sentMsg = false;
//        if (worldPtr->SimTime().sec > 10 && !sentMsg) {
//            // send air traffic message
//            msgs::Any airTrafficMsg;
//            airTrafficMsg.set_type(msgs::Any::STRING);
//            airTrafficMsg.set_string_value("activate zephyr0");
//            pubPtr->Publish(airTrafficMsg);
//            gzdbg << "air traffic message is sent" << std::endl;
//            sentMsg = true;
//        }
        // END: test

//        std::map<std::string, bool>::iterator uavIterator;
//        for (uavIterator = isUAVActive.begin(); uavIterator != isUAVActive.end(); uavIterator++) {
//            physics::ModelPtr modelPtr = worldPtr->ModelByName(uavIterator->first);
//            if (modelPtr != NULL) {
//                if (!(uavIterator->second)) {
////                    gzdbg << "uav " << uavIterator->first << " is not active" << std::endl;
//                    ignition::math::Pose3d initialModelPose = initialPoses[uavIterator->first];
//                    modelPtr->SetWorldPose(initialModelPose);
//                }
//            }
//        }
        std::set<std::string>::iterator landedIterator;
        for (landedIterator = justLanded.begin(); landedIterator != justLanded.end(); landedIterator++) {
            physics::ModelPtr modelPtr = worldPtr->ModelByName(*landedIterator);
            if (modelPtr != NULL) {
                physics::LinkPtr wingPtr = modelPtr->GetLink("zephyr_with_skid_pad::zephyr_fixed_wing::wing");

                ignition::math::Vector3d xyForce(wingPtr->WorldForce().X(), wingPtr->WorldForce().Y(), 0);
//                gzdbg << "wing force:" << wingPtr->WorldForce().Length() << " torque:"
//                      << wingPtr->WorldTorque().Length() << " xyforce:" << xyForce.Length() << std::endl;


                if (wingPtr->WorldTorque().Length() < 0.001 && xyForce.Length() < 0.001) {
                    justLanded.erase(*landedIterator);
                    continue;
                }

//                ignition::math::Vector3d negativeForce(-wingPtr->WorldForce().X(), -wingPtr->WorldForce().Y(),
//                                                       -wingPtr->WorldForce().Z());
//                wingPtr->SetForce(negativeForce);
//                ignition::math::Vector3d negativeTorque(-wingPtr->WorldTorque().X(), -wingPtr->WorldTorque().Y(),
//                                                        -wingPtr->WorldTorque().Z());
//                wingPtr->SetTorque(negativeTorque);
                ignition::math::Vector3d zeroVelocity(0, 0, 0);
                modelPtr->SetLinearVel(zeroVelocity);
                modelPtr->SetAngularVel(zeroVelocity);

                ignition::math::Pose3d initialModelPose = initialPoses[*landedIterator];
                modelPtr->SetWorldPose(initialModelPose);
            }
        }
    }

    void AirTrafficController::SetUAVStatus(std::string uavName, bool isActive) {
        isUAVActive.insert(std::pair<std::string, bool>(uavName, isActive));
//        gzdbg << "uav " << uavName << " is set as " << isActive << std::endl;
        // send air traffic information to uav controllers
        msgs::Any airTrafficMsg;
        airTrafficMsg.set_type(msgs::Any::STRING);
        if (isActive) {
            airTrafficMsg.set_string_value("activate " + uavName);
        } else {
            airTrafficMsg.set_string_value("deactivate " + uavName);
        }
        uavStatusPub->Publish(airTrafficMsg);
    }

    bool AirTrafficController::AirTrafficService(AirTraffic::Request& req, AirTraffic::Response& resp) {
//        gzdbg << "We get a service request command:" << req.command <<  " from " << req.sender << std::endl;
        resp.result = runway.ProcessCommand(req.command, req.sender);
        return true;
    }
}