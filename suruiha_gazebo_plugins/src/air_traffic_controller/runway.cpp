//
// Created by okan on 01.07.2018.
//

#include <suruiha_gazebo_plugins/air_traffic_controller/runway.h>
#include <suruiha_gazebo_plugins/air_traffic_controller/air_traffic_constants.h>
#include <gazebo/common/Console.hh>

RunWay::RunWay() {
    Init();
}

RunWay::RunWay(sdf::ElementPtr sdf) {
    Init();
    // set pose and size of the runway
//    SetSDF(sdf);
}

void RunWay::Init() {
    status = air_traffic_constants::AVAILABLE;
    allocatedUAV = "";
}

RunWay::~RunWay() {}

//void RunWay::SetSDF(sdf::ElementPtr sdf) {
//    pose = sdf->Get<ignition::math::Pose3d>("pose");
//    size = sdf->GetElement("geometry")->GetElement("plane")->Get<ignition::math::Vector2d>("size");
//    gzdbg << "pose.x:" << pose.Pos().X() << " .y:" << pose.Pos().Y() << " .z:" << pose.Pos().Z() << std::endl;
//    gzdbg << "size.x:" << size[0] << " .y:" << size[1] << std::endl;
//}

std::string RunWay::ProcessCommand(std::string& cmd, std::string& sender) {
    if (cmd == AirTrafficConstants::STATUS) {
        if (!allocatedUAV.empty() && allocatedUAV == sender) {
            return AirTrafficConstants::ToString(status);
        } else {
            if (status == air_traffic_constants::AVAILABLE) {
                return AirTrafficConstants::AVAILABLE;
            } else {
                return AirTrafficConstants::TAKEN;
            }
        }
    } else if (cmd == AirTrafficConstants::TAKEOFF_REQUEST) {
        if (status != air_traffic_constants::AVAILABLE) {
            return AirTrafficConstants::TAKEN;
        } else {
            allocatedUAV = sender;
            status = air_traffic_constants::ALLOCATED_TO_TAKEOFF;
            return AirTrafficConstants::ALLOCATED_TO_TAKEOFF;
        }
    } else if (cmd == AirTrafficConstants::LANDING_REQUEST) {
        if (status != air_traffic_constants::AVAILABLE) {
            return AirTrafficConstants::TAKEN;
        } else {
            allocatedUAV = sender;
            status = air_traffic_constants::ALLOCATED_TO_LAND;
            return AirTrafficConstants::ALLOCATED_TO_LAND;
        }
    } else if (cmd == AirTrafficConstants::LANDING_POSE) {
        std::stringstream ss;
        // start of landing pose
        ss << landingStartPose.Pos().X() << " " << landingStartPose.Pos().Y() << " " << landingStartPose.Pos().Z() << " ";
        ignition::math::Vector3d startEulerRot = landingStartPose.Rot().Euler();
        ss << startEulerRot.X() << " " << startEulerRot.Y() << " " << startEulerRot.Z() << " ";
        // end of the lading pose
        ss << landingEndPose.Pos().X() << " " << landingEndPose.Pos().Y() << " " << landingEndPose.Pos().Z() << " ";
        ignition::math::Vector3d endEulerRot = landingEndPose.Rot().Euler();
        ss << endEulerRot.X() << " " << endEulerRot.Y() << " " << endEulerRot.Z();
        return ss.str();
    } else {
        gzdbg << "unknown command:" << cmd << " from " << sender << std::endl;
        return "ERROR";
    }
}

std::string RunWay::GetAllocatedUAV() {
    return allocatedUAV;
}

air_traffic_constants::Status RunWay::GetStatus() {
    return status;
}

void RunWay::SetStatus(air_traffic_constants::Status _status) {
    status = _status;
}

void RunWay::SetLandingPose(ignition::math::Pose3d startPose, ignition::math::Pose3d endPose) {
    landingStartPose = startPose;
    landingEndPose = endPose;
}
//bool RunWay::GetIsLanding() {
//    return isLanding;
//}
//
//bool RunWay::GetIsTaken() {
//    return isTaken;
//}
