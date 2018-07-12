//
// Created by okan on 01.07.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_RUNWAY_H
#define SURUIHA_GAZEBO_PLUGINS_RUNWAY_H

#include <sdf/sdf.hh>
#include <ignition/math.hh>
#include <suruiha_gazebo_plugins/air_traffic_controller/air_traffic_constants.h>

class RunWay {

public: RunWay();
public: RunWay(sdf::ElementPtr runwaySdf);
public: void Init();
public: virtual ~RunWay();

//public: void SetSDF(sdf::ElementPtr sdf);
public: std::string ProcessCommand(std::string& cmd, std::string& sender);
public: std::string GetAllocatedUAV();

public: void SetStatus(air_traffic_constants::Status _status);
public: void SetLandingPose(ignition::math::Pose3d& startPose, ignition::math::Pose3d& endPose);
public: air_traffic_constants::Status GetStatus();
//public: bool GetIsLanding();
//public: bool GetIsTaken();

protected: ignition::math::Pose3d landingStartPose;
protected: ignition::math::Pose3d landingEndPose;
//protected: ignition::math::Vector2d size;

protected: air_traffic_constants::Status status;

//protected: bool isTaken;
protected: std::string allocatedUAV;
//protected: bool isLanding;
//protected: std::string flightStatus;
};

#endif //SURUIHA_GAZEBO_PLUGINS_RUNWAY_H
