//
// Created by okan on 02.07.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_AIR_TRAFFIC_CONSTANTS_H
#define SURUIHA_GAZEBO_PLUGINS_AIR_TRAFFIC_CONSTANTS_H

#include <string>

namespace air_traffic_constants {
    enum Status {
        AVAILABLE,
        ALLOCATED_TO_TAKEOFF,
        ALLOCATED_TO_LAND,
        READY_TO_TAKEOFF,
        LANDED
    };
}


class AirTrafficConstants {
// commands
public: static const std::string STATUS;
public: static const std::string TAKEOFF_REQUEST;
public: static const std::string LANDING_REQUEST;
public: static const std::string CANCEL;
public: static const std::string LANDING_POSE;

// status
public: static const std::string AVAILABLE;
public: static const std::string TAKEN;
public: static const std::string ALLOCATED_TO_LAND;
public: static const std::string ALLOCATED_TO_TAKEOFF;
public: static const std::string READY_TO_TAKEOFF;
public: static const std::string LANDED;

public: static std::string ToString(air_traffic_constants::Status status);
};


#endif //SURUIHA_GAZEBO_PLUGINS_AIR_TRAFFIC_CONSTANTS_H
