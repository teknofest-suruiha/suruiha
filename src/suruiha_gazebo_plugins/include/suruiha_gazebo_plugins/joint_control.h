//
// Created by okan on 30.04.2018.
//

#ifndef SURUIHA_GAZEBO_PLUGINS_JOINT_CONTROL_H
#define SURUIHA_GAZEBO_PLUGINS_JOINT_CONTROL_H

#include <string>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/PID.hh>

namespace gazebo
{
    class JointControl
    {
        enum JointType {
            POSITION,
            VELOCITY,
            EFFORT
        };

        public: JointControl();
        public: virtual ~JointControl();
        public: std::string jointName;
        public: JointControl::JointType jointType;
        public: physics::JointPtr joint;

        private: common::PID pid;

        public: void SetJointType(std::string jointTypeString);
        public: void SetPIDParams(double p, double i, double d, double imax, double imin, double cmdMax, double cmdMin);
        public: void SetCommand(double cmd, common::Time dt_);

        private: void SetPosition(double position, common::Time dt_);
        private: void SetVelocity(double velocity, common::Time dt_);
        private: void SetEffort(double force);

        private: void ApplyForce(double force);
    };
}

#endif //SURUIHA_GAZEBO_PLUGINS_JOINT_CONTROL_H
