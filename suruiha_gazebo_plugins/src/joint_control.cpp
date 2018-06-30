//
// Created by okan on 30.04.2018.
//

#include <suruiha_gazebo_plugins/joint_control.h>

namespace gazebo {

JointControl::JointControl() {
    this->pid.Init(0.1, 0, 0, 0, 1.0, -1.0);
    this->jointType = JointType::EFFORT;
}

JointControl::~JointControl() {
}

void JointControl::SetPIDParams(double p, double i, double d, double imax, double imin, double cmdMax, double cmdMin) {
    this->pid.Init(p, i, d, imax, imin, cmdMax, cmdMin);
}

void JointControl::SetJointType(std::string jointTypeString) {
    if (jointTypeString == "position") {
        this->jointType = JointType::POSITION;
    } else if (jointTypeString == "velocity") {
        this->jointType = JointType::VELOCITY;
    } else if (jointTypeString == "effort") {
        this->jointType = JointType::EFFORT;
    } else {
        gzerr << "Unknown joint type\n";
    }
}

void JointControl::SetCommand(double command, common::Time dt_) {
    switch (this->jointType) {
        case JointType::POSITION:
            this->SetPosition(command, dt_);
            break;
        case JointType::VELOCITY:
            this->SetVelocity(command, dt_);
            break;
        case JointType::EFFORT:
            this->SetEffort(command);
            break;
        default:
            gzerr << "Unknown joint type\n";
    }
}

void JointControl::SetPosition(double position, common::Time dt_) {
    const double currentPos = this->joint->Position(0);
    const double error = currentPos - position;
    const double force = this->pid.Update(error, dt_);
//    gzdbg << "position current:" << currentPos << " target:" << position << " err:" << error << " force:" << force << "\n";
    this->ApplyForce(force);
}

void JointControl::SetVelocity(double velocity, common::Time dt_) {
    const double currentVel = this->joint->GetVelocity(0);
    const double error = currentVel - velocity;
    const double force = this->pid.Update(error, dt_);
//    gzdbg << "velocity current:" << currentVel << " target:" << velocity << " err:" << error << " force:" << force << "\n";
    this->ApplyForce(force);
}

void JointControl::SetEffort(double effort) {
    this->ApplyForce(effort);
}

void JointControl::ApplyForce(double force) {
    joint->SetForce(0, force);
}

}
