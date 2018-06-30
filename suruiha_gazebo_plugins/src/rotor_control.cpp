/*
 * RotorControl.cpp
 *
 *  Created on: May 10, 2018
 *      Author: okan
 */

#include <suruiha_gazebo_plugins/rotor_control.h>

using namespace gazebo;

RotorControl::RotorControl() {
    // most of these coefficients are not used yet.
    this->rotorVelocitySlowdownSim = 10;
    this->frequencyCutoff = 5.0;
    this->samplingRate = 0.2;

    this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
}
