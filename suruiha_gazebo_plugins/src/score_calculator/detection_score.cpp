//
// Created by okan on 18.07.2018.
//

#include <suruiha_gazebo_plugins/score_calculator/detection_score.h>
#include <gazebo/common/Console.hh>
#include <suruiha_gazebo_plugins/util/util.h>
#include <chrono>
#include <suruiha_gazebo_plugins/score_calculator/geometry.h>
#include <suruiha_gazebo_plugins/score_calculator/coordinate_conversions.h>
#include <geometry_msgs/Point32.h>

using namespace gazebo;
//using namespace std::chrono;

DetectionScore::DetectionScore() {
}

DetectionScore::~DetectionScore() {
}

void DetectionScore::GetParameters(sdf::ElementPtr ownSDF) {
    detectionStartTime.Set(ownSDF->Get<double>("start_time")/1000.0);
    buildingName = ownSDF->Get<std::string>("building_name");
    falseDetectionPenalty = ownSDF->Get<double>("false_detection_penalty");
    baseScore = ownSDF->Get<double>("base_score");
    timeToDetect.Set(ownSDF->Get<double>("time_to_detect"));
    scoreFactor = ownSDF->Get<double>("factor");

    std::string detectionTopic = ownSDF->Get<std::string>("detection_topic");
    rosNode = new ros::NodeHandle("");
    detectionSub = rosNode->subscribe(detectionTopic, 100, &DetectionScore::OnDetection, this);

    isDetected = false;
    penalty = 0;
}

void DetectionScore::SetWorld(physics::WorldPtr _worldPtr) {
    worldPtr = _worldPtr;
}

void DetectionScore::UpdateStates() {
    // do nothing
}


double DetectionScore::CalculateScore() {
    double score = baseScore;
    if (isDetected) {
        double detectionPerformance = timeToDetect.Double() - detectionTime.Double();
        score += (detectionPerformance / timeToDetect.Double()) * 90; // score is given out of 100 (baseScore 10 + 90)
    }
    return score * (1.0 - penalty);
}

void DetectionScore::OnDetection(std_msgs::String::ConstPtr msg) {
    common::Time currTime = worldPtr->SimTime();
    if (currTime.Double() >= detectionStartTime.Double()) {
        if (msg->data == buildingName) {
            detectionTime = currTime;
            isDetected = true;
        } else {
            penalty += falseDetectionPenalty;
        }
    } else {
        penalty += falseDetectionPenalty;
    }
}

double DetectionScore::GetFactor() {
    return scoreFactor;
}