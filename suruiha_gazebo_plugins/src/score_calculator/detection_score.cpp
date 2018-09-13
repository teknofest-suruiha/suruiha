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
#include <limits>

using namespace gazebo;
//using namespace std::chrono;

DetectionScore::DetectionScore() {
}

DetectionScore::~DetectionScore() {
}

void DetectionScore::GetParameters(sdf::ElementPtr ownSDF) {
    falseDetectionPenalty = ownSDF->Get<double>("false_detection_penalty");
    baseScore = ownSDF->Get<double>("base_score");
    scoreFactor = ownSDF->Get<double>("factor");

    std::string detectionTopic = ownSDF->Get<std::string>("detection_topic");
    rosNode = new ros::NodeHandle("");
    detectionSub = rosNode->subscribe(detectionTopic, 100, &DetectionScore::OnDetection, this);

    penalty = 0;
}

void DetectionScore::SetWorld(physics::WorldPtr _worldPtr) {
    worldPtr = _worldPtr;

    std::vector<physics::ModelPtr> buildingModels;
    std::vector<physics::ModelPtr> actorModels;

    // get actor and building models
    std::vector<physics::ModelPtr> models = worldPtr->Models();
    for (unsigned int i = 0; i < models.size(); i++) {
        if (models[i]->GetName().find("building") != std::string::npos) {
            buildingModels.push_back(models[i]);
        } else if (models[i]->GetName().find("terrorist") != std::string::npos) {
            actorModels.push_back(models[i]);
        }
    }

    // get start and end times for the detection of the building
    double distThresh = 0.1;
    for (unsigned int i = 0; i < buildingModels.size(); i++) {
        ignition::math::Pose3d buildingPose = buildingModels[i]->WorldPose();
        double earliestActorTime = std::numeric_limits<double>::max();
        double latestActorTime = std::numeric_limits<double>::min();
        buildingPose.Pos().Z(0.0);
        for (unsigned int j = 0; j < actorModels.size(); j++) {
            ignition::math::Pose3d actorPose = actorModels[j]->WorldPose();
            actorPose.Pos().Z(0.0);
            double actorDist = Util::CalDist(buildingPose, actorPose);
            if (actorDist <= distThresh) {
                std::pair<double, double> actorTimes = Util::GetStartEndTimeOfActor(actorModels[j]->GetSDF());
                if (actorTimes.first < earliestActorTime) {
                    earliestActorTime = actorTimes.first;
                }
                if (actorTimes.second > latestActorTime) {
                    latestActorTime = actorTimes.second;
                }
            }
        }
        if (earliestActorTime < std::numeric_limits<double>::max()) {
            gzdbg << "detection building:" << buildingModels[i]->GetName() << " startTime:" << earliestActorTime << std::endl;
            detectionStartTimes.insert(std::pair<std::string, double>(buildingModels[i]->GetName(), earliestActorTime));
        }
        if (latestActorTime > std::numeric_limits<double>::min()) {
            gzdbg << "detection building:" << buildingModels[i]->GetName() << " endTime:" << latestActorTime << std::endl;
            detectionEndTimes.insert(std::pair<std::string, double>(buildingModels[i]->GetName(), latestActorTime));
        }
    }
}

void DetectionScore::UpdateStates() {
    // do nothing
}


double DetectionScore::CalculateScore() {
    if (detectionStartTimes.size() == 0) {
        return baseScore;
    }

    double score = 0;
    std::map<std::string, double>::iterator detectionIterator;
    for (detectionIterator = uavDetectionTimes.begin(); detectionIterator != uavDetectionTimes.end(); detectionIterator++) {
        double detectionPerformance = detectionIterator->second - detectionStartTimes[detectionIterator->first];
        double timeToDetect = detectionEndTimes[detectionIterator->first] - detectionStartTimes[detectionIterator->first];
        detectionPerformance = timeToDetect - detectionPerformance;
        score += (detectionPerformance / timeToDetect) * (100 - baseScore); // score is given out of 100
    }
    score = score / detectionStartTimes.size();
    score += baseScore;
    score = score * (1.0 - penalty);
    if (score < 0)
        score = 0;

    return score;
}

void DetectionScore::OnDetection(std_msgs::String::ConstPtr msg) {
//    gzdbg << "onDetection:" << msg->data << std::endl;
    if (uavDetectionTimes.find(msg->data) != uavDetectionTimes.end()) {
        // already detected do nothing
        return;
    }

    common::Time currTime = worldPtr->SimTime();
    std::map<std::string, double>::iterator buildingIterator;
    bool buildingMatched = false;
    std::string buildingName;
    for (buildingIterator = detectionStartTimes.begin(); buildingIterator != detectionStartTimes.end(); buildingIterator++) {
        if (currTime.Double() > buildingIterator->second &&
                currTime.Double() < detectionEndTimes[buildingIterator->first]) {
            if (msg->data == buildingIterator->first) {
                buildingMatched = true;
                buildingName = buildingIterator->first;
                break;
            }
        }
    }
    if (buildingMatched) {
//        gzdbg << "uavDetectionTime " << buildingName << " time:" << currTime.Double() << std::endl;
        uavDetectionTimes.insert(std::pair<std::string, double>(buildingName, currTime.Double()));
    } else {
//        gzdbg << "false detection" << std::endl;
        penalty += falseDetectionPenalty;
    }
}

double DetectionScore::GetFactor() {
    return scoreFactor;
}