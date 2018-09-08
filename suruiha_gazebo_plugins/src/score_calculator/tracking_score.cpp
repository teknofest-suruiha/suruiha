//
// Created by okan on 18.07.2018.
//

#include <suruiha_gazebo_plugins/score_calculator/tracking_score.h>
#include <gazebo/common/Console.hh>
#include <suruiha_gazebo_plugins/util/util.h>
#include <chrono>
#include <suruiha_gazebo_plugins/score_calculator/geometry.h>
#include <suruiha_gazebo_plugins/score_calculator/coordinate_conversions.h>
#include <geometry_msgs/Point32.h>
#include <vector>
#include <map>
#include <suruiha_gazebo_plugins/score_calculator/dtw.h>

using namespace gazebo;
//using namespace std::chrono;

TrackingScore::TrackingScore() {
}

TrackingScore::~TrackingScore() {
}

void TrackingScore::GetParameters(sdf::ElementPtr worldSdf, sdf::ElementPtr ownSDF) {
    terroristPrefix = ownSDF->Get<std::string>("terrorist_prefix");

    messageRate = ownSDF->Get<double>("message_rate")/1000.0;
    updateRate = ownSDF->Get<double>("update_rate")/1000.0;
    lastUpdateTime.Set(0);
    scoreFactor = ownSDF->Get<double>("factor");

    std::string trackingTopic = ownSDF->Get<std::string>("tracking_topic");
    rosNode = new ros::NodeHandle("");
    trackingSub = rosNode->subscribe(trackingTopic, 100, &TrackingScore::OnTrackingMessage, this);

    // get terrorist models
    const std::vector<physics::ModelPtr>& allModels = worldPtr->Models();
    for (unsigned int i = 0; i < allModels.size(); i++) {
        physics::ModelPtr modelPtr = allModels.at(i);
        if (modelPtr->GetName().find(terroristPrefix) != std::string::npos) {
            terrorists.insert(std::pair<std::string, physics::ModelPtr>(modelPtr->GetName(), modelPtr));
            std::vector<std::vector<double> > trajectory;
            terroristTrajectories.insert(std::pair<std::string, std::vector<std::vector<double> > >(modelPtr->GetName(), trajectory));
            trackingTrajectories.insert(std::pair<std::string, std::vector<std::vector<double> > >(modelPtr->GetName(), trajectory));
            soFarCalculatedIndex.insert(std::pair<std::string, unsigned int>(modelPtr->GetName(), 0));
            soFarCalculatedValue.insert(std::pair<std::string, unsigned int>(modelPtr->GetName(), 0.0));

            sdf::ElementPtr scriptElement = modelPtr->GetSDF()->GetElement("script");
            double startTime = 0;
            scriptElement->GetElement("delay_start")->GetValue()->Get(startTime);
            trackingStartTimes.insert(std::pair<std::string, double>(modelPtr->GetName(), startTime));

            sdf::ElementPtr child = scriptElement->GetElement("trajectory")->GetFirstElement();
            double maxEndTime = 0;
            for (; child; child = child->GetNextElement()) {
                if (child->GetName() == "waypoint") {
                    double time = 0.0;
                    child->GetElement("time")->GetValue()->Get(time);
                    if (time > maxEndTime) {
                        maxEndTime = time;
                    }
                }
            }
            trackingEndTimes.insert(std::pair<std::string, double>(modelPtr->GetName(), maxEndTime+startTime));
        }
    }
}

void TrackingScore::SetWorld(physics::WorldPtr _worldPtr) {
    worldPtr = _worldPtr;
}

void TrackingScore::UpdateStates() {
    boost::mutex::scoped_lock lock(this->updateMutex);
    common::Time currTime = worldPtr->SimTime();
    // record the true trajectories of the terrorists
    if ((currTime - lastUpdateTime).Double() > updateRate) {
        std::map<std::string, physics::ModelPtr>::iterator terroristIterator;
        for (terroristIterator = terrorists.begin(); terroristIterator != terrorists.end(); terroristIterator++) {
            // check the start and end time of this terrorist
            if (currTime.Double() >= trackingStartTimes[terroristIterator->first] &&
                currTime.Double() <= trackingEndTimes[terroristIterator->first]) {
                const ignition::math::Pose3d &pose = terroristIterator->second->WorldPose();
                std::vector<double> position(2, 0);
                position[0] = pose.Pos().X();
                position[1] = pose.Pos().Y();
                terroristTrajectories[terroristIterator->first].push_back(position);
            }
        }
        lastUpdateTime = currTime;
    }
}


double TrackingScore::CalculateScore() {
    boost::mutex::scoped_lock lock(this->updateMutex);
    double totalScore = 0;
    std::map<std::string, std::vector<std::vector<double> > >::iterator it;
//    gzdbg << "---" << std::endl;
    for (it = trackingTrajectories.begin(); it != trackingTrajectories.end(); it++) {
        std::vector<std::vector<double> > trajectory = it->second;
        std::vector<std::vector<double> > trueTrajectory = terroristTrajectories[it->first];
        if (trajectory.size() > 0 && trueTrajectory.size() > 0) {
//            gzdbg << "modelName:" << it->first << " trajectory.size:" << trajectory.size() << " trueTrajectory.size:" <<
//                  trueTrajectory.size() << " trajectory.vec.size:" << trajectory[0].size() << std::endl;
            DTW::SimpleDTW dtw(trueTrajectory.size(), trajectory.size());
//            gzdbg << "dtw initialized" << std::endl;
            double cost = dtw.EvaluateWarpingCost(trueTrajectory, trajectory);
//            gzdbg << "dtw evaluate warping const" << std::endl;
            double maxCost = CostAgainstZeroTrajectory(it->first, trueTrajectory);
            totalScore += ((maxCost - cost) / maxCost) * 100;
//            gzdbg << "model:" << it->first << " maxCost:" << maxCost << " cost:" << cost << std::endl;
        }
    }
    return totalScore / terroristTrajectories.size();
}

void TrackingScore::OnTrackingMessage(suruiha_gazebo_plugins::UAVTracking::ConstPtr trackingMsg) {
    boost::mutex::scoped_lock lock(this->updateMutex);
    common::Time currTime = worldPtr->SimTime();
    if (currTime.Double() - lastTrackingMessageTime.Double() > messageRate) {
//        if (currTime.Double() >= trackingStartTime.Double()) {
            for (unsigned int i = 0; i < trackingMsg->names.size(); i++) {
                std::string modelName = trackingMsg->names[i];
                if (currTime.Double() >= trackingStartTimes[modelName] &&
                        currTime.Double() <= trackingEndTimes[modelName]) {
                    std::map<std::string, physics::ModelPtr>::iterator search = terrorists.find(modelName);
                    if (search != terrorists.end()) {
                        geometry_msgs::Pose pose = trackingMsg->poses[i];
                        std::vector<double> trackingPosition(2, 0);
                        trackingPosition[0] = pose.position.x;
                        trackingPosition[1] = pose.position.y;
                        trackingTrajectories[modelName].push_back(trackingPosition);
//                    gzdbg << "adding position.x:" << pose.position.x << " .y:" << pose.position.y <<
//                          " size:" << trackingTrajectories[modelName].size() << " vec.size:" <<
//                          trackingTrajectories[modelName][0].size() << std::endl;
                    }
                }
            }
//        }
        lastTrackingMessageTime =  currTime;
    }
}

double TrackingScore::GetFactor() {
    return scoreFactor;
}

double TrackingScore::CostAgainstZeroTrajectory(const std::string& modelName, std::vector<std::vector<double> >& trajectory) {
    /*
    std::vector<std::vector<double> > zeroTrajectory(trajectory.size());
    for (unsigned int i = 0; i < trajectory.size(); i++) {
        std::vector<double> vec(trajectory.at(0).size(), 0);
        zeroTrajectory[i] = vec;
    }
    DTW::SimpleDTW dtw(trajectory.size(), zeroTrajectory.size());
    double cost = dtw.EvaluateWarpingCost(trajectory, zeroTrajectory);
     */
    if (trajectory.size() == 0)
        return 0.0;

    double cost = soFarCalculatedValue[modelName];
    for (unsigned int i = soFarCalculatedIndex[modelName]; i < trajectory.size(); i++) {
        double vecCost = 0;
        for (unsigned int j = 0; j < trajectory[i].size(); j++) {
            vecCost += pow(trajectory[i][j], 2);
        }
        cost += sqrt(vecCost);
    }
    soFarCalculatedValue[modelName] = cost;
    soFarCalculatedIndex[modelName] = trajectory.size();
    return cost;
}