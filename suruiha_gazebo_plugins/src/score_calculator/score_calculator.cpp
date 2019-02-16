//
// Created by okan on 11.07.2018.
//

#include <suruiha_gazebo_plugins/score_calculator/score_calculator.h>
#include <suruiha_gazebo_plugins/util/util.h>
#include <gazebo/common/common.hh>
#include <visualization_msgs/Marker.h>
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>
#include <suruiha_gazebo_plugins/UAVScore.h>

namespace gazebo{
    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(ScoreCalculator);

    ScoreCalculator::ScoreCalculator() {
        isCalculateScore = false;
        isThreadAlive = true;
        finalScore = false;
    }

    ScoreCalculator::~ScoreCalculator() {
        isThreadAlive = false;
        scoreCalculationThread->join();
        delete scoreCalculationThread;
    }

    void ScoreCalculator::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

        world = _parent;
        lastPublishTime = world->SimTime();

        // load models in to the map
        Util::GetModels(models, 6, "zephyr", _parent);
        Util::GetModels(models, 6, "iris", _parent);

        areaScore.SetWorld(_parent);
        areaScore.SetModels(models);
        // get sensor parameters from models itself not to duplicate parameters
        areaScore.GetParameters(_parent->SDF(), _sdf->GetElement("area_coverage_score"));

        detectionScore.SetWorld(_parent);
        detectionScore.GetParameters(_sdf->GetElement("detection_score"));

        trackingScore.SetWorld(_parent);
        trackingScore.GetParameters(_parent->SDF(), _sdf->GetElement("tracking_score"));

        // how often score is going to be published
        publishRate = _sdf->Get<double>("publish_rate");

        rosNode = new ros::NodeHandle("");
        scorePublisher = rosNode->advertise<suruiha_gazebo_plugins::UAVScore>("/score", 1);

        scoreCalculationThread = new boost::thread(boost::bind(&ScoreCalculator::CalculateAndPublishScore, this));

        // get the sim time
        sdf::ElementPtr worldSDF = world->SDF();
        sdf::ElementPtr pluginSdf = worldSDF->GetElement("plugin");
        while (pluginSdf != NULL) {
            std::string pluginName = pluginSdf->GetAttribute("name")->GetAsString();
            if (pluginName == "scenario_manager") {
                simDuration = pluginSdf->Get<double>("sim_duration");
            }
            pluginSdf = pluginSdf->GetNextElement("plugin");
        }
        gzdbg << "simDuration:" << simDuration << std::endl;

        // New Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&ScoreCalculator::UpdateStates, this));
    }


    void ScoreCalculator::UpdateStates() {
        boost::mutex::scoped_lock lock(updateMutex);

        areaScore.UpdateStates();
        detectionScore.UpdateStates();
        trackingScore.UpdateStates();

        common::Time currTime = world->SimTime();
        double dt = (currTime - lastPublishTime).Double() * 1000; // miliseconds
        if (dt > publishRate && !isCalculateScore) {
            isCalculateScore = true;
            lastPublishTime = currTime;
        }
        if (currTime.Double() >= simDuration) {
            finalScore = true;
            isCalculateScore = true;
        }
    }

    void ScoreCalculator::CalculateAndPublishScore() {
        while (isThreadAlive) {
            if (isCalculateScore) {
                suruiha_gazebo_plugins::UAVScore scoreMsg;
                scoreMsg.area_score = areaScore.CalculateScore();
                scoreMsg.detection_score = detectionScore.CalculateScore();
                scoreMsg.tracking_score = trackingScore.CalculateScore();
                scoreMsg.total_score = scoreMsg.area_score * areaScore.GetFactor() +
                        scoreMsg.detection_score * detectionScore.GetFactor() +
                        scoreMsg.tracking_score * trackingScore.GetFactor();
                scorePublisher.publish(scoreMsg);
                isCalculateScore = false;
                if (finalScore) {
                    gzdbg << "-- FINAL SCORE --" << std::endl;
                    gzdbg << "area_score:" << scoreMsg.area_score << std::endl;
                    gzdbg << "detection_score:" << scoreMsg.detection_score << std::endl;
                    gzdbg << "tracking_score:" << scoreMsg.tracking_score << std::endl;
                    gzdbg << "total:" << scoreMsg.total_score << std::endl;
                    finalScore = false;
                }
            } else {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
            }
        }
    }

}
