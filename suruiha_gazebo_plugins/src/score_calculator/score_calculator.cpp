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

        // how often score is going to be published
        publishRate = _sdf->Get<double>("publish_rate");

        rosNode = new ros::NodeHandle("");
        scorePublisher = rosNode->advertise<suruiha_gazebo_plugins::UAVScore>("/score", 1);

        scoreCalculationThread = new boost::thread(boost::bind(&ScoreCalculator::CalculateAndPublishScore, this));

        // New Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&ScoreCalculator::UpdateStates, this));
    }


    void ScoreCalculator::UpdateStates() {
        boost::mutex::scoped_lock lock(updateMutex);

        areaScore.UpdateStates();
        common::Time currTime = world->SimTime();
        double dt = (currTime - lastPublishTime).Double() * 1000; // miliseconds
        if (dt > publishRate && !isCalculateScore) {
            isCalculateScore = true;
            lastPublishTime = currTime;
        }
    }

    void ScoreCalculator::CalculateAndPublishScore() {
        while (isThreadAlive) {
            if (isCalculateScore) {
                suruiha_gazebo_plugins::UAVScore scoreMsg;
                scoreMsg.area_score = areaScore.CalculateScore();
                scoreMsg.detection_score = 0;
                scoreMsg.tracking_score = 0;
                scoreMsg.total_score = 0;
                scorePublisher.publish(scoreMsg);
                gzdbg << "score is published area:" << scoreMsg.area_score << std::endl;
                isCalculateScore = false;
            } else {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
            }
        }
    }

}
