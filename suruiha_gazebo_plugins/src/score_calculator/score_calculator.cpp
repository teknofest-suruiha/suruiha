//
// Created by okan on 11.07.2018.
//

#include <suruiha_gazebo_plugins/score_calculator/score_calculator.h>
#include <suruiha_gazebo_plugins/util/util.h>
#include <gazebo/common/common.hh>
#include <visualization_msgs/Marker.h>

namespace gazebo{

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(ScoreCalculator);

    ScoreCalculator::ScoreCalculator() {
//        isVisualization = false;
//        markerCounter = 0;
    }

    ScoreCalculator::~ScoreCalculator() {

    }

    void ScoreCalculator::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
        // load models in to the map
        Util::GetModels(models, 6, "zephyr", _parent);
        Util::GetModels(models, 6, "iris", _parent);

        areaScore.SetWorld(_parent);
        areaScore.SetModels(models);
        // get sensor parameters from models itself not to duplicate parameters
        areaScore.GetParameters(_parent->SDF(), _sdf->GetElement("area_coverage_score"));

        rosNode = new ros::NodeHandle("");
        serviceServer = rosNode->advertiseService("score", &ScoreCalculator::ScoreService, this);

//        std::map<std::string, physics::ModelPtr>::iterator it;
//        for (it = models.begin(); it != models.end(); it++) {
//            std::stringstream ss(it->first);
//            ss << "_sensor";
//            ros::Subscriber sub = rosNode->subscribe(ss.str().c_str(), 100, &ScoreCalculator::SensorMessage, this);
//            sensorSubs.push_back(sub);
//        }

//        if (isVisualization) {
//            std::string topicName = _sdf->GetElement("visualization")->Get<std::string>("topic_name");
//            visPub = rosNode->advertise<visualization_msgs::MarkerArray>(topicName, 1);
//        }


        // New Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&ScoreCalculator::UpdateStates, this));
    }


    void ScoreCalculator::UpdateStates() {
        boost::mutex::scoped_lock lock(updateMutex);

        areaScore.UpdateStates();

    }

    bool ScoreCalculator::ScoreService(suruiha_gazebo_plugins::Score::Request& request,
                      suruiha_gazebo_plugins::Score::Response& resp) {
        std::string sender = request.sender; // actually not important :)

        resp.area_score = areaScore.CalculateScore();
        resp.detection_score = 0;
        resp.tracking_score = 0;
        return true;
    }

}
