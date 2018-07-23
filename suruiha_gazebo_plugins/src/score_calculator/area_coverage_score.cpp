//
// Created by okan on 18.07.2018.
//

#include <suruiha_gazebo_plugins/score_calculator/area_coverage_score.h>
#include <gazebo/common/Console.hh>
#include <suruiha_gazebo_plugins/util/util.h>
#include <chrono>


using namespace gazebo;
using namespace std::chrono;

AreaCoverageScore::AreaCoverageScore() {
map[100][100][2] = 10;
    uint8_t* dynamicMap = new uint8_t[2500000000];
}

AreaCoverageScore::~AreaCoverageScore() {
    gzdbg << "# of percepted polygons:" << perceptedPolygons.size() << std::endl;
}

void AreaCoverageScore::GetParameters(sdf::ElementPtr worldSdf, sdf::ElementPtr ownSDF) {
// we assume that every uav can have different maximum height for perception

    // iterate over all models and if we have model with name iris or zephyr get its max_height uav_sensor
    sdf::ElementPtr modelSdf = worldSdf->GetElement("model");
    while (modelSdf != NULL) {
        std::string modelName = modelSdf->GetAttribute("name")->GetAsString();
        gzdbg << "checking model:" << modelName << std::endl;
        // we would like to find models having name zephyr or iris
        if (modelName.find("zephyr") != std::string::npos || modelName.find("iris") != std::string::npos) {
            sdf::ElementPtr pluginSdf = modelSdf->GetElement("plugin");
            while (pluginSdf != NULL) {
                std::string pluginName = pluginSdf->GetAttribute("name")->GetAsString();
                // we would like to find zephyr_controller or iris_controller plugins
                if (pluginName.find("zephyr") != std::string::npos || pluginName.find("iris") != std::string::npos) {
                    gzdbg << "checking plugin:" << pluginName << std::endl;
                    float maxHeight = pluginSdf->GetElement("uav_sensor")->Get<float>("max_height");
                    float minHeight = pluginSdf->GetElement("uav_sensor")->Get<float>("min_height");

                    modelPerceptionHeights.insert(std::pair<std::string, std::pair<float, float> >
                                                          (modelName, std::pair<float, float>(minHeight, maxHeight)));

                    // create model frustum
                    ignition::math::Frustum* frustum = new ignition::math::Frustum();
                    frustum->SetFOV(pluginSdf->GetElement("uav_sensor")->Get<double>("hfov"));
                    frustum->SetAspectRatio(pluginSdf->GetElement("uav_sensor")->Get<double>("aspect_ratio"));
                    frustum->SetNear(pluginSdf->GetElement("uav_sensor")->Get<double>("near"));
                    frustum->SetFar(pluginSdf->GetElement("uav_sensor")->Get<double>("far"));

                    modelFrustums.insert(std::pair<std::string, ignition::math::Frustum*>(modelName, frustum));
                    gzdbg << "model:" << modelName << " min_height:" << minHeight << " max_height:" << maxHeight << std::endl;
                    break;
                }
                pluginSdf = pluginSdf->GetNextElement("plugin");
            }
        }
        modelSdf = modelSdf->GetNextElement("model");
    }
    updateRate = ownSDF->Get<int>("update_rate");
}

void AreaCoverageScore::SetModels(std::map<std::string, physics::ModelPtr> _models) {
    models = _models;
}

void AreaCoverageScore::SetWorld(physics::WorldPtr _worldPtr) {
    worldPtr = _worldPtr;
}

void AreaCoverageScore::UpdateStates() {

    common::Time currTime = worldPtr->SimTime();
    double dt_ = (currTime - lastUpdateTime).Double() * 1000; // miliseconds
    if (dt_ > updateRate) {
        std::map<std::string, physics::ModelPtr>::iterator it;
        for (it = models.begin(); it != models.end(); it++) {
            float minHeight = modelPerceptionHeights[it->first].first;
            float maxHeight = modelPerceptionHeights[it->first].second;
            ignition::math::Pose3d uavPose = it->second->WorldPose();

            if (uavPose.Pos().Z() < maxHeight && uavPose.Pos().Z() > minHeight) {

                ignition::math::Frustum *frustum = modelFrustums[it->first];
                ignition::math::Pose3d frustumPose(uavPose);

                // correct the pose of the frustum and set it towards the bottom of the uav
                ignition::math::Vector3d rot = frustumPose.Rot().Euler();
                // rot.X(rot.X() + 1.57079);
                rot.Y(rot.Y() + 1.57079);
                rot.Z(rot.Z() - 1.57079);

                frustumPose.Rot().Euler(rot);

                frustum->SetPose(frustumPose);

                // calculate the intersection of frustum lines on the floor plane
                float farWidth = tan(frustum->FOV().Radian() / 2) * frustumPose.Pos().Z();
                float farHeight = farWidth / frustum->AspectRatio();

                ignition::math::Vector2d uavPoint(uavPose.Pos().X(), uavPose.Pos().Y());

                // calculate rectangle points from frustum width and height
                Polygon_2 p;
                ignition::math::Vector2d point(-farWidth, -farHeight);
                Util::Rotate(point, uavPose.Rot().Euler().Z());
                point += uavPoint;
                p.push_back(Point_2(point.X(), point.Y()));

                ignition::math::Vector2d point1(-farWidth, farHeight);
                Util::Rotate(point1, uavPose.Rot().Euler().Z());
                point1 += uavPoint;
                p.push_back(Point_2(point1.X(), point1.Y()));

                ignition::math::Vector2d point2(farWidth, farHeight);
                Util::Rotate(point2, uavPose.Rot().Euler().Z());
                point2 += uavPoint;
                p.push_back(Point_2(point2.X(), point2.Y()));

                ignition::math::Vector2d point3(farWidth, -farHeight);
                Util::Rotate(point3, uavPose.Rot().Euler().Z());
                point3 += uavPoint;
                p.push_back(Point_2(point3.X(), point3.Y()));

                if (p.is_clockwise_oriented()) {
                    p.reverse_orientation();
                }

//                perceptedPolygonSet.insert(p);
                perceptedPolygons.push_back(p);
            }
        }
        lastUpdateTime = currTime;

//            high_resolution_clock::time_point t1 = high_resolution_clock::now();
//
//            Polygon_with_holes_2 unionP;
//            int replaceIndex = -1;
//            for (unsigned int i = 0; i < perceptedPolygons.size(); i++) {
//                Polygon_with_holes_2& p2 = perceptedPolygons.at(i);
//                if (CGAL::join(p, p2, unionP)) {
//                    replaceIndex = i;
//                    break;
//                }
//            }
//            if (replaceIndex == -1) {
//                perceptedPolygons.push_back(Polygon_with_holes_2(p));
//            } else {
//                perceptedPolygons.at(replaceIndex) = unionP;
//            }
//            gzdbg << "perceptedPolygon count:" << perceptedPolygons.size() << std::endl;
//            high_resolution_clock::time_point t2 = high_resolution_clock::now();
//            auto duration = duration_cast<microseconds>( t2 - t1 ).count();
//            gzdbg << "elapsed time:" << duration << " nano second." << std::endl;
    }
}

double AreaCoverageScore::CalculateScore() {

    std::vector<Polygon_2> perceptedPolygonsCopy;
    perceptedPolygonsCopy.swap(perceptedPolygons);

    Polygon_2 p1;
    p1.push_back(Point_2(1, 1));
    p1.push_back(Point_2(1, 3));
    p1.push_back(Point_2(4, 3));
    p1.push_back(Point_2(4, 1));
    if (p1.is_clockwise_oriented()) {
        p1.reverse_orientation();
    }

    Polygon_2 p2;
    p2.push_back(Point_2(2, 2));
    p2.push_back(Point_2(2.5, 6));
    p2.push_back(Point_2(7, 4));
    p2.push_back(Point_2(6, 1.5));
    if (p2.is_clockwise_oriented()) {
        p2.reverse_orientation();
    }

    CGAL::set_pretty_mode(std::cout);
    gzdbg << p1 << std::endl;
    gzdbg << p2 << std::endl;
    gzdbg << "p1.area:" << fabs(CGAL::to_double(p1.area())) << std::endl;
    gzdbg << "p2.area:" << fabs(CGAL::to_double(p2.area())) << std::endl;

    Polygon_with_holes_2 unionP;

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    if (CGAL::join(p1, p2, unionP)) {
        gzdbg << "union.area:" << fabs(CGAL::to_double(unionP.outer_boundary().area())) << std::endl;
        gzdbg << "p.union:" << unionP.outer_boundary() << std::endl;
    }

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
    gzdbg << "elapsed time:" << (duration / 1000.0) << " milli second." << std::endl;

    high_resolution_clock::time_point t3 = high_resolution_clock::now();
//    MyJoin(p1, p2);
    high_resolution_clock::time_point t4 = high_resolution_clock::now();
    auto duration2 = duration_cast<microseconds>( t4 - t3 ).count();
    gzdbg << "myjoin elapsed time:" << (duration2 / 1000.0) << " milli second." << std::endl;

    high_resolution_clock::time_point t5 = high_resolution_clock::now();
    MyJoin2(p1, p2);
    high_resolution_clock::time_point t6 = high_resolution_clock::now();
    auto duration3 = duration_cast<microseconds>( t6 - t5 ).count();
    gzdbg << "myjoin elapsed time:" << (duration3 / 1000.0) << " milli second." << std::endl;



//    perceptedPolygonSet.join();



//    high_resolution_clock::time_point t2 = high_resolution_clock::now();
//    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
//    gzdbg << "elapsed time to copy:" << (duration / 1000.0) << " milli second." << std::endl;
//
//    if (perceptedPolygonsCopy.size() < 2) {
//        return CGAL::to_double(perceptedPolygonsCopy.at(0).area());
//    }
//    Polygon_with_holes_2 p(perceptedPolygonsCopy.back());
//    perceptedPolygonsCopy.pop_back();
//
//
//
//    int counter = 0;
//    while (perceptedPolygonsCopy.size() > 0) {
//        Polygon_2 p2 = perceptedPolygonsCopy.back();
//        perceptedPolygonsCopy.pop_back();
//        if (!CGAL::join(p, p2, unionP)) {
//            gzdbg << "impossible case two intercepted polygons do not intersects" << std::endl;
//        }
//        p = unionP;
//        gzdbg << "processing " << counter << " polygon and left " << perceptedPolygonsCopy.size() << std::endl;
//        counter++;
//    }
//    high_resolution_clock::time_point t3 = high_resolution_clock::now();
//    auto duration2 = duration_cast<microseconds>( t3 - t1 ).count();
//    gzdbg << "total elapsed time:" << (duration2 / 1000.0) << " milli second." << std::endl;
//
//    return CGAL::to_double(p.outer_boundary().area());

    return 0;

}

void AreaCoverageScore::MyJoin(Polygon_2 p1, Polygon_2 p2) {
    Polygon_2 unionP;
    EdgeIterator ei1;
    VertexIterator vi1 = p1.vertices_begin();
    for (ei1 = p1.edges_begin(); ei1 != p1.edges_end(); ei1++) {
        Point_2 intersection;
        EdgeIterator ei2;
        if (IsIntersection(*ei1, p2, intersection, ei2)) {
            if (IsInside(p2, ei1->source())) {
                unionP.push_back(intersection);
            } else {
                unionP.push_back(ei1->source());
                unionP.push_back(intersection);
            }
        }

        if (IsInside(p2, ei1->source())) {

        } else {

        }
//        gzdbg << "p1 edge:" << *ei1 << std::endl;
//        EdgeIterator ei2;
//        bool intersected = false;
//        Point_2 intersectionPoint;
//        for (ei2 = p2.edges_begin(); ei2 != p2.edges_end(); ei2++) {
//            CGAL::Object result = CGAL::intersection(*ei1, *ei2);
//            if (CGAL::assign(intersectionPoint, result)) {
//                gzdbg << "intersected e1:" << *ei1 << " e2:" << *ei2 << " point:" << intersectionPoint << std::endl;
//                intersected = true;
//                break;
//            }
//        }
////        unionP.push_back(*ei1[0]);
//        if (intersected) {
//            unionP.push_back(intersectionPoint);
//        }
    }

    for (unsigned int i = 0; i < p1.size(); i++) {
        gzdbg << "p1.x:" << p1[i][0] << " p1.y:" << p1[i][1] << std::endl;
        //if (p2.)
        switch(CGAL::bounded_side_2(p2.vertices_begin(), p2.vertices_end(), p1[i], Kernel())) {
            case CGAL::ON_BOUNDED_SIDE :
                gzdbg << " is inside the polygon.\n";
                break;
            case CGAL::ON_BOUNDARY:
            case CGAL::ON_UNBOUNDED_SIDE:
                gzdbg << " is outside the polygon.\n";
                unionP.push_back(p1[i]);
                break;
        }
    }

    EdgeIterator ei;
    for (ei = unionP.edges_begin(); ei != unionP.edges_end(); ei++) {
        gzdbg << "union 1 edge:" << *ei << std::endl;
    }

    if (unionP.is_clockwise_oriented()) {
        unionP.reverse_orientation();
    }

    for (ei = unionP.edges_begin(); ei != unionP.edges_end(); ei++) {
        gzdbg << "union 2 edge:" << *ei << std::endl;
    }

    gzdbg << "union area:" << fabs(CGAL::to_double(unionP.area())) << std::endl;
}

void AreaCoverageScore::MyJoin2(Polygon_2 p1, Polygon_2 p2) {

    // Compute the intersection of p1 and p2.
    Pwh_list_2                  intR;
    Pwh_list_2::const_iterator  it;
    CGAL::intersection (p1, p2, std::back_inserter(intR));
    std::cout << "The intersection:" << std::endl;
    for (it = intR.begin(); it != intR.end(); ++it) {
        std::cout << "--> " << *it << std::endl;
    }

    double totalArea = CGAL::to_double(p1.area()) + CGAL::to_double(p2.area());
    double unionArea = totalArea - CGAL::to_double((*intR.begin()).outer_boundary().area());
    gzdbg << "totalArea:" << totalArea << " unionArea:" << unionArea << std::endl;

//
//    if ((CGAL::intersection(p1, p2)))
//        std::cout << "The two polygons intersect in their interior." << std::endl;
//    else
//        std::cout << "The two polygons do not intersect." << std::endl;
}

bool AreaCoverageScore::IsInside(Polygon_2& polygon, const Point_2& point) {
    switch (CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), point, Kernel())) {
        case CGAL::ON_BOUNDED_SIDE:
            return true;
            break;
    }
    return false;
}

bool AreaCoverageScore::IsIntersection(const Segment_2& seg1, const Segment_2& seg2, Point_2& intersection) {
    CGAL::Object result = CGAL::intersection(seg1, seg2);
    if (CGAL::assign(intersection, result)) {
        return true;
    }
    return false;
}

bool AreaCoverageScore::IsIntersection(const Segment_2& seg, const Polygon_2& polygon, Point_2& intersection, EdgeIterator& polygonEi) {
    EdgeIterator ei;
    for (ei = polygon.edges_begin(); ei != polygon.edges_end(); ei++) {
        if (IsIntersection(seg, *ei, intersection)) {
            polygonEi = ei;
            return true;
        }
    }
    return false;
}