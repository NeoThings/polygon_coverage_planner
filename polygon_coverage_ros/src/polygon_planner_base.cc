/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "polygon_coverage_ros/polygon_planner_base.h"
#include "polygon_coverage_ros/ros_interface.h"

#include <functional>

#include <polygon_coverage_msgs/msg_from_xml_rpc.h>
#include <polygon_coverage_planners/cost_functions/path_cost_functions.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace polygon_coverage_planning {

PolygonPlannerBase::PolygonPlannerBase(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      wall_distance_(0.0),
      path_cost_function_(
          {std::bind(&computeEuclideanPathCost, std::placeholders::_1),
           CostFunctionType::kDistance}),
      latch_topics_(true),
      global_frame_id_("world"),
      publish_plan_on_planning_complete_(false),
      publish_visualization_on_planning_complete_(true),
      set_start_goal_from_rviz_(false),
      set_polygon_from_rviz_(true),
      planning_complete_(false) {
  // Initial interactions with ROS
  getParametersFromRos();
  advertiseTopics();

  // Publish RVIZ.
  publishVisualization();
}

void PolygonPlannerBase::advertiseTopics() {
  // Advertising the visualization and planning messages
  marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "path_markers", 1, true);
  waypoint_list_pub_ = nh_.advertise<geometry_msgs::PoseArray>(
      "waypoint_list", 1, latch_topics_);
  coverage_plan_pub_ = nh_.advertise<nav_msgs::Path>("coverage_plan", 1);
  interpolated_plan_pub_ = nh_.advertise<nav_msgs::Path>("interpolated_plan", 1);
  // Services for generating the plan.
  set_polygon_srv_ = nh_private_.advertiseService(
      "set_polygon", &PolygonPlannerBase::setPolygonCallback, this);
  convert_map_to_polygon_srv_ = nh_private_.advertiseService(
      "convert_map_to_polygon", &PolygonPlannerBase::convertMap2PolygonCallback, this);
  query_interpolated_path_srv_ = nh_private_.advertiseService(
    "query_interpolated_path", &PolygonPlannerBase::queryInterpolatedPathCallback, this);
  plan_path_srv_ = nh_private_.advertiseService(
      "plan_path", &PolygonPlannerBase::planPathCallback, this);
  // Services for performing publishing and visualization
  publish_all_srv_ = nh_private_.advertiseService(
      "publish_all", &PolygonPlannerBase::publishAllCallback, this);
  publish_visualization_srv_ = nh_private_.advertiseService(
      "publish_visualization",
      &PolygonPlannerBase::publishVisualizationCallback, this);
  publish_plan_points_srv_ = nh_private_.advertiseService(
      "publish_path_points",
      &PolygonPlannerBase::publishTrajectoryPointsCallback, this);
  // Subscribe
  clicked_point_sub_ = nh_.subscribe(
      "/clicked_point", 1, &PolygonPlannerBase::clickPointCallback, this);
  polygon_sub_ = nh_.subscribe("/polygon", 1,
                               &PolygonPlannerBase::clickPolygonCallback, this);
  map_sub_ = nh_.subscribe("/map", 1, 
                           &PolygonPlannerBase::subMapCallback, this);
}

void PolygonPlannerBase::getParametersFromRos() {
  // Load the polygon from polygon message from parameter server.
  // The altitude and the global frame ID are set from the same message.
  XmlRpc::XmlRpcValue polygon_xml_rpc;
  const std::string polygon_param_name = "polygon";
  if (nh_private_.getParam(polygon_param_name, polygon_xml_rpc)) {
    polygon_coverage_msgs::PolygonWithHolesStamped poly_msg;
    if (PolygonWithHolesStampedMsgFromXmlRpc(polygon_xml_rpc, &poly_msg)) {
      PolygonWithHoles temp_pwh;
      double temp_alt;
      if (polygonFromMsg(poly_msg, &temp_pwh, &temp_alt, &global_frame_id_)) {
        ROS_INFO_STREAM("Successfully loaded polygon.");
        ROS_INFO_STREAM("Altitude: " << temp_alt << " m");
        ROS_INFO_STREAM("Global frame: " << global_frame_id_);
        ROS_INFO_STREAM("Polygon:" << temp_pwh);
        polygon_ = std::make_optional(temp_pwh);
        altitude_ = std::make_optional(temp_alt);
      }
    } else {
      ROS_WARN_STREAM("Failed reading polygon message from parameter server.");
    }
  } else {
    ROS_WARN_STREAM(
        "No polygon file specified to parameter "
        "server (parameter \""
        << polygon_param_name
        << "\"). Expecting "
           "polygon from service call.");
  }

  // Getting control params from the server
  if (!nh_private_.getParam("wall_distance", wall_distance_)) {
    ROS_WARN_STREAM("No wall distance specified. Using default value of: "
                    << wall_distance_);
  } else
    ROS_INFO_STREAM("Wall distance: " << wall_distance_ << " m");

  // Cost function
  double temp_v_max;
  if (nh_private_.getParam("v_max", temp_v_max)) {
    v_max_ = std::make_optional(temp_v_max);
  }
  double temp_a_max;
  if (nh_private_.getParam("a_max", temp_a_max)) {
    a_max_ = std::make_optional(temp_a_max);
  }

  // Cost function type.
  int cost_function_type_int = static_cast<int>(path_cost_function_.second);
  if (!nh_private_.getParam("cost_function_type", cost_function_type_int)) {
    ROS_WARN_STREAM("No cost_function_type specified. Using default value of: "
                    << path_cost_function_.second << "("
                    << getCostFunctionTypeName(path_cost_function_.second)
                    << ").");
  }
  if (!checkCostFunctionTypeValid(cost_function_type_int)) {
    ROS_WARN_STREAM("cost_function_type not valid. Resetting to default: "
                    << path_cost_function_.second << "("
                    << getCostFunctionTypeName(path_cost_function_.second)
                    << ").");
    cost_function_type_int = static_cast<int>(path_cost_function_.second);
  }
  path_cost_function_.second =
      static_cast<CostFunctionType>(cost_function_type_int);

  ROS_INFO_STREAM(
      "Cost function: " << getCostFunctionTypeName(path_cost_function_.second));
  if (path_cost_function_.second == CostFunctionType::kTime) {
    if (!v_max_.has_value() || !a_max_.has_value()) {
      ROS_WARN_COND(!v_max_.has_value(), "Velocity 'v_max' not set.");
      ROS_WARN_COND(!a_max_.has_value(), "Acceleration 'a_max' not set.");
      ROS_WARN("Falling back to distance cost function.");
      path_cost_function_.second = CostFunctionType::kDistance;
    } else {
      ROS_INFO_STREAM("v_max: " << v_max_.value()
                                << ", a_max: " << a_max_.value());
    }
  }

  switch (path_cost_function_.second) {
    case CostFunctionType::kDistance: {
      path_cost_function_.first =
          std::bind(&computeEuclideanPathCost, std::placeholders::_1);
      break;
    }
    case CostFunctionType::kTime: {
      path_cost_function_.first =
          std::bind(&computeVelocityRampPathCost, std::placeholders::_1,
                    v_max_.value(), a_max_.value());
      break;
    }
    case CostFunctionType::kWaypoints: {
      path_cost_function_.first =
          std::bind(&computeWaypointsPathCost, std::placeholders::_1);
      break;
    }
    default: {
      ROS_WARN_STREAM("Cost function type: "
                      << getCostFunctionTypeName(path_cost_function_.second)
                      << "not implemented. Using euclidean distance.");
      break;
    }
  }

  // Getting the behaviour flags
  nh_private_.getParam("latch_topics", latch_topics_);
  nh_private_.getParam("publish_plan_on_planning_complete",
                       publish_plan_on_planning_complete_);
  nh_private_.getParam("publish_visualization_on_planning_complete",
                       publish_visualization_on_planning_complete_);
  nh_private_.getParam("global_frame_id", global_frame_id_);
  nh_private_.getParam("set_start_goal_from_rviz", set_start_goal_from_rviz_);
  nh_private_.getParam("set_polygon_from_rviz", set_polygon_from_rviz_);
}

void PolygonPlannerBase::solve(const Point_2& start, const Point_2& goal) {
  ROS_INFO_STREAM("Start solving.");
  if ((planning_complete_ = solvePlanner(start, goal))) {
    ROS_INFO_STREAM("Finished plan."
                    << std::endl
                    << "Optimization Criterion: "
                    << getCostFunctionTypeName(path_cost_function_.second)
                    << std::endl
                    << "Number of waypoints: " << solution_.size() << std::endl
                    << "Start point: " << start << std::endl
                    << "Goal point: " << goal << std::endl
                    << "Altitude: " << altitude_.value() << " [m]" << std::endl
                    << "Path length: " << computeEuclideanPathCost(solution_)
                    << " [m]");
    if (v_max_.has_value() && a_max_.has_value())
      ROS_INFO_STREAM("Path time: "
                      << computeVelocityRampPathCost(solution_, v_max_.value(),
                                                     a_max_.value())
                      << " [s]");

    // Publishing the plan if requested
    if (publish_plan_on_planning_complete_) {
      publishTrajectoryPoints();
    }
    // Publishing the visualization if requested
    if (publish_visualization_on_planning_complete_) {
      publishVisualization();
    }

    publishPlan();
  } else {
    ROS_ERROR_STREAM("Failed calculating plan.");
  }
}

//publishClusteredPlan
  // if (clustered_solution_.size() > 1) {
  //   for (unsigned int num = 1; num < clustered_solution_.size(); num++) {
  //     if (!clustered_solution_[num].empty()) {
  //       nav_msgs::Path path;
  //       path.poses.resize(clustered_solution_[num].size());

  //       path.header.frame_id = global_frame_id_;
  //       path.header.stamp = ros::Time::now();

  //       for (unsigned int i = 0; i < clustered_solution_[num].size(); i++) {
  //         path.poses[i].pose.position.x = CGAL::to_double(clustered_solution_[num][i].x());
  //         path.poses[i].pose.position.y = CGAL::to_double(clustered_solution_[num][i].y());
  //         path.poses[i].pose.position.z = 0.0;
  //       }
  //       coverage_plan_pub_.publish(path);
  //     }
  //   }
  // }

void PolygonPlannerBase::subMapCallback(const nav_msgs::OccupancyGrid& msg) {
  map_origin_x_ = msg.info.origin.position.x;
  map_origin_y_ = msg.info.origin.position.y;
  std::cout << "origin_x: " << map_origin_x_ << " " << "origin_y: " << map_origin_y_ << std::endl;
}

void PolygonPlannerBase::publishPlan() {

    nav_msgs::Path path;
    path.poses.resize(solution_.size());

    path.header.frame_id = global_frame_id_;
    path.header.stamp = ros::Time::now();

    for (unsigned int i = 0; i < solution_.size(); i++) {
        path.poses[i].pose.position.x = CGAL::to_double(solution_[i].x());
        path.poses[i].pose.position.y = CGAL::to_double(solution_[i].y());
        path.poses[i].pose.position.z = 0.0;
    }
    coverage_plan_pub_.publish(path);

}

// PathArray PolygonPlannerBase::generatePathFromWaypoints() {
//     float interval = 0.05;
//     for (auto it = clustered_solution_.begin() + 1; it!=clustered_solution_.end(); it++) {
//       if (!it->empty()) {
//         if
//       }
//     }

//   }

// LCTODO: delete when no need
// void PolygonPlannerBase::checkPathDist() {
//   partition_solution_.clear();
//   auto it = solution_.begin() + 1;
//   if (solution_.size() > 5) {
//     for (; it < (solution_.end() - 4);) {
//       Line_2 sweep_line(*it, *(it+1));
//       FT p1_dist = CGAL::squared_distance(*(it+2), sweep_line);
//       FT p2_dist = CGAL::squared_distance(*(it+3), sweep_line);
//       if (p1_dist == p2_dist) {
//         it += 2;
//       } else {
//         it += 3;
//       }
//       std::cout << "dist is: " << p1_dist << std::endl;

//     }
//   }
// }

bool PolygonPlannerBase::publishVisualization() {
  ROS_INFO_STREAM("Sending visualization messages.");

  // Delete old markers.
  for (visualization_msgs::Marker& m : markers_.markers)
    m.action = visualization_msgs::Marker::DELETE;
  marker_pub_.publish(markers_);

  // Create new markers.
  markers_.markers.clear();
  // The planned path:
  visualization_msgs::Marker path_points, path_line_strips;
  const double kPathLineSize = 0.1;
  const double kPathPointSize = 0.1;
  if (!altitude_.has_value()) {
    ROS_WARN_STREAM("Cannot send visualization because altitude not set.");
    return false;
  }

  if (!planning_complete_) {
    ROS_WARN_STREAM(
        "Cannot send solution visualization because plan has not been made.");
  } else {
    createMarkers(solution_, altitude_.value(), global_frame_id_,
                  "vertices_and_strip", Color::Red(), Color::Orange(),
                  kPathLineSize, kPathPointSize, &path_points,
                  &path_line_strips);
    markers_.markers.push_back(path_points);
    markers_.markers.push_back(path_line_strips);

    // Start and end points
    visualization_msgs::Marker start_point, end_point;
    createStartAndEndPointMarkers(solution_.front(), solution_.back(),
                                  altitude_.value(), global_frame_id_, "points",
                                  &start_point, &end_point);
    markers_.markers.push_back(start_point);
    markers_.markers.push_back(end_point);

    // Start and end text.
    visualization_msgs::Marker start_text, end_text;
    createStartAndEndTextMarkers(solution_.front(), solution_.back(),
                                 altitude_.value(), global_frame_id_, "points",
                                 &start_text, &end_text);
    markers_.markers.push_back(start_text);
    markers_.markers.push_back(end_text);
  }

  // The original polygon:
  const double kPolygonLineSize = 0.1;
  visualization_msgs::MarkerArray polygon;
  if (!polygon_.has_value()) {
    ROS_WARN_STREAM("Cannot send visualization because polygon not set.");
    return false;
  }
  createPolygonMarkers(polygon_.value(), altitude_.value(), global_frame_id_,
                       "polygon", Color::Black(), Color::Blue(),
                       kPolygonLineSize, kPolygonLineSize, &polygon);
  markers_.markers.insert(markers_.markers.end(), polygon.markers.begin(),
                          polygon.markers.end());

  // The decomposed polygons.
  visualization_msgs::MarkerArray decomposition_markers =
      createDecompositionMarkers();
  markers_.markers.insert(markers_.markers.end(),
                          decomposition_markers.markers.begin(),
                          decomposition_markers.markers.end());

  // Publishing
  marker_pub_.publish(markers_);

  // Success
  return true;
}

bool PolygonPlannerBase::publishTrajectoryPoints() {
  if (!planning_complete_) {
    ROS_WARN("Cannot send trajectory messages because plan has not been made.");
    return false;
  }
  ROS_INFO_STREAM("Sending trajectory messages");

  // Convert path to pose array.
  geometry_msgs::PoseArray trajectory_points_pose_array;
  if (!altitude_.has_value()) {
    ROS_WARN_STREAM("Cannot send trajectory because altitude not set.");
    return false;
  }
  poseArrayMsgFromPath(solution_, altitude_.value(), global_frame_id_,
                       &trajectory_points_pose_array);

  // Publishing
  waypoint_list_pub_.publish(trajectory_points_pose_array);

  // Success
  return true;
}

bool PolygonPlannerBase::queryInterpolatedPathCallback(std_srvs::Empty::Request& req,
                                                       std_srvs::Empty::Response& res) {
  //check solution is not empty

  nav_msgs::Path path;

  path.header.frame_id = global_frame_id_;
  path.header.stamp = ros::Time::now();

  double intervel_dist = 0.05;
  for (auto it = solution_.begin() + 1; it != solution_.end(); it++) {
    auto start_pose_x = CGAL::to_double((it - 1)->x());
    auto start_pose_y = CGAL::to_double((it - 1)->y());
    auto end_pose_x = CGAL::to_double(it->x());
    auto end_pose_y = CGAL::to_double(it->y());
    double line_dist = hypot(end_pose_x - start_pose_x, end_pose_y - start_pose_y);
    int intervel_num = floor(line_dist / intervel_dist);
    if (end_pose_x != start_pose_x) {
      // double slope = (end_pose_y - start_pose_y) / (end_pose_x - start_pose_x);
      double theta = atan2((end_pose_y - start_pose_y), (end_pose_x - start_pose_x));
      for (int i = 0; i < intervel_num - 1; i++) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = start_pose_x + intervel_dist * cos(theta) * i;
        pose_stamped.pose.position.y = start_pose_y + intervel_dist * sin(theta) * i;
        pose_stamped.pose.position.z = 0.0;
        path.poses.push_back(pose_stamped);
      }
    } else { //atan can handle delta x = 0, so 这个else可以不去处理
      geometry_msgs::PoseStamped pose_stamped;
      if (end_pose_y > start_pose_y) {  
        pose_stamped.pose.position.x = start_pose_x;
        pose_stamped.pose.position.y = start_pose_y + intervel_dist;
      } else {
        pose_stamped.pose.position.x = start_pose_x;
        pose_stamped.pose.position.y = start_pose_y - intervel_dist;
      }
      pose_stamped.pose.position.z = 0.0;
      path.poses.push_back(pose_stamped);
    }
  }
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose.position.x = CGAL::to_double(solution_.back().x());
  pose_stamped.pose.position.y = CGAL::to_double(solution_.back().y());
  pose_stamped.pose.position.z = 0.0;
  path.poses.push_back(pose_stamped);

  interpolated_plan_pub_.publish(path);
  return true;
}

bool PolygonPlannerBase::setPolygonCallback(
    polygon_coverage_msgs::PolygonService::Request& request,
    polygon_coverage_msgs::PolygonService::Response& response) {
  PolygonWithHoles temp_pwh;
  double temp_alt;
  if (!polygonFromMsg(request.polygon, &temp_pwh, &temp_alt,
                      &global_frame_id_)) {
    ROS_ERROR_STREAM("Failed loading correct polygon.");
    ROS_ERROR_STREAM("Planner is in an invalid state.");
    polygon_.reset();
    return false;
  }
  polygon_ = std::make_optional(temp_pwh);
  altitude_ = std::make_optional(temp_alt);

  ROS_INFO_STREAM("Successfully loaded polygon.");
  ROS_INFO_STREAM("Altitude: " << altitude_.value() << "m");
  ROS_INFO_STREAM("Global frame: " << global_frame_id_);
  ROS_INFO_STREAM("Polygon:" << polygon_.value());

  response.success = resetPlanner();
  return true;  // Still return true to identify service has been reached.
}

bool PolygonPlannerBase::convertMap2PolygonCallback(std_srvs::Empty::Request& req,
                                                    std_srvs::Empty::Response& res) {
  auto temp_pwh = convertMap2Polygon();
  double temp_alt = 0.0;

  polygon_ = std::make_optional(temp_pwh);
  altitude_ = std::make_optional(temp_alt);

  ROS_INFO_STREAM("Successfully convert map to polygon.");
  ROS_INFO_STREAM("Altitude: " << altitude_.value() << "m");
  ROS_INFO_STREAM("Global frame: " << global_frame_id_);
  ROS_INFO_STREAM("Polygon:" << polygon_.value());

  // res.success = resetPlanner();
  resetPlanner();
  return true;
}

PolygonWithHoles PolygonPlannerBase::convertMap2Polygon() {

    cv::Mat img = cv::imread("/home/neo/demobot_ws/src/DemoBot/demobot_tools/polygon_approximation/map/1515.pgm");

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    cv::Mat img_ = gray.clone();

    cv::threshold(img_, img_, 250, 255, cv::THRESH_BINARY);

    cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(12,12), cv::Point(-1,-1)); // size: robot radius
    cv::morphologyEx(img_, img_, cv::MORPH_ERODE, erode_kernel);

    cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5), cv::Point(-1,-1));
    cv::morphologyEx(img_, img_, cv::MORPH_OPEN, open_kernel);

    // cv::imshow("preprocess", img_);
    // cv::waitKey(0);

    std::vector<std::vector<cv::Point>> cnts;
    std::vector<cv::Vec4i> hierarchy; // index: next, prev, first_child, parent
    cv::findContours(img_, cnts, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE); //try to use RETR_CCOMP
    
    //Sort contour from larger area to small
    std::vector<int> cnt_indices(cnts.size());
    std::iota(cnt_indices.begin(), cnt_indices.end(), 0);
    std::sort(cnt_indices.begin(), cnt_indices.end(), [&cnts](int lhs, int rhs){return cv::contourArea(cnts[lhs]) > cv::contourArea(cnts[rhs]);});
    int ext_cnt_idx = cnt_indices.front();
    std::cout << "frist contour: " << ext_cnt_idx  << std::endl;

    cv::Mat cnt_canvas = img.clone();
    //draw external contour (polygon)
    cv::drawContours(cnt_canvas, cnts, ext_cnt_idx, cv::Scalar(0,0,255));
    std::vector<std::vector<cv::Point>> contours;
    contours.emplace_back(cnts[ext_cnt_idx]);

    //find all the contours of obstacle (holes)
    for(int i = 0; i < hierarchy.size(); i++){
        if(hierarchy[i][3]==ext_cnt_idx){ // parent contour's index equals to external contour's index
            contours.emplace_back(cnts[i]);
            cv::drawContours(cnt_canvas, cnts, i, cv::Scalar(255,0,0));
        }
    }
    
    // cv::imshow("contours", cnt_canvas);
    // cv::waitKey(0);
    
    cv::Mat poly_canvas = img.clone();

    std::vector<cv::Point> poly;
    std::vector<std::vector<cv::Point>> polys;
    for(auto& contour : contours){
        cv::approxPolyDP(contour, poly, 3, true);
        polys.emplace_back(poly);
        poly.clear();
    }
    for(int i = 0; i < polys.size(); i++){
        cv::drawContours(poly_canvas, polys, i, cv::Scalar(255,0,255));
    }

    // construct polygon with holes
    std::vector<cv::Point> outer_poly = polys.front();
    polys.erase(polys.begin());
    std::vector<std::vector<cv::Point>> inner_polys = polys;

    Polygon_2 outer_polygon;
    for(const auto& point : outer_poly){
        outer_polygon.push_back(Point_2(point.x, point.y));
    }

    int num_holes = inner_polys.size();
    std::vector<Polygon_2> holes(num_holes);
    for(int i = 0; i < inner_polys.size(); i++){
        for(const auto& point : inner_polys[i]){
            holes[i].push_back(Point_2(point.x, point.y));
        }
    }

    double trans_x = img.rows + map_origin_x_*20;
    double trans_y = map_origin_y_*20;
    
    // trans from picture frame to ros grid map frame
    CGAL::Aff_transformation_2<K> transform(1.0, 0.0, -trans_x, 0.0, -1.0, -trans_y, 20);

    for (auto it = outer_polygon.begin(); it != outer_polygon.end(); it++) {
      *it = transform(*it);
    }

    for (auto it = holes.begin(); it != holes.end(); it++) {
      for (auto sub_it = it->begin(); sub_it != it->end(); sub_it++) {
        *sub_it = transform(*sub_it);
      }
    }

    PolygonWithHoles pwh(outer_polygon, holes.begin(), holes.end());

    return pwh;
}

bool PolygonPlannerBase::planPathCallback(
    polygon_coverage_msgs::PlannerService::Request& request,
    polygon_coverage_msgs::PlannerService::Response& response) {
  planning_complete_ = false;
  if (!polygon_.has_value()) {
    ROS_WARN("Polygon not set. Cannot plan path.");
    response.success = planning_complete_;
    return true;
  }
  const Point_2 start(request.start_pose.pose.position.x,
                      request.start_pose.pose.position.y);
  const Point_2 goal(request.goal_pose.pose.position.x,
                     request.goal_pose.pose.position.y);
  solve(start, goal);  // Calculate optimal path.
  if (altitude_.has_value()) {
    msgMultiDofJointTrajectoryFromPath(solution_, altitude_.value(),
                                       &response.sampled_plan);
  } else {
    ROS_WARN("Cannot plan path. Altitude not set.");
  }
  response.success = planning_complete_;
  return true;
}

bool PolygonPlannerBase::publishAllCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  bool success_publish_trajectory = publishTrajectoryPoints();
  bool success_publish_visualization = publishVisualization();
  return (success_publish_trajectory && success_publish_visualization);
}

bool PolygonPlannerBase::publishVisualizationCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  return publishVisualization();
}

bool PolygonPlannerBase::publishTrajectoryPointsCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  return publishTrajectoryPoints();
}

// Reset the planner when a new polygon is set.
bool PolygonPlannerBase::resetPlanner() {
  ROS_ERROR_STREAM("resetPlanner is not implemented.");
  return false;
}

void PolygonPlannerBase::clickPointCallback(
    const geometry_msgs::PointStampedConstPtr& msg) {
  if (!set_start_goal_from_rviz_) return;

  if (!start_.has_value()) {
    ROS_INFO("Selecting START from RVIZ PublishPoint tool.");
    start_ = std::make_optional<Point_2>(msg->point.x, msg->point.y);
  } else if (!goal_.has_value()) {
    ROS_INFO("Selecting GOAL from RVIZ PublishPoint tool.");
    goal_ = std::make_optional<Point_2>(msg->point.x, msg->point.y);
  }

  if (start_.has_value() && goal_.has_value()) {
    solve(start_.value(), goal_.value());
    start_.reset();
    goal_.reset();
  }

  return;
}

void PolygonPlannerBase::clickPolygonCallback(
    const polygon_coverage_msgs::PolygonWithHolesStamped& msg) {
  if (!set_polygon_from_rviz_) return;

  ROS_INFO("Updating polygon from RVIZ polygon tool.");
  PolygonWithHoles temp_pwh;
  double temp_alt;
  if (polygonFromMsg(msg, &temp_pwh, &temp_alt, &global_frame_id_)) {
    ROS_INFO_STREAM("Successfully loaded polygon.");
    ROS_INFO_STREAM("Altitude: " << temp_alt << " m");
    ROS_INFO_STREAM("Global frame: " << global_frame_id_);
    ROS_INFO_STREAM("Polygon:" << temp_pwh);
    polygon_ = std::make_optional(temp_pwh);
    altitude_ = std::make_optional(temp_alt);
  }

  planning_complete_ = false;
  resetPlanner();
  publishVisualization();

  return;
}

}  // namespace polygon_coverage_planning
