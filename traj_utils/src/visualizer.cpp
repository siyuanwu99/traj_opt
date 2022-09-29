/**
 * @file visualizer.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-08-08
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <traj_utils/visualizer.hpp>

namespace visualizer {

/**
 * @brief publish trajectory and color the speed
 *
 * @param start_pos when planning starts
 * @param traj       trajectory to be visualized
 * @param max_vel    maximum velocity to visualize as red line
 */
void Visualizer::visualizePolyTraj(const Eigen::Vector3d&        start_pos,
                                   const polynomial::Trajectory& traj,
                                   double                        max_vel) {
  visualization_msgs::Marker traj_marker;
  traj_marker.header.frame_id    = _frame_id;
  traj_marker.header.stamp       = ros::Time::now();
  traj_marker.type               = visualization_msgs::Marker::LINE_LIST;
  traj_marker.pose.orientation.w = 1.00;
  traj_marker.action             = visualization_msgs::Marker::ADD;
  traj_marker.id                 = 0;
  traj_marker.ns                 = "trajectory";
  traj_marker.color.r            = 0.00;
  traj_marker.color.g            = 0.50;
  traj_marker.color.b            = 1.00;
  traj_marker.color.a            = 1.00;
  traj_marker.scale.x            = 0.10;

  double          T     = 0.05;
  Eigen::Vector3d lastX = traj.getPos(0.0) + start_pos;
  for (double t = T; t < traj.getDuration(); t += T) {
    std_msgs::ColorRGBA c;
    Eigen::Vector3d     jets = jetColorMap(traj.getVel(t).norm() / max_vel);
    c.r                      = jets[0];
    c.g                      = jets[1];
    c.b                      = jets[2];

    geometry_msgs::Point point;
    Eigen::Vector3d      X = traj.getPos(t) + start_pos;
    point.x                = lastX(0);
    point.y                = lastX(1);
    point.z                = lastX(2);
    traj_marker.points.push_back(point);
    traj_marker.colors.push_back(c);
    point.x = X(0);
    point.y = X(1);
    point.z = X(2);
    traj_marker.points.push_back(point);
    traj_marker.colors.push_back(c);
    lastX = X;
  }
  _colorful_traj_pub.publish(traj_marker);
}

/**
 * @brief visualize corridors
 * Corridor is given by normal vector and offset
 * @param corridors
 * @param pose
 * @param rviz_map_center_locked true if map center is locked
 * @param clear_corridors
 */
void Visualizer::visualizeCorridors(const traj_utils::Corridors& corridors,
                                    const Eigen::Vector3d&    map_pose) {
  displayCorridors(corridors, map_pose, _corridor_pub, _frame_id);
}

/** NOTE: There are still some bugs in this function
 * @brief if corridor is given in H-representation
 * @param corridors H-representation of corridors
 * @param map_pose
 * @param crd_pub
 * @param frame_id
 */
void displayCorridors(const std::vector<Eigen::MatrixX4d>& corridors,
                      const Eigen::Vector3d&               map_pose,
                      const ros::Publisher&                crd_pub,
                      const std::string                    frame_id = "world") {
  vec_E<Polyhedron3D> polyhedra;
  polyhedra.reserve(corridors.size());
  for (const auto& crd : corridors) {
    Polyhedron3D poly;
    for (int i = 0; i < crd.rows(); i++) {
      Eigen::Vector3d pos;

      double a  = crd(i, 0);
      double b  = crd(i, 1);
      double c  = crd(i, 2);
      double d  = crd(i, 3);
      double z0 = (c == 0) ? 0.0 : -(a + b + c) / c;
      double x0 = (c == 0) ? -(b + d) / a : 1.0;
      pos << x0, 1.0, z0;
      poly.add(Hyperplane3D(pos, crd.row(i).head<3>()));
      // std::cout << "normal: " << gcrd.row(i).head<3>() << " pos: " << pos.transpose() <<
      // std::endl;
    }
    polyhedra.push_back(poly);
  }
  decomp_ros_msgs::PolyhedronArray msg = DecompROS::polyhedron_array_to_ros(polyhedra);
  msg.header.frame_id                  = frame_id;
  msg.header.stamp                     = ros::Time::now();
  crd_pub.publish(msg);
}

/**
 * @brief if corridor is given in H-representation
 * c[0]x + c[1]y + c[2]z + c[3] <= 0
 * @param corridors
 * @param map_pose
 */
void Visualizer::visualizeCorridors(const std::vector<Eigen::MatrixX4d>& corridors,
                                    const Eigen::Vector3d&               map_pose) {
  displayCorridors(corridors, map_pose, _corridor_pub, _frame_id);
}

void Visualizer::visualizePath(const std::vector<Eigen::Vector3d>& route) {
  visualization_msgs::Marker routeMarker;
  routeMarker.id                 = 0;
  routeMarker.type               = visualization_msgs::Marker::LINE_LIST;
  routeMarker.header.stamp       = ros::Time::now();
  routeMarker.header.frame_id    = _frame_id;
  routeMarker.pose.orientation.w = 1.00;
  routeMarker.action             = visualization_msgs::Marker::ADD;
  routeMarker.ns                 = "path";
  routeMarker.color.r            = 1.00;
  routeMarker.color.g            = 0.00;
  routeMarker.color.b            = 0.00;
  routeMarker.color.a            = 1.00;
  routeMarker.scale.x            = 0.05;
  if (route.size() > 0) {
    bool            first = true;
    Eigen::Vector3d last;
    for (auto it : route) {
      if (first) {
        first = false;
        last  = it;
        continue;
      }
      geometry_msgs::Point point;

      point.x = last(0);
      point.y = last(1);
      point.z = last(2);
      routeMarker.points.push_back(point);
      point.x = it(0);
      point.y = it(1);
      point.z = it(2);
      routeMarker.points.push_back(point);
      last = it;
    }
    _astar_path_pub.publish(routeMarker);
  }

}

void Visualizer::visualizeAstarPath(const std::vector<Eigen::Vector3d>& points) {
  visualization_msgs::Marker pt_marker;
  pt_marker.header.frame_id = _frame_id;
  pt_marker.header.stamp    = ros::Time::now();

  pt_marker.type    = visualization_msgs::Marker::POINTS;
  pt_marker.ns      = "astar_path";
  pt_marker.id      = 0;
  pt_marker.action  = visualization_msgs::Marker::ADD;
  pt_marker.scale.x = 0.2;
  pt_marker.scale.y = 0.2;
  pt_marker.scale.z = 0.2;
  pt_marker.color.a = 1.0;
  pt_marker.color.r = 0.8;
  pt_marker.color.g = 0.3;
  pt_marker.color.b = 0.4;

  visualization_msgs::Marker astar_mkr;
  astar_mkr.header.frame_id = _frame_id;
  astar_mkr.header.stamp    = ros::Time::now();

  astar_mkr.type    = visualization_msgs::Marker::LINE_LIST;
  astar_mkr.action  = visualization_msgs::Marker::ADD;
  astar_mkr.ns      = "astar_path";
  astar_mkr.id      = 1;
  astar_mkr.scale.x = 0.1;
  astar_mkr.scale.y = 0.1;
  astar_mkr.scale.z = 0.1;
  astar_mkr.color.a = 0.1;
  astar_mkr.color.r = 0.9;
  astar_mkr.color.g = 0.2;
  astar_mkr.color.b = 1.0;

  astar_mkr.pose.orientation.w = 1.0;

  Eigen::Vector3d last;
  bool            first = true;
  for (const auto& point : points) {
    if (first) {
      first = false;
      last  = point;
      continue;
    }
    geometry_msgs::Point p;
    p.x = last.x();
    p.y = last.y();
    p.z = last.z();
    astar_mkr.points.push_back(p);
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    astar_mkr.points.push_back(p);
    pt_marker.points.push_back(p);
    last = point;
  }

  // _astar_path_pub.publish(astar_mkr);
  _astar_path_pub.publish(pt_marker);
}

/**
 * @brief visualize start and goal points
 *
 * @param center
 * @param radius
 * @param sg
 */
void Visualizer::visualizeStartGoal(const Eigen::Vector3d& center, int sg) {
  visualization_msgs::Marker sphereMarkers, sphereDeleter;
  float                      radius = 0.1;

  sphereMarkers.id                 = sg;
  sphereMarkers.type               = visualization_msgs::Marker::SPHERE_LIST;
  sphereMarkers.header.stamp       = ros::Time::now();
  sphereMarkers.header.frame_id    = _frame_id;
  sphereMarkers.pose.orientation.w = 1.00;
  sphereMarkers.action             = visualization_msgs::Marker::ADD;
  sphereMarkers.ns                 = "StartGoal";
  sphereMarkers.color.r            = 1.00;
  sphereMarkers.color.g            = 0.00;
  sphereMarkers.color.b            = 0.00;
  sphereMarkers.color.a            = 1.00;
  sphereMarkers.scale.x            = radius * 2.0;
  sphereMarkers.scale.y            = radius * 2.0;
  sphereMarkers.scale.z            = radius * 2.0;

  sphereDeleter        = sphereMarkers;
  sphereDeleter.action = visualization_msgs::Marker::DELETEALL;

  geometry_msgs::Point point;
  point.x = center(0);
  point.y = center(1);
  point.z = center(2);
  sphereMarkers.points.push_back(point);

  if (sg == 0) {
    _start_goal_pub.publish(sphereDeleter);
    ros::Duration(1.0e-9).sleep();
    sphereMarkers.header.stamp = ros::Time::now();
  }
  _start_goal_pub.publish(sphereMarkers);
}

void Visualizer::visualizeBezierCurve(const Eigen::Vector3d&   start_pos,
                                      const Bernstein::Bezier& traj,
                                      double                   max_vel) {
  visualization_msgs::Marker traj_marker;
  traj_marker.header.frame_id    = _frame_id;
  traj_marker.header.stamp       = ros::Time::now();
  traj_marker.type               = visualization_msgs::Marker::LINE_LIST;
  traj_marker.pose.orientation.w = 1.00;
  traj_marker.action             = visualization_msgs::Marker::ADD;
  traj_marker.id                 = 0;
  traj_marker.ns                 = "trajectory";
  traj_marker.color.r            = 0.00;
  traj_marker.color.g            = 0.50;
  traj_marker.color.b            = 1.00;
  traj_marker.color.a            = 1.00;
  traj_marker.scale.x            = 0.05;

  double          T     = 0.05;
  Eigen::Vector3d lastX = traj.getPos(0.0) + start_pos;
  for (double t = T; t < traj.getDuration(); t += T) {
    std_msgs::ColorRGBA c;
    Eigen::Vector3d     jets = jetColorMap(traj.getVel(t).norm() / max_vel);
    c.r                      = jets[0];
    c.g                      = jets[1];
    c.b                      = jets[2];

    geometry_msgs::Point point;
    Eigen::Vector3d      X = traj.getPos(t) + start_pos;
    point.x                = lastX(0);
    point.y                = lastX(1);
    point.z                = lastX(2);
    traj_marker.points.push_back(point);
    traj_marker.colors.push_back(c);
    point.x = X(0);
    point.y = X(1);
    point.z = X(2);
    traj_marker.points.push_back(point);
    traj_marker.colors.push_back(c);
    lastX = X;
  }
  _colorful_traj_pub.publish(traj_marker);
                                      }

  /* visualize control pointss */
void Visualizer::visualizeControlPoints(const Eigen::MatrixX3d& cpts) {
  visualization_msgs::Marker cpts_marker;
  cpts_marker.header.frame_id    = _frame_id;
  cpts_marker.header.stamp       = ros::Time::now();
  cpts_marker.type    = visualization_msgs::Marker::SPHERE_LIST;
  cpts_marker.pose.orientation.w = 1.00;
  cpts_marker.action  = visualization_msgs::Marker::ADD;
  cpts_marker.id      = 1;
  cpts_marker.ns      = "control_points";
  cpts_marker.color.r = 0.00;
  cpts_marker.color.g = 1.00;
  cpts_marker.color.b = 0.00;
  cpts_marker.color.a = 1.00;
  cpts_marker.scale.x = 0.10;
  cpts_marker.scale.y = 0.10;
  cpts_marker.scale.z = 0.10;

  for (int i = 0; i < cpts.rows(); i++) {
    geometry_msgs::Point point;
    point.x = cpts(i, 0);
    point.y = cpts(i, 1);
    point.z = cpts(i, 2);
    cpts_marker.points.push_back(point);
  }
  _ctrl_pts_pub.publish(cpts_marker);
}

void Visualizer::displayOptimizationInfo(const double& comp_time,
                                         const double& max_velocity,
                                         const double& max_acceleration,
                                         const double& duration) {
  visualization_msgs::Marker textMarker;
  textMarker.header.frame_id = _frame_id;
  textMarker.header.stamp    = ros::Time::now();
  textMarker.ns              = "text";
  textMarker.id              = 1;
  textMarker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
  textMarker.action          = visualization_msgs::Marker::ADD;

  textMarker.pose.position.x    = -9;
  textMarker.pose.position.y    = 0.0;
  textMarker.pose.position.z    = 6.0;
  textMarker.pose.orientation.x = 0.0;
  textMarker.pose.orientation.y = 0.0;
  textMarker.pose.orientation.z = 0.0;
  textMarker.pose.orientation.w = 1.0;
  textMarker.scale.x            = 1.0;
  textMarker.scale.y            = 1.0;
  textMarker.scale.z            = 1.0;
  textMarker.color.r            = 1.0;
  textMarker.color.g            = 0.0;
  textMarker.color.b            = 0.0;
  textMarker.color.a            = 1.0;
  textMarker.text               = "Comp: ";
  textMarker.text += std::to_string(static_cast<int>(comp_time));
  textMarker.text += ".";
  textMarker.text += std::to_string(static_cast<int>(comp_time * 10) % 10);
  textMarker.text += "ms\n";
  textMarker.text += "Max speed: ";
  textMarker.text += std::to_string(static_cast<int>(max_velocity));
  textMarker.text += ".";
  textMarker.text += std::to_string(static_cast<int>(max_velocity * 100) % 100);
  textMarker.text += "m/s\n";
  textMarker.text += "Max acceleration: ";
  textMarker.text += std::to_string(static_cast<int>(max_acceleration));
  textMarker.text += ".";
  textMarker.text += std::to_string(static_cast<int>(max_acceleration * 100) % 100);
  textMarker.text += "m/s\n";
  textMarker.text += "Total time: ";
  textMarker.text += std::to_string(static_cast<int>(duration));
  textMarker.text += ".";
  textMarker.text += std::to_string(static_cast<int>(duration * 100) % 100);
  textMarker.text += "s\n";
  _text_pub.publish(textMarker);
}

}  // namespace visualizer
