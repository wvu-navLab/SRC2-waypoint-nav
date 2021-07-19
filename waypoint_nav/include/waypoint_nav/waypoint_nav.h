/*!
 * \waypoint_nav.h
 * \brief waypoint_nav (...).
 *
 * Waypoint navigation (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \author Matteo De Petrillo, WVU - madepetrillo@mix.wvu.edu
 * \date June 01, 2020
 */

#ifndef WAYPOINT_NAV_H
#define WAYPOINT_NAV_H

// Include cpp important headers
#include <math.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <termios.h>

// ROS headers
#include <ros/ros.h>
#include <tf/tf.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <waypoint_nav/SetGoal.h>
#include <waypoint_nav/GoToGoal.h>
#include <waypoint_nav/Interrupt.h>
#include <sensor_msgs/LaserScan.h>

#define ARRIVED 0
#define GOING 1
#define UNREACHABLE 2

#define INIT_STATE 0
#define TRAV_STATE 1
#define PLAN_STATE 2
#define VOLH_STATE 3
#define LOST_STATE 4

class WaypointNavigation
{
public:
  WaypointNavigation(ros::NodeHandle &nh);

  bool commandVelocity();

  bool active_ = false;

private:
  // Node Handle
  ros::NodeHandle &nh_;

  // Subscriber
  ros::Subscriber subOdom;
  ros::Subscriber subGoal;
  ros::Subscriber subLaser;
  ros::Subscriber subSmach;

  // Publisher
  ros::Publisher pubCmdVel;
  ros::Publisher pubNavStatus;
  ros::Publisher pubWaypointUnreachable;
  ros::Publisher pubArrivedAtWaypoint;

  ros::ServiceServer srv_set_goal_;
  ros::ServiceServer srv_go_to_goal_;
  ros::ServiceServer srv_interrupt_;

  std::string robot_name_;
  std::string robot_id_;


  const double LASER_THRESH = 0.9;
  const int LASER_MAX_SET_SIZE = 20;

  int side_ = 0;
  double avoid_angle_ = 0.0;
  double Kp_yaw_ = 5.0;
  double thresh_ = .75;
  double y_offset_b_ = 0.0;
  bool rr_ = false;
  bool ll_ = false;

  bool firstOdom_;
  bool firstGoal_;
  bool flag_in_collision = false;

  geometry_msgs::Pose localPos_curr_;
  geometry_msgs::Twist localVel_curr_;
  geometry_msgs::Pose goalPos_;

  void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void smachCallback(const std_msgs::Int64::ConstPtr &msg);
  void goalCallback(const geometry_msgs::Pose::ConstPtr &msg);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
  //  void avoidObstacleCallback(const std_msgs::Float64::ConstPtr& msg);

  bool setGoal(waypoint_nav::SetGoal::Request &req, waypoint_nav::SetGoal::Response &res);
  bool goToGoal(waypoint_nav::GoToGoal::Request &req, waypoint_nav::GoToGoal::Response &res);
  bool interrupt(waypoint_nav::Interrupt::Request &req, waypoint_nav::Interrupt::Response &res);
};

#endif // WAYPOINT_NAV_H
