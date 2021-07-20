/*!
 * \waypoint_nav.cpp
 * \brief waypoint_nav (...).
 *
 * Waypoint navigation (...).
 *
 * \author Matteo Petrillo, WVU - madepetrillo@mix.wvu.edu
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \date June 20, 2020
 */

#include "waypoint_nav/waypoint_nav.h"

WaypointNavigation::WaypointNavigation(ros::NodeHandle &nh)
    : nh_(nh)
{
  if (ros::param::get("robot_name", robot_name_) == false)
  {
    ROS_FATAL("No parameter 'robot_name' specified");
    ros::shutdown();
    exit(1);
  }

  // Subscriber
  subOdom = nh_.subscribe("localization/odometry/sensor_fusion", 10, &WaypointNavigation::odometryCallback, this);
  subSmach = nh_.subscribe("state_machine/state", 10, &WaypointNavigation::smachCallback, this);
  subLaser = nh_.subscribe("laser/scan", 1, &WaypointNavigation::laserCallback, this);
  //subAvoidDirection = nh_.subscribe("driving/direction", 10, &WaypointNavigation::avoidObstacleCallback, this);

  // Publisher
  pubCmdVel = nh_.advertise<geometry_msgs::Twist>("driving/cmd_vel", 10);
  pubNavStatus = nh_.advertise<std_msgs::Int64>("navigation/status", 10);
  pubWaypointUnreachable = nh_.advertise<std_msgs::Bool>("state_machine/waypoint_unreachable", 10);
  pubArrivedAtWaypoint = nh_.advertise<std_msgs::Bool>("state_machine/arrived_at_waypoint", 10);

  // Service Servers
  srv_set_goal_ = nh_.advertiseService("navigation/set_goal", &WaypointNavigation::setGoal, this);
  srv_go_to_goal_ = nh_.advertiseService("navigation/go_to_goal", &WaypointNavigation::goToGoal, this);
  srv_interrupt_ = nh_.advertiseService("navigation/interrupt", &WaypointNavigation::interrupt, this);
}

void WaypointNavigation::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (firstOdom_ == false)
  {
    firstOdom_ = true;
  }
  localPos_curr_ = msg->pose.pose;
  localVel_curr_ = msg->twist.twist;
}

void WaypointNavigation::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  std::vector<float> ranges = msg->ranges;
  std::sort (ranges.begin(), ranges.end());
  float min_range_sum = 0;
  int set_size = 0;
  for (int i = 0; i < LASER_MAX_SET_SIZE; ++i)
  {
    // if (!isinf(ranges[i]))
    // {
      min_range_sum = min_range_sum + ranges[i];
      set_size++;
    // }
  }
  float min_range = min_range_sum / (float) set_size;

  if (min_range < LASER_THRESH)
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"WAYPOINT NAV. Collision Detected. Min range: "<< min_range << ". Set size: " << set_size);
    flag_in_collision = true;
  }
  else
  {
    flag_in_collision = false;
  }
}

bool WaypointNavigation::interrupt(waypoint_nav::Interrupt::Request &req, waypoint_nav::Interrupt::Response &res)
{
  if (req.interrupt == true)
  {
    geometry_msgs::Twist cmd_vel;

    active_ = false;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    pubCmdVel.publish(cmd_vel);
    ROS_INFO_STREAM("WAYPOINT NAV INTERRUPT");
  }
  else
  {
    active_ = true;
    ROS_INFO_STREAM("WAYPOINT NAV SET ACTIVE");
  }
  res.success = true;
  return true;
}

bool WaypointNavigation::setGoal(waypoint_nav::SetGoal::Request &req, waypoint_nav::SetGoal::Response &res)
{
  if (req.start == true)
  {
    if (firstGoal_ == false)
    {
      firstGoal_ = true;
    }

    goalPos_ = req.goal;
    // ROS_INFO_STREAM("New goal "<< goalPos_);
    res.success = false;
  }
  else
  {
    goalPos_ = localPos_curr_;
    res.success = false;
  }
  return true;
}

bool WaypointNavigation::goToGoal(waypoint_nav::GoToGoal::Request &req, waypoint_nav::GoToGoal::Response &res)
{
  if (req.start == true)
  {
    goalPos_ = req.goal;
    thresh_ = req.thresh;
    y_offset_b_ = req.y_offset_b;
    side_ = req.side;
    firstGoal_ = true;
    //  active_ = true;
    ros::Time timeOutTimer = ros::Time::now();
    ros::Duration timeOutDuration(req.timeOut);
    bool arrived = false;
    while (!arrived && ((ros::Time::now() - timeOutTimer) < timeOutDuration))
    {
      ROS_INFO_STREAM_THROTTLE(1,"[" << robot_name_ << "] " <<"WAYPOINT NAV. Commanding Velocity.");
      arrived = commandVelocity();
      ros::spinOnce();
    }
    geometry_msgs::Twist cmd_vel;
    pubCmdVel.publish(cmd_vel);
    res.success = arrived;
  }
  else
  {
    goalPos_ = localPos_curr_;
    geometry_msgs::Twist cmd_vel;
    pubCmdVel.publish(cmd_vel);
    res.success = false;
  }
  //    active_ =false;
  geometry_msgs::Twist cmd_vel;
  pubCmdVel.publish(cmd_vel);
  return true;
}

void WaypointNavigation::smachCallback(const std_msgs::Int64::ConstPtr &msg)
{
  if (msg->data == TRAV_STATE)
  {
    active_ = true;
  }
}

// void WaypointNavigation::avoidObstacleCallback(const std_msgs::Float64::ConstPtr& msg)
// {
//    if (msg->data > 0 && msg->data < 0.758 )
//    {
//         rr_ = true;
//         ll_ = false;
//         avoid_angle_ = 0.758;
//     }
//     else if(msg->data > 0 && msg->data > 0.758) {
//         ll_ = true;
//         rr_ = false;
//         avoid_angle_ = msg->data;
//     }
//     else if(msg->data < 0 && msg->data > -0.758) {
//         ll_ = true;
//         rr_ = false;
//         avoid_angle_ = -0.758;
//     }
//     else if(msg->data < 0 && msg->data < -0.758) {
//         ll_ = true;
//         rr_ = false;
//         avoid_angle_ = msg->data;
//     }
//     else if(msg->data == 0) {
//         rr_ = false;
//         ll_ = false;
//     }
// }

bool WaypointNavigation::commandVelocity()
{
  std_msgs::Int64 status;
  std_msgs::Bool arrived;
  std_msgs::Bool unreachable;
  geometry_msgs::Twist cmd_vel;
  double ex, ey, ex_b, ey_b, et, ephi, phi_d;
  double Kv;
  double roll, pitch, yaw;

  tf::Quaternion q(
      localPos_curr_.orientation.x,
      localPos_curr_.orientation.y,
      localPos_curr_.orientation.z,
      localPos_curr_.orientation.w);

  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  double pf;

  ex = goalPos_.position.x - localPos_curr_.position.x;
  ey = goalPos_.position.y - localPos_curr_.position.y;

  ex_b = (ex)*cos(yaw) + (ey)*sin(yaw);
  ey_b = -(ex)*sin(yaw) + (ey)*cos(yaw) + side_ * y_offset_b_;

  pf = std::hypot(ex_b, ey_b);
  // ROS_INFO_STREAM("error:"<<pf);
  // ROS_INFO_STREAM("goalPos_.position"<<goalPos_);
  // ROS_INFO_STREAM("localPos_curr_.position"<<localPos_curr_);
  // ROS_INFO_STREAM("yaw: "<<yaw);

  Kv = 0.2;
  // vd = 0.4;
  // if (vd<1)
  // {
  //     vd=2;
  // }
  // vFB = std::hypot(localVel_curr_.linear.x, localVel_curr_.linear.y);
  // ev = vd-vFB;

  if (pf > thresh_)
  {
    phi_d = atan2(ey, ex);

    // ROS_INFO_STREAM("phi_d: "<<phi_d);
    ephi = phi_d - yaw;
    et = atan2(sin(ephi), cos(ephi));
    // ROS_INFO_STREAM("et: "<<et);

    if (signbit(et) == 1)
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = -0.4;
      // ROS_INFO_STREAM("Rotate in place");
    }
    else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = 0.4;
      // ROS_INFO_STREAM("Rotate in place");
    }
    // if (abs(et) < M_PI_4)
    // {
    //     cmd_vel.linear.x = vd;
    //     cmd_vel.linear.y = 0.0;
    //     cmd_vel.linear.z = 0.0;
    //     cmd_vel.angular.x = 0.0;
    //     cmd_vel.angular.y = 0.0;
    //     cmd_vel.angular.z = Kp_yaw_*et;
    //     // ROS_INFO_STREAM("Double Ackermann");
    // }
    if (abs(et) < M_PI_4)
    {
      cmd_vel.linear.x = Kv * ex_b;
      cmd_vel.linear.y = Kv * ey_b;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = 0.0;
      // ROS_INFO_STREAM("Crab motion");
    }
    // if (rr_ == true && ll_ == false)
    // {
    //     cmd_vel.linear.x = ev*cos(avoid_angle_);
    //     cmd_vel.linear.y = ev*sin(avoid_angle_);
    //     cmd_vel.linear.z = 0.0;
    //     cmd_vel.angular.x = 0.0;
    //     cmd_vel.angular.y = 0.0;
    //     cmd_vel.angular.z = 0.0;
    //     // ROS_INFO_STREAM("Crab motion");
    // }
    // if (rr_ == false && ll_ == true)
    // {
    //     cmd_vel.linear.x = ev*cos(avoid_angle_);
    //     cmd_vel.linear.y = ev*sin(avoid_angle_);
    //     cmd_vel.linear.z = 0.0;
    //     cmd_vel.angular.x = 0.0;
    //     cmd_vel.angular.y = 0.0;
    //     cmd_vel.angular.z = 0.0;
    //     // ROS_INFO_STREAM("Crab motion");
    // }
    status.data = GOING;
    arrived.data = false;
  }
  else if (flag_in_collision)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    status.data = ARRIVED;
    arrived.data = true;
  }
  else
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    status.data = ARRIVED;
    arrived.data = true;
  }
  ROS_INFO_STREAM_THROTTLE(5,"[" << robot_name_ << "] " <<"WAYPOINT NAV. Commanded Velocity: " << cmd_vel);

  unreachable.data = false;

  if (firstGoal_ && firstOdom_)
  {
    pubCmdVel.publish(cmd_vel);
    pubNavStatus.publish(status);
    pubArrivedAtWaypoint.publish(arrived);
    pubWaypointUnreachable.publish(unreachable);
    return arrived.data;
  }

  return false;
}

/*!
 * \brief Creates and runs the waypoint_nav node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_nav");
  ros::NodeHandle nh("");

  ros::Rate rate(50);

  ROS_INFO("Waypoint Nav Node initializing...");
  WaypointNavigation waypoint_nav(nh);

  while (ros::ok())
  {
    // if (waypoint_nav.active_ == true)
    // {
    //     waypoint_nav.commandVelocity();
    // }
    // ROS_INFO_THROTTLE(1,"active_%d",waypoint_nav.active_);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
