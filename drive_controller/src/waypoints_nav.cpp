// Author : Matte

#include <stdio.h>
#include <unistd.h>

// Include ROS headers

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
// #include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <termios.h>
#include <math.h>


# define M_PI           3.14159265358979323846

using namespace std;

geometry_msgs::Pose localPos_curr;
// geometry_msgs::Pose pos_curr, localPos_curr;
geometry_msgs::Twist localVel_curr;
bool rr = false;
bool ll = false;
float avoid_angle;

// subscriber to the ground truth to use as pose feedback
// void current_pos(const gazebo_msgs::ModelStates::ConstPtr& pos){
//         pos_curr = pos->pose[2];
//
// }

void local_pos(const nav_msgs::Odometry::ConstPtr& local){
        localPos_curr = local->pose.pose;
        localVel_curr = local->twist.twist;
        // cout<<"LOCAL POS CURR X "<<localPos_curr.position.x<<endl;
}

void avoid(const std_msgs::Float64::ConstPtr& dir){
        if (dir->data > 0 && dir->data < 0.758 ) { //data is coming from obstacle avoidance
                rr = true; //
                ll = false;
                avoid_angle = 0.758;
        }
        else if(dir->data > 0 && dir->data > 0.758) {
                ll = true;
                rr = false;
                avoid_angle = dir->data;
        }
        else if(dir->data < 0 && dir->data > -0.758) {
                ll = true;
                rr = false;
                avoid_angle = -0.758;
        }
        else if(dir->data < 0 && dir->data < -0.758) {
                ll = true;
                rr = false;
                avoid_angle = dir->data;
        }
        else if(dir->data == 0) {
                rr = false;
                ll = false;
        }
        else if(dir->data > 1.39626){

                rr = true; //right
                ll = false; //left
                avoid_angle = 1.39626;
        }
        else if(dir->data < -1.39626){

                rr = false; //right
                ll = true; //left
                avoid_angle = -1.39626;
        }



        // cout<<"LOCAL POS CURR X "<<localPos_curr.position.x<<endl;
}

int main(int argc, char **argv){

        ros::init(argc, argv, "waypoints_nav");
        ros::NodeHandle nh;
        vector<double> x,y;
        // x = {6,6,0,0};
        // y = {0,6,6,0};
        // x = {6,6,12,0};
        // y = {0,0,0,0};



        double ex,ey,et,ephi,phi_d;
        double roll, pitch, yaw;
        double h_angle;
        int k=5;

        std_msgs::Float64 wheel_angle1,wheel_angle2,wheel_angle3,wheel_angle4, wheel_cmd,wheel_cmd2;

        if (!nh.hasParam("/waypoints/")) {
                ROS_INFO("No waypoints found");
        }else{
                ROS_INFO("waypoints loaded");
        }

        nh.getParam("/waypoints/x",x);
        nh.getParam("/waypoints/y",y);

        // ros::Subscriber truth = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",1, current_pos);
        // ros::Subscriber odom = nh.subscribe<nav_msgs::Odometry>("/dead_reckoning/odometry",1, local_pos); 
        ros::Subscriber odom = nh.subscribe<nav_msgs::Odometry>("/odometry/truth",1, local_pos);
        ros::Subscriber direction = nh.subscribe<std_msgs::Float64>("/direction",100, avoid);

        ros::Publisher fl_ang = nh.advertise<std_msgs::Float64>("/scout_1/fl_steering_arm_controller/command",1);
        ros::Publisher fr_ang = nh.advertise<std_msgs::Float64>("/scout_1/fr_steering_arm_controller/command",1);
        ros::Publisher bl_ang = nh.advertise<std_msgs::Float64>("/scout_1/bl_steering_arm_controller/command",1);
        ros::Publisher br_ang = nh.advertise<std_msgs::Float64>("/scout_1/br_steering_arm_controller/command",1);
        ros::Publisher fl_cmd = nh.advertise<std_msgs::Float64>("/scout_1/fl_wheel_controller/command",1);
        ros::Publisher fr_cmd = nh.advertise<std_msgs::Float64>("/scout_1/fr_wheel_controller/command",1);
        ros::Publisher bl_cmd = nh.advertise<std_msgs::Float64>("/scout_1/bl_wheel_controller/command",1);
        ros::Publisher br_cmd = nh.advertise<std_msgs::Float64>("/scout_1/br_wheel_controller/command",1);

        ros::Rate rate(100.0);
        int i = 0;
        int wp_size = x.size();
        double vd,vFB,kv,ev, ka;
        kv = 100;
        ka = 75;
        while (ros::ok()) {
                cout<<"WP1: X:  "<<x[i] << " Y: "<<y[i]<<endl;
                cout<<"WP size: "<<wp_size<<endl;
                cout<<"posCur: X:  "<<localPos_curr.position.x << " Y: "<<localPos_curr.position.y<<endl;
                cout<<"posCur: X:  "<<localPos_curr.position.x << " Y: "<<localPos_curr.position.y<<endl;

                // tf::Quaternion q(
                //         pos_curr.orientation.x,
                //         pos_curr.orientation.y,
                //         pos_curr.orientation.z,
                //         pos_curr.orientation.w
                //         );

                tf::Quaternion q(
                        localPos_curr.orientation.x,
                        localPos_curr.orientation.y,
                        localPos_curr.orientation.z,
                        localPos_curr.orientation.w
                        );

                tf::Matrix3x3 m(q);
                m.getRPY(roll, pitch, yaw);
                double pf;

                ex = x[i]-localPos_curr.position.x;
                ey = y[i]-localPos_curr.position.y;
                pf = sqrt(pow(ex,2)+pow(ey,2));

                vd = 1;
                if (vd<1) {
                        vd=2;
                }
                vFB = sqrt(pow(localVel_curr.linear.x,2)+pow(localVel_curr.linear.y,2));
                ev = vd-vFB;

                cout<<"VD:  "<<vd << " VFB: "<<vFB<<" EV: "<<ev<<" K*EV: "<<kv*ev<<endl;
                if (pf>0.2) {
                        phi_d = atan2(ey,ex);
                        ephi = phi_d-yaw;
                        et = atan2(sin(ephi),cos(ephi));







                        if (signbit(et)==1) { //turning in place, et is the angle
                                wheel_cmd.data = -18;
                                wheel_cmd2.data = 18;
                                wheel_angle1.data = 45*M_PI/180;
                                wheel_angle2.data = -45*M_PI/180;
                                wheel_angle3.data = 45*M_PI/180;
                                wheel_angle4.data = -45*M_PI/180;
                        }
                        else{
                                wheel_cmd.data = 18;
                                wheel_cmd2.data = -18;
                                wheel_angle1.data = 45*M_PI/180;
                                wheel_angle2.data = -45*M_PI/180;
                                wheel_angle3.data = 45*M_PI/180;
                                wheel_angle4.data = -45*M_PI/180;
                        }

                        if (abs(et)<0.07) { //dont turn in place adjust the trajectory/heading with respect to the estimated error
                                wheel_cmd.data = kv*ev; //velocity error ev
                                wheel_cmd2.data = kv*ev;
                                wheel_angle1.data = k*et;
                                wheel_angle2.data = k*et;
                                wheel_angle3.data = -k*et;   //-et is the heading error
                                wheel_angle4.data = -k*et;  //-
                        }
                        if (rr == true && ll == false){
                          ros::Duration(0.5).sleep();
                          wheel_cmd.data = kv*ev;
                          wheel_cmd2.data = kv*ev;
                          wheel_angle1.data = avoid_angle;//-45*M_PI/180;
                          wheel_angle2.data = avoid_angle;//-45*M_PI/180;
                          wheel_angle3.data = avoid_angle;//-45*M_PI/180;
                          wheel_angle4.data = avoid_angle;//-45*M_PI/180;
                        }
                        if (rr == false && ll == true){
                          ros::Duration(0.5).sleep();
                          wheel_cmd.data = kv*ev;
                          wheel_cmd2.data = kv*ev;
                          wheel_angle1.data = avoid_angle;//45*M_PI/180;
                          wheel_angle2.data = avoid_angle;//45*M_PI/180;
                          wheel_angle3.data = avoid_angle;//45*M_PI/180;
                          wheel_angle4.data = avoid_angle;//45*M_PI/180;
                        }
                        cout<<"headingangle:  "<<yaw<<endl;
                        cout<<"delta:  "<<wheel_angle1.data+yaw<<endl;
                        cout<< "errors--->"<<endl<<"eX: "<<ex<<endl<<"eY: "<<ey<<endl<<"eT: "<<et<<endl;
                        cout<<endl<< "Angles errors--->"<<endl<<"yaw: "<<yaw<<endl<<"phi_d: "<<phi_d<<endl<<"ephi: "<<ephi<<endl<<"et: "<<et<<endl;
                        cout<<"LOCAL POS CURR X "<<localPos_curr.position.x<<endl;
                        cout<<"LOCAL POS CURR Y "<<localPos_curr.position.y<<endl;
                        cout<<"WP error d " << pf <<endl;
                        cout<<"WP # "<<i<<endl;
                        fr_ang.publish(wheel_angle1);
                        fl_ang.publish(wheel_angle2);
                        bl_ang.publish(wheel_angle3);
                        br_ang.publish(wheel_angle4);
                        fl_cmd.publish(wheel_cmd2);
                        fr_cmd.publish(wheel_cmd);
                        bl_cmd.publish(wheel_cmd2);
                        br_cmd.publish(wheel_cmd);
                        // ros::spinOnce();
                        // rate.sleep();

                }else{i++;  //if youre close to waypoint the goto the next waypoint
                      // ros::spinOnce();
                      // rate.sleep();
                      //
                      //
                }

                if (i >= wp_size) {//size of the waypoints vector
                        wheel_cmd.data = 0;
                        wheel_cmd2.data = 0;
                        fl_cmd.publish(wheel_cmd2);
                        fr_cmd.publish(wheel_cmd);
                        bl_cmd.publish(wheel_cmd2);
                        br_cmd.publish(wheel_cmd);
                }

                ros::spinOnce();
                rate.sleep();

        }


}
