#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include<geometry_msgs/Twist.h>
#include <first_boong/minsooOdom.h>

double x,y,z;
double vx, vy, vz;
double tx, ty, tz;

void chatterCallback(const geometry_msgs::Twist::ConstPtr &msg){
    vx = msg->linear.x;
    vy = msg->linear.y;
    vz = msg->linear.z;
    tx = msg->angular.x;
    ty = msg->angular.y;
    tz = msg->angular.z;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "boong_broadcaster");
    ros::NodeHandle node;

    first_boong::minsooOdom odom;

    ros::Subscriber sub = node.subscribe("cmd_vel", 1000, chatterCallback);

    ros::Publisher minsoo_pub = node.advertise<first_boong::minsooOdom>("odom", 50);

    tf::TransformBroadcaster br; //링크에서 레이저로 tf를 보낼 객체 선언
    tf::TransformBroadcaster broad; //맵에서 로봇으로 tf를 보낼 객체 선언
    tf::Transform transform; 
    //tf::Quaternion q;
    tf2::Quaternion q;
    
    double V, W, x, y, dt, heading;   
    double pi = 3.141592;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(100);



    while(ros::ok){

        ros::spinOnce();
        current_time = ros::Time::now();

        br.sendTransform (
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.1, 0.0, 0.2)),
                    ros::Time::now(), "Robot", "Sensor"));

        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(tz) - vy * sin(tz)) * dt;
        double delta_y = (vx * sin(tz) + vy * cos(tz)) * dt;
            
        x += delta_x;
        y += delta_y;



    }
}