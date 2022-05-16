#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void poseCallback(const nav_msgs::Odometry::ConstPtr &msg){
    static tf::TransformBroadcaster br;

    tf::Transform transform;


    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z));
    tf::Quaternion q;


    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    q.setW(msg->pose.pose.orientation.w);

    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
    br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.1, 0.0, 0.2)), ros::Time::now(), "base_link", "base_laser"));
}   

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster_cpp");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/odom_msg", 10, &poseCallback);

  ros::spin();
  return 0;
};