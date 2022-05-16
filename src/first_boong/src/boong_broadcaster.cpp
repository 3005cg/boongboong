#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <first_boong/minsooOdom.h>

void poseCallback(const first_boong::minsooOdom::ConstPtr &msg){
    static tf::TransformBroadcaster trans;
    tf::Transform transform;
    //tf::TransformBroadcaster trans; //맵에서 로봇으로 tf를 보낼 객체 선언

    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y
                                    msg->pose.pose.position.z));


    q.setx(msg->pose.pose.orientation.x);
    q.sety(msg->pose.pose.orientation.y);
    q.setz(msg->pose.pose.orientation.z);
    q.setw(msg->pose.pose.orientation.w);

    transform.setRotation(q);

    trans.sendTransform(
        tf::StampedTransform(
            transform, ros::Time::now(),
             "world", "Robot"));

}


int main(int argc, char** argv){
    ros::init(argc, argv, "boong_broadcaster");
    ros::NodeHandle node;

    first_boong::minsooOdom msg;



    tf::TransformBroadcaster br; //링크에서 레이저로 tf를 보낼 객체 선언
    //tf::Transform transform; 
    //tf::Quaternion q;
    //q.normalize();
   // q.setRPY(0, 0, msg.theta);

    //transform.setRotation(q);

    while(ros::ok){
        br.sendTransform (
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.1, 0.0, 0.2)),
                ros::Time::now(), "Robot", "Sensor"));
                //TransformBroadcaster 를 사용하여 변환 전송하려면 5개 인수 필요
                //회전을 적용하는 Quaternion, 오프셋 적용 Vector3, 시간 말해주는 ros::Time::now()
                //그 다음 부모노트 Robot, 자식 노드 Sensor
    }

    ros::Subscriber sub = node.subscribe("/odom_msg", 10, &poseCallback);
}