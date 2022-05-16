#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
class controller{
    public:
        controller(){
            sub = n.subscribe("turtle1/pose", 1000, &controller::callback, this);
            pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
        }

        void callback(const turtlesim::Pose::ConstPtr& msg){
            std_msgs::String pub_str;
            std::stringstream ss;

            ss << "controller heard: " << msg->x << ", " << msg->y;

            pub_str.data = ss.str();

            std::cout << pub_str.data.c_str() << std::endl;

            if(msg->x >= 11)
            {
                geometry_msgs::Twist pubMsg;

                // TODO
                pubMsg.angular.z = 3.14;
                pub.publish(pubMsg);
            }

            ros::spinOnce();
        }


    private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    controller ctrl;
    
    ros::spin();

    return 0;
}