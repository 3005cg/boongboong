//STD header
#include<vector>
#include<cmath>
#include<string>
#include<iostream>

//ROS header
#include<ros/ros.h>
#include<tf/tf.h>


//Message header
#include<std_msgs/Int32.h>
#include<std_msgs/Float64.h>


using namespace std;

class indexPub{
    public:
        indexPub(){
            //pub_init_x=nh_.advertise<std_msgs::Float64>("/x_init",100);
            //pub_init_y=nh_.advertise<std_msgs::Float64>("/y_init",100);
            pub_goal_x = nh_.advertise<std_msgs::Float64>("/x_slot",100);
	        pub_goal_y = nh_.advertise<std_msgs::Float64>("/y_slot",100);
            pub_obstacle_x = nh_.advertise<std_msgs::Float64>("/x_obs",100);
            pub_obstacle_y = nh_.advertise<std_msgs::Float64>("/y_obs",100);
            pub_obstacle_r = nh_.advertise<std_msgs::Float64>("/size_obs",100);
            abc[3]=nh_.advertise<std_msgs::Float64>("abc",100);;   // 행렬 publish subsribe 어떻게 하는지..

        }
    
    private:
        ros::NodeHandle nh_;
        //ros::Publisher pub_init_x;
        //ros::Publisher pub_init_y;
        ros::Publisher pub_goal_x;
	    ros::Publisher pub_goal_y;
        ros::Publisher pub_obstacle_x;
        ros::Publisher pub_obstacle_y;
        ros::Publisher pub_obstacle_r;
        ros::Publisher abc[3];   // 행렬 publish subsribe 어떻게 하는지..

        //std_msgs::Float64 x_init;
	    //std_msgs::Float64 y_init;
        std_msgs::Float64 x_slot;
	    std_msgs::Float64 y_slot;
        std_msgs::Float64 x_obs;
        std_msgs::Float64 y_obs;
        std_msgs::Float64 size_obs;
        std_msgs::Float64 obs[3]; // 행렬 publish subsribe 어떻게 하는지..


    public:
        /*void pubInit(){
            x_init.data = 0;   // x좌표 초기 값 원점
            y_init.data = 0;   // y좌표 초기 값 원점
            pub_init_x.publish(x_init);
            pub_init_y.publish(y_init);
        }*/
        void pubIdx(){
            x_slot.data = 5;   //  offset distance: 0.5m(x좌표 최종값)
	        y_slot.data = 20;   // look ahead distance:2m (y좌표 최종값)
            pub_goal_x.publish(x_slot); 
	        pub_goal_y.publish(y_slot); 
        }
        void pubObstacle(){
            x_obs.data=0;
            y_obs.data=10;
            size_obs.data=4;
            pub_obstacle_x.publish(x_obs);
            pub_obstacle_y.publish(y_obs);
            pub_obstacle_r.publish(size_obs);    
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv,"pub_index");
    indexPub pubI;

    ros::Rate rate(0.5);
    
    while(ros::ok()){
        //pubI.pubInit();
        pubI.pubIdx();
        pubI.pubObstacle();
        ros::spinOnce();
        rate.sleep();
    }
}