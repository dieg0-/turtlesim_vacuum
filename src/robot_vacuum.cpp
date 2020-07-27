#include "geometry_msgs/Twist.h"
#include <math.h>
#include "ros/ros.h"
#include "turtlesim/Pose.h"

class RobotCleaner{
    public:
        turtlesim::Pose turtlesim_pose;

        void init_communication(){
            velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
            pose_subscriber_ = node_handle_.subscribe("/turtle1/pose", 10, &RobotCleaner::pose_clbk, this);
        }

        void pose_clbk(const turtlesim::Pose::ConstPtr& pose_message){
            turtlesim_pose.x = pose_message->x;
            turtlesim_pose.y = pose_message->y;
            turtlesim_pose.theta = pose_message->theta;
        }

    private:
        ros::NodeHandle node_handle_;
        ros::Publisher velocity_publisher_;
        ros::Subscriber pose_subscriber_;
};

int main(int argc, char** argv){
    // Initialize ROS node
    ros::init(argc, argv, "roomba");

    // Create and Initialize Robot Instance
    RobotCleaner cleaner;
    cleaner.init_communication();

    ros::spin();

    return 0;
}