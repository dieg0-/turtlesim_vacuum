#include "geometry_msgs/Twist.h"
#include <math.h>
#include "ros/ros.h"
#include <tuple>
#include "turtlesim/Pose.h"

double degrees2radians(double angle_in_degrees);
double radians2degrees(double angle_in_radians);
double euclidean_distance(double x1, double y1, double x2, double y2);

class RobotCleaner{
    public:
        turtlesim::Pose turtlesim_pose;
        std::tuple<float, float> x_limits = std::make_tuple(0.0, 11.0);
        std::tuple<float, float> y_limits = std::make_tuple(0.0, 11.0);

        void init_communication(){
            velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
            pose_subscriber_ = node_handle_.subscribe("/turtle1/pose", 10, &RobotCleaner::pose_clbk, this);
        }

        void pose_clbk(const turtlesim::Pose::ConstPtr& pose_message){
            turtlesim_pose.x = pose_message->x;
            turtlesim_pose.y = pose_message->y;
            turtlesim_pose.theta = pose_message->theta;
        }

        void move(double speed, double distance, bool isForward){
            geometry_msgs::Twist velocity_msg;

            // distance = velocity * time
            if(isForward)
                velocity_msg.linear.x = abs(speed); // A positive value
            else
                velocity_msg.linear.x = -abs(speed); // A negative value
            velocity_msg.linear.y = 0.0;
            velocity_msg.linear.z = 0.0;

            velocity_msg.angular.x = 0.0;
            velocity_msg.angular.y = 0.0;
            velocity_msg.angular.z = 0.0;

            // t0: current time
            double t0 = ros::Time::now().toSec();
            double current_distance = 0.0;
            ros::Rate loop_rate(10);

            do{
                //   Publish the velocity
                velocity_publisher_.publish(velocity_msg);

                //   Estimate the current distance = speed * (t1 - t0)
                double t1 = ros::Time::now().toSec();
                current_distance = speed * (t1 - t0);

                ros::spinOnce(); // Otherwise it will not publish.
                loop_rate.sleep();
            // until: the distance moved by the robot <= the specified distance
            }while(current_distance <= distance);

            // Stop the robot as soon as it exits the loop.
            velocity_msg.linear.x = 0.0;
            velocity_publisher_.publish(velocity_msg);
        }

        void rotate(double angular_speed, double relative_angle, bool clockwise){
            geometry_msgs::Twist velocity_msg;
            // All linear components are 0.0 because there is no linear motion
            velocity_msg.linear.x = 0.0;
            velocity_msg.linear.y = 0.0;
            velocity_msg.linear.z = 0.0;

            velocity_msg.angular.x = 0.0;
            velocity_msg.angular.y = 0.0;
            // By convention, a clockwise rotation is a negative angle.
            if(clockwise)
                velocity_msg.angular.z = -abs(angular_speed);
            else
                velocity_msg.angular.z = abs(angular_speed);

            double current_angle = 0.0;
            double t0 = ros::Time::now().toSec();
            ros::Rate loop_rate(100); // 10 times per second.

            do{
                velocity_publisher_.publish(velocity_msg);
                double t1 = ros::Time::now().toSec();
                current_angle = angular_speed * (t1 - t0);
                ros::spinOnce();
                loop_rate.sleep();
            }while(current_angle < relative_angle);

            // Stop the robot after leaving the loop
            velocity_msg.angular.z = 0.0;
            velocity_publisher_.publish(velocity_msg);
        }

        void move_to_goal(turtlesim::Pose goal_pose, double distance_tolerance){
            geometry_msgs::Twist velocity_msg;
            ros::Rate loop_rate(10);

            do{
                // Tends to 0 as the robot approaches the goal
                velocity_msg.linear.x = 1.5 * euclidean_distance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
                velocity_msg.linear.y = 0.0;
                velocity_msg.linear.z = 0.0;

                velocity_msg.angular.x = 0.0;
                velocity_msg.angular.y = 0.0;
                velocity_msg.angular.z = 4 * (atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);

                velocity_publisher_.publish(velocity_msg);

                ros::spinOnce();
                loop_rate.sleep();
            }while(euclidean_distance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance);

            velocity_msg.linear.x = 0.0;
            velocity_msg.angular.z = 0.0;
            velocity_publisher_.publish(velocity_msg);
        }

        double set_orientation(double angle_in_radians){
            // Calculate the relative orientation between the current orientation and the absolute orientation.

            double relative_angle_radians = angle_in_radians - turtlesim_pose.theta;
            bool clockwise = ((relative_angle_radians < 0)? true : false);
            // Also giving the relative angle in radians as angular speed
            rotate(abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
        }

        void grid_clean(){
            ros::Rate loop(0.5);
            turtlesim::Pose initial_pose;
            initial_pose.x = 1;
            initial_pose.y = 1;
            initial_pose.theta = 0;
            move_to_goal(initial_pose, 0.01);
            loop.sleep();
            set_orientation(0);
            loop.sleep();

            move(2.0, 9.0, true);
            loop.sleep();
            rotate(degrees2radians(10), degrees2radians(90), false);
            loop.sleep();

            while(turtlesim_pose.x > std::get<0>(x_limits) + 2){
                move(2.0, 9.0, true);
                rotate(degrees2radians(10), degrees2radians(90), false);
                loop.sleep();
                move(2.0, 1.0, true);
                rotate(degrees2radians(10), degrees2radians(90), false);
                loop.sleep();
                move(2.0, 9.0, true);
                rotate(degrees2radians(30), degrees2radians(90), true);
                loop.sleep();
                move(2.0, 1.0, true);
                rotate(degrees2radians(30), degrees2radians(90), true);
                loop.sleep();
            }
        }

        void spiral_clean(){
            geometry_msgs::Twist velocity_msg;
            double count = 0.0;

            double constant_speed = 4.0;
            double vk = 1.0;
            double wk = 2.0;
            double rk = 0.5;
            ros::Rate loop(1);

            do{
                rk = rk + 0.5;
                velocity_msg.linear.x = rk; // The radius of a circle is characterized by the linear velocity: radius = linear_vel / angular_vel
                // If the linear velocity is constant: we have a circle with a constant radius
                // If the lienar velocity increases, the radius of the circle increases
                velocity_msg.linear.y = 0.0;
                velocity_msg.linear.z = 0.0;

                velocity_msg.angular.x = 0.0;
                velocity_msg.angular.y = 0.0;
                velocity_msg.angular.z = constant_speed; // ((vk) / (0.5 + rk));

                velocity_publisher_.publish(velocity_msg);
                ros::spinOnce();

                loop.sleep();
                // vk = velocity_msg.linear.x
                // wk = velocity_msg.angular.z
                // rk = vk / wk
            }while((turtlesim_pose.x < 10.5) && (turtlesim_pose.y < 10.5));
            velocity_msg.linear.x = 0.0;
            velocity_publisher_.publish(velocity_msg);
        }

    private:
        ros::NodeHandle node_handle_;
        ros::Publisher velocity_publisher_;
        ros::Subscriber pose_subscriber_;
};

double degrees2radians(double angle_in_degrees){
    return angle_in_degrees * M_PI / 180.0;
}

double radians2degrees(double angle_in_radians){
    return angle_in_radians * 180.0 / M_PI;
}

double euclidean_distance(double x1, double y1, double x2, double y2){
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

int main(int argc, char** argv){
    // Initialize ROS node
    ros::init(argc, argv, "roomba");

    // Create and Initialize Robot Instance
    RobotCleaner cleaner;
    cleaner.init_communication();
    cleaner.spiral_clean();
    // cleaner.grid_clean();

    ros::spin();

    return 0;
}