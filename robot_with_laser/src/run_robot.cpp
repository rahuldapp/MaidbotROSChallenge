#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <tf/transform_broadcaster.h>


int main(int argc,char** argv)
{
    ros::init(argc,argv,"run_robot");

    ros::NodeHandle nh;

    ros::Publisher robot_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("robot_pose",1);

    geometry_msgs::PoseStamped robot_pose_msg;

    tf::Transform transform;
    tf::TransformBroadcaster base_frame_broadcaster;

    //Sets the rotation radius of the robot. Default value is 3.0
    double radius;
    nh.param<double>("robot/rotation_radius",radius,3.0);

    //Start from yaw = 0
    double yaw = 0.0;

    //Increase the yaw by 1 degree each instant to move the robot.
    double step = 2.0*M_PI/360.0;
 
    bool goCounter = false;

    ros::Rate rate(10);

    while(ros::ok())
    {
         
        tf::Quaternion robot_orientation_tf;

        if(goCounter)
        {
            transform.setOrigin( tf::Vector3(radius*cos(yaw), radius*sin(yaw), 0.0) );
            robot_orientation_tf.setRPY(0, 0, yaw);
            transform.setRotation(robot_orientation_tf);
        }

        else
        {
            transform.setOrigin( tf::Vector3(radius*cos(-yaw), radius*sin(-yaw), 0.0) );
            robot_orientation_tf.setRPY(0, 0, yaw);
            transform.setRotation(robot_orientation_tf);
        }

        //broadcast the dynamic transform.
        base_frame_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

        //publish robot's pose (seems valid to publish the pose message)
        robot_pose_msg.header.stamp = ros::Time::now();

        robot_pose_msg.pose.position.x = radius*cos(yaw);
        robot_pose_msg.pose.position.y = radius*sin(yaw);
        robot_pose_msg.pose.position.z = 0.0;

        robot_pose_msg.pose.orientation.w = robot_orientation_tf.getW();
        robot_pose_msg.pose.orientation.x = robot_orientation_tf.getAxis().x();
        robot_pose_msg.pose.orientation.y = robot_orientation_tf.getAxis().y();;
        robot_pose_msg.pose.orientation.z = robot_orientation_tf.getAxis().z();;

        robot_pose_publisher.publish(robot_pose_msg);

        yaw = yaw+step;  

        //rotate counter when completed one full rotation
        if(yaw - 2.0*M_PI > 1e-9)
        {
            goCounter = !goCounter;
            yaw = 0.0;
        }

        rate.sleep();
    }
}