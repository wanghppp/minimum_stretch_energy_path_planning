#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "self_pub_point");
    ros::NodeHandle pub_point;
    // ros::Rate rate(1000);
    ros::Publisher publish_point = pub_point.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100, true);
    // rate.sleep();
    geometry_msgs::PoseStamped point1, point2;
    point1.header.frame_id = "odom";
    point1.header.seq = 1;
    point1.pose.position.x = 14.466;
    point1.pose.position.y = 23.168;
    point1.pose.position.z = 0.00;
    point1.pose.orientation.x = 0.00;
    point1.pose.orientation.y = 0.00;
    point1.pose.orientation.z = -0.856;
    point1.pose.orientation.w = 0.518;
    // while (publish_point.getNumSubscribers() < 1)
    // {
        
        
    // }

    point2.header.frame_id = "odom";
    point2.header.seq = 2;
    point2.pose.position.x = -22.106;
    point2.pose.position.y = -23.421;
    point2.pose.position.z = 0.00;
    point2.pose.orientation.x = 0.00;
    point2.pose.orientation.y = 0.00;
    point2.pose.orientation.z = -0.894;
    point2.pose.orientation.w = 0.528;
    
    // while (publish_point.getNumSubscribers() < 2)
    // {
        
    // }
    // publish_point.publish(point2);
    //     loop_rate.sleep();
    // }
    ros::Rate rate(1);
    while (ros::ok())
    {
        rate.sleep();
        std::cout << publish_point.getNumSubscribers() << std::endl;
        publish_point.publish(point1);
        // publish_point.publish(point1);
        ros::spinOnce();
        rate.sleep();
        publish_point.publish(point2);
        // publish_point.publish(point1);
        ros::spin();
    }
    
}