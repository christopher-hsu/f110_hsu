#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>

#include <tf/transform_datatypes.h>
// TODO: include ROS msg type headers and libraries

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    const double min_TTC = 0.35f;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;
    ros::Publisher brake_bool_pub;
    ros::Publisher brake_pub;
    // TODO: create ROS subscribers and publishers

public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
        scan_sub = n.subscribe("scan", 10, &Safety::scan_callback, this);
        odom_sub = n.subscribe("odom", 10, &Safety::odom_callback, this);
        brake_bool_pub = n.advertise<std_msgs::Bool>("brake_bool", 10);
        brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("brake", 10);
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        speed = odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC
        int num_scans = (scan_msg->angle_max-scan_msg->angle_min)/(scan_msg->angle_increment)+1;
        float TTC;
        float range_dot;
        bool brake_bool = false;
        for (int ii = 0; ii < num_scans; ii++)
        {
            range_dot = speed * cos(scan_msg->angle_min + ii * scan_msg->angle_increment);
            
            
            TTC = scan_msg->ranges[ii] / ((range_dot > 0) ? range_dot : 0);
            if (TTC < min_TTC)
            {
                ROS_INFO("TTC: %f", TTC);
                brake_bool = true;
                break;
            }
            /*if (-2 * 8.26 * 0.3 <= range_dot* range_dot + 2 * 8.26 * -scan_msg->ranges[ii] && (range_dot > 0.1 || range_dot < -0.1))
            {
                brake_bool = true;
                ROS_INFO("range: %f", (range_dot* range_dot + 2 * 8.26 * -scan_msg->ranges[ii]) / (-2 * 8.26));
                ROS_INFO("dot: %f", range_dot);
                break;
            }*/
            
        }
        std_msgs::Bool brake_bool_msg;
        brake_bool_msg.data = brake_bool;
        brake_bool_pub.publish(brake_bool_msg);
        if (brake_bool)
        {
            ackermann_msgs::AckermannDriveStamped brake_msg;
            brake_msg.drive.speed = 0.0f;
            brake_pub.publish(brake_msg);
        }
        /*ROS_INFO("calc_angle: %f", scan_msg->angle_min + (num_scans-1) * scan_msg->angle_increment);
        ROS_INFO("actual: %f", scan_msg->angle_min);
        ROS_INFO("num: %i", num_scans);
        ROS_INFO("incr: %f", scan_msg->angle_increment);
        ROS_INFO("range 0: %f", scan_msg->ranges[0]);
        ROS_INFO("range f: %f", scan_msg->ranges[1079]);*/
        // TODO: publish drive/brake message
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}