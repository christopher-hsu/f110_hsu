#include <ros/ros.h>
// #include <iostream>
// #include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <tf/transform_datatypes.h>
#include <vector>
// TODO: include ROS msg type headers and libraries

class ReactiveFollowGap{
// The class follows the largest gap
private:
    int const BUBBLE_SZ = 10;
    // TODO: create ROS subscribers and publishers
    ros::NodeHandle n;
    ros::Subscriber lidar_sub;
    ros::Publisher drive_pub;


public:
    struct gap {
            int start_i;
            int end_i;
    };
    struct action {
        double speed;
        double angle;
    };
    struct lidar {
        std::vector<float> ranges;
        std::vector<float> angles;
    };  
    ReactiveFollowGap() {
        n = ros::NodeHandle();
        lidar_sub = n.subscribe("/scan", 10, &ReactiveFollowGap::lidar_callback, this);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);  
    }

    lidar preprocess_lidar(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        std::cout << scan_msg->angle_increment;
        lidar data;
        std::cout << scan_msg->angle_increment;
        float scan_angle = scan_msg->angle_min;

        // data.ranges = scan_msg->ranges;

        int num_scans = (scan_msg->angle_max-scan_msg->angle_min)/(scan_msg->angle_increment)+1;
        std::cout << num_scans;
        for (int ii = 0; ii < num_scans; ii++) 
        {
            scan_angle += scan_msg->angle_increment;
            data.angles[ii] = scan_angle;
            data.ranges[ii] = scan_msg->ranges[ii];
        }
        std::cout << data.angles[20];
        return data;
    }

    int closest_point(const sensor_msgs::LaserScan::ConstPtr &scan_msg, int &num_scans) {
        // Find closest point to lidar
        double poi = 101.0; //point of interest, closest point
        int idx;
        for (int ii = 0; ii < num_scans; ii++)
        {
        	if (scan_msg->ranges[ii] < poi)
        	{
        		poi = scan_msg->ranges[ii];
        		idx = ii;
        	}
       	}
       	return idx;
    }

    std::vector<float> bubble(const sensor_msgs::LaserScan::ConstPtr &scan_msg, int &poi_idx) {
        // Zero out all ranges in the bubble radius
        std::vector<float> virt_lidar = scan_msg->ranges;
        for (int ii = 0; ii < BUBBLE_SZ; ii++)
        {
            virt_lidar[poi_idx-ii] = 0.0;
            virt_lidar[poi_idx+ii] = 0.0;
        } 
        // for (float n : virt_lidar){
        //     std::cout << n<< '\n';
        // }
        // std::cout << "rand"<<virt_lidar[234]<< '\n'<<"min"<< virt_lidar[poi_idx]<<'\n';
        return virt_lidar;
    }

    gap find_max_gap(std::vector<float> &virt_lidar, int &num_scans) {
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        gap bigGap;
        int start_temp;
        int largest = 0;
        int count = 0;
        for (int ii = 0; ii < num_scans; ii++)
        {
            if ((virt_lidar[ii] != 0) && (ii == 0))
            {
                start_temp = ii;
                count++;
            }
            else if ((virt_lidar[ii-1] == 0) && (virt_lidar[ii] != 0))
            {
                start_temp = ii;
                count++;
            }
            else if ((virt_lidar[ii-1] != 0) && (virt_lidar[ii] != 0))
            {
                count++;
            }
            else if ((virt_lidar[ii-1] != 0) && (virt_lidar[ii] == 0))
            {
                if (count > largest)
                {
                    bigGap.start_i = start_temp;
                    bigGap.end_i = ii-1;
                    largest = count;
                    count = 0;
                }
                else
                {
                    count = 0;
                }
            }
            else 
            {
                count = 0;
            }
        }
        // for (float n : virt_lidar){
        //     std::cout << n<< '\n';
        // }
        // std::cout << "start_i"<< start_i << '\n';
        // std::cout << "end_i" << end_i << '\n';
        return bigGap;
    }

    int find_best_point(std::vector<float> &virt_lidar, gap &bigGap) {
        // Return the index of the best point in ranges
        int best_idx;
        float best_range = 0;
        for (int ii = bigGap.start_i; ii < bigGap.end_i+1; ii++)
        {
            if (virt_lidar[ii] > best_range)
            {
                best_idx = ii;
                best_range = virt_lidar[ii];
            }
        }
        return best_idx;
    }

    action drive_action(const lidar &data, int &best_idx){
        // Determine speed and angle to execute
        action drive;
        std::cout << data.angles[best_idx]<< '/n';
        // drive.angle = data.angles[best_idx];
        // // if (drive.angle)
        // drive.speed = 1.0;

        return drive;
    }

    void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // Follow the gap algorithm
        int num_scans = (scan_msg->angle_max-scan_msg->angle_min)/(scan_msg->angle_increment)+1;
        lidar data;
        gap bigGap;
        action drive;

        // Preprocess scan_msg ranges and angles
        data = ReactiveFollowGap::preprocess_lidar(scan_msg);
        // std::cout << data.angles[20] << '\n';
        // Find closest point to LiDAR
        int poi_idx = ReactiveFollowGap::closest_point(scan_msg, num_scans);
        // Eliminate all points inside 'bubble' (set them to zero) 
        std::vector<float> virt_lidar = ReactiveFollowGap::bubble(scan_msg, poi_idx);
        // Find max length gap indexes
        bigGap = ReactiveFollowGap::find_max_gap(virt_lidar, num_scans);
        // Find the index of best (furthest) point in the gap 
        int best_idx = ReactiveFollowGap::find_best_point(virt_lidar, bigGap);
        // Determine speed and angle
        // drive = ReactiveFollowGap::drive_action(data, best_idx);
        // Publish drive message
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = drive.angle;
        drive_msg.drive.speed = drive.speed;
        drive_pub.publish(drive_msg);
    }

};
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "gap_follow_node");
    ReactiveFollowGap rfg;
    ros::spin();
    return 0;
}