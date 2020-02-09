// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"

#include <cmath>


class LidarProcessing
{
public:
  LidarProcessing()
  {
    //Topic you want to publish
    pub_close = n_.advertise<std_msgs::Float64>("/closest_point", 1);
    pub_far = n_.advertise<std_msgs::Float64>("/farthest_point", 1);

    //Topic you want to subscribe
    sub_scan = n_.subscribe("/scan", 1, &LidarProcessing::callback, this);
  }

  void callback(const sensor_msgs::LaserScan& msg)
  {
    // initialize message to be published
    // sensor_msgs::LaserScan output;
    std_msgs::Float64 closest_point;
    std_msgs::Float64 farthest_point;
    //.... do something with the input and generate the output...
    std::cout << "hello" << std::endl;




    std::array<float> ranges = msg->ranges;

    double ranges = ranges[std::isfinite(ranges)];

    std::array<double> output

    pub_close.publish(output);
    pub_far.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_close;
  ros::Publisher pub_far;
  ros::Subscriber sub_scan;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "lidar_processing");

  //Create an object of class SubscribeAndPublish that will take care of everything
  LidarProcessing lidar;

  ros::spin();

  return 0;
}