#include <iostream>
#include <fstream>
#include <iomanip>

#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace ros;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_recorder");

  if (argc < 2)
  {
    ROS_ERROR("Usage: rosrun lego_loam pose_recorder output.txt");
    return 1;
  }

  // open output file
  ofstream output(argv[1]);
  if (!output.is_open())
  {
    ROS_ERROR_STREAM("Can not open output file: " << argv[1]);
    return 1;
  }

  output << "# timestamp(us) x y z roll pitch yaw" << endl;

  ros::NodeHandle nh;

  tf::TransformListener listener;

  uint64_t last_stamp = 0;

  ros::Rate rate(30.0);
  while (nh.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

      // get timestamp
      uint64_t stamp = transform.stamp_.toNSec();

      // check if updated
      if (stamp == last_stamp)
        continue;
      else
        last_stamp = stamp;

      // get pose
      double x, y, z;
      x = transform.getOrigin().x();
      y = transform.getOrigin().y();
      z = transform.getOrigin().z();

      double roll, pitch, yaw;
      transform.getBasis().getRPY(roll, pitch, yaw);

      // write to file
      output << static_cast<uint64_t>(stamp / 1e3) << fixed << setprecision(6)
             << ' ' << x << ' ' << y << ' ' << z << ' ' << roll << ' ' << pitch
             << ' ' << yaw << endl;
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }

  ROS_INFO_STREAM("Pose data are written to file: " << argv[1]);

  return 0;
}
