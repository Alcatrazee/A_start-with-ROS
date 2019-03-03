#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_node");
  ros::NodeHandle nh;
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 10);

  nav_msgs::Path path;
  ros::Rate rate(1);
  geometry_msgs::PoseStamped poses[30];
  int k = 1;
  while (ros::ok())
  {
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    if (k % 2 == 0)
    {
      for (int i = 0; i < 30; i++)
      {
        poses[i].header.frame_id = "map";
        poses[i].header.stamp = ros::Time::now();
        poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(0);
        poses[i].pose.position.x = (float)i / 10 * k;
        poses[i].pose.position.y = (float)i / 10 * k;
        poses[i].pose.position.z = 0;
        path.poses.push_back(poses[i]);
      }
    }
    else
    {
      path.poses.clear();
    }
    k += 1;

    path_pub.publish(path);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}