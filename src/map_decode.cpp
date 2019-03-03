#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <stdio.h>
using namespace std;

void map_rec_callback(const nav_msgs::OccupancyGrid map)
{
  ROS_INFO("map got!\r\n");
  int map_arr[map.info.width][map.info.height];
  int count = 0;
  for (int i = 0; i < map.info.width; i++)
  {
    for (int j = 0; j < map.info.height; j++)
    {
      map_arr[i][j] = map.data[count];
      count++;
      printf("%3d", map_arr[i][j]);
    }
    cout << endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_decode_node");
  ros::NodeHandle nh;
  ros::Subscriber suber = nh.subscribe("/map", 1, map_rec_callback);
  ros::Rate rate(1);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}