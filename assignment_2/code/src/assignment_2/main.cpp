#include <ros/ros.h>
#include "line_following.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "stdr_line_following", ros::init_options::AnonymousName);

  assignment_2::LineFollowing obj(argc, argv);

  ros::spin();

  return 0;

  //ros::NodeHandle node;

  //ROS_INFO_STREAM("hello world!");

}
