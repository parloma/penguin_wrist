#include "ros/ros.h"
#include <penguin_ros/penguin.h>
#include "penguin_wrist.hpp"
#include "math.h"


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  if (argc != 4) {
    std::cout << "Usage: " << argv[0] << " roll pitch yaw" << std::endl;
    return 0;
  }


  PenguinWrist penguin;

  ros::init(argc, argv, "penguin");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<penguin_ros::penguin>("penguin_msgs", 1000);

  ros::Rate loop_rate(40);

  float r = 0, p = 0, y = 0;
  int i = 0;
  while (ros::ok()){

    y = 20*sin(0.1*i++)*M_PI/180;
    penguin.set_rpy(r,p,y);
    std::cout << penguin.get_solution().transpose()*180/M_PI << std::endl;
    penguin_ros::penguin msg;
    msg.up = penguin.get_solution()(0)*180/M_PI;
    msg.middle = penguin.get_solution()(1)*180/M_PI;
    msg.down = penguin.get_solution()(2)*180/M_PI;


    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
