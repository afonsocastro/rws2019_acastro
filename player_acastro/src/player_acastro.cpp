#include <ros/ros.h>
#include <iostream>

const int MY_VAR = 2;
struct A
{
  int B;
};

int doSomething()
{
  return 0;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "player_acastro");
  ros::NodeHandle n;
  for (int i = 0; i < 10; i++)
  {
    std::cout << i << std::endl;
  }
  return 0;
}