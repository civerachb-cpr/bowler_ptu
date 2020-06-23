#include <iostream>
#include <string>
#include "ros/ros.h"
#include "pelcod.h"

// serial i/o stuff
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace std;
using namespace pelco_d;

int main(int argc, const char* argv[])
{
  if(argc != 3)
  {
    ROS_ERROR("Wrong number of arguments! Expected: %s PORT BAUD", argv[0]);
    exit(1);
  }

  const char* device = argv[1];
  int baud = atoi(argv[2]);
  ROS_INFO("Opening %s @ %d baud", device, baud);
}
