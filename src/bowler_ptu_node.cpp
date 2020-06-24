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

  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  int port = open(device, O_RDWR);
  if(tcgetattr(port, &tty) != 0)
  {
    ROS_ERROR("Failed to open serial port: %i (%s)", errno, strerror(errno));
    exit(1);
  }

  // configure for 8n1 at the specified baud rate
  tty.c_cflag &= ~PARENB;     // disable parity
  tty.c_cflag &= ~CSTOPB;     // single parity bit
  tty.c_cflag |= CS8;         // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS;    // disable RTS/CTS flow
  tty.c_lflag &= ~ICANON;     // disable canonical mode
  tty.c_cflag &= ~ECHO;       // disable echo
  tty.c_lflag &= ~ECHOE;      // disable erasure
  tty.c_lflag &= ~ECHONL;     // disable newline echo
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  switch(baud)
  {
    case 0:
      cfsetispeed(&tty, B0);
      cfsetospeed(&tty, B0);
      break;

    case 50:
      cfsetispeed(&tty, B50);
      cfsetospeed(&tty, B50);
      break;

    case 75:
      cfsetispeed(&tty, B75);
      cfsetospeed(&tty, B75);
      break;

    case 110:
      cfsetispeed(&tty, B110);
      cfsetospeed(&tty, B110);
      break;

    case 134:
      cfsetispeed(&tty, B134);
      cfsetospeed(&tty, B134);
      break;

    case 200:
      cfsetispeed(&tty, B200);
      cfsetospeed(&tty, B200);
      break;

    case 300:
      cfsetispeed(&tty, B0);
      cfsetospeed(&tty, B0);
      break;

    case 600:
      cfsetispeed(&tty, B300);
      cfsetospeed(&tty, B300);
      break;

    case 1200:
      cfsetispeed(&tty, B1200);
      cfsetospeed(&tty, B1200);
      break;

    case 2400:
      cfsetispeed(&tty, B2400);
      cfsetospeed(&tty, B2400);
      break;

    case 4800:
      cfsetispeed(&tty, B4800);
      cfsetospeed(&tty, B4800);
      break;

    case 9600:
      cfsetispeed(&tty, B9600);
      cfsetospeed(&tty, B9600);
      break;

    case 19200:
      cfsetispeed(&tty, B19200);
      cfsetospeed(&tty, B19200);
      break;

    case 38400:
      cfsetispeed(&tty, B38400);
      cfsetospeed(&tty, B38400);
      break;

    case 57600:
      cfsetispeed(&tty, B57600);
      cfsetospeed(&tty, B57600);
      break;

    case 115200:
      cfsetispeed(&tty, B115200);
      cfsetospeed(&tty, B115200);
      break;

    case 230400:
      cfsetispeed(&tty, B230400);
      cfsetospeed(&tty, B230400);
      break;

    case 460800:
      cfsetispeed(&tty, B460800);
      cfsetospeed(&tty, B460800);
      break;

    default:
      ROS_WARN("Non-standard baud rate specified: %i", baud);
      cfsetispeed(&tty, baud);
      cfsetospeed(&tty, baud);
      break;
  }

  if (tcsetattr(port, TCSANOW, &tty) != 0)
  {
    ROS_ERROR("Failed to configure serial port: %i (%s)\n", errno, strerror(errno));
    exit(1);
  }

  int n;
  uint8_t buffer[PELCO_D_MSG_LENGTH];
  uint8_t checksum;
  uint8_t test_command[] = {
    PELCO_D_SYNC,
    0x00,
    ((PELCO_D_CMD_QUERY_PAN >> 8) & 0xff),
    (PELCO_D_CMD_QUERY_PAN & 0xff),
    0x00,
    0x00,
    0x00
  };
  test_command[PELCO_D_CSUM_BYTE] = pelco_d::calculate_checksum(test_command);

  for(;;)
  {
    n = write(port, test_command, PELCO_D_MSG_LENGTH);

    if(n < PELCO_D_MSG_LENGTH)
    {
      ROS_WARN("Only write %d bytes!", n);
    }
    else
    {
      n = read(port, buffer, PELCO_D_MSG_LENGTH);

      if(n < PELCO_D_MSG_LENGTH)
      {
        ROS_WARN("Did not read whole packet! Only %d bytes recv'd", n);
      }
      else
      {
        uint8_t checksum = pelco_d::calculate_checksum(buffer);
        if(checksum != buffer[PELCO_D_CSUM_BYTE])
        {
          ROS_WARN("Checksum mismatch! e:%x r:%x", checksum ,buffer[PELCO_D_CSUM_BYTE]);
        }

        ROS_INFO("Response: %x %x %x %x %x %x %x", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6]);
      }
    }
  }// loop
}// mains
