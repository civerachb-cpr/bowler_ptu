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

  // use a state machine to listen to the serial port
  typedef enum PROGRAM_STATE{
    INIT           = 0,     // initial state, waiting for header/sync byte
    BYTE_1_RECV    = 1,     // received sync, waiting for address
    BYTE_2_RECV    = 2,     // received address, waiting for command 1
    BYTE_3_RECV    = 3,     // received command 1, waiting for command 2
    BYTE_4_RECV    = 4,     // received command 2, waiting for data 1
    BYTE_5_RECV    = 5,     // received data 1, waiting for data 2
    BYTE_6_RECV    = 6,     // received data 2, waiting for checksum. Process the packet when we exit this state
  } state_t;

  int n;
  uint8_t ch;
  state_t state;
  uint8_t buffer[PELCO_D_MSG_LENGTH];
  uint8_t checksum;
  for(;;)
  {
    n = read(port, &ch, 1);

    if(n < 0)
    {
      ROS_WARN("Error reading data from serial port: %i", n);
    }
    else if(n > 0) // we've successfully read data!
    {
      buffer[state] = ch;

      switch(state)
      {
        case INIT:
          if (ch == PELCO_D_SYNC)
          {
            state = BYTE_1_RECV;
          }
          break;

          case BYTE_1_RECV:
            state = BYTE_2_RECV;
            break;

          case BYTE_2_RECV:
            state = BYTE_3_RECV;
            break;

          case BYTE_3_RECV:
            state = BYTE_4_RECV;
            break;

          case BYTE_4_RECV:
            state = BYTE_5_RECV;
            break;

          case BYTE_5_RECV:
            state = BYTE_6_RECV;
            break;

          case BYTE_6_RECV:
            checksum = pelco_d::calculate_checksum(buffer);
            if(checksum != ch)
            {
              ROS_WARN("Checksums do not match! r: %x e: %x", ch, checksum);
            }

            ROS_INFO("Received data: %x %x %x %x %x %x %x", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6]);

            state = INIT;
            break;
      }// switch
    }// read-data success
  }// loop
}// mains
