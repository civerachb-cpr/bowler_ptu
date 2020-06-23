#ifndef __PELCO_D_HEADER__
#define __PELCO_D_HEADER__

// Pelco-D messages are 7 bytes long
#define PELCO_D_MSG_LENGTH    7

// definition for each byte in the pelco-d packet
#define PELCO_D_SYNC_BYTE     0
#define PELCO_D_ADDR_BYTE     1
#define PELCO_D_CMD1_BYTE     2
#define PELCO_D_CMD2_BYTE     3
#define PELCO_D_PAN_BYTE      4
#define PELCO_D_TILT_BYTE     5
#define PELCO_D_CSUM_BYTE     6

// the sync byte is fixed to 0xff
#define PELCO_D_SYNC          0xff

// maximum & minimum speeds for the pan/tilt motors
#define PELCO_D_PAN_TILT_STOP 0x00
#define PELCO_D_PAN_TILT_MAX  0x3f

// pan also defines a turbo mode
#define PELCO_PAN_TURBO       0xff

// bit-masks for checking/setting fields in COMMAND1
// bits 5 and 6 are reserved
#define PELCO_D_CMD1_SENSE_BIT        (1 << 7)
#define PELCO_D_CMD1_SCAN_BIT         (1 << 4)
#define PELCO_D_CMD1_CAM_ON_OFF_BIT   (1 << 3)
#define PELCO_D_CMD1_IRIS_CLOSE_BIT   (1 << 2)
#define PELCO_D_CMD1_IRIS_OPEN_BIT    (1 << 1)
#define PELCO_D_CMD1_FOCUS_NEAR_BIT   1

// bit-masks for checking/setting fields in COMMAND2
// bit 0 is fixed to 0x00
#define PELCO_D_CMD2_FOCUS_FAR_BIT    (1 << 7)
#define PELCO_D_CMD2_ZOOM_WIDE_BIT    (1 << 6)
#define PELCO_D_CMD2_ZOOM_TELE_BIT    (1 << 5)
#define PELCO_D_CMD2_TILT_DOWN_BIT    (1 << 4)
#define PELCO_D_CMD2_TILT_UP_BIT      (1 << 3)
#define PELCO_D_CMD2_PAN_LEFT_BIT     (1 << 2)
#define PELCO_D_CMD2_PAN_RIGHT_BIT    (1 << 1)

namespace pelco_d
{
  /*!
  \brief Calculate the checksum of a Pelco-D message

  Note: we assume the array is at least PELCO_D_MSG_LENGTH bytes long
  If it's not, this may cause an array overflow!
  */
  inline uint8_t calculate_checksum(uint8_t message[])
  {
    uint8_t sum = 0x00;
    for(int i=0; i<PELCO_D_MSG_LENGTH-1; i++)
      sum += message[i];

    return sum;
  }
}

#endif
