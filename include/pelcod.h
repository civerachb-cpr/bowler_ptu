#ifndef __PELCO_D_HEADER__
#define __PELCO_D_HEADER__

// Pelco-D messages are 7 bytes long
#define PELCO_D_MSG_LENGTH    7

// definition for each byte in the pelco-d packet
#define PELCO_D_SYNC_BYTE     0
#define PELCO_D_ADDR_BYTE     1
#define PELCO_D_CMD1_BYTE     2
#define PELCO_D_CMD2_BYTE     3
#define PELCO_D_DATA_1_BYTE   4    // PAN SPEED
#define PELCO_D_DATA_2_BYTE   5    // TILT_SPEED
#define PELCO_D_CSUM_BYTE     6

// the sync byte is fixed to 0xff
#define PELCO_D_SYNC          0xff

// bit-masks for checking/setting fields in COMMAND1
// bits 5 and 6 are reserved
#define PELCO_D_CMD1_SENSE_BIT        (1 << 7)
#define PELCO_D_CMD1_SCAN_BIT         (1 << 4)
#define PELCO_D_CMD1_CAM_ON_OFF_BIT   (1 << 3)
#define PELCO_D_CMD1_IRIS_CLOSE_BIT   (1 << 2)
#define PELCO_D_CMD1_IRIS_OPEN_BIT    (1 << 1)
#define PELCO_D_CMD1_FOCUS_NEAR_BIT   1

// bit-masks for checking/setting fields in COMMAND2
#define PELCO_D_CMD2_FOCUS_FAR_BIT    (1 << 7)
#define PELCO_D_CMD2_ZOOM_WIDE_BIT    (1 << 6)
#define PELCO_D_CMD2_ZOOM_TELE_BIT    (1 << 5)
#define PELCO_D_CMD2_TILT_DOWN_BIT    (1 << 4)
#define PELCO_D_CMD2_TILT_UP_BIT      (1 << 3)
#define PELCO_D_CMD2_PAN_LEFT_BIT     (1 << 2)
#define PELCO_D_CMD2_PAN_RIGHT_BIT    (1 << 1)


/////////////////////////////////////////////////////////////////////
// START 2-byte commands made by combining the bitmasks above      //
/////////////////////////////////////////////////////////////////////

// MOVEMENT COMMANDS
// see https://www.epiphan.com/userguides/LUMiO12x/Content/UserGuides/PTZ/3-operation/PELCODcommands.htm
// commands can be combined with bitwise-OR
// e.g. up+left --> (PELCO_D_COMMAND_TILT_UP | PELCO_D_COMMAND_PAN_LEFT)
// DATA_1 is the pan speed (see a)
// DATA_2 is the tilt speed
#define PELCO_D_CMD_TILT_UP           0x0008
#define PELCO_D_CMD_TILT_DOWN         0x0010
#define PELCO_D_CMD_PAN_LEFT          0x0004
#define PELCO_D_CMD_PAN_RIGHT         0x0002
#define PELCO_D_CMD_ZOOM_IN           0x0020
#define PELCO_D_CMD_ZOOM_OUT          0x0040
#define PELCO_D_CMD_FOCUS_NEAR        0x0100
#define PELCO_D_CMD_FOCUS_FAR         0x0080

// maximum & minimum speeds for the pan/tilt motors
#define PELCO_D_PAN_TILT_STOP         0x00
#define PELCO_D_PAN_TILT_MAX          0x3f

// pan also defines a turbo mode
#define PELCO_PAN_TURBO               0xff

// POSITION QUERY
#define PELCO_D_CMD_QUERY_PAN         0x0051
#define PELCO_D_CMD_QUERY_TILT        0x0053
#define PELCO_D_CMD_QUERY_ZOOM        0x0055

// QUERY RESPONSES
// DATA_1 is the high byte
// DATA_2 is the low byte
#define PELCO_D_CMD_QUERY_PAN_RESP    0x0059
#define PELCO_D_CMD_QUERY_TILT_RESP   0x005B
#define PELCO_D_CMD_QUERY_ZOOM_RESP   0x005D

// PRESETS
// these use 0x00 and <ID> as DATA_1 and DATA_2 bytes
#define PELCO_D_CMD_SET_PRESET        0x0003
#define PELCO_D_CMD_CLEAR_PRESET      0x0005
#define PELCO_D_CMD_CALL_PRESET       0x0007

/////////////////////////////////////////////////////////////////////
// END COMMANDS                                                    //
/////////////////////////////////////////////////////////////////////

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
