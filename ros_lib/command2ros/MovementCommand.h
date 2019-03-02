#ifndef _ROS_command2ros_MovementCommand_h
#define _ROS_command2ros_MovementCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace command2ros
{

  class MovementCommand : public ros::Msg
  {
    public:
      typedef int32_t _serialID_type;
      _serialID_type serialID;
      typedef int32_t _driveDirection_type;
      _driveDirection_type driveDirection;
      typedef int32_t _stop_type;
      _stop_type stop;
      typedef int32_t _packIn_type;
      _packIn_type packIn;
      typedef int32_t _packOut_type;
      _packOut_type packOut;
      typedef int32_t _raiseBucketChain_type;
      _raiseBucketChain_type raiseBucketChain;
      typedef int32_t _lowerBucketChain_type;
      _lowerBucketChain_type lowerBucketChain;
      typedef int32_t _speedBucketChain_type;
      _speedBucketChain_type speedBucketChain;
      typedef int32_t _raiseConveyorBelt_type;
      _raiseConveyorBelt_type raiseConveyorBelt;
      typedef int32_t _lowerConveyorBelt_type;
      _lowerConveyorBelt_type lowerConveyorBelt;
      typedef int32_t _speedConveyorBelt_type;
      _speedConveyorBelt_type speedConveyorBelt;
      typedef int32_t _misc1_type;
      _misc1_type misc1;
      typedef int32_t _misc2_type;
      _misc2_type misc2;

    MovementCommand():
      serialID(0),
      driveDirection(0),
      stop(0),
      packIn(0),
      packOut(0),
      raiseBucketChain(0),
      lowerBucketChain(0),
      speedBucketChain(0),
      raiseConveyorBelt(0),
      lowerConveyorBelt(0),
      speedConveyorBelt(0),
      misc1(0),
      misc2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_serialID;
      u_serialID.real = this->serialID;
      *(outbuffer + offset + 0) = (u_serialID.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_serialID.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_serialID.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_serialID.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->serialID);
      union {
        int32_t real;
        uint32_t base;
      } u_driveDirection;
      u_driveDirection.real = this->driveDirection;
      *(outbuffer + offset + 0) = (u_driveDirection.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_driveDirection.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_driveDirection.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_driveDirection.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->driveDirection);
      union {
        int32_t real;
        uint32_t base;
      } u_stop;
      u_stop.real = this->stop;
      *(outbuffer + offset + 0) = (u_stop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stop.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stop);
      union {
        int32_t real;
        uint32_t base;
      } u_packIn;
      u_packIn.real = this->packIn;
      *(outbuffer + offset + 0) = (u_packIn.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_packIn.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_packIn.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_packIn.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->packIn);
      union {
        int32_t real;
        uint32_t base;
      } u_packOut;
      u_packOut.real = this->packOut;
      *(outbuffer + offset + 0) = (u_packOut.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_packOut.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_packOut.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_packOut.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->packOut);
      union {
        int32_t real;
        uint32_t base;
      } u_raiseBucketChain;
      u_raiseBucketChain.real = this->raiseBucketChain;
      *(outbuffer + offset + 0) = (u_raiseBucketChain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_raiseBucketChain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_raiseBucketChain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_raiseBucketChain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->raiseBucketChain);
      union {
        int32_t real;
        uint32_t base;
      } u_lowerBucketChain;
      u_lowerBucketChain.real = this->lowerBucketChain;
      *(outbuffer + offset + 0) = (u_lowerBucketChain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lowerBucketChain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lowerBucketChain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lowerBucketChain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lowerBucketChain);
      union {
        int32_t real;
        uint32_t base;
      } u_speedBucketChain;
      u_speedBucketChain.real = this->speedBucketChain;
      *(outbuffer + offset + 0) = (u_speedBucketChain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speedBucketChain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speedBucketChain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speedBucketChain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speedBucketChain);
      union {
        int32_t real;
        uint32_t base;
      } u_raiseConveyorBelt;
      u_raiseConveyorBelt.real = this->raiseConveyorBelt;
      *(outbuffer + offset + 0) = (u_raiseConveyorBelt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_raiseConveyorBelt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_raiseConveyorBelt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_raiseConveyorBelt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->raiseConveyorBelt);
      union {
        int32_t real;
        uint32_t base;
      } u_lowerConveyorBelt;
      u_lowerConveyorBelt.real = this->lowerConveyorBelt;
      *(outbuffer + offset + 0) = (u_lowerConveyorBelt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lowerConveyorBelt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lowerConveyorBelt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lowerConveyorBelt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lowerConveyorBelt);
      union {
        int32_t real;
        uint32_t base;
      } u_speedConveyorBelt;
      u_speedConveyorBelt.real = this->speedConveyorBelt;
      *(outbuffer + offset + 0) = (u_speedConveyorBelt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speedConveyorBelt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speedConveyorBelt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speedConveyorBelt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speedConveyorBelt);
      union {
        int32_t real;
        uint32_t base;
      } u_misc1;
      u_misc1.real = this->misc1;
      *(outbuffer + offset + 0) = (u_misc1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_misc1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_misc1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_misc1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->misc1);
      union {
        int32_t real;
        uint32_t base;
      } u_misc2;
      u_misc2.real = this->misc2;
      *(outbuffer + offset + 0) = (u_misc2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_misc2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_misc2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_misc2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->misc2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_serialID;
      u_serialID.base = 0;
      u_serialID.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_serialID.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_serialID.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_serialID.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->serialID = u_serialID.real;
      offset += sizeof(this->serialID);
      union {
        int32_t real;
        uint32_t base;
      } u_driveDirection;
      u_driveDirection.base = 0;
      u_driveDirection.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_driveDirection.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_driveDirection.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_driveDirection.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->driveDirection = u_driveDirection.real;
      offset += sizeof(this->driveDirection);
      union {
        int32_t real;
        uint32_t base;
      } u_stop;
      u_stop.base = 0;
      u_stop.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stop.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stop.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stop.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stop = u_stop.real;
      offset += sizeof(this->stop);
      union {
        int32_t real;
        uint32_t base;
      } u_packIn;
      u_packIn.base = 0;
      u_packIn.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_packIn.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_packIn.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_packIn.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->packIn = u_packIn.real;
      offset += sizeof(this->packIn);
      union {
        int32_t real;
        uint32_t base;
      } u_packOut;
      u_packOut.base = 0;
      u_packOut.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_packOut.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_packOut.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_packOut.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->packOut = u_packOut.real;
      offset += sizeof(this->packOut);
      union {
        int32_t real;
        uint32_t base;
      } u_raiseBucketChain;
      u_raiseBucketChain.base = 0;
      u_raiseBucketChain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_raiseBucketChain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_raiseBucketChain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_raiseBucketChain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->raiseBucketChain = u_raiseBucketChain.real;
      offset += sizeof(this->raiseBucketChain);
      union {
        int32_t real;
        uint32_t base;
      } u_lowerBucketChain;
      u_lowerBucketChain.base = 0;
      u_lowerBucketChain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lowerBucketChain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lowerBucketChain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lowerBucketChain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lowerBucketChain = u_lowerBucketChain.real;
      offset += sizeof(this->lowerBucketChain);
      union {
        int32_t real;
        uint32_t base;
      } u_speedBucketChain;
      u_speedBucketChain.base = 0;
      u_speedBucketChain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speedBucketChain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speedBucketChain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speedBucketChain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speedBucketChain = u_speedBucketChain.real;
      offset += sizeof(this->speedBucketChain);
      union {
        int32_t real;
        uint32_t base;
      } u_raiseConveyorBelt;
      u_raiseConveyorBelt.base = 0;
      u_raiseConveyorBelt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_raiseConveyorBelt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_raiseConveyorBelt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_raiseConveyorBelt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->raiseConveyorBelt = u_raiseConveyorBelt.real;
      offset += sizeof(this->raiseConveyorBelt);
      union {
        int32_t real;
        uint32_t base;
      } u_lowerConveyorBelt;
      u_lowerConveyorBelt.base = 0;
      u_lowerConveyorBelt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lowerConveyorBelt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lowerConveyorBelt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lowerConveyorBelt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lowerConveyorBelt = u_lowerConveyorBelt.real;
      offset += sizeof(this->lowerConveyorBelt);
      union {
        int32_t real;
        uint32_t base;
      } u_speedConveyorBelt;
      u_speedConveyorBelt.base = 0;
      u_speedConveyorBelt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speedConveyorBelt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speedConveyorBelt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speedConveyorBelt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speedConveyorBelt = u_speedConveyorBelt.real;
      offset += sizeof(this->speedConveyorBelt);
      union {
        int32_t real;
        uint32_t base;
      } u_misc1;
      u_misc1.base = 0;
      u_misc1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_misc1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_misc1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_misc1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->misc1 = u_misc1.real;
      offset += sizeof(this->misc1);
      union {
        int32_t real;
        uint32_t base;
      } u_misc2;
      u_misc2.base = 0;
      u_misc2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_misc2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_misc2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_misc2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->misc2 = u_misc2.real;
      offset += sizeof(this->misc2);
     return offset;
    }

    const char * getType(){ return "command2ros/MovementCommand"; };
    const char * getMD5(){ return "4d53d58ab1133b4758635512ae9b0bf6"; };

  };

}
#endif