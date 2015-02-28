#ifndef _ROS_volti_float32_3_h
#define _ROS_volti_float32_3_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace volti
{

  class float32_3 : public ros::Msg
  {
    public:
      float data[3];

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_datai;
      u_datai.base = 0;
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data[i] = u_datai.real;
      offset += sizeof(this->data[i]);
      }
     return offset;
    }

    const char * getType(){ return "volti/float32_3"; };
    const char * getMD5(){ return "b97789d46afcb9293725ec7b2cfcf8bb"; };

  };

}
#endif