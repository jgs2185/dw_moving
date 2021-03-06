#include <errno.h>
#include <signal.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "target.cpp"
#include "radar.hpp"

//TODO use init list
Radar::Radar(std::string fw,double x_in, double y_in, double theta_in)
{
  radar_firmware = fw;
  numTargets     = 0;
  (this -> pose).x     = x_in;
  (this -> pose).y     = y_in;
  (this -> pose).theta = theta_in;
}

Radar::~Radar()
{
  delete[] targetArray;
}

bool Radar::init()
{
  bool init_okay = false;

  init_okay = check_firmware();
  init_okay = config_socketcan();

  return init_okay;
}


//send the starting command
bool Radar::activate()
{
  //TODO implement checking
  bool ret = true;
  int nbytes;
  struct can_frame frame;

  //format frame to the start command!
  //this is standard frame format
  frame.can_id = 0x100;
  frame.can_dlc = 8;
  frame.data[0] = 0x01;
  frame.data[1] = 0x00;
  frame.data[2] = 0xff;
  frame.data[3] = 0xff;
  frame.data[4] = 0xff;
  frame.data[5] = 0xff;
  frame.data[6] = 0xff;
  frame.data[7] = 0xff;

  nbytes = write(s, &frame, sizeof(struct can_frame));
  std::cout << "bytes wroten: " << nbytes << std::endl;
  return ret;
}

bool Radar::deactivate()
{
  //TODO implement checking
  bool ret = true;
  int nbytes;
  struct can_frame frame;

  //format frame to the start command!
  //this is standard frame format
  frame.can_id = 0x100;
  frame.can_dlc = 8;
  frame.data[0] = 0x02;
  frame.data[1] = 0x00;
  frame.data[2] = 0xff;
  frame.data[3] = 0xff;
  frame.data[4] = 0xff;
  frame.data[5] = 0xff;
  frame.data[6] = 0xff;
  frame.data[7] = 0xff;

  nbytes = write(s, &frame, sizeof(struct can_frame));
  std::cout << "bytes wroten: " << nbytes << std::endl;
  return ret;
}

//TODO Add functions for switching the mode and or output type(raw detection vs target tracking..)

//look for a header ID, then parse all targets until you see footer ID
bool Radar::get_scan()
{
  bool ret = true;
  bool footerFound = false;
  bool headerFound = false;
  int  targetCount = 0;
  int  nbytes;

  while(footerFound == false)
  {

    struct can_frame frame;
    nbytes = read(s, &frame, sizeof(struct can_frame));
    //make sure frame is something
    if(nbytes < 0)
    {
      std::cout << "can raw socket read error" << std::endl;
      return false;
    }
    //make sure frame is a frame
    else if((unsigned)nbytes < sizeof(struct can_frame))
    {
      std::cout << "read error: incomplete frame" << std::endl;
      return false;
    }
    //frame is 'safer' to parse
    else
    {
      int frame_id = frame.can_id;
      //can frame ID is header ID
      if(frame_id == (this -> header_id))
      {
        headerFound = true;
        //TODO get # of targets for comparing in footer
      }
      //can frame ID is footer ID and header has been processed
      else if(frame_id == (this -> footer_id) && headerFound)
      {
        footerFound = true;
        numTargets = targetCount;
      }
      //can frame ID is in the target range and header has been processed.
      else if( (frame_id >= (this -> target_frame_min) ) && (frame_id <= (this -> target_frame_max) ) && (headerFound) )
      {
        //processing can frame into radar target object.
        float range = (int16_t)( (frame.data[2] << 8) + frame.data[3]) / 100.0;
        float az = (int16_t)( (frame.data[6] << 8) + frame.data[7]) / 100.0 * -1;
        float vel = (int16_t)( (frame.data[4] << 8) + frame.data[5]) / 100.0;
        float snr = frame.data[1];
        int id = frame.data[0]; 
        targetArray[targetCount].set_id(id);
        targetArray[targetCount].set_range(range);
        targetArray[targetCount].set_az(az);
        targetArray[targetCount].set_velocity(vel);
        targetArray[targetCount].set_snr(snr);
        targetCount++;
      }
      //add more can frame id checks starting here. elifs
    }

  }

  return ret;
}

void Radar::print_scan_info()
{
  std::cout << "Number of Targets in radar: " << numTargets << std::endl;
  for(int x = 0; x < numTargets; x++)
  {
    std::cout << "Id: " << targetArray[x].get_id() << ", ";
    std::cout << "Range: " << targetArray[x].get_range() << ", ";
    std::cout << "Velocity: " << targetArray[x].get_velocity() << std::endl;
    std::cout << "Azimuth Angle: " << targetArray[x].get_az() << ", ";
    std::cout << "SNR: " << targetArray[x].get_snr() << std::endl;
  }
}



bool Radar::check_firmware()
{
//TODO put actual numbers in here
  bool firmware_matched = false;

  if(radar_firmware == "k77_default")
  {
    max_targets = 65;
    header_id = 1086;
    footer_id = 1087;
    target_frame_min = 1024;
    target_frame_max = 1084;
    targetArray = new Target[max_targets];
    firmware_matched = true;
  }
  else if(radar_firmware == "t79_short_range")
  {
    max_targets = 65;
    header_id   = 1086;
    footer_id   = 1087;
    target_frame_min = 1024;
    target_frame_max = 1084;
    targetArray = new Target[max_targets];
    firmware_matched = true;
  }
  else if(radar_firmware == "t79_bsd")
  {
    max_targets = 65;
    header_id   = 1086;
    footer_id   = 1087;
    target_frame_min = 1024;
    target_frame_max = 1084;
    targetArray = new Target[max_targets];
    firmware_matched = true;
  }

  return firmware_matched;
}


bool Radar::config_socketcan()
{
  //open the socket
  if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    std::cout << "socket failed" << std::endl;
    return false;
  }
  
  //determine interface index
  strcpy(ifr.ifr_name, "can0");
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
  {
    std::cout << "ioctl failed" << std::endl;
    return false;
  }
  
  //assign interface index
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if(bind( s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    std::cout << "bind failed" << std::endl;
    return false;
  }

  //if you made it here you're a success
  return true;

}





