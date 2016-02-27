#ifndef ROISELECTOR_H
#define ROISELECTOR_H

#include <ros/ros.h>

class RoiSelector
{
public:
  ros::NodeHandle nh_;///< Private NodeHandle
  ros::Publisher rect_pub_;///<Publish rectangle coordinates
  RoiSelector()
  {
      //Get Params
      //Declare publishers
  }
};

#endif // ROISELECTOR_H
