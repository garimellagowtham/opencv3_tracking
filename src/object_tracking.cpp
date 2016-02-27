#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>



using namespace cv;

class ImageTracker
{
  ros::NodeHandle nh_;///< Private NodeHandle
  image_transport::ImageTransport it_;///< Transposter for getting the image of right format
  image_transport::Subscriber image_sub_;///< Subscriber to image
  image_transport::Publisher image_pub_;///< Publish the resulting image
  ros::Subscriber roi_subscriber_;///< Region of Interest subscriber
  ros::Publisher roi_publisher_;///< Tracked Region of Interest publisher
  cv::Rect2d roi_rect_;
  Ptr<Tracker> tracker;///< OpenCV object tracker
  bool tracker_initialized_;///< If Tracker has been initialized with a ROI to track
  bool roi_received_;///< flag to specify roi has been received and tracker should be reinitialized

public:
  ImageTracker(std::string tracker_algorithm)
    : it_(nh_), tracker_initialized_(false), roi_received_(false)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("image", 1,
                               &ImageTracker::imageCb, this);
    image_pub_ = it_.advertise("/object_tracker/output", 1);

    roi_subscriber_ = nh_.subscribe("roi",1,&ImageTracker::roiCallback,this);

    roi_publisher_ = nh_.advertise<sensor_msgs::RegionOfInterest>("roi_out",1);

    //Create Tracker
    tracker = Tracker::create( tracker_algorithm );

    if( tracker == NULL )
    {
      ROS_ERROR("***Error in the instantiation of the tracker...***");
      return;
    }
  }

  void roiCallback(const sensor_msgs::RegionOfInterestConstPtr& msg)
  {
    roi_rect_.x = msg->x_offset;
    roi_rect_.y = msg->y_offset;
    roi_rect_.width = msg->width;
    roi_rect_.height = msg->height;
    roi_received_ = true;
    ROS_INFO("Roi received");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr_image_copy_;///< Copy of the image received
    try
    {
      cv_ptr_image_copy_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //Initialize Tracker:
    if(roi_received_)
    {
      roi_received_ = false;
      if(roi_rect_.height > 0 && roi_rect_.width > 0)
      {
        ROS_INFO("Initializing tracker");
        //Initialize tracker:
        if( !tracker->init(cv_ptr_image_copy_->image, roi_rect_) )
        {
          ROS_ERROR("***Could not initialize tracker...***\n");
          tracker_initialized_ = false;
        }
        else
        {
          //Tracker Initialized
          rectangle(cv_ptr_image_copy_->image,roi_rect_, Scalar(255,0,0),3,8,0);
          tracker_initialized_ = true;
        }
      }
    }
    else
    {
      //Track the template and draw bounding box:
      if(tracker_initialized_)
      {
        if( tracker->update( cv_ptr_image_copy_->image, roi_rect_ ) )
        {
          Point point1 = Point(roi_rect_.x, roi_rect_.y);
          Point point2 = Point(roi_rect_.x + roi_rect_.width, roi_rect_.y+roi_rect_.height);
          rectangle(cv_ptr_image_copy_->image, point1, point2, Scalar(255,0,0),3,8,0);
          //Publish ros msg:
          sensor_msgs::RegionOfInterest roi_out_msg;
          roi_out_msg.x_offset = roi_rect_.x;
          roi_out_msg.y_offset = roi_rect_.y;
          roi_out_msg.width = roi_rect_.width;
          roi_out_msg.height = roi_rect_.height;
          roi_publisher_.publish(roi_out_msg);
        }
      }
    }

    // Output modified video stream
    if(image_pub_.getNumSubscribers()>0)
    {
      image_pub_.publish(cv_ptr_image_copy_->toImageMsg());
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  //Get Tracker Algorithm
  std::string tracking_algorithm = "MIL";
  ros::param::get("tracking_algorithm",tracking_algorithm);
  ROS_INFO("Tracking algo used: %s",tracking_algorithm.c_str());
  ImageTracker ic(tracking_algorithm);
  ros::spin();
  return 0;
}
