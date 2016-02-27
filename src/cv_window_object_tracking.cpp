#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/utility.hpp>


static const std::string OPENCV_WINDOW = "Object Tracker";

using namespace cv;

class ImageTracker
{
  ros::NodeHandle nh_;///< Private NodeHandle
  image_transport::ImageTransport it_;///< Transposter for getting the image of right format
  image_transport::Subscriber image_sub_;///< Subscriber to image
  image_transport::Publisher image_pub_;///< Publish the resulting image
  static cv::Rect2d roiRect;
  static bool drag;///< Drag flag to select ROI
  static bool select_flag;///< Select flag
  static bool track_flag;///< Flag to start tracking once ROI has been selected
  Ptr<Tracker> tracker;///< OpenCV object tracker

public:
  ImageTracker(std::string tracker_algorithm)
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("image", 1,
                               &ImageTracker::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //Create Tracker
    tracker = Tracker::create( tracker_algorithm );

    if( tracker == NULL )
    {
      ROS_ERROR("***Error in the instantiation of the tracker...***");
      return;
    }

    //Create Windows
    namedWindow(OPENCV_WINDOW);

    //Set Opencv Callbacks
    setMouseCallback(OPENCV_WINDOW,mouseHandler);
  }

  ~ImageTracker()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  static void mouseHandler(int event, int x, int y, int flags, void* param)
  {
    if (event == CV_EVENT_LBUTTONDOWN && !drag)
    {
      /* left button clicked. ROI selection begins */
      roiRect.x = x;
      roiRect.y = y;
      roiRect.width = roiRect.height = 0;
      select_flag = false;//Reselecting template
      track_flag = false;
      drag = true;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag)
    {
      /* mouse dragged. ROI being selected */
      roiRect.width = std::abs(roiRect.x - x);
      roiRect.height = std::abs(roiRect.y - y);
    }

    if (event == CV_EVENT_LBUTTONUP && drag)
    {
      roiRect.width = std::abs(roiRect.x - x);
      roiRect.height = std::abs(roiRect.y - y);
      select_flag = true;
      drag = false;
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //Get a window from the image and say thats the template
    if(drag)
    {
      Point point1 = Point(roiRect.x, roiRect.y);
      Point point2 = Point(roiRect.x + roiRect.width, roiRect.y+roiRect.height);
      rectangle(cv_ptr->image, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);//Draw Rectangle when dragged
    }

    if(select_flag)
    {
      select_flag = false;
      //Roi is selected
      if(roiRect.height > 0 && roiRect.width > 0)
      {
        //Initialize tracker:
        if( !tracker->init(cv_ptr->image, roiRect) )
        {
          ROS_ERROR("***Could not initialize tracker...***\n");
          track_flag = false;
          return;
        }
        else
        {
          //Tracker Initialized
          rectangle(cv_ptr->image,roiRect, Scalar(255,0,0),3,8,0);
          cv::imshow(OPENCV_WINDOW, cv_ptr->image);
          cv::waitKey(3);
          track_flag = true;
          return;
        }
      }
    }
    //Track the template and draw bounding box:
    if(track_flag)
    {
      if( tracker->update( cv_ptr->image, roiRect ) )
      {
        Point point1 = Point(roiRect.x, roiRect.y);
        Point point2 = Point(roiRect.x + roiRect.width, roiRect.y+roiRect.height);
        rectangle(cv_ptr->image, point1, point2, Scalar(255,0,0),3,8,0);
      }
    }
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

Rect2d ImageTracker::roiRect = Rect2d();
bool ImageTracker::drag = false;
bool ImageTracker::select_flag = false;
bool ImageTracker::track_flag = false;

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
