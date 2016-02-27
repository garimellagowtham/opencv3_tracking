#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string ROI_WINDOW = "ROI window";

using namespace cv;

class ImageConverter
{
  ros::NodeHandle nh_;///< Private NodeHandle
  image_transport::ImageTransport it_;///< Transposter for getting the image of right format
  image_transport::Subscriber image_sub_;///< Subscriber to image
  image_transport::Publisher image_pub_;///< Publish the resulting image
  static Point point1, point2;///< Point1 and Point2 of ROI selected
  static bool drag;///< Drag flag to select ROI
  static bool select_flag;///< Select flag

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
                               &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    namedWindow(OPENCV_WINDOW);
    setMouseCallback(OPENCV_WINDOW,mouseHandler);
    namedWindow(ROI_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
    destroyWindow(ROI_WINDOW);
  }

  static void mouseHandler(int event, int x, int y, int flags, void* param)
  {
    if (event == CV_EVENT_LBUTTONDOWN && !drag)
    {
      /* left button clicked. ROI selection begins */
      point1 = Point(x, y);
      select_flag = false;//Reselecting template
      drag = true;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag)
    {
      /* mouse dragged. ROI being selected */
      point2 = Point(x, y);
    }

    if (event == CV_EVENT_LBUTTONUP && drag)
    {
      point2 = Point(x, y);
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
      rectangle(cv_ptr->image, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);//Draw Rectangle when dragged
    }
    if(select_flag)
    {
      //Roi is selected
      Rect roiRect = Rect(point1.x,point1.y,point2.x-point1.x,point2.y-point1.y);
      if(roiRect.height > 0 && roiRect.width > 0)
      {
        Mat roiImg = cv_ptr->image(roiRect);
        imshow(ROI_WINDOW,roiImg);
        cv::waitKey(3);
      }
      select_flag = false;
    }
    //Track the template and draw bounding box:

    // Draw an example circle on the video stream
    /* if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    */
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

Point ImageConverter::point1 = Point();
Point ImageConverter::point2 = Point();
bool ImageConverter::drag = false;
bool ImageConverter::select_flag = false;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
