#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>



using namespace cv;

class ImageTracker
{
  ros::NodeHandle nh_;///< Private NodeHandle
  image_transport::ImageTransport it_;///< Transposter for getting the image of right format
  image_transport::Subscriber image_sub_;///< Subscriber to image
  image_transport::Subscriber roi_image_sub_;///< Subscriber to image
  image_transport::Publisher image_pub_;///< Publish the resulting image
  ros::Subscriber roi_subscriber_;///< Region of Interest subscriber
  ros::Subscriber camera_info_subscriber_;///< Camera Info subscriber
  ros::Publisher roi_publisher_;///< Tracked Region of Interest publisher
  cv::Rect2d roi_rect_;
  cv_bridge::CvImagePtr roi_image_copy_;///< Copy of the Roi image received
  Ptr<Tracker> tracker;///< OpenCV object tracker
  bool tracker_initialized_;///< If Tracker has been initialized with a ROI to track
  bool roi_received_;///< flag to specify roi has been received and tracker should be reinitialized
  bool roi_image_received_;///< Received roi and corresponding image
  bool cam_info_received_;
  bool use_features_; ///< use feature matching to initialize roi in latest image
  std::string feature_type_; ///< type of feature to use for feature matching

  double fx_, fy_, cx_, cy_;
  double yaw_offset_;

  Ptr<FeatureDetector> detector_;
  std::vector<KeyPoint> roi_kps_;
  Mat roi_descs_;

public:
  ImageTracker(std::string tracker_algorithm)
    : it_(nh_), tracker_initialized_(false), roi_received_(false), cam_info_received_(false)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("image", 1,
                               &ImageTracker::imageCb, this);
    camera_info_subscriber_ = nh_.subscribe("camera_info", 1,
                               &ImageTracker::cameraInfoCb, this);
    roi_image_sub_ = it_.subscribe("roi_image", 1,
                               &ImageTracker::roiImageCb, this);
    image_pub_ = it_.advertise("/object_tracker/output", 1);

    roi_subscriber_ = nh_.subscribe("roi",1,&ImageTracker::roiCallback,this);

    roi_publisher_ = nh_.advertise<sensor_msgs::RegionOfInterest>("roi_out",1);

    nh_.param<bool>("/object_tracker/use_features", use_features_, false);
    if(use_features_)
    {
      ROS_INFO("Using feature matching for ROI initialization");
      nh_.param<std::string>("feature_type", feature_type_, "ORB");
      if(feature_type_ == "ORB")
      {
        detector_ = ORB::create(2000);
      }
      else
      {
        ROS_ERROR("\"%s\" is not a valid feature type", feature_type_.c_str());
      }
    }

    //Create Tracker
#if (CV_MINOR_VERSION < 3)
    { tracker = Tracker::create(tracker_algorithm); }
#else
    {
      if (tracker_algorithm == "BOOSTING")
        tracker = TrackerBoosting::create();
      if (tracker_algorithm == "MIL")
        tracker = TrackerMIL::create();
      if (tracker_algorithm == "KCF")
        tracker = TrackerKCF::create();
      if (tracker_algorithm == "TLD")
        tracker = TrackerTLD::create();
      if (tracker_algorithm == "MEDIANFLOW")
        tracker = TrackerMedianFlow::create();
      if (tracker_algorithm == "GOTURN")
        tracker = TrackerGOTURN::create();
      if (tracker_algorithm == "MOSSE")
        tracker = TrackerMOSSE::create();
    }
#endif

    if( tracker == NULL )
    {
      ROS_ERROR("***Error in the instantiation of the tracker...***");
      return;
    }
  }

  bool detectROIFeatures()
  {
    if( roi_rect_.height == 0 || roi_rect_.width == 0)
      return false;
    Mat roi_mask(roi_image_copy_->image.rows, roi_image_copy_->image.cols, CV_8UC1, Scalar(0));
    roi_mask(Range(roi_rect_.y, roi_rect_.y+roi_rect_.height-1),
      Range(roi_rect_.x, roi_rect_.x+roi_rect_.width-1)) = Scalar(255); 
    detector_->detectAndCompute(roi_image_copy_->image, roi_mask, roi_kps_, roi_descs_);
    return true;
  }

  void filterMatchesEpipolarConstraint(
    std::vector<cv::Point2f>& pts1,
    std::vector<cv::Point2f>& pts2)
  {
    std::vector<unsigned char> status;
    std::vector<cv::Point2f> filt_pts1, filt_pts2;
    cv::Mat fMat = findFundamentalMat(pts1, pts2, CV_FM_RANSAC, 3, .99, status);

    for(size_t i = 0; i < status.size(); i++)
    {
      if(status[i]) 
      {
        filt_pts1.push_back(pts1[i]);
        filt_pts2.push_back(pts2[i]);
      }
    }
    pts1 = filt_pts1;
    pts2 = filt_pts2;
  }
  
  std::vector<DMatch> filterMatchesRatioTest(const std::vector< std::vector< DMatch> >& matches,
    double ratio)
  {
    std::vector<DMatch> good_matches;
    for(size_t j = 0; j < matches.size(); ++j)
    {
      if(matches[j][0].distance < ratio*matches[j][1].distance)
      {
        good_matches.push_back(matches[j][0]);
      }
    }
    return good_matches;
  }

  std::vector<Point2f> filterPointsBoundingBox(const std::vector<Point2f>& pts,
    const cv::Rect2d& roi_rect, int num_samples)
  {
     num_samples = std::min(num_samples, int(pts.size()));
     std::vector<Point2f> best_pts;
     for(int i = 0; i < num_samples; i++)
     {
       int r = rand() % pts.size();
       const Point2f& mid = pts[r];
       std::vector<Point2f> good_pts;
       for(size_t j = 0; j < pts.size(); j++)
       {
         if(pts[j].x >= mid.x-roi_rect.width/2. && pts[j].x < mid.x+roi_rect.width/2. &&
            pts[j].y >= mid.y-roi_rect.height/2. && pts[j].y < mid.y+roi_rect.height/2.)
         {
           good_pts.push_back(pts[j]);
         }
       }
       if(good_pts.size() > best_pts.size())
         best_pts = good_pts;
     }
     return best_pts;
  }

  cv::Rect2d roiFromPoints(const std::vector<Point2f>& img_pts)
  {
    cv::Rect2d roi_rect;
    float left_x, right_x, top_y, bottom_y;
    left_x = right_x = img_pts[0].x;
    top_y = bottom_y = img_pts[0].y;
    for(size_t i = 1; i < img_pts.size(); i++)
    {
      right_x = std::max(img_pts[i].x, right_x);
      left_x = std::min(img_pts[i].x, left_x);
      top_y = std::min(img_pts[i].y, top_y);
      bottom_y = std::max(img_pts[i].y, bottom_y);
    }
    roi_rect.x = int(left_x);
    roi_rect.width = int(right_x - left_x + 1);
    roi_rect.y = int(top_y);
    roi_rect.height = int(bottom_y - top_y + 1);
    return roi_rect;
  }
  

  void roiCallback(const sensor_msgs::RegionOfInterestConstPtr& msg)
  {
    roi_rect_.x = msg->x_offset;
    roi_rect_.y = msg->y_offset;
    roi_rect_.width = msg->width;
    roi_rect_.height = msg->height;
    if(roi_image_received_ && use_features_)
    {
      detectROIFeatures();
    }
    nh_.param<double>("/tracking/yaw_offset_tracking", yaw_offset_, 0);
    ROS_INFO("Yaw Offset: %f",yaw_offset_);
    roi_received_ = true;
    ROS_INFO("Roi received");
  }

  void roiImageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      roi_image_copy_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      if(roi_received_ && use_features_)
      {
        detectROIFeatures();
      }
      roi_image_received_ = true;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    if(!cam_info_received_)
    {
      fx_ = msg->K[0];
      fy_ = msg->K[4];
      cx_ = msg->K[2];
      cy_ = msg->K[5];
      cam_info_received_ = true;
      ROS_INFO("Camera info received by tracker");
    }        
    else
      camera_info_subscriber_.shutdown();
 
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    int number_of_image_subscribers = image_pub_.getNumSubscribers();
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
    if(roi_received_ && roi_image_received_)
    {
      roi_received_ = false;
      roi_image_received_ = false;//Consumed
      if(roi_rect_.height > 0 && roi_rect_.width > 0)
      {
        ROS_INFO("Initializing tracker");
        if(use_features_) 
        {
          if(roi_kps_.size() == 0)
          {
            ROS_WARN("ROI has no keypoints");
            return;
          }

          std::vector<KeyPoint> img_kps;
          Mat img_descs;
          detector_->detectAndCompute(cv_ptr_image_copy_->image,
            Mat(cv_ptr_image_copy_->image.rows, cv_ptr_image_copy_->image.cols, CV_8UC1, Scalar(255)),
            img_kps, img_descs);

          if(img_kps.size() == 0)
          {
            ROS_WARN("Image has no keypoints");
            return;
          }

          BFMatcher matcher(NORM_HAMMING);
          std::vector<std::vector<DMatch> > matches;
          std::vector<DMatch> filtered_matches;
          matcher.knnMatch(roi_descs_, img_descs, matches, 2);

          if(matches.size() == 0)
          {
            ROS_WARN("No matches");
            return;
          }

          filtered_matches = filterMatchesRatioTest(matches, 0.8);

          if(filtered_matches.size() == 0)
          {
            ROS_WARN("No features after ratio test");
            return;
          }

          std::vector<Point2f> roi_pts, img_pts;
          std::vector<KeyPoint> roi_kps_filt, img_kps_filt;
          for(size_t i = 0; i < filtered_matches.size(); ++i)
          { 
            roi_pts.push_back(roi_kps_[filtered_matches[i].queryIdx].pt);
            roi_kps_filt.push_back(roi_kps_[filtered_matches[i].queryIdx]);
            img_pts.push_back(img_kps[filtered_matches[i].trainIdx].pt);
            img_kps_filt.push_back(img_kps[filtered_matches[i].trainIdx]);
          }
          //Mat image_matches;
          //drawMatches(roi_image_copy_->image, roi_kps_, cv_ptr_image_copy_->image, img_kps,
          //  filtered_matches, image_matches);
          //imshow("matches", image_matches);
          //waitKey(1);

          filterMatchesEpipolarConstraint(roi_pts, img_pts);
          if(img_pts.size() == 0)
          {
            ROS_WARN("No features after epipolar constraint");
            return;
          }

          std::vector<Point2f> roi_filt_img_pts = filterPointsBoundingBox(img_pts, roi_rect_, 100);
          roi_rect_ = roiFromPoints(roi_filt_img_pts);

          if (roi_rect_.width <= 0 || roi_rect_.height <=0)
          {
            ROS_WARN("ROI from features has dimensions less than 0");
            return;
          }
          if(!tracker->init(cv_ptr_image_copy_->image, roi_rect_))
          {
            ROS_ERROR("***Could not initialize tracker...***\n");
            tracker_initialized_ = false;
          }
          else
          {
            tracker_initialized_ = true;
          }
        }
        //Initialize tracker:
        else if( !tracker->init(roi_image_copy_->image, roi_rect_) )
        {
          ROS_ERROR("***Could not initialize tracker...***\n");
          tracker_initialized_ = false;
        }
        else
        {
          //Tracker Initialized
          if(tracker->update( cv_ptr_image_copy_->image, roi_rect_) && number_of_image_subscribers > 0)
          {
            rectangle(cv_ptr_image_copy_->image,roi_rect_, Scalar(255,0,0),3,8,0);
          }
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
          if(number_of_image_subscribers > 0)
          {
            Point point1 = Point(roi_rect_.x, roi_rect_.y);
            Point point2 = Point(roi_rect_.x + roi_rect_.width, roi_rect_.y+roi_rect_.height);
            rectangle(cv_ptr_image_copy_->image, point1, point2, Scalar(255,0,0),3,8,0);
            if(cam_info_received_)
            {
              double px = -fx_*tan(yaw_offset_) + cx_;
              circle(cv_ptr_image_copy_->image, Point(px, cy_), 5, Scalar(0,0,255), 5);
            }
          }
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
    if(number_of_image_subscribers >0)
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
