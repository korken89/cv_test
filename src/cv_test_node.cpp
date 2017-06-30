#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("cam0/throttled", 1,
      &ImageConverter::imageCb, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray, out;
    std::vector<cv::Point2f> corners;

    cv::cvtColor( cv_ptr->image, gray, CV_RGB2GRAY );

    cv::goodFeaturesToTrack(gray, corners, 200, 0.01, 1);


    out = cv_ptr->image.clone();
    out.setTo(0);

    int prad = 15;
    for (const auto corner : corners)
    {
      if (corner.x < out.cols - prad && corner.x > prad &&
          corner.y < out.rows - prad && corner.y > prad)
      {
        //out(cv::Rect(corner.x, corner.y, 2*prad+1, 2*prad+1)) =
        //    cv_ptr->image(cv::Rect(corner.x, corner.y, 2*prad+1, 2*prad+1));
        cv_ptr->image(cv::Rect(corner.x - prad - 1, corner.y - prad - 1, 2*prad+1, 2*prad+1)).copyTo(out(cv::Rect(corner.x - prad - 1, corner.y - prad - 1, 2*prad+1, 2*prad+1)));
      }
    }

    int r = 4;
    for (const auto corner : corners)
    {
      if (corner.x < out.cols - prad && corner.x > prad &&
          corner.y < out.rows - prad && corner.y > prad)
        cv::circle( out,
                    corner,
                    r,
                    CV_RGB(255, 0, 0) );
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, out);//cv_ptr->image);
    cv::waitKey(3);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ROS_INFO("Up and running...");
  ros::spin();
  return 0;
}
