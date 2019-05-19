#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Mat img, maskHSV, hsv;
cv::Mat r_maskHSV, b_maskHSV, y_maskHSV;
cv::Mat r_edges, y_edges, b_edges;
cv::Mat y_blurred, y_opened, y_morphed;
cv::Mat r_blurred, r_opened, r_morphed;
cv::Mat b_blurred, b_opened, b_morphed;
double r_area, y_area, b_area;

int y_h_min, y_h_max, y_s_min , y_s_max , y_v_min, y_v_max ;
int r_h_min, r_h_max, r_s_min , r_s_max , r_v_min, r_v_max ;
int b_h_min, b_h_max, b_s_min , b_s_max , b_v_min, b_v_max ;
int gauss_ksize=5, bil_ksize=9, open_ksize=3, close_ksize=9, sigma=75;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher obj_pub;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("colour_image", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("threshold_image", 1);
  }

  ~ImageConverter(){}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    nh_.getParam("threshold/yellow/h_max", y_h_max);
    nh_.getParam("threshold/yellow/h_min", y_h_min);
    nh_.getParam("threshold/yellow/s_max", y_s_max);
    nh_.getParam("threshold/yellow/s_min", y_s_min);
    nh_.getParam("threshold/yellow/v_max", y_v_max);
    nh_.getParam("threshold/yellow/v_min", y_v_min);

    nh_.getParam("threshold/red/h_max", r_h_max);
    nh_.getParam("threshold/red/h_min", r_h_min);
    nh_.getParam("threshold/red/s_max", r_s_max);
    nh_.getParam("threshold/red/s_min", r_s_min);
    nh_.getParam("threshold/red/v_max", r_v_max);
    nh_.getParam("threshold/red/v_min", r_v_min);

    nh_.getParam("threshold/blue/h_max", b_h_max);
    nh_.getParam("threshold/blue/h_min", b_h_min);
    nh_.getParam("threshold/blue/s_max", b_s_max);
    nh_.getParam("threshold/blue/s_min", b_s_min);
    nh_.getParam("threshold/blue/v_max", b_v_max);
    nh_.getParam("threshold/blue/v_min", b_v_min);

    nh_.getParam("threshold/gaussian/ksize", gauss_ksize);
    nh_.getParam("threshold/bilateral/ksize", bil_ksize);
    nh_.getParam("threshold/bilateral/sigma", sigma);
    nh_.getParam("threshold/morph_open/ksize", open_ksize);
    nh_.getParam("threshold/morph_close/ksize", close_ksize);

    r_area = 0; y_area = 0; b_area = 0;

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

    img=cv_ptr->image;
    cv::cvtColor(img,hsv,CV_BGR2HSV);
    
    cv::GaussianBlur(hsv, hsv, cv::Size(5,5), 0, 0);

    cv::inRange(hsv,cv::Scalar(r_h_min,r_s_min,r_v_min),cv::Scalar(r_h_max,r_s_max,r_v_max),r_maskHSV);
    cv::inRange(hsv,cv::Scalar(b_h_min,b_s_min,b_v_min),cv::Scalar(b_h_max,b_s_max,b_v_max),b_maskHSV);
    cv::inRange(hsv,cv::Scalar(y_h_min,y_s_min,y_v_min),cv::Scalar(y_h_max,y_s_max,y_v_max),y_maskHSV);

    cv::bilateralFilter(y_maskHSV, y_blurred,bil_ksize,sigma,sigma);
    cv::bilateralFilter(r_maskHSV, r_blurred,bil_ksize,sigma,sigma);
    cv::bilateralFilter(b_maskHSV, b_blurred,bil_ksize,sigma,sigma);

    cv::morphologyEx(y_blurred, y_opened, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_ksize,open_ksize), cv::Point(1,1)));
    cv::morphologyEx(r_blurred, r_opened, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_ksize,open_ksize), cv::Point(1,1)));
    cv::morphologyEx(b_blurred, b_opened, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_ksize,open_ksize), cv::Point(1,1)));
    
    cv::morphologyEx(y_opened, y_morphed, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_ksize,close_ksize), cv::Point(1,1)));
    cv::morphologyEx(r_opened, r_morphed, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_ksize,close_ksize), cv::Point(1,1)));
    cv::morphologyEx(b_opened, b_morphed, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_ksize,close_ksize), cv::Point(1,1)));

    for(int i=0; i<y_morphed.rows; i++)
    {
      for(int j=0; j<y_morphed.cols;j++)
      {
        if(y_morphed.at<uchar>(i,j) != 0)
        {
          y_area++;
        }
      }
    }

    for(int i=0; i<b_morphed.rows; i++)
    {
      for(int j=0; j<b_morphed.cols;j++)
      {
        if(b_morphed.at<uchar>(i,j) != 0)
        {
          b_area++;
        }      
      }
    }

    for(int i=0; i<r_morphed.rows; i++)
    {
      for(int j=0; j<r_morphed.cols;j++)
      {
        if(r_morphed.at<uchar>(i,j) != 0)
        {
          r_area++;
        }
      }
    }

    if(r_area>b_area)
    {
      if(r_area>y_area)
      {
        maskHSV = r_morphed;
      }
      else
      {
        maskHSV = y_morphed;
      }
    }
    else
    {
      if(b_area>y_area)
      {
        maskHSV = b_morphed;
      }
      else
      {
        maskHSV = y_morphed;
      }
    }

    cv::cvtColor(maskHSV,img,cv::COLOR_GRAY2BGR);
    cv_bridge::CvImage in_msg;
    in_msg.header.stamp = ros::Time::now();
    in_msg.encoding = sensor_msgs::image_encodings::BGR8;
    in_msg.image = img;
    image_pub_.publish(in_msg.toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "threshold");
  ImageConverter ic;
  ros::spin();
  return 0;
}