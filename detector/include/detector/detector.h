#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <detector_msgs/BBoxes.h>
#include <detector_msgs/BBox.h>
#include <detector_msgs/BBPose.h>
#include <detector_msgs/BBPoses.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

#define MAX_BOXES 10000
#define MAX_CONTOUR_POINTS 2500

#define min(a,b) (a>b)?b:a
#define sign(x) (x>0)?1:0 
#define check(X) std::cout<<"check "<<X<<std::endl

cv::Vec3b BLACK = (0,0,0);
cv::Vec3b RED = (255,0,0);
cv::Vec3b BLUE = (0,0,255);
cv::Vec3b GREEN = (0,255,0);

int redType = -10, yellowType = -20, blueType = -30;
int borderType = (int)10E4;
int contourType = (int)10E6;

bool eigenCheckFlag = true;
bool diagCheckFlag = true;
bool areaCheckFlag = true;
bool sizeCheckFlag = true;

bool debug = true;
bool verbose = false;
bool isRectified = false;

int minSize = 4000;
float maxAreaIndex = 1;
float maxDiagIndex = 100;
float maxEigenIndex = 1.07;
float minSizeHeight = 3.000;

bool eigenPassed = false;
bool diagPassed = false;
bool areaPassed = false;
bool sizePassed = false;
bool passed = false;

int YHMax = 20 , YHMin = 40 , YSMax = 255, YSMin = 100, YVMax = 255, YVMin = 100;
int RHMax = 0  , RHMin = 20 , RSMax = 255, RSMin = 100, RVMax = 255, RVMin = 100;
int BHMax = 120, BHMin = 100, BSMax = 255, BSMin = 100, BVMax = 255, BVMin = 100;

int imageID = 0;
nav_msgs::Odometry odom;
cv::Mat intrinsic = cv::Mat_<double>(3,3);
cv::Mat distCoeffs = cv::Mat_<double>(1,5);
cv::Mat img_, undistImg_, markedImg_;

Eigen::Matrix3f camMatrix, invCamMatrix, camToQuad, quadToCam;
Eigen::Vector3f tCam;

float redSize, yellowSize, blueSize, delSize;

struct bbox
{
    int rangeX[2], rangeY[2];
    float x_mean, y_mean;
    int warning, id, pixSize, contourSize, type;
    float areaIndex, diagIndex;
    float eigenIndex, eigvl0, eigvl1, v0, v1;
    int contourX[MAX_CONTOUR_POINTS];
    int contourY[MAX_CONTOUR_POINTS];
    int cornerX[4], cornerY[4];
};

// clean
int sgnArea(double x1, double x2, double x3, double y1, double y2, double y3)
{
    return sign(x1*y2 + x2*y3 + x3*y1 - y1*x2 - y2*x3 - y3*x1);
}

void loadParams(ros::NodeHandle nh)
{
    std::vector<double> tempList;
    int tempIdx=0;

    nh.getParam("detector/distortion_coefficients/data", tempList);
    for(int i=0; i<5; i++)
    {
        distCoeffs.at<double>(i) = tempList[i];
    }
   
    nh.getParam("detector/camera_matrix/data", tempList);
    tempIdx=0;
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            intrinsic.at<double>(i,j) = tempList[tempIdx++];
        }
    }

    nh.getParam("detector/flags/diagCheck", diagCheckFlag);
    nh.getParam("detector/flags/eigenCheck", eigenCheckFlag);
    nh.getParam("detector/flags/areaCheck", areaCheckFlag);
    nh.getParam("detector/flags/sizeCheck", sizeCheckFlag);

    nh.getParam("detector/flags/debug", debug);
    nh.getParam("detector/flags/verbose", verbose);

    nh.getParam("detector/box/minSize", minSize);
    nh.getParam("detector/box/maxAreaIndex", maxAreaIndex);
    nh.getParam("detector/box/maxEigenIndex", maxEigenIndex);
    nh.getParam("detector/box/maxDiagIndex", maxDiagIndex);

    nh.getParam("detector/camera/translation", tempList);
    for (int i = 0; i < 3; i++)
    {
        tCam(i) = tempList[i];
    }

    nh.getParam("detector/camera/rotation", tempList);
    tempIdx = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            quadToCam(i,j) = tempList[tempIdx++];
        }
    }

    nh.getParam("detector/camera/is_rectified", isRectified);

    nh.getParam("detector/yellow/h_max", YHMax);
    nh.getParam("detector/yellow/h_min", YHMin);
    nh.getParam("detector/yellow/s_max", YSMax);
    nh.getParam("detector/yellow/s_min", YSMin);
    nh.getParam("detector/yellow/v_max", YVMax);
    nh.getParam("detector/yellow/v_min", YVMin);

    nh.getParam("detector/red/h_max", RHMax);
    nh.getParam("detector/red/h_min", RHMin);
    nh.getParam("detector/red/s_max", RSMax);
    nh.getParam("detector/red/s_min", RSMin);
    nh.getParam("detector/red/v_max", RVMax);
    nh.getParam("detector/red/v_min", RVMin);

    nh.getParam("detector/blue/h_max", BHMax);
    nh.getParam("detector/blue/h_min", BHMin);
    nh.getParam("detector/blue/s_max", BSMax);
    nh.getParam("detector/blue/s_min", BSMin);
    nh.getParam("detector/blue/v_max", BVMax);
    nh.getParam("detector/blue/v_min", BVMin);

    nh.getParam("detector/sizes/red", redSize);
    nh.getParam("detector/sizes/blue", blueSize);
    nh.getParam("detector/sizes/yellow", yellowSize);
    nh.getParam("detector/sizes/tolerance", delSize);
    nh.getParam("detector/sizes/minHeight", minSizeHeight);

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            camMatrix(i,j) = intrinsic.at<double>(i,j);
        }
    }

    invCamMatrix = camMatrix.inverse();
    camToQuad = quadToCam.inverse();

    std::cout << "Parameters Loaded." << std::endl;

    return;
}

void imageCallback(const sensor_msgs::Image msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  img_ = cv_ptr->image;
  imageID = msg.header.seq;
  markedImg_ = cv::Mat(msg.height, msg.width, CV_8UC3);
  return;
}

void odomCallback(const nav_msgs::Odometry msg)
{
    odom = msg;
    return;
}

detector_msgs::BBPoses findPoses(std::vector<struct bbox> *ptr)
{
    detector_msgs::BBPoses msg;
    struct bbox *box;

    Eigen::Matrix3f scaleUp, quadToGlob;

    tf::Quaternion q1(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    Eigen::Quaternionf quat = Eigen::Quaternionf(q1.w(), q1.x(), q1.y(), q1.z());
    quadToGlob = quat.toRotationMatrix();

    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            if(i==j) scaleUp(i,j) = odom.pose.pose.position.z;
            else scaleUp(i,j) = 0;
        }
    }

    for(int i=0; i<ptr->size(); i++)
    {
        box = &(ptr->at(i));
        float area = 0;

        if(sizeCheckFlag)
        {
            if(odom.pose.pose.position.z < minSizeHeight)
            {
                area = -1;
                sizePassed = true;
            }
            else
            {
                Eigen::MatrixXf globCorners(4,2);
                for(int i=0; i<4; i++)
                {
                    Eigen::Vector3f imgVec(box->cornerX[i],box->cornerY[i],1);
                    Eigen::Vector3f quadCoord = (camToQuad*scaleUp*invCamMatrix*imgVec) + tCam;

                    Eigen::Vector3f globCoord = quadToGlob*quadCoord;
                    globCorners(i,0) = globCoord(0) + odom.pose.pose.position.x;
                    globCorners(i,1) = globCoord(1) + odom.pose.pose.position.y;
                }

                for(int i=0; i<4; i++)
                {
                    area += globCorners(i,0)*globCorners((i+1)%4,1) - globCorners(i,1)*globCorners((i+1)%4,0);
                }
                area = (float)fabs(area/2);

                if (box->type == redType) sizePassed = (fabs(area-redSize)<=delSize);
                else if (box->type == yellowType) sizePassed = (fabs(area-yellowSize)<=delSize);
                else if (box->type == blueType) sizePassed = (fabs(area-blueSize)<=delSize);
            }
        }
        
        if(!sizeCheckFlag || sizePassed)
        { 
            detector_msgs::BBPose temp; 
            temp.boxID = box->id;

            if(sizeCheckFlag)
            {
                temp.area = area;

                if(box->type == redType) temp.colour = 'red';
                else if(box->type == yellowType) temp.colour = 'yellow';
                else if(box->type == blueType) temp.colour = 'blue';
            }

            Eigen::Vector3f imgVec(box->x_mean,box->y_mean,1);
            Eigen::Vector3f quadCoord = (camToQuad*scaleUp*invCamMatrix*imgVec) + tCam;

            Eigen::Vector3f globCoord = quadToGlob*quadCoord;
            temp.position.x = globCoord(0) + odom.pose.pose.position.x;
            temp.position.y = globCoord(1) + odom.pose.pose.position.y;
            temp.position.z = globCoord(2) + odom.pose.pose.position.z;

            msg.object_poses.push_back(temp);
        }
    }

    return msg;
}
