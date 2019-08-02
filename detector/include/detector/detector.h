#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <opencv2/highgui/highgui.hpp>

#include <mav_utils_msgs/BBoxes.h>
#include <mav_utils_msgs/BBox.h>
#include <mav_utils_msgs/BBPose.h>
#include <mav_utils_msgs/BBPoses.h>
#include <mav_utils_msgs/signal.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic_reconfigure/server.h>
#include <detector/reconfigConfig.h>

#define MAX_BOXES 10000
#define MAX_CONTOUR_POINTS 2500

#define min(a,b) (a>b)?b:a
#define sign(x) (x>0)?1:0 
#define sq(x) (x)*(x)
#define check(X) std::cout<<"check "<<X<<std::endl

#define exit execFlag == -1
#define run execFlag == 1

int execFlag = 0;
int color_num = 0;

cv::Vec3b BLACK = (0,0,0);
cv::Vec3b RED = (255,0,0);
cv::Vec3b BLUE = (0,0,255);
cv::Vec3b GREEN = (0,255,0);

int redType = -10, yellowType = -20, blueType = -30, orangeType = -40;
int borderType = (int)10E4;
int contourType = (int)10E6;

bool eigenCheckFlag = true;
bool diagCheckFlag = true;
bool areaCheckFlag = true;
bool sizeCheckFlag = true;
bool centreCorrect = true;

bool debug = true;
bool verbose = false;
bool isRectified = false;

int minSize = 4000;
float maxAreaIndex = 1;
float maxDiagIndex = 100;
float maxEigenIndex = 1.07;
float centreCorrectIndex = 4;
float minSizeHeight = 3.000;
float maxCentreDist = 0.5;

bool eigenPassed = false;
bool diagPassed = false;
bool areaPassed = false;
bool sizePassed = false;
bool passed = false;

int YHMin = 110, YHMax = 130, YSMin = 100, YSMax = 255, YVMin = 100, YVMax = 255;
int RHMin = 90 , RHMax = 110, RSMin = 100, RSMax = 255, RVMin = 100, RVMax = 255;
int BHMin = 10 , BHMax = 30 , BSMin = 100, BSMax = 255, BVMin = 100, BVMax = 255;
int OHMin = 105, OHMax = 125, OSMin = 100, OSMax = 255, OVMin = 100, OVMax = 255;

int imageID = 0;
nav_msgs::Odometry odom;
cv::Mat intrinsic = cv::Mat_<double>(3,3);
cv::Mat distCoeffs = cv::Mat_<double>(1,5);
cv::Mat img_, undistImg_, markedImg_;

Eigen::Matrix3f camMatrix, invCamMatrix, camToQuad, quadToCam;
Eigen::Vector3f tCam;

float redSize, yellowSize, blueSize, orangeSize, delSize;

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
    bool store;
};

// clean
int sgnArea(double x1, double x2, double x3, double y1, double y2, double y3)
{
    return sign(x1*y2 + x2*y3 + x3*y1 - y1*x2 - y2*x3 - y3*x1);
}

void cfgCallback(detector::reconfigConfig &config, uint32_t level){
    if(run){
        switch(level){
            case 0: isRectified = config.is_rectified;
                    ROS_INFO("Set isRectified to %d", isRectified); break;

            case 1: tCam(0) = config.groups.camera_translation.t_x;
                    ROS_INFO("Set t_x to %f", tCam(0)); break;
            case 2: tCam(1) = config.groups.camera_translation.t_y;
                    ROS_INFO("Set t_y to %f", tCam(1)); break;
            case 3: tCam(2) = config.groups.camera_translation.t_z;
                    ROS_INFO("Set t_z to %f", tCam(2)); break;

            case 4: quadToCam(0,0) = config.groups.camera_rotation.r_xx;
                    ROS_INFO("Set r_xx to %f", quadToCam(0,0)); break;
            case 5: quadToCam(0,1) = config.groups.camera_rotation.r_xy;
                    ROS_INFO("Set r_xy to %f", quadToCam(0,1)); break;
            case 6: quadToCam(0,2) = config.groups.camera_rotation.r_xz;
                    ROS_INFO("Set r_xz to %f", quadToCam(0,2)); break;
            case 7: quadToCam(1,0) = config.groups.camera_rotation.r_yx;
                    ROS_INFO("Set r_yx to %f", quadToCam(1,0)); break;
            case 8: quadToCam(1,1) = config.groups.camera_rotation.r_yy;
                    ROS_INFO("Set r_yy to %f", quadToCam(1,1)); break;
            case 9: quadToCam(1,2) = config.groups.camera_rotation.r_yz;
                    ROS_INFO("Set r_yz to %f", quadToCam(1,2)); break;
            case 10: quadToCam(2,0) = config.groups.camera_rotation.r_zx;
                    ROS_INFO("Set r_zx to %f", quadToCam(2,0)); break;
            case 11: quadToCam(2,1) = config.groups.camera_rotation.r_zy;
                    ROS_INFO("Set r_zy to %f", quadToCam(2,1)); break;
            case 12: quadToCam(2,2) = config.groups.camera_rotation.r_zz;
                    ROS_INFO("Set r_zz to %f", quadToCam(2,2)); break;
        
            case 13: minSize = config.groups.box.minSize;
                     ROS_INFO("Set minSize to %d", minSize); break;
            case 14: maxAreaIndex = config.groups.box.maxAreaIndex;
                     ROS_INFO("Set maxAreaIndex to %f", maxAreaIndex); break;
            case 15: maxEigenIndex = config.groups.box.maxEigenIndex;
                     ROS_INFO("Set maxEigenIndex to %f", maxEigenIndex); break;
            case 16: maxDiagIndex = config.groups.box.maxDiagIndex;
                     ROS_INFO("Set maxDiagIndex to %f", maxDiagIndex); break;
            case 17: centreCorrectIndex = config.groups.box.centreCorrectIndex;
                     ROS_INFO("Set centreCorrectIndex to %f", centreCorrectIndex); break;
            case 37: maxCentreDist = config.groups.box.maxCentreDist;
                     ROS_INFO("Set maxCentreDist to %f", maxCentreDist); break;

            case 18: debug = config.groups.flags.debug;
                     ROS_INFO("Set debug to %d", debug); break;
            case 19: verbose = config.groups.flags.verbose;
                     ROS_INFO("Set verbose to %d", verbose); break;
            case 20: areaCheckFlag = config.groups.flags.areaCheck;
                     ROS_INFO("Set areaCheckFlag to %d", areaCheckFlag); break;
            case 21: eigenCheckFlag = config.groups.flags.eigenCheck;
                     ROS_INFO("Set eigenCheckFlag to %d", eigenCheckFlag); break;
            case 22: diagCheckFlag = config.groups.flags.diagCheck;
                     ROS_INFO("Set diagCheckFlag to %d", diagCheckFlag); break;
            case 23: sizeCheckFlag = config.groups.flags.sizeCheck;
                     ROS_INFO("Set sizeCheckFlag to %d", sizeCheckFlag); break;
            case 24: centreCorrect = config.groups.flags.centreCorrect;
                     ROS_INFO("Set centreCorrect to %d", centreCorrect); break;

            case 25: color_num = config.groups.hsv.color; break;
            
            case 26: switch(color_num){
                        case 0: RHMin = (config.groups.hsv.h_min + 90) % 180;
                                ROS_INFO("Set RHMin to %d", RHMin); break;
                        case 1: BHMin = (config.groups.hsv.h_min + 90) % 180;
                                ROS_INFO("Set BHMin to %d", BHMin); break;
                        case 2: YHMin = (config.groups.hsv.h_min + 90) % 180;
                                ROS_INFO("Set YHMin to %d", YHMin); break;
                        case 3: OHMin = (config.groups.hsv.h_min + 90) % 180;
                                ROS_INFO("Set OHMin to %d", OHMin); break;
                     } break;
            
            case 27: switch(color_num){
                        case 0: RHMax = (config.groups.hsv.h_max + 90) % 180;
                                ROS_INFO("Set RHMax to %d", RHMax); break;
                        case 1: BHMax = (config.groups.hsv.h_max + 90) % 180;
                                ROS_INFO("Set BHMax to %d", BHMax); break;
                        case 2: YHMax = (config.groups.hsv.h_max + 90) % 180;
                                ROS_INFO("Set YHMax to %d", YHMax); break;
                        case 3: OHMax = (config.groups.hsv.h_max + 90) % 180;
                                ROS_INFO("Set OHMax to %d", OHMax); break;        
                     } break;
            
            case 28: switch(color_num){
                        case 0: RSMin = config.groups.hsv.s_min;
                                ROS_INFO("Set RSMin to %d", RSMin); break;
                        case 1: BSMin = config.groups.hsv.s_min;
                                ROS_INFO("Set BSMin to %d", BSMin); break;
                        case 2: YSMin = config.groups.hsv.s_min;
                                ROS_INFO("Set YSMin to %d", YSMin); break;
                        case 3: OSMin = config.groups.hsv.s_min;
                                ROS_INFO("Set OSMin to %d", OSMin); break;
                     } break;
            
            case 29: switch(color_num){
                        case 0: RSMax = config.groups.hsv.s_max;
                                ROS_INFO("Set RSMax to %d", RSMax); break;
                        case 1: BSMax = config.groups.hsv.s_max;
                                ROS_INFO("Set BSMax to %d", BSMax); break;
                        case 2: YSMax = config.groups.hsv.s_max;
                                ROS_INFO("Set YSMax to %d", YSMax); break;
                        case 3: OSMax = config.groups.hsv.s_max;
                                ROS_INFO("Set OSMax to %d", OSMax); break;
                     } break;
            
            case 30: switch(color_num){
                        case 0: RVMin = config.groups.hsv.v_min;
                                ROS_INFO("Set RVMin to %d", RVMin); break;
                        case 1: BVMin = config.groups.hsv.v_min;
                                ROS_INFO("Set BVMin to %d", BVMin); break;
                        case 2: YVMin = config.groups.hsv.v_min;
                                ROS_INFO("Set YVMin to %d", YVMin); break;
                        case 3: OVMin = config.groups.hsv.v_min;
                                ROS_INFO("Set OVMin to %d", OVMin); break;
                     } break;
           
            case 31: switch(color_num){
                        case 0: RVMax = config.groups.hsv.v_max;
                                ROS_INFO("Set RVMax to %d", RVMax); break;
                        case 1: BVMax = config.groups.hsv.v_max;
                                ROS_INFO("Set BVMax to %d", BVMax); break;
                        case 2: YVMax = config.groups.hsv.v_max;
                                ROS_INFO("Set YVMax to %d", YVMax); break;
                        case 3: OVMax = config.groups.hsv.v_max;
                                ROS_INFO("Set OVMax to %d", OVMax); break;
                     } break;

            case 32: redSize = config.groups.sizes.r_size;
                     ROS_INFO("Set redSize to %f", redSize); break;
            case 33: blueSize = config.groups.sizes.b_size;
                     ROS_INFO("Set blueSize to %f", blueSize); break;
            case 34: yellowSize = config.groups.sizes.y_size;
                     ROS_INFO("Set yellowSize to %f", yellowSize); break;
            case 38: orangeSize = config.groups.sizes.o_size;
                     ROS_INFO("Set orangeSize to %f", orangeSize); break;
            case 35: delSize = config.groups.sizes.del_size;
                     ROS_INFO("Set delSize to %f", delSize); break;
            case 36: minSizeHeight = config.groups.sizes.min_height;
                     ROS_INFO("Set minSizeHeight to %f", minSizeHeight); break;
        }

        return;
    } 
}

void loadParams(ros::NodeHandle nh)
{
    std::vector<double> tempList;
    int tempIdx=0;

    nh.getParam("distortion_coefficients/data", tempList);
    for(int i=0; i<5; i++)
    {
        distCoeffs.at<double>(i) = tempList[i];
    }
   
    nh.getParam("camera_matrix/data", tempList);
    tempIdx=0;
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            intrinsic.at<double>(i,j) = tempList[tempIdx++];
        }
    }

    nh.getParam("flags/diagCheck", diagCheckFlag);
    nh.getParam("flags/eigenCheck", eigenCheckFlag);
    nh.getParam("flags/areaCheck", areaCheckFlag);
    nh.getParam("flags/sizeCheck", sizeCheckFlag);
    nh.getParam("flags/centreCorrect", centreCorrect);

    nh.getParam("flags/debug", debug);
    nh.getParam("flags/verbose", verbose);

    nh.getParam("box/minSize", minSize);
    nh.getParam("box/maxAreaIndex", maxAreaIndex);
    nh.getParam("box/maxEigenIndex", maxEigenIndex);
    nh.getParam("box/maxDiagIndex", maxDiagIndex);
    nh.getParam("box/centerCorrectIndex", centreCorrectIndex);
    nh.getParam("box/maxCentreDist", maxCentreDist);

    nh.getParam("camera/translation", tempList);
    for (int i = 0; i < 3; i++)
    {
        tCam(i) = tempList[i];
    }

    nh.getParam("camera/rotation", tempList);
    tempIdx = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            quadToCam(i,j) = tempList[tempIdx++];
        }
    }

    nh.getParam("camera/is_rectified", isRectified);

    nh.getParam("yellow/h_max", YHMax);
    nh.getParam("yellow/h_min", YHMin);
    nh.getParam("yellow/s_max", YSMax);
    nh.getParam("yellow/s_min", YSMin);
    nh.getParam("yellow/v_max", YVMax);
    nh.getParam("yellow/v_min", YVMin);

    nh.getParam("red/h_max", RHMax);
    nh.getParam("red/h_min", RHMin);
    nh.getParam("red/s_max", RSMax);
    nh.getParam("red/s_min", RSMin);
    nh.getParam("red/v_max", RVMax);
    nh.getParam("red/v_min", RVMin);

    nh.getParam("blue/h_max", BHMax);
    nh.getParam("blue/h_min", BHMin);
    nh.getParam("blue/s_max", BSMax);
    nh.getParam("blue/s_min", BSMin);
    nh.getParam("blue/v_max", BVMax);
    nh.getParam("blue/v_min", BVMin);

    nh.getParam("orange/h_max", OHMax);
    nh.getParam("orange/h_min", OHMin);
    nh.getParam("orange/s_max", OSMax);
    nh.getParam("orange/s_min", OSMin);
    nh.getParam("orange/v_max", OVMax);
    nh.getParam("orange/v_min", OVMin);

    nh.getParam("sizes/red", redSize);
    nh.getParam("sizes/blue", blueSize);
    nh.getParam("sizes/yellow", yellowSize);
    nh.getParam("sizes/orange", orangeSize);
    nh.getParam("sizes/tolerance", delSize);
    nh.getParam("sizes/minHeight", minSizeHeight);

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

bool serviceCall(mav_utils_msgs::signal::Request &req, mav_utils_msgs::signal::Response &res)
{
    switch(req.signal){
        case -1:
        case 0:
        case 1: execFlag = req.signal;
                switch(execFlag){
                    case 0: ROS_INFO("Paused"); break;
                    case 1: ROS_INFO("Starting"); break;
                    case -1: ROS_INFO("Exiting"); break;
                }
                res.success = true;
                break;
        default: res.success = false;
                break;
    }
    return true;
}

void imageCallback(const sensor_msgs::Image msg)
{
  if(run)
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
  }
  return;
}

void odomCallback(const nav_msgs::Odometry msg)
{
    if(run) odom = msg;
    return;
}

mav_utils_msgs::BBPoses findPoses(std::vector<struct bbox> *ptr)
{
    mav_utils_msgs::BBPoses msg;
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
                else if (box->type == orangeType) sizePassed = (fabs(area-orangeSize)<=delSize);
            }
        }
        
        if(!sizeCheckFlag || sizePassed)
        { 
            mav_utils_msgs::BBPose temp; 
            temp.boxID = box->id;
            temp.store = box->store;
            temp.type = box->type;

            if(sizeCheckFlag) temp.area = area;

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
