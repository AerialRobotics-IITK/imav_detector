#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <detector/BBoxes.h>
#include <detector/BBox.h>
#define MAX_BOXES 10000
#define MAX_CONTOUR_POINTS 2500
#define min(a,b) (a>b)?b:a

cv::Vec3b BLACK = (0,0,0);
cv::Vec3b RED = (255,0,0);
cv::Vec3b BLUE = (0,0,255);
cv::Vec3b GREEN = (0,255,0);

int whiteType = -10;
int borderType = (int)10E4;
int contourType = (int)10E6;

bool strictCheckFlag = false;
bool rqt = true;
bool debug = true;

int imageID = 0;
cv::Mat img_, undistImg_, markedImg_;

struct bbox
{
    int x_min, x_max, y_min, y_max;
    float x_mean, y_mean, v0, v1, eigvl0, eigvl1;
    int warning, id, pixSize, contourSize;
    float areaIndex, eigenIndex;
    int contourX[MAX_CONTOUR_POINTS];
    int contourY[MAX_CONTOUR_POINTS];
    int cornerX[4];
    int cornerY[4];
};


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

std::vector<struct bbox> floodFill(cv::Mat *input, ros::NodeHandle nh, cv::Mat *output)
{
    std::vector<struct bbox> bboxes;
    *output = *input;

    int minSize = 4000;
    double minAreaIndex = 0.4;
    nh.getParam("/detect_node/box/minSize", minSize);
    nh.getParam("/detect_node/box/minAreaIndex", minAreaIndex);

    nh.getParam("/detect_node/flags/strictCheck", strictCheckFlag);
    nh.getParam("/detect_node/flags/debug", debug);
    nh.getParam("/detect_node/flags/rqt", rqt);

    int width = input->cols;
    int height = input->rows;
    int res = width*height;

    int expand[4] = {width, -width, 1, -1};
    
    int stack_index = 0;
    int contour_index = 0;
    int numBoxes = 0;
    int pixType = 0;

    int *stack = (int *)calloc(res, sizeof(int));
    int *contour = (int *)calloc(res, sizeof(int));

    int* buffer = (int*)calloc(res, sizeof(int));
    for(int i=0; i<res; i++)
    {
        if(input->at<cv::Vec3b>(cv::Point(i%width, i/width))!=BLACK)
        {
            buffer[i]=whiteType;
        }
    }

    int topPix = 0;
    int botPix = height-1;
    int leftPix = 0;
    int rightPix = width-1;

    for(int i=leftPix; i<rightPix; i++)
    {
        buffer[topPix*width + i] = borderType;
        buffer[botPix*width + i] = borderType;
    }

    for(int i=topPix; i<botPix; i++)
    {
        buffer[width*i+leftPix] = borderType;
        buffer[width*i+rightPix] = borderType;
    }

    int startPos = 0;
    int pixPos = 0;
    int queueStart = 0;
    int queueEnd = 0;
    int nPix = 0;
    int dx, dy, maxX, minX, minY, maxY;

    for(int i=0; i<res; i++)
    {
        if(buffer[i]<0 && numBoxes < MAX_BOXES)
        {
            queueEnd = queueStart = 0;
            contour_index = 0;

            struct bbox Box;
            Box.id = numBoxes+1;
            Box.warning = 0;
            Box.pixSize = 1;            
            
            maxX = minX = Box.x_max = Box.x_min = i%width;
            maxY = minY = Box.y_max = Box.y_min = i/width;

            pixType = buffer[i];
            buffer[i] = ++numBoxes;
            stack[queueEnd++] = i;

            while(queueEnd > queueStart)
            {
                startPos = stack[queueStart++];
                nPix=0;
                for(int i=0; i<4; i++)
                {
                    pixPos = startPos + expand[i];

                    if(buffer[pixPos]!=0)
                    {
                        nPix++;
                    }

                    if(buffer[pixPos]==whiteType)
                    {
                        stack[queueEnd++] = pixPos;
                        buffer[pixPos] = numBoxes;
                        dx = pixPos%width;
                        dy = pixPos/width;

                        if(dx > maxX) maxX = dx;
                        if(dx < minX) minX = dx;
                        if(dy > maxY) maxY = dy;
                        if(dy < minY) minY = dy;
                    }

                    if(buffer[pixPos]==borderType) Box.warning = 1;
                }

                if (nPix!=4) contour[contour_index++] = pixPos;
            }

            if(queueEnd > minSize)
            {
                Box.areaIndex = fabs((float)(maxX-minX+1)*(maxY-minY+1)/queueEnd - 1);
                if(Box.areaIndex > minAreaIndex) queueEnd = 0; 
            }
            else
            {
                queueEnd = 0;
            }

            if(queueEnd>0)
            {
                for(int j=0; j<contour_index; j++)
                {
                    pixPos = contour[j];
                    buffer[pixPos] = contourType + numBoxes;
                    // does not show colors other than blue
                    if(rqt) output->at<cv::Vec3b>(cv::Point(pixPos%width, pixPos/width)) = BLUE;
                }
                
                long long int sx=0, sy=0;
                long long int cxx=0,cxy=0,cyy=0; 

                for(int s=0; s<queueEnd; s++)
                {
                    pixPos = stack[s];
                    sx += pixPos%width;
                    sy += pixPos/width;

                    if(strictCheckFlag)
                    {
                        cxx += (pixPos%width)*(pixPos%width);
                        cxy += (pixPos%width)*(pixPos/width);
                        cyy += (pixPos/width)*(pixPos/width);
                    }
                }

                float fsx = (float)sx/queueEnd;
                float fsy = (float)sy/queueEnd;

                if(strictCheckFlag)
                {
                    float fcxx = ((float)cxx/queueEnd-fsx*fsx);
                    float fcxy = ((float)cxy/queueEnd-fsx*fsy);
                    float fcyy = ((float)cyy/queueEnd-fsy*fsy);

                    float det = (fcxx+fcyy)*(fcxx+fcyy)-4*(fcxx*fcyy-fcxy*fcxy);
                    if(det>0) det = sqrt(det); else det = 0;

                    float eigvl0 = ((fcxx+fcyy)+det)/2;
                    float eigvl1 = ((fcxx+fcyy)-det)/2;
                    float eivec = (fcxy*fcxy+(fcxx-eigvl0)*(fcxx-eigvl0));

                    if(fcyy!=0 && eivec > 0)
                    {
                        Box.v0 = -fcxy/sqrt(eivec);
                        Box.v1 = (fcxx-eigvl0)/sqrt(eivec);
                    }
                    else
                    {
                        Box.v0 = Box.v1 = 0;
                        if(fcxx > fcyy) Box.v0 = 1.0; else Box.v1 = 1.0;
                    }

                    Box.eigenIndex = M_PI*4*sqrt(eigvl0)*sqrt(eigvl1)/queueEnd;
                    Box.eigvl0 = eigvl0;
                    Box.eigvl1 = eigvl1;
                    
                }

                Box.x_mean = fsx;
                Box.y_mean = fsy;
                Box.pixSize = queueEnd;

                Box.x_max = maxX;
                Box.x_min = minX;
                Box.y_min = minY;
                Box.y_max = maxY;

                int corners = 0;
                float cx=0, cy=0;
                float cX[4], cY[4];
                float dist=0, maxDist=0;

                for(int i=0; i<4; i++)
                {
                    maxDist=0;
                    for(int s=0; s<contour_index; s++)
                    {
                        pixPos = contour[s];
                        cx = pixPos%width - fsx;
                        cy = pixPos/width - fsy;

                        dist = 0;
                        if(i > 0)
                        {
                            for(int j=0; j<i; j++)
                            {
                                dist += sqrt((cx - cX[j])*(cx-cX[j])+(cy-cY[j])*(cy-cY[j]));
                            }
                        }
                        else
                        {
                            dist = cx*cx + cy*cy;

                            if(debug)
                            {   
                                if(s < MAX_CONTOUR_POINTS)
                                {
                                    Box.contourX[s] = cx+fsx-minX;
                                    Box.contourY[s] = cy+fsy-minY;
                                }
                            }
                        }

                        if(dist > maxDist)
                        {
                            cX[i] = cx;
                            cY[i] = cy;
                            maxDist = dist;
                        }
                    }
                }
                Box.contourSize = min(contour_index, MAX_CONTOUR_POINTS);

                for(int i=0; i<4; i++)
                {
                    Box.cornerX[i] = cX[i]+fsx;
                    Box.cornerY[i] = cY[i]+fsy;
                }

                bboxes.push_back(Box);
            }
            else
            {
                numBoxes--;
            }   
        }
    }

    free(stack);
    free(contour);
    free(buffer);

    return bboxes;
}  

detector::BBoxes createMsg(std::vector<struct bbox> *ptr)
{
    detector::BBoxes obj_msg; 
    struct bbox *box;

    for(unsigned int i=0; i < ptr->size(); i++)
    {
        box = &(ptr->at(i));
        detector::BBox temp;

        temp.boxID = box->id;
        temp.minX = box->x_min;
        temp.minY = box->y_min;
        temp.maxX = box->x_max;
        temp.maxY = box->y_max;
        temp.centreX = box->x_mean;
        temp.centreY = box->y_mean;
        temp.pixSize = box->pixSize;
        temp.contourSize = box->contourSize;
        temp.areaIndex = box->areaIndex;
        temp.full = !box->warning;

        for(int j=0; j<4; j++)
        {
            temp.cornerX.push_back(box->cornerX[j]);
            temp.cornerY.push_back(box->cornerY[j]);
        }

        if(debug)
        {
            for(int j=0; j<box->contourSize; j++)
            {
                temp.contourX.push_back(box->contourX[j]);
                temp.contourY.push_back(box->contourY[j]);
            }
        }

        if(strictCheckFlag)
        {
            temp.eigenVal.push_back(box->eigvl0);
            temp.eigenVal.push_back(box->eigvl1);
            temp.eigenVec.push_back(box->v0);
            temp.eigenVec.push_back(box->v1);
        }

        obj_msg.objects.push_back(temp);
    }
    return obj_msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "detector");

    ros::NodeHandle nh;
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("image", 1000, imageCallback);
    ros::Publisher bbox_pub = nh.advertise<detector::BBoxes>("bounding_boxes",10);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher undist_imgPub = it.advertise("undist_image",10);
    image_transport::Publisher marked_imgPub = it.advertise("marked_image", 10);
    
    ros::Rate loop_rate(30);
    std::vector<double> tempList;

    cv::Mat distCoeffs = cv::Mat_<double>(1,5);
    nh.getParam("/detect_node/camera/distortion_coefficients", tempList);
    for(int i=0; i<5; i++)
    {
        distCoeffs.at<double>(i) = tempList[i];
    }

    cv::Mat intrinsic = cv::Mat_<double>(3,3);
    nh.getParam("/detect_node/camera/intrinsic_parameters", tempList);
    int tempIdx=0;
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            intrinsic.at<double>(i,j) = tempList[tempIdx++];
        }
    }

    while (nh.ok())
    {
        while(imageID < 1) ros::spinOnce();
        cv::undistort(img_, undistImg_, intrinsic, distCoeffs);

        if(rqt)
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistImg_).toImageMsg();  
            undist_imgPub.publish(msg);
        }

        std::vector<struct bbox> objects = floodFill(&undistImg_, nh, &markedImg_);
        detector::BBoxes msg = createMsg(&objects);
        msg.stamp = ros::Time::now();
        msg.imageID = imageID;
        bbox_pub.publish(msg);

        if(rqt)
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", markedImg_).toImageMsg();
            marked_imgPub.publish(msg);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}