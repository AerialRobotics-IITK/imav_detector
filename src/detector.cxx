#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <detector/BBoxes.h>
#include <detector/BBox.h>
#define MAX_BOXES 10000
#define MAX_CONTOUR_POINTS 2500
#define min(a,b) (a>b)?b:a

int whiteType = -10;
int borderType = (int)10E4;
int contourType = (int)10E6;
bool strictCheckFlag = true;
int imageID = 0;
cv::Mat img_, undistImg_;

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
  return;
}

std::vector<struct bbox> floodFill(cv::Mat *input, ros::NodeHandle nh)
{
    std::vector<struct bbox> bboxes;

    int minSize = 40;
    int minAreaIndex = 0.4;
    nh.getParam("box/minSize", minSize);
    nh.getParam("box/minAreaIndex", minAreaIndex);
    nh.getParam("box/strictCheck", strictCheckFlag);

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
        if(input->at<uchar>(i/width, i%width)!=0)
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
                        dx = i%width;
                        dy = i/width;
                        if(maxX < dx) maxX = dx;
                        if(minX > dx) minX = dx;
                        if(minY > dy) minY = dy;
                        if(maxY < dy) maxY = dy;
                    }

                    if(buffer[pixPos]==borderType) Box.warning = 1;
                }

                if (nPix!=4) contour[contour_index++] = pixPos;
            }

            if(queueEnd <= minSize)
            {
                queueEnd = 0;
            }
            else
            {
                Box.areaIndex = fabs((maxX-minX+1)*(maxY-minY+1)/queueEnd - 1);
                if(Box.areaIndex > minAreaIndex) queueEnd = 0; 
            }

            if(queueEnd>0)
            {
                for(int j=0; j<contour_index; j++)
                {
                    pixPos = contour[j];
                    buffer[pixPos] = contourType + numBoxes;
                }
                
                long long int sx, sy;
                long long int cxx,cxy,cyy; 

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

                int corners = 0, cx, cy;
                int cX[4], cY[4];
                float dist, maxDist;
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
                                dist += sqrt((cx - cX[i])*(cx-cX[i])+(cy-cY[i])*(cy-cY[i]));
                            }
                        }
                        else
                        {
                            dist = cx*cx + cy*cy;
                            if(s < MAX_CONTOUR_POINTS)
                            {
                                Box.contourX[s] = cx+fsx;
                                Box.contourY[s] = cy+fsy;
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
        for(int j=0; j<box->contourSize; j++)
        {
            temp.contourX.push_back(box->contourX[j]);
            temp.contourY.push_back(box->contourY[j]);
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
    std::cout<<"check1"<<std::endl;
    ros::Publisher bbox_pub = nh.advertise<detector::BBoxes>("bounding_boxes",10);
    
    std::vector<double> tempList;
    cv::Mat distCoeffs = cv::Mat_<double>(1,5);
    nh.getParam("detect_node/camera/distortion_coefficients", tempList);
        std::cout<<"check2"<<std::endl;

    for(int i=0; i<5; i++)
    {
        distCoeffs.at<double>(i) = tempList[i];
    }
        std::cout<<"check3"<<std::endl;

    cv::Mat intrinsic = cv::Mat_<double>(3,3);
    nh.getParam("detect_node/camera/intrinsic_parameters", tempList);
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
        std::vector<struct bbox> objects = floodFill(&undistImg_, nh);
        detector::BBoxes msg = createMsg(&objects);
        msg.stamp = ros::Time::now();
        msg.imageID = imageID;
        bbox_pub.publish(msg);
        ros::spinOnce();
    }
    return 0;
}