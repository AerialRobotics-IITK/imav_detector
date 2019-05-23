#include "detector.h"

std::vector<struct bbox> floodFill(cv::Mat *input, ros::NodeHandle nh, cv::Mat *output)
{
    std::vector<struct bbox> bboxes;
    *output = *input;
    
    nh.getParam("/detector/flags/diagCheck", diagCheckFlag);
    nh.getParam("/detector/flags/eigenCheck", eigenCheckFlag);
    nh.getParam("/detector/flags/areaCheck", areaCheckFlag);

    nh.getParam("/detector/flags/debug", debug);
    nh.getParam("/detector/flags/verbose", verbose);

    int minSize = 4000;
    nh.getParam("/detector/box/minSize", minSize);

    // if areaCheckFlag
    float maxAreaIndex = 1;
    nh.getParam("/detector/box/maxAreaIndex", maxAreaIndex);

    // if (eigenCheckFlag)
    float maxEigenIndex = 1.07;
    nh.getParam("/detector/box/maxEigenIndex", maxEigenIndex);

    // if(diagCheckFlag)
    float maxDiagIndex = 100;
    nh.getParam("/detector/box/maxDiagIndex", maxDiagIndex);

    int width = input->cols;
    int height = input->rows;
    int res = width*height;

    int expand[4] = {width, -width, 1, -1};
    
    int stack_index = 0;
    int numBoxes = 0;
    int pixType = 0;
    int contour_index = 0;

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

    int startPos = 0, pixPos = 0, nPix = 0;
    int queueStart = 0, queueEnd = 0;
    int dx=0, dy=0, maxX = width, minX = 0, minY = 0, maxY = height;

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
            
            maxX = minX = Box.rangeX[0] = Box.rangeX[1] = i%width;
            maxY = minY = Box.rangeY[0] = Box.rangeY[1] = i/width;

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

            if(queueEnd <= minSize) queueEnd = 0;

            if(queueEnd>0)
            {
                long long int sx=0, sy=0, cxx=0,cxy=0,cyy=0; 

                for(int s=0; s<queueEnd; s++)
                {
                    pixPos = stack[s];
                    sx += pixPos%width;
                    sy += pixPos/width;

                    if(eigenCheckFlag)
                    {
                        cxx += (pixPos%width)*(pixPos%width);
                        cxy += (pixPos%width)*(pixPos/width);
                        cyy += (pixPos/width)*(pixPos/width);
                    }
                }

                float fsx = (float)sx/queueEnd;
                float fsy = (float)sy/queueEnd;

                if(eigenCheckFlag)
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
                    Box.eigvl0 = 2*sqrt(M_PI)*sqrt(eigvl0);
                    Box.eigvl1 = 2*sqrt(M_PI)*sqrt(eigvl1);

                    if(Box.eigenIndex < maxEigenIndex) eigenPassed = true; else eigenPassed = false;
                }

                Box.x_mean = fsx;
                Box.y_mean = fsy;
                Box.pixSize = queueEnd;

                Box.rangeX[0] = minX;
                Box.rangeX[1] = maxX;
                Box.rangeY[0] = minY;
                Box.rangeY[1] = maxY;

                float cx=0, cy=0;
                int cX[4], cY[4];
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
                                dist += sqrt((cx-cX[j])*(cx-cX[j])+(cy-cY[j])*(cy-cY[j]));
                            }
                        }
                        else
                        {
                            dist = cx*cx + cy*cy;

                            if(verbose)
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

                for(int i=0; i<4; i++)
                {
                    cX[i] = cX[i]+fsx;
                    cY[i] = cY[i]+fsy;
                }

                bool sgn1 = sgnArea(cX[0], cX[1], cX[2], cY[0], cY[1], cY[2]);
                bool sgn2 = sgnArea(cX[0], cX[2], cX[3], cY[0], cY[2], cY[3]);
                bool sgn3 = sgnArea(cX[0], cX[1], cX[3], cY[0], cY[1], cY[3]);

                if(sgn1 == sgn2)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        Box.cornerX[i] = cX[i];
                        Box.cornerY[i] = cY[i];
                    }
                }
                else
                {
                    if(sgn2 ==  sgn3)
                    {
                        Box.cornerX[0] = cX[0]; Box.cornerY[0] = cY[0];
                        Box.cornerX[1] = cX[3]; Box.cornerY[1] = cY[3];
                        Box.cornerX[2] = cX[1]; Box.cornerY[2] = cY[1];
                        Box.cornerX[3] = cX[2]; Box.cornerY[3] = cY[2];
                    }
                    else
                    {
                        Box.cornerX[0] = cX[0]; Box.cornerY[0] = cY[0];
                        Box.cornerX[1] = cX[1]; Box.cornerY[1] = cY[1];
                        Box.cornerX[2] = cX[3]; Box.cornerY[2] = cY[3];
                        Box.cornerX[3] = cX[2]; Box.cornerY[3] = cY[2];
                    }
                    
                }
                
                Box.contourSize = min(contour_index, MAX_CONTOUR_POINTS);

                if(areaCheckFlag)
                {
                    float area = 0;
                    for(int i=0; i<4; i++)
                    {
                        area += Box.cornerX[i]*Box.cornerY[(i+1)%4] - Box.cornerY[i]*Box.cornerX[(i+1)%4];
                    }
                    area = (float)fabs(area/2);
                    Box.areaIndex = fabs((float)area/queueEnd -1);
                    if(Box.areaIndex < maxAreaIndex) areaPassed = true; else areaPassed = false;
                }

                if(Box.warning) diagCheckFlag = false;
                if(diagCheckFlag)
                {
                    float diag1=0, diag2=0;

                    diag1 = (Box.cornerX[2]-Box.cornerX[0])*(Box.cornerX[2]-Box.cornerX[0]) + (Box.cornerY[2]-Box.cornerY[0])*(Box.cornerY[2]-Box.cornerY[0]);
                    diag2 = (Box.cornerX[1]-Box.cornerX[3])*(Box.cornerX[1]-Box.cornerX[3]) + (Box.cornerY[3]-Box.cornerY[1])*(Box.cornerY[3]-Box.cornerY[1]);

                    Box.diagIndex = fabs(sqrt(diag1)-sqrt(diag2));
                    if(Box.diagIndex < maxDiagIndex) diagPassed = true; else diagPassed = false;
                }

                passed = (!eigenCheckFlag || eigenPassed) && (!diagCheckFlag || diagPassed) && (!areaCheckFlag || areaPassed);
                if(passed)
                {
                    bboxes.push_back(Box);

                    for(int j=0; j<Box.contourSize; j++)
                    {
                        pixPos = contour[j];
                        buffer[pixPos] = contourType + numBoxes;
                        // does not show colors other than blue
                        if(debug) output->at<cv::Vec3b>(cv::Point(pixPos%width, pixPos/width)) = BLUE;
                    }
                }
                else
                {
                    numBoxes--;
                }
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

        for(int i=0; i<2; i++)
        {
            temp.rangeX.push_back(box->rangeX[i]);
            temp.rangeY.push_back(box->rangeY[i]);
        }

        temp.centre.push_back(box->x_mean);
        temp.centre.push_back(box->y_mean);

        temp.pixSize = box->pixSize;
        temp.contourSize = box->contourSize;       
        temp.full = !box->warning;

        for(int j=0; j<4; j++)
        {
            temp.cornerX.push_back(box->cornerX[j]);
            temp.cornerY.push_back(box->cornerY[j]);
        }

        if(eigenCheckFlag)
        {
            temp.eigenIndex = box->eigenIndex;
            temp.eigenVal.push_back(box->eigvl0);
            temp.eigenVal.push_back(box->eigvl1);
            temp.eigenVec.push_back(box->v0);
            temp.eigenVec.push_back(box->v1);
        }

        if(diagCheckFlag) temp.diagIndex = box->diagIndex;
        if(areaCheckFlag) temp.areaIndex = box->areaIndex;

        if(verbose)
        {
            for(int j=0; j<box->contourSize; j++)
            {
                temp.contourX.push_back(box->contourX[j]);
                temp.contourY.push_back(box->contourY[j]);
            }
        }

        obj_msg.objects.push_back(temp);
    }
    return obj_msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "detector");

    ros::NodeHandle nh;
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("image", 30, imageCallback);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odometry", 10, odomCallback);
    ros::Publisher bbox_pub = nh.advertise<detector::BBoxes>("bounding_boxes",10);
    ros::Publisher pose_pub = nh.advertise<detector::BBPoses>("object_poses",10);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher undist_imgPub = it.advertise("undist_image",10);
    image_transport::Publisher marked_imgPub = it.advertise("marked_image", 10);
    
    ros::Rate loop_rate(30);
    std::vector<double> tempList;

    cv::Mat distCoeffs = cv::Mat_<double>(1,5);
    nh.getParam("/detector/distortion_coefficients/data", tempList);
    for(int i=0; i<5; i++)
    {
        distCoeffs.at<double>(i) = tempList[i];
    }

    nh.getParam("/detector/camera_matrix/data", tempList);
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
        std::vector<struct bbox> objects = floodFill(&undistImg_, nh, &markedImg_);
        if(objects.size()>0)
        {
            detector::BBPoses pose_msg = findPoses(&objects, nh);
            pose_msg.stamp = ros::Time::now();
            pose_msg.imageID = imageID;
            pose_pub.publish(pose_msg);
        }
        if(debug)
        {   
            sensor_msgs::ImagePtr undist_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistImg_).toImageMsg();  
            undist_imgPub.publish(undist_msg);
            if(objects.size() > 0)
            {    
                detector::BBoxes msg = createMsg(&objects);
                msg.stamp = ros::Time::now();
                msg.imageID = imageID;
                bbox_pub.publish(msg);
            }
            sensor_msgs::ImagePtr marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", markedImg_).toImageMsg();
            marked_imgPub.publish(marked_msg);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
