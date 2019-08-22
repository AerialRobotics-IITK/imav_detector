#include <detector/detector.h>

int getObjectType(cv::Vec3b pixel)
{
    int h = (pixel[0] + 90) % 180, s = pixel[1], v = pixel[2];
    if(h>=RHMin && h<=RHMax && s>=RSMin && s<=RSMax && v>=RVMin && v<=RVMax) return redType;
    else if(h>=YHMin && h<=YHMax && s>=YSMin && s<=YSMax && v>=YVMin && v<=YVMax) return yellowType;
    else if(h>=BHMin && h<=BHMax && s>=BSMin && s<=BSMax && v>=BVMin && v<=BVMax) return blueType;
    else if(h>=OHMin && h<=OHMax && s>=OSMin && s<=OSMax && v>=OVMin && v<=OVMax) return orangeType;
    else return 0;
}

std::vector<struct bbox> floodFill(cv::Mat *input, cv::Mat *output, cv::Mat *buf)
{
    std::vector<struct bbox> bboxes;
    if(debug) *output = *input;
    if(debug) *buf = *input;
    cv::Mat hsv(input->cols, input->rows, CV_8UC3);
    cv::cvtColor(*input, hsv, cv::COLOR_BGR2HSV);

    int width = input->cols;
    int height = input->rows;
    int res = width*height;

    int expand[4] = {width, -width, 1, -1};
    
    int stack_index = 0;
    int numBoxes = 0;
    int pixType = 0;
    int contour_index = 0;

    int* stack = (int*)calloc(res, sizeof(int));
    int* contour = (int*)calloc(res, sizeof(int));
    int* buffer = (int*)calloc(res, sizeof(int));

    for(int i=0; i<res; i++)
    {
        buffer[i] = getObjectType(hsv.at<cv::Vec3b>(cv::Point(i%width, i/width)));
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

            eigenPassed = false;
            diagPassed = false;
            areaPassed = false;
            sizePassed = false;
            passed = false;

            struct bbox Box;
            Box.id = numBoxes+1;
            Box.warning = 0;
            Box.type = 0;
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

                    if(buffer[pixPos]==pixType)
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
                Box.type = pixType;

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
                Box.store = (sq(Box.x_mean - width/2) + sq(Box.y_mean - height/2) > maxCentreDist) ? false : true;

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

                if(centreCorrect && Box.warning == 1) 
                {
                    Box.x_mean = (Box.x_mean >= width/2) ? width - (width - Box.x_mean) / centreCorrectIndex : Box.x_mean / centreCorrectIndex;
                    Box.y_mean = (Box.y_mean >= height/2) ? height - (height - Box.y_mean) / centreCorrectIndex : Box.y_mean / centreCorrectIndex;
                }

                if(diagCheckFlag && Box.warning != 1)
                {
                    float diag1=0, diag2=0;

                    diag1 = (Box.cornerX[2]-Box.cornerX[0])*(Box.cornerX[2]-Box.cornerX[0]) + (Box.cornerY[2]-Box.cornerY[0])*(Box.cornerY[2]-Box.cornerY[0]);
                    diag2 = (Box.cornerX[1]-Box.cornerX[3])*(Box.cornerX[1]-Box.cornerX[3]) + (Box.cornerY[3]-Box.cornerY[1])*(Box.cornerY[3]-Box.cornerY[1]);

                    Box.diagIndex = fabs(sqrt(diag1)-sqrt(diag2));
                    if(Box.diagIndex < maxDiagIndex) diagPassed = true; else diagPassed = false;
                }

                if (debug)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        int pos = pixPos + expand[i];
                        buf->at<cv::Vec3b>(cv::Point(pos % width, pos / width)) = BLUE;
                    }
                }

                passed = (!eigenCheckFlag || eigenPassed) && (!diagCheckFlag || diagPassed || Box.warning == 1) && (!areaCheckFlag || areaPassed);
                if(passed)
                {
                    bboxes.push_back(Box);

                    for(int j=0; j<Box.contourSize; j++)
                    {
                        pixPos = contour[j];
                        buffer[pixPos] = contourType + numBoxes;
                        if(debug)
                        {
                            for(int i=0; i<4; i++)
                            {
                                int pos = pixPos + expand[i];
                                output->at<cv::Vec3b>(cv::Point(pos%width, pos/width)) = BLUE;
                            }
                        }
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

mav_utils_msgs::BBoxes createMsg(std::vector<struct bbox> *ptr)
{
    mav_utils_msgs::BBoxes obj_msg; 
    struct bbox *box;

    for(unsigned int i=0; i < ptr->size(); i++)
    {
        box = &(ptr->at(i));
        mav_utils_msgs::BBox temp;
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
        temp.store = box->store;

        if(box->type == redType) temp.colour = "red";
        else if(box->type == yellowType) temp.colour = "yellow";
        else if(box->type == blueType) temp.colour = "blue";
        else if(box->type == orangeType) temp.colour = "orange";
        else temp.colour = "wrong";

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

    ros::NodeHandle sh, ph("~");
    
    ros::ServiceServer exec_server = ph.advertiseService("terminate", serviceCall);

    ros::Subscriber image_sub = sh.subscribe<sensor_msgs::Image>("image", 30, imageCallback);
    ros::Subscriber odom_sub = sh.subscribe<nav_msgs::Odometry>("odometry", 10, odomCallback);
    
    ros::Publisher pose_pub = sh.advertise<mav_utils_msgs::BBPoses>("object_poses",10);
    ros::Publisher bbox_pub = ph.advertise<mav_utils_msgs::BBoxes>("bounding_boxes",10);
    
    ros::Publisher undist_imgPub = ph.advertise<sensor_msgs::Image>("undist_image", 1);
    ros::Publisher marked_imgPub = ph.advertise<sensor_msgs::Image>("marked_image", 1);
    ros::Publisher buffer_imgPub = ph.advertise<sensor_msgs::Image>("flood_image", 1);

    dynamic_reconfigure::Server<detector::reconfigConfig> cfg_server;
    dynamic_reconfigure::Server<detector::reconfigConfig>::CallbackType call_f = boost::bind(&cfgCallback, _1, _2);
    cfg_server.setCallback(call_f);

    ros::Rate loop_rate(10);
    int cols, rows;
    loadParams(ph); 

    while(ros::ok() && !(exit))
    {
        if(run)
        {
            while((imageID < 1 || odom.pose.pose.position.z == 0) && !(exit)){
                if(run) ros::spinOnce();
            }

            if(!isRectified) cv::undistort(img_, undistImg_, intrinsic, distCoeffs);
            else undistImg_ = img_;
            cols = undistImg_.cols; rows = undistImg_.rows;

            std::vector<struct bbox> objects = floodFill(&undistImg_, &markedImg_, &bufferImg_);

            if(objects.size()>0)
            {
                mav_utils_msgs::BBPoses pose_msg = findPoses(&objects);
                pose_msg.stamp = ros::Time::now();
                pose_msg.imageID = imageID;
                if(pose_msg.object_poses.size()>0)
                {
                    pose_pub.publish(pose_msg);
                }
            }

            if(debug)
            {   
                sensor_msgs::ImagePtr undist_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistImg_).toImageMsg();  
                undist_imgPub.publish(undist_msg);
                if(objects.size() > 0)
                {    
                    mav_utils_msgs::BBoxes msg = createMsg(&objects);
                    msg.stamp = ros::Time::now();
                    msg.imageID = imageID;
                    bbox_pub.publish(msg);
                }
                sensor_msgs::ImagePtr marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", markedImg_).toImageMsg();
                marked_imgPub.publish(marked_msg);
                sensor_msgs::ImagePtr buffer_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bufferImg_).toImageMsg();
                buffer_imgPub.publish(buffer_msg);
                // int data[rows][cols];
                // for(int i=0; i<rows*cols; i++){
                    // if(buf[i]>0) data[i/cols][i%cols] = 255;
                    // else data[i/cols][i%cols] = 0;
                // }
                // cv::Mat imgBuf(rows, cols, CV_8UC1, &data);
                // sensor_msgs::ImagePtr buffer_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgBuf).toImageMsg(); 
                // buffer_imgPub.publish(buffer_msg)
                // cv::imshow("frame", imgBuf);
                // cv::waitKey(0);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
