#include <stdio.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
    cv::Mat img = cv::imread(argv[1]);
    cv::imshow("First IMage", img);
    cv::waitKey(0);
}
