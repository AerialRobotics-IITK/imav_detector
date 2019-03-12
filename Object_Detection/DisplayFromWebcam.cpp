#include <opencv2/opencv.hpp>
using namespace cv;

int main(int argc, char **argv)
{
    VideoCapture cap(0);
    Mat frame;
A:
    cap >> frame;
    imshow("MuGsHoT", frame);
    // 24 ms refresh rate, press 'A' to exit
    if ((char)waitKey(24) == 'a')
    {
        return 0;
    }
    goto A;
}