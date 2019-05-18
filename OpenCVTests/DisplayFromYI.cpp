#include <opencv2/opencv.hpp>
using namespace cv;

int main(int argc, char **argv)
{
    VideoCapture cap("rtsp://192.168.42.1/live");
    Mat frame;
A:
    cap >> frame;
    imshow("MuGsHoT", frame);
    // 24 ms refresh rate, press 'Q' to exit
    if ((char)waitKey(24) == 'q')
    {
        return 0;
    }
    goto A;
}