#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
    VideoCapture cap("udp://localhost:5000");

    Mat frame;

    while (cap.isOpened())
    {
        cap >> frame;

        if (frame.empty())
            continue;

        imshow("Stream", frame);

        if (waitKey(1)=='q')
            break;
    }

    cap.release();
    return 0;
}