#include "cvni2.h"
#include <opencv2/highgui.hpp>
#include <iostream>
using namespace std;
int main(int argc,char **argv){

    cv::VideoWriter vw;
    CvNI2 vin;vin.open(argv[1]);
    cv::Mat color;
    int f=0;
    int k=0;
    while(vin.grab() && k!=27){
        vin.retrieve(color);
        if (!vw.isOpened())
            vw.open(argv[2], CV_FOURCC('X', '2', '6', '4'), 30,color.size()  , color.channels()!=1);
        vw<<color;
        cerr<<f++<<" ";
        cv::imshow("image",color);
        int k=cv::waitKey(10);
        if (k==27) break;
    }
    vw.release();
}
