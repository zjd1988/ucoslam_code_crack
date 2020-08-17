#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
using namespace std;
int main(int argc,char **argv){

    if(argc!=5){
        cout<<"Usage: in w h out "<<endl;
        return -1;
    }
    cv::Size size;
    size.width=stoi(argv[2]);
    size.height=stoi(argv[3]);
    cv::VideoCapture vcap(argv[1]);
    cv::VideoWriter vout(argv[4],CV_FOURCC('D','I','V','X'), 30, size);
    while(vcap.grab()){
        cv::Mat ima,ima2;
        vcap.retrieve(ima);
        cv::resize(ima,ima2,size);
        vout<<ima2;
        cv::imshow("image",ima2);
        char k=cv::waitKey(10);
        if (k==27)break;
    }

}
