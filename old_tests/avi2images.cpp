#include <opencv2/highgui.hpp>
#include <iostream>
using namespace  std;

int main(int argc,char**argv ){


    if (argc!=3){cerr<<"video outdir"<<endl;return -1;}

    cv::VideoCapture vcap;
    vcap.open(argv[1]);
    cv::Mat image;
    uint32_t fidx=0;
    uint32_t totalFrames=vcap.get(CV_CAP_PROP_FRAME_COUNT);
    while(vcap.grab()){
        vcap.retrieve(image);
        std::string number=std::to_string(fidx++);
        while(number.size()!=5) number="0"+number;
        cv::imwrite(argv[2]+string("/")+number+".jpg",image);
        if (fidx%10==0)
        cerr<< 100*float(fidx)/float(totalFrames)<<"%,";
    }

}
