#include "featureextractors/ORBextractor.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "private_basictypes//timers.h"
#include "featureextractors/feature2dserializable.h"
using namespace std;
uint64_t imsig(cv::Mat &im){
    uint64_t sum=0;
    assert(im.type()==CV_8U);
    for(int y=0;y<im.rows;y++){
        auto ptr=im.ptr<uchar>(y);
        for(int x=0;x<im.cols;x++)
            sum+=ptr[x];
    }
    return sum;
}
uint64_t kpsig(cv::Mat &desc,const std::vector<cv::KeyPoint> &kps){
    uint64_t  sum=0;
    for(auto &k:kps) sum+=k.pt.x*1000+k.pt.y*1000+k.octave;
    return sum+cv::sum(desc)[0];
}

int main(int argc,char **argv){
    if (argc==1){cerr<<"Usage: video descriptor nthreads"<<endl;return -1;}
    auto extractor=ucoslam::Feature2DSerializable::create(ucoslam::DescriptorTypes::fromString(argv[2]),stoi(argv[3]),2000,2,1.2);
     cv::VideoCapture vcap;
    vcap.open(argv[1]);
    if(!vcap.isOpened()){cerr<<"ERROR IN"<<endl;return -1;}


    while(vcap.grab()  ){

        cv::Mat im;
        vcap.retrieve(im);
        cv::Mat im2;
        cv::cvtColor(im,im2,CV_BGR2GRAY);

        vector<cv::KeyPoint> kpts;cv::Mat desc;
        {
            ucoslam::ScopeTimer t("compute");
            extractor->detectAndCompute(im2,cv::Mat(),kpts,desc);
        }
         cout<<"Sim="<<imsig(im2)<< " sig="<<kpsig(desc,kpts)<<endl;
    }
}
