#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include <iostream>
//#include <opencv2/cudafeatures2d.hpp>
using namespace std;
int main(int argc,char **argv){

    if(argc!=2){
        cout<<"USage : video"<<endl;
        return -1;
    }
    cout<<"haveocl="<<cv::ocl::haveOpenCL	()<<" use="<<cv::ocl::useOpenCL ()<<endl;
     cv::VideoCapture vcap;
    vcap.open(argv[1]);
    cv::Mat image;
    vcap>>image;
    cv::cvtColor(image,image,CV_BGR2GRAY);
    cv::UMat umat,desc;
    image.copyTo(umat);
    float t1,t2;
     auto orb=cv::ORB::create();
     orb->setMaxFeatures(2000);
     orb->setScaleFactor(1.2);
     orb->setEdgeThreshold(20);
     orb->setFastThreshold(7);
     vector<cv::KeyPoint> kpts;
     t1=cv::getTickCount();
     orb->detectAndCompute(umat,cv::UMat(),kpts,desc);
     t2=cv::getTickCount();

     t1=cv::getTickCount();
     orb->detectAndCompute(umat,cv::UMat(),kpts,desc);
     t2=cv::getTickCount();
     cout<<float(t2-t1)/float(cv::getTickFrequency())*1000<<" ms"<<endl;
     cout<<kpts.size()<<endl;

     cv::ocl::setUseOpenCL(false);
     t1=cv::getTickCount();
     orb->detectAndCompute(image,cv::Mat(),kpts,desc);
     t2=cv::getTickCount();
     cout<<float(t2-t1)/float(cv::getTickFrequency())*1000<<" ms"<<endl;
     cout<<kpts.size()<<endl;
//     cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());

//     cv::cuda::GpuMat gpu_img,gpu_desc,keypointsGpu;
//         cv::cuda::GpuMat dst;
//         t1=cv::getTickCount();
//         gpu_img.upload(image);
//         t2=cv::getTickCount();
//         cout<<"upload image"<<float(t2-t1)/float(cv::getTickFrequency())*1000<<" ms"<<endl;
//     auto cudaorb=cv::cuda::ORB::create();
//     cudaorb->setMaxFeatures(2000);
//     cudaorb->setScaleFactor(1.2);
//     cudaorb->setEdgeThreshold(20);
//     cudaorb->setFastThreshold(7);
//     cout<<"orb created image"<<endl;

//t1=cv::getTickCount();
//cudaorb->detectAndCompute(gpu_img,cv::cuda::GpuMat(),kpts,gpu_desc);
//t2=cv::getTickCount();
//cout<<float(t2-t1)/float(cv::getTickFrequency())*1000<<" ms"<<endl;
//cout<<kpts.size()<<endl;


//t1=cv::getTickCount();
//cudaorb->detectAndCompute(gpu_img,cv::cuda::GpuMat(),kpts,gpu_desc);
//t2=cv::getTickCount();
//cout<<float(t2-t1)/float(cv::getTickFrequency())*1000<<" ms"<<endl;
//cout<<kpts.size()<<endl;
}
