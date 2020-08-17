#include <opencv2/highgui.hpp>
#include "imageparams.h"
#include <iostream>
#include <fstream>
using namespace  std;

int main(int argc,char**argv ){


    if (argc!=3){cerr<<"camerafile outname"<<endl;return -1;}
    ucoslam::ImageParams ip;
    ip.readFromXMLFile(argv[1]);

    ofstream ofile(argv[2]);
    ofile<< ip.fx() <<" "<<ip.fy() <<" "<< ip.cx() <<" "<<ip.cy() <<" "<<   ip.k1()<<" "<<ip.k2()<<" "<<ip.p1()<<" "<<ip.p2()<< endl;
    ofile<<ip.CamSize.width<<" "<<ip.CamSize.height<<endl;
    ofile<<"full"<<endl<<"640 480"<<endl;

    ofile.close();

//    ofile.open(argv[2]+string("dso.txt"));
//    float w=ip.CamSize.width;
//    float h=ip.CamSize.height;

//    ofile<< "RadTan "<<ip.fx()/w <<" "<<ip.fy()/h <<" "<< ip.cx()/w <<" "<<ip.cy()/h <<" "<<   ip.k1()<<" "<<ip.k2()<<" "<<ip.p1()<<" "<<ip.p2()<< endl;
//    ofile<<ip.CamSize.width<<" "<<ip.CamSize.height<<endl;
//    ip.resize(cv::Size(640,480));

//    w=ip.CamSize.width;
//    h=ip.CamSize.height;
//    ofile<< ip.fx()/w <<" "<<ip.fy()/h <<" "<< ip.cx()/w <<" "<<ip.cy()/h <<" "<<   ip.k1()<<" "<<ip.k2()<<" "<<ip.p1()<<" "<<ip.p2()<< endl;
//    ofile<<ip.CamSize.width<<" "<<ip.CamSize.height<<endl;

   // ip.saveToFile(argv[2]+string("small.yml"));


}
