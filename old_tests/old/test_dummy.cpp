#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <cmath>
using namespace std;


std::map<uint32_t,cv::Mat> getPosesFromFile(string filePath,bool invert){
    ifstream ifile(filePath);
    if(!ifile) throw std::runtime_error("Could not open file:"+filePath);

    std::map<uint32_t,cv::Mat> result;
    cv::Mat m=cv::Mat::eye(4,4,CV_32F);
    float *mptr=m.ptr<float>(0);
    while(!filePath.eof()){
        uint32_t id;
        filePath>>id;
        if (filePath.eof()) break;
        for(int i=0;i<16;i++)
            ifile>>mptr[i];
        if (!invert)        result.insert({id,m.clone()});
        else  result.insert({id,m.inv()});

    }
    return result;
}
float getFColor(uchar r,uchar g,uchar b){
    float fcolor=0;
    uchar *c=(uchar*)&fcolor;
    c[0]=r;
    c[1]=g;
    c[2]=b;
    return fcolor;
}

int main(int argc,char **argv){
    try{
        if(argc==1){
            cerr<<"Usage: infile out.pcd [invertPose(0,1)]"<<endl;return -1;
        }
        bool invert=false;
        if (argc>=4)
            invert=stoi(argv[3]);

        auto IdPoses=getPosesFromFile(argv[1],invert);

        ofstream outPCD(argv[2]);
        if(!outPCD)throw std::runtime_error("Could not open file:"+argv[2]);

        outPCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<IdPoses.size()<<"\nHEIGHT 1\nPOINTS "<<IdPoses.size()<<"\nDATA binary\n";

        cv::Vec4f point;

        point[3]=getFColor(255,0,0);//Set the color here

        for(const auto &id_p:IdPoses){
            point[0]=id_p.at<float>(0,3);
            point[1]=id_p.at<float>(1,3);
            point[2]=id_p.at<float>(2,3);
            outPCD.write((char*)&point[0],sizeof(cv::Vec4f));
        }


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }


}
