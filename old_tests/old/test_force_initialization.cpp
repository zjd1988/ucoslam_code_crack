#include "slam.h"
#include "debug.h"
#include "viewer.h"
#include "utils.h"
#include <aruco/markermap.h>
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

cv::Size readInpuSize(string s){
    for(auto &c:s)if(c==':')c =' ';
    stringstream sstr(s.c_str());
    cv::Size size;
    if ( sstr>>size.width>>size.height) return size;
    else return cv::Size(0,0);
}

cv::Mat resize(cv::Mat &in,cv::Size size){
    if (size.area()<=0)return in;
    cv::Mat ret;
    cv::resize(in,ret,size);
    return ret;
}

int main(int argc,char **argv){
    ucoslam::debug::Debug::setLevel(10);
    CmdLineParser cml(argc,argv);
    if (argc<6 || cml["-h"]){
        cerr<<"Usage: video  camera_params.yml map.yml  f1 f2 [-d descriptor 0:orb 1:AKAZE] [-size markers_size]   [-dict <dictionary>]   [-nomarkers] [-vsize w:h]"<<endl;return -1;
    }
    cv::VideoCapture vcap(argv[1]);
    if (!vcap.isOpened()) throw std::runtime_error("Video not opened");
    ucoslam::Viewer TViewer;
    ucoslam::Slam Slam;
    cv::Size vsize(0,0);
    cv::Mat in_image;


    if(cml["-vsize"]) vsize=readInpuSize(cml("-vsize"));
    ucoslam::ImageParams image_params_org,image_params;
    image_params_org.readFromXMLFile(argv[2]);
    image_params=image_params_org;
    if (vsize.area()>0) image_params.resize(vsize);
    ucoslam::Slam::Params params;
    params.descriptor=  stoi(cml("-d","0"));
    params.markerSize=stof(cml("-size","1"));
    params.markerDictionary=cml("-dic","ARUCO_MIP_36h12");
    params.detectMarkers=!cml["-nomarkers"]; //work only with keypoints!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    Slam.setParams(image_params,params);



    aruco::MarkerMap map;
    map.readFromFile(argv[3]);
    //detect the location in the image
    aruco::MarkerDetector mdetector;
    aruco::MarkerDetector::Params p;
    p._thresParam1_range=7;
    p._thresParam1=7;
    p._subpix_wsize=5;
    mdetector.setDictionary(cml("-dict","ARUCO_MIP_36h12"));




    auto f1=stoi(argv[4]);
    auto f2=stoi(argv[5]);
    vector<cv::Mat> images;
    vector<ucoslam::se3> poses;
    bool done=false;
    while(images.size()!=2){
        cv::Mat image;
        vcap>>image;
        auto fidx=int(vcap.get(CV_CAP_PROP_POS_FRAMES));
        if (fidx==f1||fidx==f2) {
            auto markers=mdetector.detect(image);
            auto r_t=map.calculateExtrinsics(markers,0.05,image_params_org.CameraMatrix,image_params_org.Distorsion);
            if(r_t.first.empty()) throw std::runtime_error("Could not detect pose in the frame!!");
            vector<cv::Point3f> p3d;vector<cv::Point2f> p2d;
            for(auto m:markers){
                p2d.insert(p2d.end(),m.begin(),m.end());
                auto m3d=map.getMarker3DInfo(m.id);
                p3d.insert(p3d.end(),m3d.begin(),m3d.end());
            }
            cout<<"err="<<ucoslam::reprj_error( p3d,p2d,image_params_org,r_t.first,r_t.second)<<endl;

            for(auto m:markers) m.draw(image,cv::Scalar(255,0,0));
            cv::imshow("image",image);
            cv::waitKey(0);
            images.push_back(resize(image,vsize));

            poses.push_back(ucoslam::se3(r_t.first,r_t.second));
        }
    }
    Slam.initialize(images[0],f1,poses[0],images[1],f2,poses[1]);
    cout<<"global err="<< Slam.globalReprojError()<<endl;
    Slam.saveToFile("world.bin");
    Slam.saveToPcd("world.pcd");


}
