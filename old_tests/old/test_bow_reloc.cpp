//! \brief Tests BOW-based relocalization
//! \author mjmarin
//! \date Dec/2016

#include "optimization/levmarq.h"
#include "ucoslam.h"
#include "private_basictypes//debug.h"
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

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
//    vector<cv::Point3f> points3d;
//    vector<cv::Point2f> points2d;

//    auto errorsin=[&](const ucoslam::LevMarq<float>::eVector &sol,ucoslam::LevMarq<float>::eVector &err){
//        vector<cv::Point2f> projections;

//        cv::projectPoints(poin3d,.....,projections);

//        err.resize(points2d.size()*2);
//        for(int i=0;i<points2d.size();i++){
//            err(i*2)=   points2d[i].x- projections[i].x;
//            err(i*2+1)=   points2d[i].x- projections[i].x;
//        }
//    };

//    ucoslam::LevMarq<float>  levm;
//    ucoslam::LevMarq<float>::eVector sol;
//    sol.resize(6);
//    sol(0)=0.1;
//    levm.solve(sol, std::bind( errorsin,std::placeholders::_1,std::placeholders::_2) );



   // else  solver.solve(sol,std::bind( error_fast,std::placeholders::_1,std::placeholders::_2),std::bind(jacobian_f,std::placeholders::_1,std::placeholders::_2) );





//    CmdLineParser cml(argc,argv);
//    if (argc<4 || cml["-h"]){
//        cerr<<"Usage: video  camera_params.yml  world [-f filter_method: 0 Homography, 1 Fundamental Matrix] [-d descriptor 0:orb 1:AKAZE] [-size markers_size] [-pcd out.pcd] [-dict <dictionary>] [-cvba out.cvba] [-ms markerset.yml] [-nomarkers] [-vsize w:h] [-debug level] [-voc bow_volcabulary_file] [-fdt n:number of threads of the feature detector]"<<endl;return -1;
//    }
//    cv::VideoCapture vcap(argv[1]);
//    ucoslam::debug::Debug::setLevel( stoi(cml("-debug","0")));
//    if (!vcap.isOpened()) throw std::runtime_error("Video not opened");
//    ucoslam::Viewer TViewer;
//    ucoslam::Slam Slam;
//    cv::Size vsize(0,0);
//    cv::Mat in_image;


//    if(cml["-vsize"]) vsize=readInpuSize(cml("-vsize"));

//    Slam.readFromFile(argv[3]);
//    if (ucoslam::debug::Debug::getLevel() >= 0)
//        cout << "World loaded!"  << endl;

//    // TODO: read frames from video and, for each one, relocalise
//    bool finish=false;
//    while(in_image.empty())vcap>>in_image;
//    int waitTime=0;
//    auto myIdx = Slam.getLastProcessedFrame();
//    while(!finish && !in_image.empty()){
//        in_image=resize(in_image,vsize);

//        //Slam.process(in_image,int(vcap.get(CV_CAP_PROP_POS_FRAMES)));
//        //TViewer.update(Slam);
//        ucoslam::Frame cFrame;
//        Slam.fextractor.process(in_image, cFrame, ++myIdx);
//        ucoslam::se3 pose;
//        Slam.relocalize(cFrame,pose);

//        cv::imshow("image",Slam.getShowImage("press s to start/stop"));
//        int k=cv::waitKey(waitTime);
//        if (char(k)=='k')for(int i=0;i<5;i++) vcap>>in_image;
//        if (char(k)=='s') waitTime= (waitTime==0?30:0);
//        if(char(k)==27 )finish=true;
//        vcap>>in_image;
//    }



//    TViewer.stop();

    return 0;
}
