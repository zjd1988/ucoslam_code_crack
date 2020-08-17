


#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/calib3d/calib3d.hpp>
 #include <memory>
#include <iostream>
#include <fstream>
#include "optimization/loopclosure.h"


using namespace std;
void writeRefSystems(map<uint32_t,cv::Mat> &mapC,string pcdname){

    auto getFColor=[](cv::Scalar color){
         float fcolor;uchar *c=(uchar*)&fcolor;
         for(int i=0;i<3;i++)c[i]=color[i];
         return fcolor;
    };

    auto getPoints=[&](float size,cv::Scalar color=cv::Scalar(-1,-1,-1)){
        std::vector<cv::Vec4f> points;

        auto red=getFColor(cv::Scalar(0,0,255));
        auto green=getFColor(cv::Scalar(0,255,0));
        auto blue=getFColor(cv::Scalar(255,0,0));
        auto inColor=getFColor(color);


        bool useInColor=color[0]!=-1;
        for(float y=0;y<size;y+=0.025)
            points.push_back(cv::Vec4f(0,y,0,useInColor?inColor:red));
        for(float x=0;x<size;x+=0.025)
            points.push_back(cv::Vec4f(x,0,0,useInColor?inColor:green));
        for(float z=0;z<size;z+=0.025)
            points.push_back(cv::Vec4f(0,0,z,useInColor?inColor:blue));
        return points;
    };


    auto transformPoints=[](vector<cv::Vec4f> &points,cv::Mat &m){
      for(auto &p:points){
          float *mptr=m.ptr<float>(0);
          cv::Vec4f res;
          res[0]= p[0]*mptr[0]+p[1]*mptr[1]+p[2]*mptr[2]+mptr[3];
          res[1]= p[0]*mptr[4]+p[1]*mptr[5]+p[2]*mptr[6]+mptr[7];
          res[2]= p[0]*mptr[8]+p[1]*mptr[9]+p[2]*mptr[10]+mptr[11];
          res[3]=p[3];
          p=res;
      }
    };

    std::vector<cv::Vec4f> points2write;

    for(auto m:mapC){
          std::vector<cv::Vec4f>  points;
          if (m.first==7)
              points=getPoints(0.5,cv::Scalar(125,125,255));
              else
         points=getPoints(0.5);

        transformPoints(points,m.second);
        points2write.insert(points2write.end(),points.begin(),points.end());
    }

    std::ofstream filePCD ( pcdname, std::ios::binary );

    filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points2write.size()<<"\nHEIGHT 1\nPOINTS "<<points2write.size()<<"\nDATA binary\n";


    filePCD.write((char*)&points2write[0],points2write.size()*sizeof(points2write[0]));

}

int main(int argc, char *argv[])

{

        vector<pair<uint32_t,uint32_t>> v_edges_poses;

        map<uint32_t,cv::Mat> mapLC;



        cv::FileStorage loadWork("/home/salinas/tmp/debug.xml", cv::FileStorage::READ);


        cv::FileNode features = loadWork["v_edges_poses"];

        cv::FileNodeIterator it = features.begin(), it_end = features.end();

        // iterate through a sequence using FileNodeIterator

        int idx = 0;

        for( ; it != it_end; ++it, idx++ )

        {

            std::pair<int,int> ii;

            ii.first =  (int)(*it)["a"] ;

            ii.second = (int)(*it)["b"] ;

            v_edges_poses.push_back(ii);


        }

        cv::Mat expectedPose;

        int IdClosesLoopA,IdClosesLoopB;

        loadWork["expectedPose"] >> expectedPose;

        loadWork["IdClosesLoopA"]>> IdClosesLoopA;

        loadWork["IdClosesLoopB"]>>IdClosesLoopB;


        cv::FileNode features2 = loadWork["mapLC"];

        cv::FileNodeIterator it2 = features2.begin(), it2_end = features2.end();

        // iterate through a sequence using FileNodeIterator

        int idx2 = 0;

        for( ; it2 != it2_end; ++it2, idx2++ )

        {

            int temp1 =  (int)(*it2)["i"] ;

            cv::Mat temp2;

            (*it2)["mat"] >> temp2;

            mapLC.insert(std::pair<uint32_t,cv::Mat>(temp1,temp2));

        }


        // is it working? YES:

        for(map<uint32_t,cv::Mat>::const_iterator it = mapLC.begin(); it != mapLC.end(); ++it)

        {

            std::cout << it->second << endl;

        }


        cout<<"expectedPose"<<expectedPose<<endl;


        auto mapCopy=mapLC;
        mapLC[7]=expectedPose;
        writeRefSystems(mapLC,"refBef.pcd");
        cout<<mapLC[6]<<endl;
       ucoslam::loopclosure::loopClosurePathOptimization_cv(v_edges_poses, IdClosesLoopA,IdClosesLoopB,expectedPose, mapLC);
       writeRefSystems(mapLC,"refAft.pcd");
       cout<<mapLC[6]<<endl;


        //

        return 0;

}




//#include "stuff/utils.h"
//#include <aruco/aruco.h>
//#include <iostream>
//#include <opencv2/highgui.hpp>
//#include "optimization/ippe.h"
//using namespace std;
//class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

//cv::Mat resize(cv::Mat &in,cv::Size size){
//    if (size.area()<=0)return in;
//    cv::Mat ret;
//    cv::resize(in,ret,size);
//    return ret;
//}
//cv::Size readInpuSize(string s){
//    for(auto &c:s)if(c==':')c =' ';
//    stringstream sstr(s.c_str());
//    cv::Size size;
//    if ( sstr>>size.width>>size.height) return size;
//    else return cv::Size(0,0);
//}

//int main(int argc,char **argv){
//    try{
//        CmdLineParser cml(argc,argv);
//        if (argc!=4){
//            cerr<<"In: video camera_params dict "<<endl;
//            return -1;
//        }

//        aruco::CameraParameters camp;
//        cv::VideoCapture vcap(argv[1]);
//        camp.readFromXMLFile(argv[2]);
//        cv::Mat img;
//        aruco::MarkerDetector md;
//        md.setDictionary(argv[3]);
//        auto md_params=md.getParams();
//        md.setParams(md_params);
//        pair<double,uint32_t> avrg_ratio(0,0);
//        bool finish=false;
//        while(vcap.grab() && !finish){
//            vcap.retrieve(img);
//            auto markers=md.detect(img);
//            for(auto m:markers){
//                m.draw(img,cv::Scalar(255,0,0));
//                auto res=IPPE::solvePnP_(0.125f,m,camp.CameraMatrix,camp.Distorsion);
//                avrg_ratio.first+=res[1].second/res[0].second;
//                avrg_ratio.second++;
//            }
//            cout<<"avrg ratio="<<avrg_ratio.first/double(avrg_ratio.second)<<endl;
//            cv::imshow("iamge",img);
//            char k=cv::waitKey(5);
//            if (k==27)finish=true;
//        }
//    }catch(std::exception &ex){
//        cout<<ex.what()<<endl;
//    }
//}
