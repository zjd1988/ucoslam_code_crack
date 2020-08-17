#include <iostream>
#include <opencv2/highgui.hpp>
#include "slam.h"
#include "timers.h"
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};
cv::Mat resize(cv::Mat &in,cv::Size size){
    if (size.area()<=0)return in;
    cv::Mat ret;
    cv::resize(in,ret,size);
    return ret;
}


int main(int argc,char **argv){
    if (argc!=3){cerr<<"Usage: ucoslam.ucs inputvideo"<<endl;return -1;}

    try{


        ucoslam::Slam slam;
        slam.readFromFile(argv[1]);
        cv::VideoCapture vcap(argv[2]);
        if(!vcap.isOpened())throw std::runtime_error("COuld not open video");


        char k=0;
        int counter=0;
        for(int i=0;i<38;i++) vcap.grab();
        while( k!=27 && vcap.grab()){
            cout<<"frame==="<<counter<<endl;
            cv::Mat img;
            vcap.retrieve(img);
            img=resize(img,slam.TheImageParams.CamSize);
            ucoslam::Frame f;
            slam.fextractor.process(img,f,vcap.get(CV_CAP_PROP_POS_FRAMES),100);
            vector<uint32_t> kfcandidates;
            {
                ucoslam::ScopeTimer Timer("reloc");
                kfcandidates=slam.TheMap->TheKFDataBase.relocalizationCandidates(f,slam.TheMap->keyframes,slam.TheMap->TheCVGraph);
            }
            for(auto c:kfcandidates)cout<<c<<" ";cout<<endl;
            cv::imshow("image",img);
            cv::imwrite("image"+to_string(counter++)+".png",img);
            k=cv::waitKey(0);
        }


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
