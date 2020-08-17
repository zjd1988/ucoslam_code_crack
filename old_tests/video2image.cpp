//Reads video and save images when press 'w'

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace std;
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

cv::Mat resize(cv::Mat &in,cv::Size size){
    if (size.area()<=0)return in;
    cv::Mat ret;
    cv::resize(in,ret,size);
    return ret;
}
cv::Size readInpuSize(string s){
    for(auto &c:s)if(c==':')c =' ';
    stringstream sstr(s.c_str());
    cv::Size size;
    if ( sstr>>size.width>>size.height) return size;
    else return cv::Size(0,0);
}
int main(int argc,char **argv){
    CmdLineParser cml(argc,argv);
    try{
        if (argc!=2){cerr<<"USage: in.avi [-size w:h]"<<endl;return -1;}
        cv::VideoCapture vcap(argv[1]);
        cv::Size size=readInpuSize(cml("-size","0:0"));
        char k=0;
        int counter=0;
        while(vcap.grab() && k!=27){
            cv::Mat image_in,imresized;
            vcap.retrieve(image_in);
            imresized=resize(image_in,size);
            cv::imshow("image",imresized);
            k=cv::waitKey(0);
            if (k=='w'){
                cv::imwrite(string("image-")+std::to_string(counter++)+string(".jpg"),imresized);
                cerr<<"image saved"<<endl;
            }
        }


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
