#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "basic_types.h"
#include "utils.h"
#include "track_utils.h"
#include "debug.h"
#include "ORBextractor.h"
#include <cvba/bundler.h>
#include <aruco/markermap.h>
#include <aruco/posetracker.h>
#include <aruco/markerdetector.h>
#include "Initializer.h"

using namespace std;


template<typename T>
void   fromStream__ (  std::vector<T> &v,istream &str ) {

    uint32_t s;
    str.read( ( char* ) &s,sizeof ( s) );
    v.resize(s);
    for(const auto &e:v)       str.read(  (char*)&e,sizeof(e));
}

/**
 */

void  fromStream__ ( cv::Mat &m,istream &str ) {
    int r,c,t;
    str.read ( ( char* ) &r,sizeof ( int ) );
    str.read ( ( char* ) &c,sizeof ( int ) );
    str.read ( ( char* ) &t,sizeof ( int ) );
    cout<<r<<" "<<c<<" "<<t<<endl;
    m.create ( r,c,t );
    for ( int y=0; y<m.rows; y++ )
        str.read ( m.ptr<char> ( y ),m.cols *m.elemSize() );
}
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

int main(int argc,char **argv){

    ucoslam::debug::Debug::setLevel(10);
    CmdLineParser cml(argc,argv);
    if (argc!=2 || cml["-h"]){
        cerr<<"Usage: in.dat "<<endl;return -1;
    }
    ifstream file( argv[1]);
    cv::Mat mK;
    vector<cv::KeyPoint> mvKeys1,mvKeys2;
    vector<int> mvMatches12;
int mag;
file.read((char*)&mag,sizeof(int));

    cerr<<"readed:"<<mag<<endl;
    fromStream__(mK,file);
    cerr<<"readed"<<endl;
    fromStream__(mvKeys1,file);
    cerr<<"readed"<<endl;
    fromStream__(mvKeys2,file);
    cerr<<"readed"<<endl;
    fromStream__(mvMatches12,file);
    cerr<<"readed"<<endl;
    file.close();
cerr<<"readed END"<<endl;
    vector<cv::DMatch> matches;
    for(size_t i=0;i<mvMatches12.size();i++){
        if (mvMatches12[i]!=-1)
            matches.push_back(cv::DMatch(i,mvMatches12[i],1));
    }

    ucoslam::Initializer initializer ;
    cv::Mat R12,t12;
    vector<cv::Point3f> p3d;
    vector<bool> triangulated;
    auto res=initializer.getRtFromMatches(mK,mvKeys1,mvKeys2,matches,R12,t12,p3d,triangulated);
    cout<<"initialize="<<res<<" "<< p3d.size()<<" "<<triangulated.size()<< endl;
    cout<<R12<<" "<<t12<<endl;
    if( R12.type()==CV_32F) cerr<<"Is 32"<<endl;
//    while( char(cv::waitKey(0))!=27 ) ;


}


