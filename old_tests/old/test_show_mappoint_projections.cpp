#include "basic_types.h"
#include "slam.h"
#include <opencv2/imgproc.hpp>
#include "utils.h"
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};


struct GridComposer{


    int _maxCols=4;

    cv::Mat _composed,_resized;
    int _nCols=-1;
    int _nRows=-1;


    cv::Size _imageSize;

    cv::Size _userTotalImgeSize;

    void setParams(cv::Size totalImageSize){
        _userTotalImgeSize=totalImageSize;
    }



    cv::Mat operator()(ucoslam::Slam &world,uint32_t kpidx){
        if (kpidx>=world.TheMap->map_points.size())return cv::Mat() ;
        if (world.TheMap->map_points.count(kpidx)==0)return  cv::Mat();

        int nimages=world.TheMap->map_points[kpidx].frames.size();
        _nCols=min(nimages,_maxCols);
        _nRows=nimages/_nCols;
        if( nimages%_nCols!=0)_nRows++;


        _imageSize.width = _userTotalImgeSize.width/_nCols;
        _imageSize.height= _userTotalImgeSize.height/_nRows;

        cv::Size finalSize;
        finalSize.width=_imageSize.width*_nCols;
        finalSize.height=_imageSize.height*_nRows;

        _composed.create(finalSize.height,finalSize.width,CV_8UC3);
        _composed.setTo(cv::Scalar::all(0));
        //check composed image size
        int cw=0;
        for(auto f_i:world.TheMap->map_points[kpidx].frames){
            auto &im=world.TheMap->keyframes[f_i.first].image;
            cv::Mat im_color;
            cv::cvtColor(im,im_color,CV_GRAY2BGR);
            //now, draw the point
            cv::Point2f pt=world.TheMap->keyframes[f_i.first].und_kpts[ f_i.second].pt;
            pt=world.TheImageParams.distortPoint(pt);
            cv::rectangle(im_color,pt-cv::Point2f(3,3),pt+cv::Point2f(3,3),cv::Scalar(0,0,255),2);
            addImage(cw++,im_color);
        }
        return _composed;
    }

private:
    void addImage(int idx,cv::Mat &image){


        int r= idx/_nCols;
        int c= idx - _nCols*r;
        cv::Mat sub=_composed.rowRange( r*_imageSize.height,(r+1)*_imageSize.height ).colRange(c*_imageSize.width,(c+1)*_imageSize.width);
        if ( _imageSize!=image.size()){
            cv::resize(image,_resized,_imageSize);
        }
        else _resized=image;
        _resized.copyTo(sub);
    }

};

//void composeImage(ucoslam::Slam &world,uint32_t kpidx,        GridComposer gc ){
//    if (kpidx>=world.TheMap->map_points.size())return  ;
//    if (world.TheMap->map_points.count(kpidx)==0)return  ;

//    gc.reset();
//    //check composed image size
//    int cw=0;
//    for(auto f_i:world.TheMap->map_points[kpidx].frames){
//        auto &im=world.TheFrameSet[f_i.first].image;
//        cv::Mat im_color;
//        cv::cvtColor(im,im_color,CV_GRAY2BGR);
//        //now, draw the point
//        cv::Point2f pt=world.TheFrameSet[f_i.first].und_kpts[ f_i.second].pt;
//        pt=world.TheImageParams.distortPoint(pt);
//        cv::rectangle(im_color,pt-cv::Point2f(3,3),pt+cv::Point2f(3,3),cv::Scalar(0,0,255),2);
//        gc.addImage(cw++,im_color);
//    }
//}

int main(int argc,char **argv){

    CmdLineParser cml(argc,argv);
    if (argc!=2){cerr<<"Usage: wroldint "<<endl;return -1;}

    try{

        ucoslam::Slam Tworld;
        Tworld.readFromFile(argv[1]);
        cout<<"Readed "<<Tworld.TheMap->map_points.size()<<endl;
        std::vector<uint32_t> points;
        int  maxImageViews=0;
        //find the mappoints that projec in three
        for(const auto &p:Tworld.TheMap->map_points)
            if (p.frames.size()>=4){
                points.push_back(p.id);
                if ( maxImageViews<p.frames.size())maxImageViews=p.frames.size();
            }

        GridComposer gcompo;
        gcompo.setParams(cv::Size(1680,1050));

        cout<<"NPOINTS="<<points.size()<<endl;
         for(size_t i=0;i<points.size();i++){
             cv::imshow("image",gcompo(Tworld,points[i]));
           int k= cv::waitKey(0);
           if (k==27)exit(0);
         }

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
