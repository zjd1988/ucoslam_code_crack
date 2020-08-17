#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include "private_basictypes//se3.h"
 #include <map>
#include <Eigen/Geometry>
using namespace std;
using namespace ucoslam;
std::map<uint32_t,se3> flog1,flog2;

void  getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,float &qx,float &qy,float &qz,float &qw,float &tx,float &ty,float &tz);
cv::Mat  getMatrix(double tx,double ty ,double tz,double qx,double qy, double qz,double qw);




std::map<uint32_t,se3> loadFile(std::string fp,bool invert=false){
    std::map<uint32_t,se3> fmap;
    ifstream file(fp);
    float stamp;
    float tx,ty,tz,qx,qy,qz,qw;
    while(!file.eof()){
        string line;
        std::getline(file,line);

        stringstream sline;sline<<line;
         if (sline>>stamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw){
             auto m=getMatrix(qx,qy,qz,qw,tx,ty,tz);
             if (invert)m=m.inv();
            fmap.insert(make_pair (stamp, se3(m)));
        }
     }
    return fmap;
}


cv::Vec4f convert(cv::Point3f p,cv::Scalar color){

    float fcolor;uchar *c=(uchar*)&fcolor;

    for(int i=0;i<3;i++)c[i]=color[i];
    return cv::Vec4f(p.x,p.y,p.z,fcolor);
}

std::vector<cv::Vec4f> getLine(const  cv::Point3f &a,const  cv::Point3f &b,cv::Scalar color,int npoints){
    float fcolor;uchar *c=(uchar*)&fcolor;
    for(int i=0;i<3;i++)c[i]=color[i];

    std::vector<cv::Vec4f>  points;
    cv::Vec4f pa(a.x,a.y,a.z,fcolor);
    cv::Vec4f pb(b.x,b.y,b.z,fcolor);
    cv::Vec4f d =pb-pa;
    d*=1./cv::norm(d);
    double step=  cv::norm(pb-pa)*( 1./ double(npoints));
    //cout<<"step="<<step<<endl;
    for(int i=0;i<npoints;i++){
        points.push_back(pa+ (d*step*double(i)));
    }
  //  cout<<pa<<" "<<pb<<" "<<pa+ (d*step*double(npoints))<<endl;
    return points;

}

class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                                    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };



se3 inverseComposition(se3 gt_i, se3 gt_j){

    cv::Mat cv_gti=gt_i.convert();
    cv::Mat cv_gtj=gt_j.convert();
    cv::Mat  res= cv_gtj*cv_gti.inv();
    return se3(res);
}


std::pair<double,double> getKITTIError(const vector<std::pair<se3,se3> > &gt_other_){

    std::pair<double,double> sum_err(0,0);
     int n=0;

     for(size_t i=0;i<gt_other_.size()-1;i++){

        se3 gt_comp= inverseComposition(gt_other_[i].first, gt_other_[i+1].first);
        se3 other_comp= inverseComposition(gt_other_[i].second, gt_other_[i+1].second);
        se3 terr=inverseComposition(gt_comp,other_comp);
        sum_err.first+=cv::norm(terr.getTvec());
        sum_err.second+=360* cv::norm(terr.getRvec())/3.141516;
        n++;
    }

    sum_err.first/=double(n);
    sum_err.second/=double(n);
    return sum_err;
}


vector<std::pair<se3,se3> > getMatchedLocations(std::map<uint32_t,se3> &gt,std::map<uint32_t,se3> &other){
   vector<std::pair<se3,se3> > res;
        for(auto frame_est:other){
            auto frame_gt=gt.find( frame_est.first);
            if (frame_gt!=gt.end())
                res.push_back( {frame_gt->second, frame_est.second});
           }
        return res;

}



void  alignAndScaleToGroundTruth( std::vector<std::pair<se3,se3> > &gt_other ){

    vector<cv::Point3f> points_other,points_gt;
    for(auto gto:gt_other){
        points_gt.push_back(cv::Point3f(gto.first[3],gto.first[4],gto.first[5]));
        points_other.push_back(cv::Point3f(gto.second[3],gto.second[4],gto.second[5]));
    }

    cv::Mat best_T=ucoslam::rigidBodyTransformation_Horn1987(points_other,points_gt,false);
 //   best_T=cv::Mat::eye(4,4,CV_32F);
    for(auto &gto:gt_other)
         gto.second = best_T*gto.second.convert();
}
void savePcd(string filepath,const vector<cv::Vec4f> & points){
    std::ofstream filePCD (filepath, std::ios::binary );
   filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points.size()<<"\nHEIGHT 1\nPOINTS "<<points.size()<<"\nDATA binary\n";
   filePCD.write((char*)&points[0],points.size()*sizeof(points[0]));
}

void savePoses(string filepath,std::map<uint32_t,se3> &poses){

    vector<se3> vposes;
    for(auto p:poses) vposes.push_back(p.second);

    vector<cv::Vec4f> pcdpoints;


    for(size_t i=1;i<vposes.size();i++){
        cv::Point3f p0(vposes[i-1][3],vposes[i-1][4],vposes[i-1][5]);
        cv::Point3f p1(vposes[i][3],vposes[i][4],vposes[i][5]);
        auto pp=getLine(p0,p1,cv::Scalar(0,0,255),10);
        pcdpoints.insert(pcdpoints.end(),pp.begin(),pp.end());
}
    savePcd(filepath,pcdpoints);

}

int main(int argc,char **argv){
    CmdLineParser cml(argc,argv);

    if(argc<3 || cml["-h"])
        {cerr<<"logfile1_groundtruth logfile2   [-flip_y 0:gt 1:est]  "<<endl;return -1;}

    auto f_gt=loadFile(argv[1],false);
    auto f_est=loadFile(argv[2],false);
    savePoses("ingt.pcd",f_gt);
    savePoses("in_other.pcd",f_est);
    //double min_err=std::numeric_limits<double>::max();
    if (cml["-flip_y"]){
        if (stoi( cml("-flip_y","0"))==0)
            for(auto &frame:f_gt)frame.second.rt[4]*=-1;
        else
            for(auto &frame:f_est)frame.second.rt[4]*=-1;
    }

    vector<std::pair<se3,se3> > gt_other_=getMatchedLocations( f_gt,f_est);
    alignAndScaleToGroundTruth(gt_other_);

    //get the ATE error
    double e=0;
    for(auto p:gt_other_){
        cv::Point3f pgt(p.first[3],p.first[4],p.first[5]);
        cv::Point3f poth=cv::Point3f(p.second[3],p.second[4],p.second[5]);
        e+=cv::norm(pgt-poth);
    }
    cout<<"ATE="<<e/double(gt_other_.size())<<endl;


    std::pair<double,double> KittiError=getKITTIError(gt_other_);

    cout<<"KittiError translation="<<KittiError.first<<" rotation="<<KittiError.second<<endl;

    vector<cv::Vec4f> pcdpoints_gt,pcdpoints_est,joinlines;

    for(size_t i=1;i<gt_other_.size();i++){
        cv::Point3f p0(gt_other_[i-1].first[3],gt_other_[i-1].first[4],gt_other_[i-1].first[5]);
        cv::Point3f p1(gt_other_[i].first[3],gt_other_[i].first[4],gt_other_[i].first[5]);
        auto pp=getLine(p0,p1,cv::Scalar(0,0,255),10);
        pcdpoints_gt.insert(pcdpoints_gt.end(),pp.begin(),pp.end());

        auto p0o=cv::Point3f(gt_other_[i-1].second[3],gt_other_[i-1].second[4],gt_other_[i-1].second[5]);
        auto p1o=cv::Point3f(gt_other_[i].second[3],gt_other_[i].second[4],gt_other_[i].second[5]);
        pp=getLine(p0o,p1o,cv::Scalar(0,255,0),10);
        pcdpoints_est.insert(pcdpoints_est.end(),pp.begin(),pp.end());

        pp=getLine(p0,p0o,cv::Scalar(255,0,0),30);
        joinlines.insert(joinlines.end(),pp.begin(),pp.end());



    }
    savePcd("gt.pcd",pcdpoints_gt);
    savePcd("other.pcd",pcdpoints_est);
    savePcd("joinlines.pcd",joinlines);




//    vector<std::pair<cv::Point3f,cv::Point3f> > matches;
//    for(auto frame_est:f_est){
//        auto frame_gt=f_gt.find( frame_est.first);
//        if (frame_gt!=f_gt.end())
//            matches.push_back(make_pair(cv::Point3f(frame_gt->second.rt[3],frame_gt->second.rt[4],frame_gt->second.rt[5]),cv::Point3f( frame_est.second.rt[3],frame_est.second.rt[4],frame_est.second.rt[5])));
//             }


//    {
//        vector<cv::Point3f> points_gt,points_est;
//        for(auto m:matches){
//            points_gt.push_back( m.first);
//            points_est.push_back(m.second);
//        }
//        best_T=ucoslam::rigidBodyTransformation_Horn1987(points_est,points_gt,false);
//        double err=0;
//        for(auto m:matches)
//            err+=cv::norm(m.first-ucoslam::mult(best_T,m.second));
//        err/=double(matches.size());
//        cout<<"Err="<<err<<endl;

//    }


//    cout<<"....2:"<<best_T<<endl;

//    //also, make both initial points the same
//    auto p0_res=ucoslam::mult(best_T,matches[0].second);
//    cout<<"....3"<<endl;

//    cv::Point3f dif= matches[0].first-p0_res;
//    cv::Mat t_m=cv::Mat::eye(4,4,CV_32F);
//    t_m.at<float>(0,3)=dif.x;
//    t_m.at<float>(1,3)=dif.y;
//    t_m.at<float>(2,3)=dif.z;
//    best_T=t_m*best_T;

//    cout<<"...."<<endl;
//     //create points
//    vector<cv::Point3f> points_gt,points_est;

//    for(auto m:matches){
//        points_gt.push_back( m.first);
//        points_est.push_back( ucoslam::mult(best_T,m.second) );
//     }


//     cout<<"...."<<endl;



//    if (cml["-out_pcd"] || cml["-out_pcd_gt"]|| cml["-out_pcd_est"]){

//        cv::Scalar color_est_points(0,0,255),color_gt_points(0,0,0);
//        auto read_color=[](string str){
//            cv::Scalar  color;
//            for(auto &c:str) if(c==',')c=' ';
//             sscanf(str.c_str(),"%lf %lf %lf",&color[0],&color[1],&color[2]);
//            return color;
//        };

//        if(cml["-est_color"]) color_est_points=read_color(cml("-est_color"));
//        if(cml["-gt_color"]) color_gt_points=read_color(cml("-gt_color"));

//        vector<cv::Vec4f> pcdpoints_gt,pcdpoints_est;
//        //transfor points





//        for(size_t i=1;i<points_est.size();i++)        {
//            auto pp=getLine(points_est[i-1],points_est[i],color_est_points,100);
//            pcdpoints_est.insert(pcdpoints_est.end(),pp.begin(),pp.end());
//        }

//        if (cml["-out_pcd"]){
//            vector<cv::Vec4f> pcdpoints=pcdpoints_gt;
//            pcdpoints.insert(pcdpoints.end(),pcdpoints_est.begin(),pcdpoints_est.end());
//            std::ofstream filePCD ( cml("-out_pcd"), std::ios::binary );
//            filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<pcdpoints.size()<<"\nHEIGHT 1\nPOINTS "<<pcdpoints.size()<<"\nDATA binary\n";
//            filePCD.write((char*)&pcdpoints[0],pcdpoints.size()*sizeof(pcdpoints[0]));
//        }
//        if (cml["-out_pcd_gt"]){
//            vector<cv::Vec4f> pcdpoints=pcdpoints_gt;
//             std::ofstream filePCD ( cml("-out_pcd_gt"), std::ios::binary );
//            filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<pcdpoints.size()<<"\nHEIGHT 1\nPOINTS "<<pcdpoints.size()<<"\nDATA binary\n";
//            filePCD.write((char*)&pcdpoints[0],pcdpoints.size()*sizeof(pcdpoints[0]));
//        }
//        if (cml["-out_pcd_est"]){
//            vector<cv::Vec4f> pcdpoints=pcdpoints_est;
//              std::ofstream filePCD ( cml("-out_pcd_est"), std::ios::binary );
//            filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<pcdpoints.size()<<"\nHEIGHT 1\nPOINTS "<<pcdpoints.size()<<"\nDATA binary\n";
//            filePCD.write((char*)&pcdpoints[0],pcdpoints.size()*sizeof(pcdpoints[0]));
//        }


//    }

//    if (cml["-gt_scaled_translated_log"]){
//        cout<<"-gt_scaled_translated_log"<<endl;
//        //transform the log locations  and save them
//        ofstream ofile_d(cml("-gt_scaled_translated_log"));
//        if (!ofile_d){cerr<<"could not open log output file"<<endl;exit(0);}
//        cv::Mat scaleMatrix=cv::Mat::eye(4,4,CV_32F);
//        scaleMatrix.at<float>(0,0)=scaleMatrix.at<float>(1,1)=scaleMatrix.at<float>(2,2)=scale_best;
//        for(auto &fp:f_gt){
//            fp.second=best_T* scaleMatrix*fp.second;
//            double tx,ty,tz,qx,qy,qz,qw;
//            getQuaternionAndTranslationfromMatrix44(fp.second,qx,qy,qz,qw,tx,ty,tz);
//            ofile_d<<fp.first<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;
//            cerr<<"d:"<<fp.first<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;

//        }
//        //now, save to file


//    }


}
cv::Mat  getMatrix(double qx,double qy, double qz,double qw,double tx,double ty ,double tz){


    double qx2 = qx*qx;
    double qy2 = qy*qy;
    double qz2 = qz*qz;


    cv::Mat m=cv::Mat::eye(4,4,CV_32F);

    m.at<float>(0,0)=1 - 2*qy2 - 2*qz2;
    m.at<float>(0,1)=2*qx*qy - 2*qz*qw;
    m.at<float>(0,2)=2*qx*qz + 2*qy*qw;
    m.at<float>(0,3)=tx;

    m.at<float>(1,0)=2*qx*qy + 2*qz*qw;
    m.at<float>(1,1)=1 - 2*qx2 - 2*qz2;
    m.at<float>(1,2)=2*qy*qz - 2*qx*qw;
    m.at<float>(1,3)=ty;

    m.at<float>(2,0)=2*qx*qz - 2*qy*qw	;
    m.at<float>(2,1)=2*qy*qz + 2*qx*qw	;
    m.at<float>(2,2)=1 - 2*qx2 - 2*qy2;
    m.at<float>(2,3)=tz;
    return m;
}
