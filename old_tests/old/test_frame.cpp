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
#include "ippe.h"
#include <aruco/markerdetector.h>
#include "Initializer.h"
#include <random>
#include <marker_mapper/mapper_types.h>
#include <marker_mapper/optimizers/fullsceneoptimizer.h>
#include <marker_mapper/utils/utils3d.h>
using namespace std;
cvba::BundlerData<float> easyBundler(const ImageParams &ip, const Frame &f1,const Frame &f2,const std::vector<cv::DMatch> &matches,const vector<cv::Point3f>  &p3d=vector<cv::Point3f>() );




class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

int main(int argc,char **argv){

    ucoslam::debug::Debug::setLevel(10);
    CmdLineParser cml(argc,argv);
    if (argc<4 || cml["-h"]){
        cerr<<"Usage: im1 im2   camera_params.yml  [-f filter_method: 0 Homography, 1 Fundamental Matrix] [-d descriptor 0:orb 1:AKAZE] [-size markers_size] [-pcd out.pcd] [-dict <dictionary>] [-cvba out.cvba] [-ms markerset.yml]"<<endl;return -1;
    }
    //load images
    cv::Mat im1=cv::imread(argv[1],IMREAD_GRAYSCALE);
    cv::Mat im2=cv::imread(argv[2],IMREAD_GRAYSCALE);


    //read dictionary
    string aruco_dict=cml("-dic","ARUCO_MIP_36h12");
    //read image parameres
    ucoslam::ImageParams image_params;
    image_params.readFromXMLFile(argv[3]);
    image_params.resize(im1.size());
    //select the aruco parameters
    aruco::MarkerDetector::Params md_params;
    md_params._thresParam1_range=5;//enable multirange
    md_params._thresParam1=8;



    //select the feature detector
    cv::Ptr<cv::Feature2D> fdetector;
    if (stoi(cml("-d","0"))==0){
        auto orb=cv::ORB::create();
        orb->setMaxFeatures(2000);
        fdetector=orb;
    }
    else if (stoi(cml("-d","0"))==1){
        fdetector=cv:: AKAZE::create();
    }
    else if (stoi(cml("-d","0"))==2){
        fdetector= new ucoslam::ORBextractor(2000,1.2,8,20,7);
    }

    //create the frame extractor
    ucoslam::FrameExtractor fextractor(image_params,fdetector,md_params,aruco_dict);
    //extract keypoints and markers
    auto frame1=fextractor(im1);
    auto frame2=fextractor(im2);

    ucoslam::Initializer map_initializer;
    map_initializer.setParams(image_params,stof(cml("-size","1")));

    auto matches=ucoslam::match_frames(frame1,frame2);

    auto rt=map_initializer.computeRt(frame1,frame2,matches);//pose from frame 1 to frame 2


    if (!rt.empty()){

        //set poses in frames
        frame1.pose_f2g=cv::Mat::eye(4,4,CV_32F);
        frame2.pose_f2g=rt;//transform moving from Global To Frame2


        //refine matches by reprojection error
        auto matches_filtered=filter_by_reprojection_error(image_params , frame1 ,frame2 ,matches,rt,2);

        //triangulate the filtered matches
        vector<cv::Point3f> p3d;
        triangulate(image_params,frame1,frame2,matches_filtered,p3d,rt);
        //print the reprj error just for the sake of curiosity
        cout<<"rerr="<<reprj_error(image_params,frame1,frame2,matches_filtered,p3d,0,rt)<<endl;

        //save 3d points if requested
        if (cml["-pcd"])  savePointsToPCD(p3d, cml("-pcd"));

        //draw image with matches
        cv::Mat resImg;
        drawMatches(im1,frame1.kpts,im2,frame2.kpts, matches_filtered,resImg);
        cv::imshow("resImg",resImg);

        //save cvba if requested
        if( cml["-cvba"]){
            //add some noise
            std::default_random_engine generator;
            std::normal_distribution<double> distribution(0,1e-2);
            for(auto &p:p3d) p+=cv::Point3f(distribution(generator),distribution(generator),distribution(generator));
            easyBundler(image_params,frame1,frame2,matches_filtered,p3d).saveToFile(cml("-cvba"));
        }
    }

    for(auto m:frame1.markers)m.draw(im1,cv::Scalar(0,255,0));
    for(auto m:frame2.markers)m.draw(im2,cv::Scalar(0,255,0));

    cv::imshow("im1",im1);
    cv::imshow("im2",im2);
    while(cv::waitKey(0)!=27 ) ;


}





cvba::BundlerData<float> easyBundler(const ImageParams &ip, const Frame &f1,const Frame &f2,const std::vector<cv::DMatch> &matches,const vector<cv::Point3f>  &p3d  ){

    cvba::BundlerData<float> Bdata(2,matches.size());
    if (p3d.size()==0){

        triangulate(ip,f1,f2,matches,Bdata.points);
        cerr<<",,,"<<endl;
    }
    else Bdata.points=p3d;

    cout<<Bdata.points.size()<<" "<<matches.size()<<endl;
    for(uint i=0;i<matches.size();i++){
        auto m=matches[i];
        Bdata.imagePoints[ 0][i]=(f1.kpts[m.queryIdx].pt);
        Bdata.imagePoints[ 1][i]=(f2.kpts[m.trainIdx].pt);
    }
    Bdata.R[0]=f1.pose_f2g.getRvec();
    Bdata.T[0]=f1.pose_f2g.getTvec();
    Bdata.R[1]=f2.pose_f2g.getRvec();
    Bdata.T[1]=f2.pose_f2g.getTvec();
    Bdata.cameraMatrix[0]=ip.CameraMatrix.clone();
    Bdata.cameraMatrix[1]=ip.CameraMatrix.clone();
    Bdata.distCoeffs[0]=ip.Distorsion.clone();
    Bdata.distCoeffs[1]=ip.Distorsion.clone();
    Bdata.setFullVisibilityVector();
    return Bdata;
}

