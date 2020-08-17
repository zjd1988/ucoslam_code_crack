#include <iostream>
#include "map.h"
#include "nanogo.h"
#include <cmath>
using namespace std;
using namespace ucoslam;

//class MapPoint3d:public nanogo::Variable<double> {
//public:
//    MapPoint3d(cv::Point3d p){
//        resize(3);
//        (*this)(0)=p.x;
//        (*this)(1)=p.y;
//        (*this)(2)=p.z;
//        marginalize=true;
//    }
//};

//class Camera:public nanogo::Variable<double> {
//public:
//    double  rt[12];


//    void setParams(ucoslam::Frame &kf){
//        //needPreparation=true;
//        resize(6+4+5);
//        ucoslam::se3 pose=kf.pose_f2g;
//        for(int i=0;i<6;i++) (*this)(i)=pose(i);
//        (*this)(6)=kf.imageParams.fx();
//        (*this)(7)=kf.imageParams.fy();
//        (*this)(8)=kf.imageParams.cx();
//        (*this)(9)=kf.imageParams.cy();
//        for(int i=10;i<15;i++) (*this)(i)=kf.imageParams.Distorsion.ptr<float>(0)[i-10];
//    }

//    inline double fx(){return (*this)(6);}
//    inline double fy(){return (*this)(7);}
//    inline double cx(){return (*this)(8);}
//    inline double cy(){return (*this)(9);}

//   inline void prepare(){
//        const double &rx=(*this)(0);
//        const double &ry=(*this)(1);
//        const double &rz=(*this)(2);
//        const double &tx=(*this)(3);
//        const double &ty=(*this)(4);
//        const double &tz=(*this)(5);
//        double nsqa=rx*rx + ry*ry + rz*rz;
//        double a=std::sqrt(nsqa);
//        double i_a=a?1./a:0;
//        double rnx=rx*i_a;
//        double rny=ry*i_a;
//        double rnz=rz*i_a;
//        double cos_a=std::cos(a);
//        double sin_a=std::sin(a);
//        double _1_cos_a=1.-cos_a;
//        rt[0] =cos_a+rnx*rnx*_1_cos_a;
//        rt[1]=rnx*rny*_1_cos_a- rnz*sin_a;
//        rt[2]=rny*sin_a + rnx*rnz*_1_cos_a;
//        rt[3]=tx;
//        rt[4]=rnz*sin_a +rnx*rny*_1_cos_a;
//        rt[5]=cos_a+rny*rny*_1_cos_a;
//        rt[6]=-rnx*sin_a+ rny*rnz*_1_cos_a;
//        rt[7]=ty;
//        rt[8]= -rny*sin_a + rnx*rnz*_1_cos_a;
//        rt[9]= rnx*sin_a + rny*rnz*_1_cos_a;
//        rt[10]=cos_a+rnz*rnz*_1_cos_a;
//        rt[11]=tz;
//    }

//};


////Reprojection error of a point in a camera
//class ReprjError:public nanogo::Error<double>{
//    cv::Point2f _rprj;
//public:
//    cv::Mat CamMatrix,DistCoeffs,Rvec,Tvec;

//    ReprjError(MapPoint3d *point,Camera*cam,cv::Point2f rprj,double information=1,double hubberDelta=0){
//        resize(2);
//        at(0)=cam;
//        at(1)=point;
//        _rprj=rprj;
//        setInformation(information);
//        setDeltaHubber(hubberDelta);
//        CamMatrix=cv::Mat::ones(3,3,CV_64F);
//        DistCoeffs=cv::Mat::zeros(1,5,CV_64F);
//        Rvec=cv::Mat::zeros(1,3,CV_64F);
//        Tvec=cv::Mat::zeros(1,3,CV_64F);
//    }
//    //computes the reprojection error
//    inline void compute(Eigen::Matrix<double, Eigen::Dynamic, 1> &error){
//        Camera *cam=(Camera *)at(0);
//        MapPoint3d *point=(MapPoint3d*)at(1);
////        Eigen::Vector3d _p3d_incam= mult(*point,cam->rt);//move the point from global to camera
//        //now, distortion
//        for(int i=0;i<3;i++) Rvec.ptr<double>(0)[i]=(*cam)(i);
//        for(int i=0;i<3;i++) Tvec.ptr<double>(0)[i]=(*cam)(i+3);
//        CamMatrix.at<double>(0,0)=cam->fx();
//        CamMatrix.at<double>(1,1)=cam->fy();
//        CamMatrix.at<double>(0,2)=cam->cx();
//        CamMatrix.at<double>(1,2)=cam->cy();
//        for(int i=0;i<5;i++) DistCoeffs.ptr<double>(0)[i]=(*cam)(10+i);

//        //compute projection error
//        std::vector<cv::Point3d> p3d(1);
//        std::vector<cv::Point2f> p2d(1);
//        p3d[0].x=(*point)(0);
//        p3d[0].y=(*point)(1);
//        p3d[0].z=(*point)(2);
//        cv::projectPoints(p3d,Rvec,Tvec,CamMatrix,DistCoeffs,p2d);
//        error.resize(2);
//        error(0)=_rprj.x-p2d[0].x;
//        error(1)=_rprj.y-p2d[0].y;
//    }

//    inline Eigen::Vector3d mult(Eigen::Matrix<double, Eigen::Dynamic, 1> &pointInGlobal, double *RT){
//        Eigen::Vector3d pointInCam;
//        pointInCam(0)= RT[0]*pointInGlobal(0)+RT[1]*pointInGlobal(1)+RT[2]*pointInGlobal(2)+RT[3];
//        pointInCam(1)= RT[4]*pointInGlobal(0)+RT[5]*pointInGlobal(1)+RT[6]*pointInGlobal(2)+RT[7];
//        pointInCam(2)= RT[8]*pointInGlobal(0)+RT[9]*pointInGlobal(1)+RT[10]*pointInGlobal(2)+RT[11];
//        return pointInCam;
//    }

//};

//void ngo_globalOptimization(std::shared_ptr<ucoslam::Map> TheMap,int niters ){

//    nanogo::Graph<double> graph;

//    //first, the cameras
//    map<uint32_t, Camera > camera_map;
//    for(auto &kf:TheMap->keyframes){
//        if (kf.isBad())continue;
//         camera_map[kf.idx].setParams(kf);
//        graph.add(&camera_map[kf.idx]);
//    }
//    //now points and projections
//    vector<MapPoint3d> points;
//    points.reserve(TheMap->map_points.size());

//    list<ReprjError> errors;
//    for(auto &p:TheMap->map_points){
//        if (p.isBad())continue;
//        points.push_back(MapPoint3d(p.getCoordinates()));
//        graph.add(&points.back());
//        for(auto of:p.getObservingFrames()){
//            assert(camera_map.count(of.first)!=0);
//            auto &kf=TheMap->keyframes[of.first];
//            if ( kf.isBad() )continue;
//            errors.push_back( ReprjError(&points.back(),&camera_map[kf.idx], kf.kpts[of.second],
//                    kf.scaleFactors[kf.und_kpts[of.second].octave],sqrt(5.99)));
//            graph.add(&errors.back());
//        }
//    }

//    nanogo::SparseGraphSolver<double> SPGSolver;
//    nanogo::SparseGraphSolver<double>::Params params;
//    params.verbose=true;
//    SPGSolver.solve(graph,params);

//}


int main(int argc,char **argv){

//    std::shared_ptr<ucoslam::Map> map=std::make_shared<ucoslam::Map>();
//    map->readFromFile(argv[1]);
//     ngo_globalOptimization(map,10);
//  // globalOptimization(map,10);


}
