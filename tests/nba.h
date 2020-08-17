#ifndef NanoBundleAdjustment
#define NanoBundleAdjustment
#include "nanogo.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "cvprojectpoint.h"
//nano bundle adjustment
namespace nba {

class MPoint:public nanogo::Variable<double> {
public:
    MPoint(){}
    MPoint(cv::Point3d p){
        resize(3);
        (*this)(0)=p.x;
        (*this)(1)=p.y;
        (*this)(2)=p.z;
     }
   inline cv::Point3d getPoint3d()const{return cv::Point3d((*this)(0),(*this)(1),(*this)(2));}
};

class CameraIntrinsics:public nanogo::Variable<double> {
    bool _isSet=false;
public:
    bool isSet()const{return _isSet;}
    void setParams(const cv::Mat &CameraMatrix,const cv::Mat &Dist  ){
        resize(4+5);
        cv::Mat Cam64,Dist64;
        CameraMatrix.convertTo(Cam64,CV_64F);
        Dist.convertTo(Dist64,CV_64F);
        (*this)(0)=Cam64.at<double>(0,0);//fx
        (*this)(1)=Cam64.at<double>(1,1);//fy
        (*this)(2)=Cam64.at<double>(0,2);//cx
        (*this)(3)=Cam64.at<double>(1,2);//cy
        for(int i=0;i<5;i++) (*this)(4+i)=Dist64.ptr<double>(0)[i];//distortion coeffs
        _isSet=true;
    }
    inline double fx()const{return (*this)(0);}
    inline double fy()const{return (*this)(1);}
    inline double cx()const{return (*this)(2);}
    inline double cy()const{return (*this)(3);}
    inline const double * dist()const{return &(*this)(4);}
};

class SE3Pose:public nanogo::Variable<double> {
public:
    cv::Mat R,derR;
    void setParams( const cv::Mat &rvec,const cv::Mat &tvec){
        needPreparation=true;
        cv::Mat R64,T64;
        rvec.convertTo(R64,CV_64F);
        tvec.convertTo(T64,CV_64F);
        resize(6);
        for(int i=0;i<3;i++) (*this)(i)=R64.ptr<double>(0)[i];
        for(int i=0;i<3;i++) (*this)(3+i)=T64.ptr<double>(0)[i];
    }
    double *getRvec(){return &(*this)(0);}
    double *getTvec(){return &(*this)(3);}
    double *getRMat(){return R.ptr<double>(0);}
    double *getDervMat(){return derR.ptr<double>(0);}

    inline void prepare(){
        cv::Mat rvec(1,3,CV_64F, &(*this)(0));
        cv::Rodrigues(rvec,R,derR);
    }
};

//Reprojection error of a point in a camera
class ReprjError:public nanogo::Error<double>{
    cv::Point2f _rprj;
    double dervs[2*18];//First row,  horizontal pixel error: rx,ry,rz,tx,ty,tx,fx,fy,cx,cy,k1,k2,p1,p2,k3,X,Y,Z Second row: same for vertical pixel error
 public:

    ReprjError(MPoint *point,CameraIntrinsics*cam,SE3Pose *pos ,cv::Point2f rprj,double information=1,double hubberDelta=0){
        resize(3);
        at(0)=point;
        at(1)=cam;
        at(2)=pos;
        _rprj=rprj;
        setInformation(information);
        setDeltaHubber(hubberDelta);
    }
    //computes the reprojection error
    inline void compute(Eigen::Matrix<double, Eigen::Dynamic, 1> &error){
        MPoint &point=*(MPoint*)at(0);
        CameraIntrinsics &cam=*(CameraIntrinsics *)at(1);
        SE3Pose &pos=*(SE3Pose *)at(2);
        //compute projection error
        auto proj=__projectPoint(point.getPoint3d(), pos.getRvec(),pos.getTvec(), cam.fx(),cam.fy(),cam.cx(),cam.cy(),cam.dist(),dervs,pos.getRMat(),pos.getDervMat());
         error.resize(2);
        error(0)=proj.x-_rprj.x;
        error(1)=proj.y-_rprj.y;
    }


    virtual inline void derv(uint32_t var,Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> &dervMatrix ){
        if(var==0){//Point Derivatives
            dervMatrix.resize(2,3);
            for(int r=0;r<2;r++){
                for(int c=0;c<3;c++){
                    dervMatrix(r,c)=dervs[r*18+15+c];
                }
            }

        } if(var==1){//Camera derivatives
            dervMatrix.resize(2,9);
            for(int r=0;r<2;r++)
                for(int c=0;c<9;c++) dervMatrix(r,c)=dervs[ r*18+ 6+c];
        }
        if(var==2){//Pose derivatives
            dervMatrix.resize(2,6);
             for(int r=0;r<2;r++)
                 for(int c=0;c<6;c++) dervMatrix(r,c)=dervs[ r*18+ c];
        }
    }

};
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#endif
