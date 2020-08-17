#include <iostream>
#include "picoflann.h"
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "featureextractors/ORBextractor.h"
#include <chrono>
#include "stuff/timers.h"
#include <fstream>
#include <list>
#include "slam.h"
#include "nanogo.h"
#include "stuff/se3.h"
#include "optimization/sparselevmarq.h"
#include "stuff/utils.h"
#include <Eigen/Geometry>
#include "optimization/globaloptimizer.h"
#include <cmath>
using namespace std;
using namespace ucoslam;

class Point:public nanogo::Variable<double> {
public:
    Point(cv::Point3f p){
        resize(3);
        (*this)(0)=p.x;
        (*this)(1)=p.y;
        (*this)(2)=p.z;
        marginalize=true;
    }
};

class CameraPose:public nanogo::Variable<double> {
public:
    float fx,fy,cx,cy;

    double  rt[12];


    void setParams(ucoslam::Frame &kf){
        needPreparation=true;
        fx=kf.imageParams.fx();
        fy=kf.imageParams.fy();
        cx=kf.imageParams.cx();
        cy=kf.imageParams.cy();
        resize(6);
        ucoslam::se3 pose=kf.pose_f2g;
        for(int i=0;i<6;i++) (*this)(i)=pose(i);
    }
   inline void prepare(){
        const double &rx=(*this)(0);
        const double &ry=(*this)(1);
        const double &rz=(*this)(2);
        const double &tx=(*this)(3);
        const double &ty=(*this)(4);
        const double &tz=(*this)(5);
        double nsqa=rx*rx + ry*ry + rz*rz;
        double a=std::sqrt(nsqa);
        double i_a=a?1./a:0;
        double rnx=rx*i_a;
        double rny=ry*i_a;
        double rnz=rz*i_a;
        double cos_a=std::cos(a);
        double sin_a=std::sin(a);
        double _1_cos_a=1.-cos_a;
        rt[0] =cos_a+rnx*rnx*_1_cos_a;
        rt[1]=rnx*rny*_1_cos_a- rnz*sin_a;
        rt[2]=rny*sin_a + rnx*rnz*_1_cos_a;
        rt[3]=tx;
        rt[4]=rnz*sin_a +rnx*rny*_1_cos_a;
        rt[5]=cos_a+rny*rny*_1_cos_a;
        rt[6]=-rnx*sin_a+ rny*rnz*_1_cos_a;
        rt[7]=ty;
        rt[8]= -rny*sin_a + rnx*rnz*_1_cos_a;
        rt[9]= rnx*sin_a + rny*rnz*_1_cos_a;
        rt[10]=cos_a+rnz*rnz*_1_cos_a;
        rt[11]=tz;
    }

};


//Reprojection error of a point in a camera
class ReprjError:public nanogo::Error<double>{
    cv::Point2f _rprj;
public:
    ReprjError(Point *point,CameraPose*cam,cv::Point2f rprj,double information=1,double hubberDelta=0){
        resize(2);
        at(0)=cam;
        at(1)=point;
        _rprj=rprj;
        setInformation(information);
        setDeltaHubber(hubberDelta);
    }
    //computes the reprojection error
    inline void compute(Eigen::Matrix<double, Eigen::Dynamic, 1> &error){
        CameraPose *cam=(CameraPose *)at(0);
        Point *point=(Point*)at(1);
        Eigen::Vector3d _p3d_incam= mult(*point,cam->rt);//move the point from global to camera
        //compute projection error
        error.resize(2);
        error(0)=_rprj.x- (_p3d_incam(0)/_p3d_incam(2)*cam->fx +cam->cx);
        error(1)=_rprj.y-(_p3d_incam(1)/_p3d_incam(2)*cam->fy +cam->cy);
    }
    inline Eigen::Vector3d mult(Eigen::Matrix<double, Eigen::Dynamic, 1> &pointInGlobal, double *RT){
        Eigen::Vector3d pointInCam;
        pointInCam(0)= RT[0]*pointInGlobal(0)+RT[1]*pointInGlobal(1)+RT[2]*pointInGlobal(2)+RT[3];
        pointInCam(1)= RT[4]*pointInGlobal(0)+RT[5]*pointInGlobal(1)+RT[6]*pointInGlobal(2)+RT[7];
        pointInCam(2)= RT[8]*pointInGlobal(0)+RT[9]*pointInGlobal(1)+RT[10]*pointInGlobal(2)+RT[11];
        return pointInCam;
    }

    inline void derv(uint32_t var,Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> &dervMatrix ){
        CameraPose *cam=(CameraPose *)at(0);
        Point *point=(Point*)at(1);
        Eigen::Vector3d xyz_trans= mult(*point,cam->rt);//move the point from global to camera

//        g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
//      g2o::SE3Quat T(vj->estimate());
//      g2o::VertexSBAPointXYZ* vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
//      Eigen::Vector3d xyz = vi->estimate();
//      Eigen::Vector3d xyz_trans = T.map(xyz);

      const double &x = xyz_trans[0];
      const double & y = xyz_trans[1];
      const double & z = xyz_trans[2];
      const double z_2 = z*z;

        if (var==0){//Derv Error wrt camera

            dervMatrix.resize(2,6);

            dervMatrix(0,0) =  x*y/z_2 *cam->fx;
            dervMatrix(0,1) = -(1+(x*x/z_2)) *cam->fx;
            dervMatrix(0,2) = y/z *cam->fx;
            dervMatrix(0,3) = -1./z *cam->fx;
            dervMatrix(0,4) = 0;
            dervMatrix(0,5) = x/z_2 *cam->fx;

            dervMatrix(1,0) = (1+y*y/z_2) *cam->fy;
            dervMatrix(1,1) = -x*y/z_2 *cam->fy;
            dervMatrix(1,2) = -x/z *cam->fy;
            dervMatrix(1,3) = 0;
            dervMatrix(1,4) = -1./z *cam->fy;
            dervMatrix(1,5) = y/z_2 *cam->fy;


        }
        else{
            Eigen::Matrix<double,2,3> tmp;
            tmp(0,0) = cam->fx;
            tmp(0,1) = 0;
            tmp(0,2) = -x/z*cam->fx;

            tmp(1,0) = 0;
            tmp(1,1) = cam->fy;
            tmp(1,2) = -y/z*cam->fy;
            Eigen::Matrix<double,3,3> R;
            R(0,0)=cam->rt[0];
            R(0,1)=cam->rt[1];
            R(0,2)=cam->rt[2];
            R(1,0)=cam->rt[4];
            R(1,1)=cam->rt[5];
            R(1,2)=cam->rt[6];
            R(2,0)=cam->rt[8];
            R(2,1)=cam->rt[9];
            R(2,2)=cam->rt[10];

            dervMatrix =  (-1./z * tmp *R) ;

        }

    }

};




void ngo_globalOptimization(std::shared_ptr<ucoslam::Map> TheMap,int niters ){

    nanogo::Graph<double> graph;

    //first, the cameras
    map<uint32_t, CameraPose > camera_map;
    for(auto &kf:TheMap->keyframes){
        if (kf.isBad)continue;
         camera_map[kf.idx].setParams(kf);
        graph.add(&camera_map[kf.idx]);
    }
    //now points and projections
    vector<Point> points;
    points.reserve(TheMap->map_points.size());

    list<ReprjError> errors;
    for(auto &p:TheMap->map_points){
        if (p.isBad)continue;
        points.push_back(Point(p.getCoordinates()));
        graph.add(&points.back());
        for(auto of:p.getObservingFrames()){
            assert(camera_map.count(of.first)!=0);
            auto &kf=TheMap->keyframes[of.first];
            if ( kf.isBad )continue;
            errors.push_back( ReprjError(&points.back(),&camera_map[kf.idx], kf.und_kpts[of.second].pt,
                    kf.scaleFactors[kf.und_kpts[of.second].octave],sqrt(5.99)));
            graph.add(&errors.back());
        }
    }

    nanogo::SparseGraphSolver<double> SPGSolver;
    nanogo::SparseGraphSolver<double>::Params params;
    params.verbose=true;
    SPGSolver.solve(graph,params);

}

void   globalOptimization(std::shared_ptr<ucoslam::Map> TheMap,int niters ){
     GlobalOptimizer::ParamSet params( debug::Debug::getLevel()>=5);
    //must set the first and second as fixed?
    params.fixFirstFrame=true;
    params.nIters=niters;

    params.verbose=true;

    if (params.fixed_frames.size()==0 && TheMap->map_markers.size()==0){//fixed second one to keep the scale if no markers
        //get the second frame if there is one and set it fixed
        auto it=TheMap->keyframes.begin();
        params.fixed_frames.insert(it->idx);
        ++it;
        if (it!=TheMap->keyframes.end()) {
            if(params.used_frames.count(it->idx) || params.used_frames.size()==0 )
                params.fixed_frames.insert(it->idx);
        }
    }

    auto Gopt=GlobalOptimizer::create(Slam::getParams().global_optimizer);
    Gopt->setParams( TheMap,params);
    Gopt->optimize();
}

int main(int argc,char **argv){

    std::shared_ptr<ucoslam::Map> map=std::make_shared<ucoslam::Map>();
    map->readFromFile(argv[1]);
     ngo_globalOptimization(map,10);
  // globalOptimization(map,10);


}

//computes the inverse SE3 transform given in rodrigues + translation form
void inverseTransform(cv::Mat &rvec_io,cv::Mat &tvec_io){
    //convert to double
    cv::Mat rv64,tv64;
    rvec_io.convertTo(rv64,CV_64F);
    tvec_io.convertTo(tvec_io,CV_64F);
    //create the 4x4 transform matrix
    double rt_44[16];

    double rx=rv64.ptr<float>(0)[0];
    double ry=rv64.ptr<float>(0)[1];
    double rz=rv64.ptr<float>(0)[2];
    double tx=tv64.ptr<float>(0)[0];
    double ty=tv64.ptr<float>(0)[1];
    double tz=tv64.ptr<float>(0)[2];
    double nsqa=rx*rx + ry*ry + rz*rz;
    double a=std::sqrt(nsqa);
    double i_a=a?1./a:0;
    double rnx=rx*i_a;
    double rny=ry*i_a;
    double rnz=rz*i_a;
    double cos_a=cos(a);
    double sin_a=sin(a);
    double _1_cos_a=1.-cos_a;
    rt_44[0] =cos_a+rnx*rnx*_1_cos_a;
    rt_44[1]=rnx*rny*_1_cos_a- rnz*sin_a;
    rt_44[2]=rny*sin_a + rnx*rnz*_1_cos_a;
    rt_44[3]=tx;
    rt_44[4]=rnz*sin_a +rnx*rny*_1_cos_a;
    rt_44[5]=cos_a+rny*rny*_1_cos_a;
    rt_44[6]= -rnx*sin_a+ rny*rnz*_1_cos_a;
    rt_44[7]=ty;
    rt_44[8]= -rny*sin_a + rnx*rnz*_1_cos_a;
    rt_44[9]= rnx*sin_a + rny*rnz*_1_cos_a;
    rt_44[10]=cos_a+rnz*rnz*_1_cos_a;
    rt_44[11]=tz;
    rt_44[12]=rt_44[13]=rt_44[14]=0;
    rt_44[15]=1;


    //obtain the inverse
    cv::Mat   cMinv(4,4,CV_32F);
    double *Minv=cMinv.ptr<double>(0);
    Minv[0]=rt_44[0];
    Minv[1]=rt_44[4];
    Minv[2]=rt_44[8];
    Minv[4]=rt_44[1];
    Minv[5]=rt_44[5];
    Minv[6]=rt_44[9];
    Minv[8]=rt_44[2];
    Minv[9]=rt_44[6];
    Minv[10]=rt_44[10];

    Minv[3] = -  ( rt_44[3]*Minv[0]+rt_44[7]*Minv[1]+rt_44[11]*Minv[2]);
    Minv[7] = -  ( rt_44[3]*Minv[4]+rt_44[7]*Minv[5]+rt_44[11]*Minv[6]);
    Minv[11]= -  ( rt_44[3]*Minv[8]+rt_44[7]*Minv[9]+rt_44[11]*Minv[10]);
    Minv[12]=Minv[13]=Minv[14]=0;
    Minv[15]=1;
    //convert back to rodrigues notation

    //extract the rotation part
    cv::Mat r33=cv::Mat ( cMinv,cv::Rect ( 0,0,3,3 ) );
    cv::Rodrigues ( r33,rv64 );
    //and translation
    for ( int i=0; i<3; i++ )
         tv64.ptr<double> ( 0 ) [i]=cMinv.at<double> ( i,3 );

    //copy back to the input data type (float or double)
    rv64.convertTo(rvec_io,rvec_io.type());
    tv64.convertTo(tvec_io,rvec_io.type());


}


//vector<cv::Point3f> cv_Points;
//vector<cv::Point2f> cv_PointProjections;
//vector<ucoslam::se3> cameraPoses;
//cv::Mat cameraMatrix(3,3,CV_32F);
//std::map<uint64_t,cv::Point2f> pointProjection;

//uint64_t join(uint32_t a ,uint32_t b){
//    if( a>b)swap(a,b);
//    uint64_t a_b;
//    uint32_t *_a_b_16=(uint32_t*)&a_b;
//    _a_b_16[0]=b;
//    _a_b_16[1]=a;
//    return a_b;
//}


//void optimize_SLM(){


//    auto poseEstimationError=[&](const SparseLevMarq<double>::eVector &solution,SparseLevMarq<double>::eVector &errV){
//        //get the camera matrix
//        vector<cv::Point3f> points(cv_Points.size());
//        vector<ucoslam::se3> poses(cameraPoses.size());
//        int cur=0;
//        for(size_t i=0;i<cv_Points.size();i++){
//            points[i].x=solution(cur++);
//            points[i].y=solution(cur++);
//            points[i].z=solution(cur++);
//        }
//        for(size_t i=0;i<cameraPoses.size();i++){
//            for(int j=0;j<6;j++)
//                poses[i][j]=solution(cur++);
//        }

//        errV.resize(cv_Points.size()*points.size()*2);
//        cur=0;

//        for(size_t i=0;i<points.size();i++)
//            for(size_t j=0;j<poses.size();j++){
//                cv::Point2f pp=ucoslam::project(points[i], cameraMatrix,poses[j].convert());
//                auto    err=pp-pointProjection[join(i,j)];
//                errV(cur++)=err.x;
//                errV(cur++)=err.y;
//            }
//    };

//    SparseLevMarq<double> SLM;
//    SparseLevMarq<double>::eVector solution(cv_Points.size()*3+cameraPoses.size()*6);
//    int cur=0;
//    for(size_t i=0;i<cv_Points.size();i++){
//        solution(cur++)=cv_Points[i].x;
//        solution(cur++)=cv_Points[i].y;
//        solution(cur++)=cv_Points[i].z;
//    }
//    for(size_t i=0;i<cameraPoses.size();i++){
//        for(int j=0;j<6;j++)
//            solution(cur++)=cameraPoses[i][j];
//    }


//    SparseLevMarq<double>::Params params;;
//    params.verbose=true;

//    SparseLevMarq<double>::eVector error;
//    poseEstimationError(solution,error);
//    cout<<error<<endl;
//    SLM.setParams(params);
//    SLM.solve(solution,std::bind( poseEstimationError,std::placeholders::_1,std::placeholders::_2) );

//    poseEstimationError(solution,error);
//    cout<<"Final solution="<<solution<<endl;

//}


//void fillData()
//{
//    cameraMatrix.at<float>(0,0)=cameraMatrix.at<float>(1,1)=500;
//    cameraMatrix.at<float>(0,2)=320;
//    cameraMatrix.at<float>(1,2)=240;
//    //set the camera locations
//    cameraPoses.clear();
//    cameraPoses.push_back(se3(0,0,0,0,0,0));
//    cameraPoses.push_back(se3(0,0,0,0.1,0.2,0.4));
//    cameraPoses.push_back(se3(0.06,0.02,0.001,0.1,0.2,0.4));

//    cv_Points.clear();
//    cv_Points.push_back(cv::Point3f(1,1,1));
//    cv_Points.push_back(cv::Point3f(1,1,4));
//    cv_Points.push_back(cv::Point3f(0,1,2));
//    //now, project all points in all cameras
//    for(size_t p=0;p<cv_Points.size();p++)
//        for(size_t c=0;c<cameraPoses.size();c++){
//            auto prj=ucoslam::project(cv_Points[p], cameraMatrix,cameraPoses[c].convert());;
//            pointProjection[ join(p,c)]=prj;
//        }



//    //now, lets add some noise
//    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
//     std::default_random_engine generator (seed);

//     std::normal_distribution<double> distribution (0.0,0.04);
//     for(auto &p:cv_Points) {
//         p.x+= 0.02;//distribution(generator);
//         p.y+= 0.014;//distribution(generator);
//         p.z+= 0.02;//distribution(generator);
//     }
//}




//int main(int argc,char **argv){
//    try{

//        fillData();
//        optimize_SLM();
//        cout<<endl<<endl;


//        fillData();
//        std::vector<CameraPose> ngo_cameraPoses(cameraPoses.size());

//        for(size_t c=0;c<ngo_cameraPoses.size();c++){
//            ngo_cameraPoses[c].resize(6);
//            for(int i=0;i<6;i++) ngo_cameraPoses[c](i)=cameraPoses[c][i];
//            ngo_cameraPoses[c].fx=cameraMatrix.at<float>(0,0);
//            ngo_cameraPoses[c].fy=cameraMatrix.at<float>(1,1);
//            ngo_cameraPoses[c].cx=cameraMatrix.at<float>(0,2);
//            ngo_cameraPoses[c].cy=cameraMatrix.at<float>(1,2);
//        }
//        std::vector<nanogo::Variable<double> > ngo_points(cv_Points.size());
//        for(size_t p=0;p<ngo_points.size();p++){
//              ngo_points[p].resize(3);
//              ngo_points[p](0)=cv_Points[p].x;
//              ngo_points[p](1)=cv_Points[p].y;
//              ngo_points[p](2)=cv_Points[p].z;
//        }

//        std::vector<ReprjError> ngo_repjErrors;
//        for(size_t p=0;p<ngo_points.size();p++)
//            for(size_t c=0;c<ngo_cameraPoses.size();c++)
//                   ngo_repjErrors.push_back( ReprjError(&ngo_points[p],&ngo_cameraPoses[c], pointProjection[ join(p,c)]));

//        nanogo::Graph<double> G;
//        for(auto &var:ngo_cameraPoses) G.add(&var);
//        for(auto &var:ngo_points) G.add(&var);
//        for(auto &err:ngo_repjErrors) G.add(&err);
//        nanogo::SparseGraphSolver<double> GraphSolver;
//        GraphSolver.solve(G);


//        cout<<"Points"<<endl;
//        for(auto &var:ngo_points)  cout<<var<<endl;
//        cout<<"Cameras"<<endl;
//        for(auto &var:ngo_cameraPoses)  cout<<var<<endl;


//    }catch(std::exception &ex){
//        cerr<<ex.what()<<endl;
//    }


//}
