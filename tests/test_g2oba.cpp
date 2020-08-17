#include <eigen3/Eigen/Core>
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include <iostream>
#include <exception>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <type_traits>
#include "cvprojectpoint.h"
#include "ucoslam.h"
class CmdLineParser{int argc; char **argv;
                public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                    std::vector<std::string> getAllInstances(string str){
                        std::vector<std::string> ret;
                        for(int i=0;i<argc-1;i++){
                            if (string(argv[i])==str)
                                ret.push_back(argv[i+1]);
                        }
                        return ret;
                    }
                   };
namespace g2oba {


typedef Eigen::Matrix<double,9,1,Eigen::ColMajor>    Vector9D;
typedef Eigen::Matrix<double,6,1,Eigen::ColMajor>    Vector6D;


class   CameraParams : public g2o::BaseVertex<9, Vector9D>
{
    bool _isSet=false;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setParams(const cv::Mat &CameraMatrix,const cv::Mat &Dist  ){
      cv::Mat Cam64,Dist64;
      CameraMatrix.convertTo(Cam64,CV_64F);
      Dist.convertTo(Dist64,CV_64F);
      Eigen::Matrix<double,9,1> obs;
      obs(0)=Cam64.at<double>(0,0);//fx
      obs(1)=Cam64.at<double>(1,1);//fy
      obs(2)=Cam64.at<double>(0,2);//cx
      obs(3)=Cam64.at<double>(1,2);//cy
      for(int i=0;i<5;i++) obs(4+i)=Dist64.ptr<double>(0)[i];//distortion coeffs
      setEstimate(obs);
      _isSet=true;

   }
  bool isSet()const{return _isSet;}

   virtual void setToOriginImpl() {
    _estimate.fill(0);
  }
  virtual void oplusImpl(const number_t* update)
  {
    Eigen::Map<const Vector9D> v(update);
    _estimate += v;
  }
  inline double fx()const{return _estimate(0);}
  inline double fy()const{return _estimate(1);}
  inline double cx()const{return _estimate(2);}
  inline double cy()const{return _estimate(3);}
  inline const double * dist()const{return &_estimate(4);}

  bool read(std::istream& is){throw std::runtime_error("Not implemented");}
  bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}

};

/**
 * \brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map
 */
class   CameraPose : public g2o::BaseVertex<6, Vector6D>{
    cv::Mat R,derR;
    uint32_t _id;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    bool read(std::istream& is){throw std::runtime_error("Not implemented");}
    bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}

    virtual void setToOriginImpl() {
        _estimate.fill(0);
    }

    virtual void oplusImpl(const number_t* update)
    {
        Eigen::Map<const Vector6D> v(update);
        _estimate += v;
         prepare();
    }
    CameraPose(uint32_t id, const cv::Mat &rvec,const cv::Mat &tvec):g2o::BaseVertex<6, Vector6D>(){
        _id=id;
        cv::Mat R64,T64;
        rvec.convertTo(R64,CV_64F);
        tvec.convertTo(T64,CV_64F);
        Eigen::Matrix<double,6,1> obs;
        for(int i=0;i<3;i++) obs(i)=R64.ptr<double>(0)[i];
        for(int i=0;i<3;i++) obs(3+i)=T64.ptr<double>(0)[i];
        setEstimate(obs);
        prepare();
    }
    uint32_t getid()const{return _id;}
      inline void prepare(){
          if(sizeof(_estimate(0))!=sizeof(double)) throw  std::runtime_error("MUST BE DOUBLE");
          cv::Mat rvec(1,3,CV_64F, &_estimate(0));
          cv::Rodrigues(rvec,R,derR);
      }

      inline   double operator()(const int pos)const{return _estimate(pos);}
    inline const double * getRvec()const{return &_estimate(0);}
    inline const double * getTvec()const{return &_estimate(3);}

    const double *getRMat()const{return R.ptr<double>(0);}
    const double *getDervMat()const{return derR.ptr<double>(0);}
};

/**
 * \brief Point vertex, XYZ
 */
 class   MapPoint : public g2o::BaseVertex<3, g2o::Vector3>
{
     uint32_t _id;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      MapPoint(uint32_t id,const cv::Point3d &p):g2o::BaseVertex<3, g2o::Vector3>(){
        Eigen::Matrix<double,3,1> obs;
        obs<<p.x,p.y,p.z;
        _id=id;
        setEstimate(obs);
    }
    uint32_t getid()const{return _id;}
    virtual void oplusImpl(const number_t* update)
    {
      Eigen::Map<const g2o::Vector3> v(update);
      _estimate += v;
    }
    virtual void setToOriginImpl() {
      _estimate.fill(0);
    }
    cv::Point3d getPoint3d()const{
        return cv::Point3d(_estimate(0),_estimate(1),_estimate(2));
    }
    bool read(std::istream& is){throw std::runtime_error("Not implemented");}
    bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}

 };


 class  ProjectionEdge: public  g2o::BaseMultiEdge<2, g2o::Vector2>
 {

      double dervs[2*18];//First row,  horizontal pixel error: rx,ry,rz,tx,ty,tx,fx,fy,cx,cy,k1,k2,p1,p2,k3,X,Y,Z Second row: same for vertical pixel error
     uint32_t point_id,frame_id;
 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW


   ProjectionEdge(uint32_t pointid,uint32_t frameid,cv::Point2d Proj){
       resize(3);
       point_id=pointid;
       frame_id=frameid;
    }

   inline void computeError()  {
       const  MapPoint* point= static_cast<const  g2oba::MapPoint *>(_vertices[0]);//
       const  CameraParams* cam= static_cast<const  g2oba::CameraParams*>(_vertices[1]);//
       const  CameraPose* pos= static_cast<const  g2oba::CameraPose*>(_vertices[2]);//


//       auto proj=__projectPoint(point->getPoint3d(), pos->getRvec(),pos->getTvec(), cam->fx(),cam->fy(),cam->cx(),cam->cy(),cam->dist()
                                //,dervs,pos->getRMat(),pos->getDervMat()
//                                );


       auto proj=__projectPoint(point->getPoint3d(), pos->getRvec(),pos->getTvec(), cam->fx(),cam->fy(),cam->cx(),cam->cy(),cam->dist()
                                ,dervs,pos->getRMat(),pos->getDervMat());
       _error.resize(2);
       _error(0)=proj.x-_measurement(0);
       _error(1)=proj.y-_measurement(1);
   }
      virtual void linearizeOplus(){



               {//Point Derivatives
                    for(int r=0;r<2;r++){
                       for(int c=0;c<3;c++){
                            _jacobianOplus[0](r,c)=dervs[r*18+15+c];
                       }
                   }

               }
                {//Camera derivatives
                   for(int r=0;r<2;r++)
                       for(int c=0;c<9;c++) _jacobianOplus[1](r,c)=dervs[ r*18+ 6+c];
               }
               {//Pose derivatives
                    for(int r=0;r<2;r++)
                        for(int c=0;c<6;c++) _jacobianOplus[2](r,c)=dervs[ r*18+ c];
               }
      }

   virtual bool read(std::istream& is){throw std::runtime_error("Not implemented");}
   virtual bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}
 };
 }

int main(int argc,char **argv){

    try {
        if(argc<3)throw std::runtime_error("Usage: inmap outmap [iterations]");

        ucoslam::Map TheMap;
        cout<<"reading map"<<endl;
        TheMap.readFromFile(argv[1]);
        cout<<"Done"<<endl;
        int niters=50;
        if(argc>=4)niters=stoi(argv[3]);

        std::shared_ptr<g2o::SparseOptimizer> Optimizer;

        Optimizer=std::make_shared<g2o::SparseOptimizer>();
        std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver=g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));


        Optimizer->setAlgorithm(solver);

        int id=0;
        //first, the cameras
        g2oba::CameraParams * camera = new g2oba::CameraParams();
        camera->setId(id++);

        map<uint32_t, g2oba::CameraPose * > camera_poses;

        for(auto &kf:TheMap.keyframes){
            if(!camera->isSet()){
                camera->setParams(kf.imageParams.CameraMatrix,kf.imageParams.Distorsion);
                Optimizer->addVertex(camera);
            }
            g2oba::CameraPose * pose= new g2oba::CameraPose(kf.idx, kf.pose_f2g.getRvec(),kf.pose_f2g.getTvec());
            pose->setId(id++);
            Optimizer->addVertex(pose);
            camera_poses.insert({kf.idx,pose});
        }

        list<g2oba::ProjectionEdge *> projectionsInGraph;
        list<g2oba::MapPoint *> mapPoints;
        for(auto &p:TheMap.map_points){
             g2oba::MapPoint *point=new g2oba::MapPoint (p.id, p.getCoordinates());
             point->setId(id++);
             point->setMarginalized(true);
             Optimizer->addVertex(point);
             mapPoints.push_back(point);

            for(auto of:p.getObservingFrames()){
                auto &kf=TheMap.keyframes[of.first];
                if ( kf.isBad() )continue;

                auto Proj=new g2oba::ProjectionEdge(p.id,kf.idx, kf.kpts[of.second]);
                Proj->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(point));
                Proj->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( camera));
                Proj->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>( camera_poses.at(kf.idx)));
                Eigen::Matrix<double,2,1> obs;
                obs<<kf.kpts[of.second].x,kf.kpts[of.second].y;
                Proj->setMeasurement(obs);
                Proj->setInformation(Eigen::Matrix2d::Identity()* 1./ kf.scaleFactors[kf.und_kpts[of.second].octave]);
             //   Proj->computeError();
                g2o::RobustKernelHuber* rk = new  g2o::RobustKernelHuber();
                rk->setDelta(sqrt(5.99));
                Proj->setRobustKernel(rk);
                Optimizer->addEdge(Proj);
                projectionsInGraph.push_back(Proj);



             }
        }

     Optimizer->initializeOptimization();
//    Optimizer->setForceStopFlag( );
    Optimizer->setVerbose(true);
    Optimizer->optimize(niters,0.001);
    //now remove outliers
    for(auto &p:projectionsInGraph){
        if(p->chi2()>5.99) p->setLevel(1);
         p->setRobustKernel(0);
    }
    Optimizer->optimize(niters,0.001);

    //copy data back to the map
    //move data back to the map
    int ip=0;
    for(auto &mp: mapPoints ){
        TheMap.map_points[ mp->getid()].setCoordinates(mp->getPoint3d());
    }

    //now, the cameras
    for(auto pose:camera_poses){
        TheMap.keyframes[pose.first].pose_f2g=ucoslam::se3((*pose.second)(0),(*pose.second)(1),(*pose.second)(2),(*pose.second)(3),(*pose.second)(4),(*pose.second)(5));
        TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(0,0)=camera->fx();
        TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(1,1)=camera->fy();
        TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(0,2)=camera->cx();
        TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(1,2)=camera->cy();
        for(int p=0;p<5;p++)
            TheMap.keyframes[pose.first].imageParams.Distorsion.ptr<float>(0)[p]=camera->dist()[p];
    }
    cout<<"Final Camera Params "<<endl;
    cout<<TheMap.keyframes.begin()->imageParams.CameraMatrix<<endl;
    cout<<TheMap.keyframes.begin()->imageParams.Distorsion<<endl;

    TheMap.saveToFile(argv[2]);

    } catch (const std::exception &ex) {
     std::cerr<<ex.what()<<std::endl;
    }
}
