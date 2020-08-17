#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
 #include <map>
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include "kittieval/matrix.h"
using namespace std;

 cv::Mat  getMatrix(double tx,double ty ,double tz,double qx,double qy, double qz,double qw);




std::map<uint32_t,cv::Mat> loadFile(std::string fp,bool invert=false){
    std::map<uint32_t,cv::Mat> fmap;
    ifstream file(fp);
    float stamp;
    float tx,ty,tz,qx,qy,qz,qw;
    cv::Mat firstFrameT;
    while(!file.eof()){
        string line;
        std::getline(file,line);

        stringstream sline;sline<<line;
         if (sline>>stamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw){
             auto m=getMatrix(qx,qy,qz,qw,tx,ty,tz);
             if (invert)m=m.inv();
             if (firstFrameT.empty())
                 firstFrameT=m.inv();
            fmap.insert( {stamp, firstFrameT*m});//refers everything to the first frame
        }
     }

    //now, find the transform from every frame to the first
    return fmap;
}



class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                                    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };




vector<std::pair<cv::Mat,cv::Mat> > getMatchedLocations(std::map<uint32_t,cv::Mat> &gt,std::map<uint32_t,cv::Mat> &other){
   vector<std::pair<cv::Mat,cv::Mat> > res;
        for(auto frame_est:other){
            auto frame_gt=gt.find( frame_est.first);
            if (frame_gt!=gt.end())
                res.push_back( {frame_gt->second.clone(), frame_est.second.clone()});
           }
        return res;

}


cv::Mat rigidBodyTransformation_Horn1987 (const std::vector<cv::Point3f> &org, const std::vector<cv::Point3f> &dst,bool mbFixScale){
    auto ComputeCentroid=[](cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
    {
        cv::reduce(P,C,1,CV_REDUCE_SUM);
        C = C/P.cols;
        for(int i=0; i<P.cols; i++)
            Pr.col(i)=P.col(i)-C;
    };

    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    //Create the P1 an P2 matrices
    cv::Mat P1(3,org.size(),CV_32F);
    cv::Mat P2(3,org.size(),CV_32F);
    for(size_t i=0;i<org.size();i++){
        P1.at<float>(0,i)=org[i].x;
        P1.at<float>(1,i)=org[i].y;
        P1.at<float>(2,i)=org[i].z;
        P2.at<float>(0,i)=dst[i].x;
        P2.at<float>(1,i)=dst[i].y;
        P2.at<float>(2,i)=dst[i].z;
    }


        // Step 1: Centroid and relative coordinates

        cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
        cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
        cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
        cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

        ComputeCentroid(P1,Pr1,O1);
        ComputeCentroid(P2,Pr2,O2);

        // Step 2: Compute M matrix

        cv::Mat M = Pr2*Pr1.t();

        // Step 3: Compute N matrix

        double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

        cv::Mat N(4,4,P1.type());

        N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
        N12 = M.at<float>(1,2)-M.at<float>(2,1);
        N13 = M.at<float>(2,0)-M.at<float>(0,2);
        N14 = M.at<float>(0,1)-M.at<float>(1,0);
        N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
        N23 = M.at<float>(0,1)+M.at<float>(1,0);
        N24 = M.at<float>(2,0)+M.at<float>(0,2);
        N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
        N34 = M.at<float>(1,2)+M.at<float>(2,1);
        N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

        N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                     N12, N22, N23, N24,
                                     N13, N23, N33, N34,
                                     N14, N24, N34, N44);


        // Step 4: Eigenvector of the highest eigenvalue

        cv::Mat eval, evec;

        cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

        cv::Mat vec(1,3,evec.type());
        (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

        // Rotation angle. sin is the norm of the imaginary part, cos is the real part
        double ang=atan2(norm(vec),evec.at<float>(0,0));

        if (norm(vec)<1e-7)return cv::Mat::eye(4,4,CV_32F);

        vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

        cv::Mat mR12i(3,3,P1.type());

        cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

        // Step 5: Rotate set 2

        cv::Mat P3 = mR12i*Pr2;

        // Step 6: Scale
        float ms12i;

        if(!mbFixScale)
        {
            double nom = Pr1.dot(P3);
            cv::Mat aux_P3(P3.size(),P3.type());
            aux_P3=P3;
            cv::pow(P3,2,aux_P3);
            double den = 0;

            for(int i=0; i<aux_P3.rows; i++)
            {
                for(int j=0; j<aux_P3.cols; j++)
                {
                    den+=aux_P3.at<float>(i,j);
                }
            }

            ms12i = nom/den;
        }
        else
            ms12i = 1.0f;

        // Step 7: Translation

        cv::Mat  mt12i(1,3,P1.type());
        mt12i = O1 - ms12i*mR12i*O2;

        // Step 8: Transformation

        // Step 8.1 T12
        cv::Mat mT12i = cv::Mat::eye(4,4,P1.type());

        cv::Mat sR = ms12i*mR12i;

        sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
        mt12i.copyTo(mT12i.rowRange(0,3).col(3));
//        return mT12i;

//        // Step 8.2 T21

        cv::Mat mT21i = cv::Mat::eye(4,4,P1.type());

        cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

        sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
        cv::Mat tinv = -sRinv*mt12i;
        tinv.copyTo(mT21i.rowRange(0,3).col(3));
        return mT21i;
}


void  alignAndScaleToGroundTruth( std::vector<std::pair<cv::Mat,cv::Mat> > &gt_other ){

    vector<cv::Point3f> points_other,points_gt;
    for(auto gto:gt_other){
        assert(gto.first.type()==CV_64F);
        points_gt.push_back(cv::Point3f(gto.first.at<double>(0,3),gto.first.at<double>(1,3),gto.first.at<double>(2,3)));
        points_other.push_back(cv::Point3f(gto.second.at<double>(0,3),gto.second.at<double>(1,3),gto.second.at<double>(2,3)));
    }

    cv::Mat best_T= rigidBodyTransformation_Horn1987(points_other,points_gt,false);
     cv::Mat best_T64;
    if( best_T.type()!=CV_64F) best_T.convertTo(best_T64,CV_64F);
    else best_T64=best_T;
    cout<<best_T64<<endl;
    for(auto &gto:gt_other){
         gto.second = best_T64* gto.second;
    }

}





std::vector<float> lengths = {0.250};

struct errors {
  int32_t first_frame;
  float   r_err;
  float   t_err;
  float   len;
   errors (int32_t first_frame,float r_err,float t_err,float len ) :
    first_frame(first_frame),r_err(r_err),t_err(t_err),len(len)  {}
};
vector<float> trajectoryDistances (vector<Matrix> &poses) {
  vector<float> dist;
  dist.push_back(0);
  for (int32_t i=1; i<poses.size(); i++) {
    Matrix P1 = poses[i-1];
    Matrix P2 = poses[i];
    float dx = P1.val[0][3]-P2.val[0][3];
    float dy = P1.val[1][3]-P2.val[1][3];
    float dz = P1.val[2][3]-P2.val[2][3];
    dist.push_back(dist[i-1]+sqrt(dx*dx+dy*dy+dz*dz));
  }
  return dist;
}
int32_t lastFrameFromSegmentLength(vector<float> &dist,int32_t first_frame,float len) {
  for (int32_t i=first_frame; i<dist.size(); i++)
    if (dist[i]>dist[first_frame]+len)
      return i;
  return -1;
}

inline float rotationError(Matrix &pose_error) {
  float a = pose_error.val[0][0];
  float b = pose_error.val[1][1];
  float c = pose_error.val[2][2];
  float d = 0.5*(a+b+c-1.0);
  return acos(max(min(d,1.0f),-1.0f));
}

inline float translationError(Matrix &pose_error) {
  float dx = pose_error.val[0][3];
  float dy = pose_error.val[1][3];
  float dz = pose_error.val[2][3];
  return sqrt(dx*dx+dy*dy+dz*dz);
}
vector<errors> calcSequenceErrors (vector<Matrix> &poses_gt,vector<Matrix> &poses_result) {

  // error vector
  vector<errors> err;

  // parameters
  int32_t step_size = 1; // every second

  // pre-compute distances (from ground truth as reference)
  vector<float> dist = trajectoryDistances(poses_gt);

  // for all start positions do
  for (int32_t first_frame=0; first_frame<poses_gt.size(); first_frame+=step_size) {

    // for all segment lengths do
    for (int32_t i=0; i<lengths.size(); i++) {

      // current length
      float len = lengths[i];

      // compute last frame
      int32_t last_frame = lastFrameFromSegmentLength(dist,first_frame,len);

      // continue, if sequence not long enough
      if (last_frame==-1)
        continue;

      // compute rotational and translational errors
      Matrix pose_delta_gt     = Matrix::inv(poses_gt[first_frame])*poses_gt[last_frame];
       Matrix pose_delta_result = Matrix::inv(poses_result[first_frame])*poses_result[last_frame];
       Matrix pose_error        = Matrix::inv(pose_delta_result)*pose_delta_gt;

       float r_err = rotationError(pose_error);
      float t_err = translationError(pose_error);

      // compute speed
      float num_frames = (float)(last_frame-first_frame+1);

      // write to file
      err.push_back(errors(first_frame,r_err/len,t_err/len,len));
    }
  }

  // return error vector
  return err;
}
bool eval (vector<std::pair<cv::Mat,cv::Mat> > &gt_other_ ) {


    auto setMatrixValues=[]( Matrix &m,const cv::Mat &cvm){
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                m.val[i][j]=cvm.at<double>(i,j);
    };
  // total errors
  vector<errors> total_err;

    // file name

    // read ground truth and result poses
  vector<Matrix> poses_gt,poses_result;
  for(auto e_gt:gt_other_){
      poses_gt.push_back( Matrix());
      poses_gt.back()= Matrix::eye(4);
      setMatrixValues(poses_gt.back(),e_gt.first);
      poses_result.push_back( Matrix());
      poses_result.back()= Matrix::eye(4);
      setMatrixValues(poses_result.back(), e_gt.second);
   }

    // plot status

    // check for errors
    if (poses_gt.size()==0 || poses_result.size()!=poses_gt.size()) {
        cerr<<"ERROR: Couldn't read (all) poses "<<endl;
      return false;
    }

    // compute sequence errors
    vector<errors> seq_err = calcSequenceErrors(poses_gt,poses_result);

    // add to total errors
    total_err.insert(total_err.end(),seq_err.begin(),seq_err.end());



  // save + plot total errors + summary statistics
  if (total_err.size()>0) {
      float t_err = 0;
      float r_err = 0;
      // for all errors do => compute sum of t_err, r_err
      for (vector<errors>::iterator it=total_err.begin(); it!=total_err.end(); it++) {
        t_err += it->t_err;
        r_err += it->r_err;
      }
      // save errors
      float num = total_err.size();
      cout<<"KITTI ERRORS t="<<t_err/num <<" r="<<r_err/num<<endl;

  }

  // success
    return true;
}
void savePcd(string filepath,const vector<cv::Vec4f> & points){
    std::ofstream filePCD (filepath, std::ios::binary );
   filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points.size()<<"\nHEIGHT 1\nPOINTS "<<points.size()<<"\nDATA binary\n";
   filePCD.write((char*)&points[0],points.size()*sizeof(points[0]));
}
cv::Point3f getT(cv::Mat &m){
    if(m.type()==CV_64F)
        return cv::Point3f(m.at<double>(0,3),m.at<double>(1,3),m.at<double>(2,3));
    else
        return cv::Point3f(m.at<float>(0,3),m.at<float>(1,3),m.at<float>(2,3));
}
std::vector<cv::Vec4f> getLine(const  cv::Point3f &a,const  cv::Point3f &b,cv::Scalar color,int npoints);
int main(int argc,char **argv){
    CmdLineParser cml(argc,argv);

    if(argc<3 || cml["-h"])
        {cerr<<"logfile1_groundtruth logfile2    [-dummy_vehicle]  "<<endl;return -1;}

    auto f_gt=loadFile(argv[1], cml["-inverse"]);
    auto f_est=loadFile(argv[2],cml["-inverse"]);

   for(auto &frame:f_gt)frame.second.at<double>(1,3)*=-1;


    if (cml["-dummy_vehicle"]){
         for(auto &frame:f_est) {
             frame.second=cv::Mat::eye(4,4,CV_64F);
         }
    }



    vector<std::pair<cv::Mat,cv::Mat> > gt_other_=getMatchedLocations( f_gt,f_est);
   alignAndScaleToGroundTruth(gt_other_);

   //get the ATE error
   double e=0;
   for(auto p:gt_other_)
       e+=cv::norm(getT(p.first)-getT(p.second));
   cout<<"ATE="<<e/double(gt_other_.size())<<endl;

    eval(gt_other_);

    //print the percentage of frames effectively tracked
    cout<<"Frames tracked="<< 100.*float(gt_other_.size())/ float(f_gt.size())<<"%"<<endl;

    vector<cv::Vec4f> pcdpoints_gt,pcdpoints_est,joinlines;
    for(size_t i=1;i<gt_other_.size();i++){
        cv::Point3f p0=getT(gt_other_[i-1].first);
        cv::Point3f p1=getT(gt_other_[i].first);

        auto pp=getLine(p0,p1,cv::Scalar(0,0,255),10);
        pcdpoints_gt.insert(pcdpoints_gt.end(),pp.begin(),pp.end());

        auto p0o=getT( gt_other_[i-1].second);
        auto p1o=getT( gt_other_[i].second);
        pp=getLine(p0o,p1o,cv::Scalar(0,255,0),10);
        pcdpoints_est.insert(pcdpoints_est.end(),pp.begin(),pp.end());

        pp=getLine(p0,p0o,cv::Scalar(255,0,0),30);
        joinlines.insert(joinlines.end(),pp.begin(),pp.end());



    }

    savePcd("gt.pcd",pcdpoints_gt);
    savePcd("other.pcd",pcdpoints_est);
    savePcd("joinlines.pcd",joinlines);



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
cv::Mat  getMatrix(double qx,double qy, double qz,double qw,double tx,double ty ,double tz){


    double qx2 = qx*qx;
    double qy2 = qy*qy;
    double qz2 = qz*qz;


    cv::Mat m=cv::Mat::eye(4,4,CV_64F);

    m.at<double>(0,0)=1 - 2*qy2 - 2*qz2;
    m.at<double>(0,1)=2*qx*qy - 2*qz*qw;
    m.at<double>(0,2)=2*qx*qz + 2*qy*qw;
    m.at<double>(0,3)=tx;

    m.at<double>(1,0)=2*qx*qy + 2*qz*qw;
    m.at<double>(1,1)=1 - 2*qx2 - 2*qz2;
    m.at<double>(1,2)=2*qy*qz - 2*qx*qw;
    m.at<double>(1,3)=ty;

    m.at<double>(2,0)=2*qx*qz - 2*qy*qw	;
    m.at<double>(2,1)=2*qy*qz + 2*qx*qw	;
    m.at<double>(2,2)=1 - 2*qx2 - 2*qy2;
    m.at<double>(2,3)=tz;
    return m;
}
