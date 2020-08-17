#include "rpp.h"
#include <cstdio>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace std;
using namespace cv;

static double TOL = 1E-5;
static double EPSILON = 1E-8;

namespace rpp
{
#define MAXDEGREE 100
#define MDP1 MAXDEGREE+1
#define M_PI 3.14159265359
#define M_PI_2 1.57079632679

void rpoly_ak1(double op[MDP1], int* Degree, double zeror[MAXDEGREE], double zeroi[MAXDEGREE]);
void Fxshfr_ak1(int L2, int* NZ, double sr, double v, double K[MDP1], int N, double p[MDP1], int NN, double qp[MDP1], double u, double* lzi, double* lzr, double* szi, double* szr);
void QuadSD_ak1(int NN, double u, double v, double p[MDP1], double q[MDP1], double* a, double* b);
int calcSC_ak1(int N, double a, double b, double* a1, double* a3, double* a7, double* c, double* d, double* e, double* f, double* g, double* h, double K[MDP1], double u, double v, double qk[MDP1]);
void nextK_ak1(int N, int tFlag, double a, double b, double a1, double* a3, double* a7, double K[MDP1], double qk[MDP1], double qp[MDP1]);
void newest_ak1(int tFlag, double* uu, double* vv, double a, double a1, double a3, double a7, double b, double c, double d, double f, double g, double h, double u, double v, double K[MDP1], int N, double p[MDP1]);
void QuadIT_ak1(int N, int* NZ, double uu, double vv, double* szr, double* szi, double* lzr, double* lzi, double qp[MDP1], int NN, double* a, double* b, double p[MDP1], double qk[MDP1], double* a1, double* a3, double* a7, double* c, double* d, double* e, double* f, double* g, double* h, double K[MDP1]);
void RealIT_ak1(int* iFlag, int* NZ, double* sss, int N, double p[MDP1], int NN, double qp[MDP1], double* szr, double* szi, double K[MDP1], double qk[MDP1]);
void Quad_ak1(double a, double b1, double c, double* sr, double* si, double* lr, double* li);


bool solveRpp(const vector<cv::Point3f> & objpoints,const vector<cv::Point2f> &imPoints,const cv::Mat &camera_matrix,const cv::Mat &dist,cv::Mat &rotation,cv::Mat &translation){
   cv::Mat rt=solveRpp(objpoints,imPoints,camera_matrix,dist);
   if (rt.empty())return false;
    cv::Mat r33=rt(cv::Range(0,3),cv::Range(0,3));
    cv::Rodrigues(r33,rotation);
    translation.create(1,3,CV_32F);
    for(int i=0;i<3;i++)
        translation.ptr<float>(0)[i]=rt.at<float>(i,3);

    return true;
}


//returns rt mat of empty if not possible
cv::Mat solveRpp(const vector<cv::Point3f> & objpoints,const vector<cv::Point2f> &imPoints,const cv::Mat &camera_matrix,const cv::Mat &dist ){
     vector<cv::Point2f> undP;
    cv::undistortPoints(imPoints,undP,camera_matrix,dist);

    cv::Mat model = cv::Mat::zeros( objpoints.size(),3, CV_64F); // 3D points, z is zero
    cv::Mat iprts = cv::Mat::ones( imPoints.size(),3, CV_64F); // 2D points, homogenous points

    for(int i=0;i<imPoints.size();i++){
        model.at<double>(i,0)=objpoints[i].x;
        model.at<double>(i,1)=objpoints[i].y;
        model.at<double>(i,2)=objpoints[i].z;
        iprts.at<double>(i,0)=undP[i].x;
        iprts.at<double>(i,1)=undP[i].y;

    }
    model=model.t();
    iprts=iprts.t();
    int iterations;
    double obj_err;
    double img_err;

    cv::Mat r33,t64;
    if(!rpp::Rpp(model, iprts, r33, t64, iterations, obj_err, img_err))
        return cv::Mat();

    cv::Mat rt44=cv::Mat::eye(4,4,CV_64F);
    for(int i=0;i<3;i++){
        memcpy(rt44.ptr<double>(i),r33.ptr<double>(i),3*sizeof(double));
        rt44.at<double>(i,3)=t64.ptr<double>(0)[i];
    }
     cv::Mat rt44_32;
    rt44.convertTo(rt44_32,CV_32F);
    return rt44_32;

}

cv::Point3f mult( cv::Point3f in,cv::Mat &r33,cv::Mat &t){
cv::Point3f res;
assert(r33.type()==CV_64F);
double *rptr=r33.ptr<double>(0);
double *tptr=t.ptr<double>(0);
res.x= in.x* rptr[0]+in.y* rptr[1]+in.z* rptr[2]+rptr[0];
res.y= in.x* rptr[3]+in.y* rptr[4]+in.z* rptr[5]+rptr[1];
res.z= in.x* rptr[6]+in.y* rptr[7]+in.z* rptr[8]+rptr[2];
return res;
}

double   cameraMarkerDotProduct(cv::Mat & r33,cv::Mat & t){
    cv::Point3f p0=mult(cv::Point3f(0,0,0),r33,t ),p1=  mult(cv::Point3f(0,0,1),r33,t);
    cv::Point3f normal_=p1-p0;
    normal_*=1./cv::norm(normal_);
    return normal_.z;
}


bool Rpp(const Mat &model_3D, const Mat &iprts,
         Mat &Rlu, Mat &tlu, int &it1, double &obj_err1, double &img_err1)
{
    // get a first guess of the pose.
	if(Rlu.data) {
		ObjPose(model_3D, iprts, Rlu, Rlu, tlu, it1, obj_err1, img_err1);
	}
	else {
	    ObjPose(model_3D, iprts, cv::Mat(), Rlu, tlu, it1, obj_err1, img_err1);
	}

    // get 2nd Pose
    vector <Solution> sol;
    bool status = Get2ndPose_Exact(iprts, model_3D, Rlu, tlu, sol);

    if(!status) {
        return false;
    }

    //refine with lu
    int bestIdx=-1;
    double lowestErr = 1e6;

    for(unsigned int i=0; i < sol.size(); i++) {
        ObjPose(model_3D, iprts, sol[i].R, Rlu, sol[i].t, it1, obj_err1, img_err1);
        sol[i].R = Rlu;
        sol[i].obj_err = obj_err1;
        sol[i].img_err = img_err1;

        if (cameraMarkerDotProduct(sol[i].R, sol[i].t)<0){ //avoid invalid solutions
            if(obj_err1 < lowestErr) {
                lowestErr = obj_err1;
                bestIdx = i;
            }
        }
    }
if (bestIdx==-1)return false;
    Rlu = sol[bestIdx].R;
    tlu = sol[bestIdx].t;
    obj_err1 = sol[bestIdx].obj_err;
    img_err1 = sol[bestIdx].img_err;


    //printf("Best solution: %d\n", bestIdx);

    //printf("Final R\n");
    //Print(Rlu);

    //printf("Final t\n");
    //Print(tlu);
    if (bestIdx!=-1)    return true;
    else return false;
}


std::vector<std::pair<cv::Mat,double> > Rpp2(const vector<cv::Point3f> &objpoints, const vector<cv::Point2f> &imPoints,const cv::Mat &cameraMatrix,const cv::Mat &distCoeffs)
{


    vector<cv::Point2f> undP;
   cv::undistortPoints(imPoints,undP,cameraMatrix,distCoeffs);

   cv::Mat model = cv::Mat::zeros( objpoints.size(),3, CV_64F); // 3D points, z is zero
   cv::Mat iprts = cv::Mat::ones( imPoints.size(),3, CV_64F); // 2D points, homogenous points

   for(int i=0;i<imPoints.size();i++){
       model.at<double>(i,0)=objpoints[i].x;
       model.at<double>(i,1)=objpoints[i].y;
       model.at<double>(i,2)=objpoints[i].z;
       iprts.at<double>(i,0)=undP[i].x;
       iprts.at<double>(i,1)=undP[i].y;

   }
 cv::Mat model_3D=model.t();
   iprts=iprts.t();



     cv::Mat Rlu, tlu;
     int it1;
     double obj_err1,img_err1;
    // get a first guess of the pose.
    if(Rlu.data) {
        ObjPose(model_3D, iprts, Rlu, Rlu, tlu, it1, obj_err1, img_err1);
    }
    else {
        ObjPose(model_3D, iprts, cv::Mat(), Rlu, tlu, it1, obj_err1, img_err1);
    }


    // get 2nd Pose
    vector <Solution> sol;
    if(! Get2ndPose_Exact(iprts, model_3D, Rlu, tlu, sol) ) return {}  ;

     //refine with lu
    int bestIdx=-1;
    double lowestErr = 1e6;
 std::vector<std::pair<cv::Mat,double> > ret_v;
    for(unsigned int i=0; i < sol.size(); i++) {
        ObjPose(model_3D, iprts, sol[i].R, Rlu, sol[i].t, it1, obj_err1, img_err1);
        sol[i].R = Rlu;
        assert(sol[i].R.type()==CV_64F);
        sol[i].obj_err = obj_err1;
        sol[i].img_err = img_err1;
      // if (cameraMarkerDotProduct(sol[i].R, sol[i].t)<0){ //avoid invalid solutions
             cv::Mat rt44=cv::Mat::eye(4,4,CV_64F);
            for(int j=0;j<3;j++){
                memcpy(rt44.ptr<double>(j),sol[i].R.ptr<double>(j),3*sizeof(double));
                rt44.at<double>(j,3)=sol[i].t.ptr<double>(0)[j];
            }

            cv::Mat rt44_32;
            rt44.convertTo(rt44_32,CV_32F);
            ret_v.push_back(make_pair(rt44,img_err1));
        //}
    }
     std::sort(ret_v.begin(),ret_v.end(),[](const std::pair<cv::Mat,double> &a,const std::pair<cv::Mat,double> &b ){return a.second<b.second;});
     return ret_v;
}

void ObjPose(const Mat _P, Mat Qp, Mat initR,
             Mat &R, Mat &t, int &it, double &obj_err, double &img_err)
{
    Mat P = _P.clone(); // make a copy, else _P will get modified, nasty GOTCHA!

    int n = P.cols;
    it = 0;

    Mat pbar = Sum(P,2) / n;

    // move the origin to the center of P
    for(int i=0; i < n; i++) {
        P.at<double>(0,i) -= pbar.at<double>(0,0);
        P.at<double>(1,i) -= pbar.at<double>(1,0);
        P.at<double>(2,i) -= pbar.at<double>(2,0);
    }

    // compute projection matrices
    vector <Mat> F(n);
    Mat V(3,1,CV_64F);
    Mat ret(1,1,CV_64F);

    for(int i=0; i < n; i++) {
        F[i].create(3,3,CV_64F);

        V.at<double>(0,0) = Qp.at<double>(0,i);
        V.at<double>(1,0) = Qp.at<double>(1,i);
        V.at<double>(2,0) = Qp.at<double>(2,i);

        ret = V.t()*V;

        F[i] = (V*V.t()) / ret.at<double>(0,0);
    }

    // compute the matrix factor required to compute t
    Mat sumF = Mat::zeros(3,3,CV_64F);

    for(int i=0; i < n; i++) {
        sumF += F[i];
    }

    Mat tFactor = (Mat::eye(3,3,CV_64F)-sumF/n).inv()/n;

    double old_err;
    Mat Qi;
    Mat Ri, ti;

    if(initR.data) {
        Ri = initR;

        Mat sum_ = Mat::zeros(3,1,CV_64F);

        Mat _eye = Mat::eye(3,3,CV_64F);
        Mat PP(3,1,CV_64F);

        for(int i=0; i < n; i++) {
            PP.at<double>(0,0) = P.at<double>(0,i);
            PP.at<double>(1,0) = P.at<double>(1,i);
            PP.at<double>(2,0) = P.at<double>(2,i);

            sum_ = sum_ + (F[i] - _eye)*Ri*PP;
        }

        ti = tFactor*sum_;

        // calculate error
        Qi = Xform(P, Ri, ti);

        old_err = 0;

        Mat vec(3,1,CV_64F);
        Mat QiCol(3,1,CV_64F);

        for(int i=0; i < n; i++) {
            QiCol.at<double>(0,0) = Qi.at<double>(0,i);
            QiCol.at<double>(1,0) = Qi.at<double>(1,i);
            QiCol.at<double>(2,0) = Qi.at<double>(2,i);

            vec = (_eye - F[i])*QiCol;

            double x = vec.at<double>(0,0);
            double y = vec.at<double>(1,0);
            double z = vec.at<double>(2,0);

            old_err += (x*x + y*y + z*z);
        }
    }
    else {
        // no initial guess; use weak-perspective approximation
        AbsKernel(P, Qp, F, tFactor, Ri, ti, Qi, old_err);
        it = 1;

        //printf("SVD\n");
        //Print(Ri);
    }

    // compute next pose estimate
    double new_err;

    AbsKernel(P, Qi, F, tFactor, Ri, ti, Qi, new_err);

    it = it + 1;

    //printf("before while\n");

    while(fabs((old_err-new_err)/old_err) > TOL && (new_err > EPSILON)) {
        old_err = new_err;
        // compute the optimal estimate of R
        AbsKernel(P, Qi, F, tFactor, Ri, ti, Qi, new_err);

        //printf("it %d\n", it);
        it = it + 1;
    }

    R = Ri;
    t = ti;
    obj_err = sqrt(new_err/n);

    // calculate image-space error
    Mat Pcol(3,1,CV_64F);
    Mat Qproj;

    img_err = 0;

    for(int i=0; i < n; i++) {
        Pcol.at<double>(0,0) = P.at<double>(0,i);
        Pcol.at<double>(1,0) = P.at<double>(1,i);
        Pcol.at<double>(2,0) = P.at<double>(2,i);

        Qproj = Ri*Pcol + ti;

        double xx = (Qproj.at<double>(0,0)/Qproj.at<double>(2,0)) - Qp.at<double>(0,0);
        double yy = (Qproj.at<double>(1,0)/Qproj.at<double>(2,0)) - Qp.at<double>(1,0);

        img_err += (xx*xx + yy*yy);
    }

    img_err = sqrt(img_err/n);

    // get back to original refernce frame

    t = t - Ri*pbar;
}

Mat EstimateT(const Mat &R, const Mat &G, const vector <Mat> &F, const Mat &P)
{
    Mat sum = Mat::zeros(3,1,CV_64F);

    Mat PP(3,1,CV_64F);

    for(int i=0; i < P.cols; i++) {
        PP.at<double>(0,0) = P.at<double>(0,i);
        PP.at<double>(1,0) = P.at<double>(1,i);
        PP.at<double>(2,0) = P.at<double>(2,i);

        sum += F[i]*R*PP;
    }

    Mat ret = G*sum;

    return ret;
}

void AbsKernel(Mat P, Mat Q, const vector <Mat> &F, const Mat &G,
               Mat &R, Mat &t, Mat &Qout, double &err2)
{
    int n = P.cols;

    Mat QQ(3,1,CV_64F);

    for(int i=0; i < n; i++) {
        QQ.at<double>(0,0) = Q.at<double>(0,i);
        QQ.at<double>(1,0) = Q.at<double>(1,i);
        QQ.at<double>(2,0) = Q.at<double>(2,i);

        QQ= F[i]*QQ;

        Q.at<double>(0,i) = QQ.at<double>(0,0);
        Q.at<double>(1,i) = QQ.at<double>(1,0);
        Q.at<double>(2,i) = QQ.at<double>(2,0);
    }

    Mat pbar = Sum(P,2)/n;
    Mat qbar = Sum(Q,2)/n;

    // compute P' and Q'
    for(int i=0; i < n; i++) {
        P.at<double>(0,i) -= pbar.at<double>(0,0);
        P.at<double>(1,i) -= pbar.at<double>(1,0);
        P.at<double>(2,i) -= pbar.at<double>(2,0);
    }

    // use SVD solution
    // compute M matrix
    Mat M = Mat::zeros(3,3,CV_64F);
    Mat PP(3,1,CV_64F);

    for(int i=0; i < n; i++) {
        PP.at<double>(0,0) = P.at<double>(0,i);
        PP.at<double>(1,0) = P.at<double>(1,i);
        PP.at<double>(2,0) = P.at<double>(2,i);

        QQ.at<double>(0,0) = Q.at<double>(0,i);
        QQ.at<double>(1,0) = Q.at<double>(1,i);
        QQ.at<double>(2,0) = Q.at<double>(2,i);

        M += PP*QQ.t();
    }

    SVD decomp(M);

    R = decomp.vt.t()*(decomp.u.t());
    Mat v = decomp.vt.t();

    if(sign(determinant(R)) == 1) {
        t = EstimateT(R,G,F,P);

        if(t.at<double>(2,0) < 0) {
            // we need to invert the t
            // negate the 3rd column

            v.at<double>(0,2) = -v.at<double>(0,2);
            v.at<double>(1,2) = -v.at<double>(1,2);
            v.at<double>(2,2) = -v.at<double>(2,2);

            // we need to invert the t
            R = -v*decomp.u.t();
            t = EstimateT(R,G,F,P);
        }
    }
    else {
        // negate the 3rd column
        v.at<double>(0,2) = -v.at<double>(0,2);
        v.at<double>(1,2) = -v.at<double>(1,2);
        v.at<double>(2,2) = -v.at<double>(2,2);

        R = v*decomp.u.t();
        t = EstimateT(R,G,F,P);

        if(t.at<double>(2,0) < 0) {
            R = -v*(decomp.u.t());
            t = EstimateT(R,G,F,P);
        }
    }


    // calculate error
    Mat _eye = Mat::eye(3,3,CV_64F);
    Mat vec(3,1,CV_64F);

    err2 = 0;
    Qout = Xform(P, R, t);

    for(int i=0; i < n; i++) {
        QQ.at<double>(0,0) = Qout.at<double>(0,i);
        QQ.at<double>(1,0) = Qout.at<double>(1,i);
        QQ.at<double>(2,0) = Qout.at<double>(2,i);

        vec = (_eye - F[i])*QQ;

        double x = vec.at<double>(0,0);
        double y = vec.at<double>(1,0);
        double z = vec.at<double>(2,0);

        err2 += (x*x + y*y + z*z);
    }
}

Mat NormRv(const Mat &R)
{
    Mat ret(R.rows, R.cols, CV_64F);

    for(int i=0; i < R.cols; i++) {

        double mag = R.at<double>(0,i)*R.at<double>(0,i) +
                    R.at<double>(1,i)*R.at<double>(1,i) +
                    R.at<double>(2,i)*R.at<double>(2,i);

        double m = 1.f/sqrt(mag);

        ret.at<double>(0,i) = R.at<double>(0,i)*m;
        ret.at<double>(1,i) = R.at<double>(1,i)*m;
        ret.at<double>(2,i) = R.at<double>(2,i)*m;
    }

    return ret;
}

Mat NormRv(const Vec3d &V)
{
    Mat ret(3,1,CV_64F);

    double mag = sqrt(V[0]*V[0] + V[1]*V[1] + V[2]*V[2]);

    ret.at<double>(0,0) = V[0] / mag;
    ret.at<double>(1,0) = V[1] / mag;
    ret.at<double>(2,0) = V[2] / mag;

    return ret;
}

Quaternion Quaternion_byAngleAndVector(double q_angle, const Vec3d &q_vector)
{
    //printf("Quaternion_byAngleAndVector\n");

    // rotation_axis=q_vector/norm(q_vector);
    // matlab norm with default argument of 2 is largest singular value
    double n = Norm(Vec2Mat(q_vector));

    //printf("n = %f\n", n);

    Vec3d rotation_axis = q_vector;

    rotation_axis[0] /= n;
    rotation_axis[1] /= n;
    rotation_axis[2] /= n;

    rotation_axis[0] *= sin(q_angle*0.5);
    rotation_axis[1] *= sin(q_angle*0.5);
    rotation_axis[2] *= sin(q_angle*0.5);

    Quaternion Q_ = Quaternion_byVectorAndScalar(rotation_axis,	cos(q_angle*0.5));

    //printf("Q_\n");
    //Print(Q_);

//    double a = 1/Quaternion_Norm(Q_);

    //printf("a = %f\n", a);

    Quaternion Q = Quaternion_multiplyByScalar(Q_, 1/Quaternion_Norm(Q_));

    return Q;
}

Mat quat2mat(const Quaternion &Q)
{
    double a = Q.scalar;
    double b = Q.vector[0];
    double c = Q.vector[1];
    double d = Q.vector[2];

    Mat R(3,3,CV_64F);

    R.at<double>(0,0) = a*a + b*b - c*c -d*d;
    R.at<double>(0,1) = 2*(b*c - a*d);
    R.at<double>(0,2) = 2*(b*d + a*c);

    R.at<double>(1,0) = 2*(b*c + a*d);
    R.at<double>(1,1) = a*a + c*c - b*b - d*d;
    R.at<double>(1,2) = 2*(c*d - a*b);

    R.at<double>(2,0) = 2*(b*d - a*c);
    R.at<double>(2,1) = 2*(c*d + a*b);
    R.at<double>(2,2) = a*a + d*d - b*b -c*c;

    return R;
}

Mat GetRotationbyVector(const Vec3d &v1, const Vec3d &v2)
{
    //printf("GetRotationbyVector\n");

    double winkel = acos(v2.dot(v1));

    //printf("winkel = %f\n", winkel);

    Quaternion QU = Quaternion_byAngleAndVector(winkel,v2.cross(v1));

    //printf("QU\n");
    //Print(QU);

    Mat R = quat2mat(QU);

    //printf("R\n");
    //Print(R);

    Mat a = Sum(Sq(NormRv(v1) - R*NormRv(v2)));

    //printf("a\n");
    //Print(a);

    double mag = a.at<double>(0,0)*a.at<double>(0,0);

    if(mag > 1e-3) {
        fprintf(stderr, "Error in GetRotationbyVector()\n");
        exit(1);
    }

    return R;
}

Mat Sum(const Mat &m, int dim)
{
    // columns
    if(dim == 1) {
        Mat ret(1, m.cols, CV_64F);

        for(int j=0; j < m.cols; j++) {
            double sum = 0;

            for(int i=0; i < m.rows; i++) {
                sum += m.at<double>(i,j);
            }

            ret.at<double>(0,j) = sum;
        }

        return ret;
    }
    else {
        Mat ret(m.rows, 1, CV_64F);

        for(int i=0; i < m.rows; i++) {
            double sum = 0;

            for(int j=0; j < m.cols; j++) {
                sum += m.at<double>(i,j);
            }

            ret.at<double>(i,0) = sum;
        }

        return ret;
    }
}

Mat Mul(const Mat &a, const Mat &b)
{
    assert(a.rows == b.rows && a.cols == b.cols);

    Mat ret(a.rows, a.cols, CV_64F);

    for(int i=0; i < a.rows; i++) {
        for(int j=0; j < a.cols; j++) {
            ret.at<double>(i,j) = a.at<double>(i,j)*b.at<double>(i,j);
        }
    }

    return ret;
}

Mat Mean(const Mat &m)
{
    Mat ret(1, m.cols, CV_64F);

    for(int j=0; j < m.cols; j++) {
        double sum = 0;

        for(int i=0; i < m.rows; i++) {
            sum += m.at<double>(i,j);
        }

        ret.at<double>(0,j) = sum / m.cols;
    }

    return ret;
}

Mat Point2Mat(const vector <Point3d> &pts)
{
    Mat ret(3, pts.size(), CV_64F);

    for(unsigned int i=0; i < pts.size(); i++) {
        ret.at<double>(0,i) = pts[i].x;
        ret.at<double>(1,i) = pts[i].y;
        ret.at<double>(2,i) = pts[i].z;
    }

    return ret;
}

Mat Point2Mat(const vector <Point2d> &pts)
{
    Mat ret(3, pts.size(), CV_64F);

    for(unsigned int i=0; i < pts.size(); i++) {
        ret.at<double>(0,i) = pts[i].x;
        ret.at<double>(1,i) = pts[i].y;
        ret.at<double>(2,i) = 1.f;
    }

    return ret;
}

Mat Vec2Mat(const Vec3d &v)
{
    Mat ret(3,1,CV_64F);

    ret.at<double>(0,0) = v[0];
    ret.at<double>(1,0) = v[1];
    ret.at<double>(2,0) = v[2];

    return ret;
}


Mat RpyMat(const Vec3d &angs)
{
    double cosA = cos(angs[2]);
    double sinA = sin(angs[2]);
    double cosB = cos(angs[1]);
    double sinB = sin(angs[1]);
    double cosC = cos(angs[0]);
    double sinC = sin(angs[0]);

    double cosAsinB = cosA*sinB;
    double sinAsinB = sinA*sinB;

    Mat R(3,3,CV_64F);

    R.at<double>(0,0) = cosA*cosB;
    R.at<double>(0,1) = cosAsinB*sinC-sinA*cosC;
    R.at<double>(0,2) = cosAsinB*cosC+sinA*sinC;

    R.at<double>(1,0) = sinA*cosB;
    R.at<double>(1,1) = sinAsinB*sinC+cosA*cosC;
    R.at<double>(1,2) = sinAsinB*cosC-cosA*sinC;

    R.at<double>(2,0) = -sinB;
    R.at<double>(2,1) = cosB*sinC;
    R.at<double>(2,2) = cosB*cosC;

    return R;
}

bool RpyAng(const Mat &R, Vec3d &ret)
{
    //printf("\nRpyAng\n");

    Vec3d angs;

    double R11 = R.at<double>(0,0);
    double R12 = R.at<double>(0,1);
    double R13 = R.at<double>(0,2);

    double R21 = R.at<double>(1,0);
    double R22 = R.at<double>(1,1);
    double R23 = R.at<double>(1,2);

    double R31 = R.at<double>(2,0);
    double R32 = R.at<double>(2,1);
    double R33 = R.at<double>(2,2);

    double sinB = -R31;
    double cosB = sqrt(R11*R11 + R21*R21);

    if(fabs (cosB) > 1e-15) {
        double sinA = R21 / cosB;
        double cosA = R11 / cosB;
        double sinC = R32 / cosB;
        double cosC = R33 / cosB;
        angs[0] = atan2(sinC,cosC);
        angs[1] = atan2(sinB,cosB);
        angs[2] = atan2(sinA,cosA);
    }
    else {
        double sinC = (R12 - R23) / 2;
        double cosC = (R22 + R13) / 2;
        angs[0] = atan2(sinC,cosC);
        angs[1] = M_PI_2;
        angs[2] = 0;

        if(sinB < 0)
            angs = -angs;
    }

    //printf("R\n");
    //Print(R);

    //printf("angs\n");
    //Print(Vec2Mat(angs));

    //printf("angs: %f %f %f\n", angs[0], angs[1], angs[2]);

    Mat a = R-RpyMat(angs);

    //printf("a\n");
    //Print(a);

    //double a = Norm(R-RpyMat(angs));
    //printf("a = %f\n", a);

    if(Norm(R-RpyMat(angs)) > 1e-6) {
        fprintf(stderr, "rpyMat: Error not correct Solution\n");
        return false;
    }

    ret = angs;

    return true;
}

bool RpyAng_X(const Mat &R, Vec3d &ret)
{
    Vec3d ang_zyx;
    bool status = RpyAng(R, ang_zyx);

    if(!status) {
        return false;
    }

    if(fabs(ang_zyx[0]) > M_PI_2) {
         // test the same R
        while ( fabs(ang_zyx[0]) > M_PI_2 ) {
            if(ang_zyx[0] > 0) {
                ang_zyx[0] = ang_zyx[0]+M_PI;
                ang_zyx[1] = 3*M_PI-ang_zyx[1];
                ang_zyx[2] = ang_zyx[2]+M_PI;

                ang_zyx[0] -= 2*M_PI;
                ang_zyx[1] -= 2*M_PI;
                ang_zyx[2] -= 2*M_PI;
            }
            else {
              ang_zyx[0] = ang_zyx[0]+M_PI;
              ang_zyx[1] = 3*M_PI-ang_zyx[1];
              ang_zyx[2] = ang_zyx[2]+M_PI;
            }
        }
    }

    ret = ang_zyx;

    return true;
}

bool Get2ndPose_Exact(const Mat &v, const Mat &P, const Mat &R, const Mat &t, vector<Solution> &ret)
{
    //printf("Get2ndPose_Exact\n");

    Mat cent = NormRv(Mean(NormRv(v).t()).t());
    Vec3d _cent;

    _cent[0] = cent.at<double>(0,0);
    _cent[1] = cent.at<double>(1,0);
    _cent[2] = cent.at<double>(2,0);

    Mat Rim = GetRotationbyVector(Vec3d(0,0,1), _cent);

    Mat v_ = Rim*v;
    cent = NormRv(Mean(NormRv(v_).t()).t());

    Mat R_ = Rim*R;
    Mat t_ = Rim*t;
/*
    printf("cent\n");
    Print(cent);

    printf("Rim\n");
    Print(Rim);

    printf("R\n");
    Print(R);

    printf("t\n");
    Print(t);

    printf("R_\n");
    Print(R_);

    printf("t_\n");
    Print(t_);
*/
    vector<Solution> sol;

    bool status = GetRfor2ndPose_V_Exact(v_,P,R_,t_, sol);

    if(!status) {
        return false;
    }

    // de Normalise the Pose
    for(unsigned int i=0; i < sol.size(); i++) {
        //printf("BEFORE\n");
        //Print(sol[i].R);

        sol[i].R = Rim.t()*sol[i].R;

        //printf("AFTER\n");
        //Print(sol[i].R);
        sol[i].t = Rim.t()*sol[i].t;
    }

    ret = sol;

    return true;
}

bool DecomposeR(const Mat &R, Mat &Rz2, Mat &ret)
{
    //printf("\nDecomposeR\n");

    double cl = atan2(R.at<double>(2,1), R.at<double>(2,0));
    Mat Rz = RpyMat(Vec3d(0,0,cl));
/*
    printf("cl = %f\n", cl);
    printf("Rz\n");
    Print(Rz);
    printf("R\n");
    Print(R);
*/
    Mat R_ = R*Rz;

    //printf("R_\n");
    //Print(R_);

    if(R_.at<double>(2,1) > 1e-3) {
        fprintf(stderr, "error in DecomposeR 1\n");
        return false;
    }

    Vec3d ang_zyx;
    bool status = RpyAng_X(R_, ang_zyx);

    if(!status) {
        return false;
    }

    if(fabs(ang_zyx[0]) > 1e-3) {
        fprintf(stderr, "error in DecomposeR 2\n");
        return false;
    }

    Rz2 = Rz*RpyMat(Vec3d(0,0,M_PI));
    R_ = R*Rz2;

    if(R_.at<double>(2,1) > 1e-3) {
        fprintf(stderr, "error in DecomposeR 3\n");
        return false;
    }

    // why do we do this?
    status = RpyAng_X(R_, ang_zyx);

    if(!status) {
        return false;
    }

    ret = Rz;

    return true;
}

Mat Sq(const Mat &m)
{
    Mat ret(m.rows, m.cols, CV_64F);

    for(int i=0; i < m.rows; i++) {
        for(int j=0; j < m.cols; j++) {
            ret.at<double>(i,j) = m.at<double>(i,j)*m.at<double>(i,j);
        }
    }

    return ret;
}

Mat Xform(const Mat &P, const Mat &R, const Mat &t)
{
    Mat ret(3,P.cols,CV_64F);

    for(int i=0; i < P.cols; i++) {
        double x = P.at<double>(0,i);
        double y = P.at<double>(1,i);
        double z = P.at<double>(2,i);

        ret.at<double>(0,i) = R.at<double>(0,0)*x + R.at<double>(0,1)*y + R.at<double>(0,2)*z + t.at<double>(0,0);
        ret.at<double>(1,i) = R.at<double>(1,0)*x + R.at<double>(1,1)*y + R.at<double>(1,2)*z + t.at<double>(1,0);
        ret.at<double>(2,i) = R.at<double>(2,0)*x + R.at<double>(2,1)*y + R.at<double>(2,2)*z + t.at<double>(2,0);
    }

    return ret;
}

double Norm(const Mat &m)
{
    SVD decomp(m);

    return decomp.w.at<double>(0,0);
}

bool GetRfor2ndPose_V_Exact(const Mat &v, const Mat &P, const Mat &R, const Mat &t, vector<Solution> &ret)
{
    //printf("\nGetRfor2ndPose_V_Exact\n");
    //printf("R\n");
    //Print(R);

    Mat Rz2;
    Mat RzN;

    bool status = DecomposeR(R, Rz2, RzN);

    if(!status) {
        return false;
    }

    Mat R_ = R*RzN;

    Mat P_ = RzN.t()*P;

    Vec3d ang_zyx;

    status = RpyAng_X(R_, ang_zyx);

    if(!status) {
        return false;
    }

    Mat Ry = RpyMat(Vec3d(0,ang_zyx[1],0));
    Mat Rz = RpyMat(Vec3d(0,0,ang_zyx[2]));

    vector <double> bl;
    Mat Tnew;
    vector <double> at;

    GetRotationY_wrtT(v ,P_,t,Rz, bl, Tnew, at);

    // Suggestion by csantos
    if(bl.empty()) {
        return false;
    }

    for(unsigned int i=0; i < bl.size(); i++) {
        bl[i] = bl[i]/180*M_PI;
    }

    // we got 2 solutions. YEAH
    vector <Mat> V(v.cols);

    Mat tmp(3,1,CV_64F);

    for(int i=0; i < v.cols; i++) {
        tmp.at<double>(0,0) = v.at<double>(0,i);
        tmp.at<double>(1,0) = v.at<double>(1,i);
        tmp.at<double>(2,0) = v.at<double>(2,i);

        Mat a = tmp.t()*tmp;

        V[i] = tmp*tmp.t() / a.at<double>(0,0);
    }

    vector <Solution> sol(bl.size());

    for(unsigned int j=0; j < bl.size(); j++) {
        sol[j].bl = bl[j];
        sol[j].at = at[j];

        Ry = RpyMat(Vec3d(0,bl[j],0));
        sol[j].R = Rz*Ry*RzN.t();

        //printf("sol[j].R\n");
        //Print(sol[j].R);


        sol[j].t.at<double>(0,0) = Tnew.at<double>(0,j);
        sol[j].t.at<double>(1,0) = Tnew.at<double>(1,j);
        sol[j].t.at<double>(2,0) = Tnew.at<double>(2,j);

        // test the Error
        double E=0;
        Mat _eye = Mat::eye(3,3,CV_64F);
        Mat Pcol(3,1,CV_64F);

        for(int i=0; i < v.cols; i++) {
            Pcol.at<double>(0,0) = P.at<double>(0,i);
            Pcol.at<double>(1,0) = P.at<double>(1,i);
            Pcol.at<double>(2,0) = P.at<double>(2,i);

            Mat a = Sum(Sq((_eye - V[i])*(sol[j].R*Pcol + sol[j].t)));

            E = E + a.at<double>(0,0);
        }

        sol[j].E = E;
    }

    ret = sol;

    return true;
}

void GetRotationY_wrtT(const Mat &v, const Mat &p, const Mat &t, const Mat &Rz,
                        vector <double> &al, Mat &tnew, vector <double> &at)
{

/*
if nargin == 4,
 Rz = eye(3);
end
*/

    vector <Mat> V(v.cols);
    Mat vv(3,1,CV_64F);

    for(int i=0; i < v.cols; i++) {
        vv.at<double>(0,0) = v.at<double>(0,i);
        vv.at<double>(1,0) = v.at<double>(1,i);
        vv.at<double>(2,0) = v.at<double>(2,i);

        Mat tmp = vv.t()*vv;
        double a = tmp.at<double>(0,0);
        V[i] = (vv*vv.t()) / a;
    }

    //generate G
    Mat G = Mat::zeros(3,3,CV_64F);

    for(int i=0; i < v.cols; i++) {
       G += V[i];
    }

    Mat _eye = Mat::eye(3,3,CV_64F);

    G = (_eye - G/v.cols).inv()/v.cols;

    //printf("G\n");
    //Print(G);

    // generate opt_t*[bt^2 bt 1]

    Mat opt_t = Mat::zeros(3,3,CV_64F);

    for(int i=0; i < v.cols; i++) {
        double v11 = V[i].at<double>(0,0);
        double v12 = V[i].at<double>(0,1);
        double v13 = V[i].at<double>(0,2);
        double v21 = V[i].at<double>(1,0);
        double v22 = V[i].at<double>(1,1);
        double v23 = V[i].at<double>(1,2);
        double v31 = V[i].at<double>(2,0);
        double v32 = V[i].at<double>(2,1);
        double v33 = V[i].at<double>(2,2);

        double px = p.at<double>(0,i);
        double py = p.at<double>(1,i);
        double pz = p.at<double>(2,i);

        double r1 = Rz.at<double>(0,0);
        double r2 = Rz.at<double>(0,1);
        double r3 = Rz.at<double>(0,2);

        double r4 = Rz.at<double>(1,0);
        double r5 = Rz.at<double>(1,1);
        double r6 = Rz.at<double>(1,2);

        double r7 = Rz.at<double>(2,0);
        double r8 = Rz.at<double>(2,1);
        double r9 = Rz.at<double>(2,2);

        opt_t.at<double>(0,0) += (((v11-1)*r2+v12*r5+v13*r8)*py+(-(v11-1)*r1-v12*r4-v13*r7)*px+(-(v11-1)*r3-v12*r6-v13*r9)*pz);
        opt_t.at<double>(0,1) += ((2*(v11-1)*r1+2*v12*r4+2*v13*r7)*pz+(-2*(v11-1)*r3-2*v12*r6-2*v13*r9)*px);
        opt_t.at<double>(0,2) += ((v11-1)*r1+v12*r4+v13*r7)*px+((v11-1)*r3+v12*r6+v13*r9)*pz+((v11-1)*r2+v12*r5+v13*r8)*py;

        opt_t.at<double>(1,0) += ((v21*r2+(v22-1)*r5+v23*r8)*py+(-v21*r1-(v22-1)*r4-v23*r7)*px+(-v21*r3-(v22-1)*r6-v23*r9)*pz);
        opt_t.at<double>(1,1) += ((2*v21*r1+2*(v22-1)*r4+2*v23*r7)*pz+(-2*v21*r3-2*(v22-1)*r6-2*v23*r9)*px);
        opt_t.at<double>(1,2) += (v21*r1+(v22-1)*r4+v23*r7)*px+(v21*r3+(v22-1)*r6+v23*r9)*pz+(v21*r2+(v22-1)*r5+v23*r8)*py;

        opt_t.at<double>(2,0) += ((v31*r2+v32*r5+(v33-1)*r8)*py+(-v31*r1-v32*r4-(v33-1)*r7)*px+(-v31*r3-v32*r6-(v33-1)*r9)*pz);
        opt_t.at<double>(2,1) += ((2*v31*r1+2*v32*r4+2*(v33-1)*r7)*pz+(-2*v31*r3-2*v32*r6-2*(v33-1)*r9)*px);
        opt_t.at<double>(2,2) += (v31*r1+v32*r4+(v33-1)*r7)*px+(v31*r3+v32*r6+(v33-1)*r9)*pz+(v31*r2+v32*r5+(v33-1)*r8)*py;
    }

    opt_t = G*opt_t;

    Mat E_2 = Mat::zeros(1,5,CV_64F);


    // estimate Error function E
    for(int i=0; i < v.cols; i++) {
        double px = p.at<double>(0,i);
        double py = p.at<double>(1,i);
        double pz = p.at<double>(2,i);

        Mat Rpi(3,3,CV_64F);

        Rpi.at<double>(0,0) = -px;
        Rpi.at<double>(0,1) = 2*pz;
        Rpi.at<double>(0,2) = px;

        Rpi.at<double>(1,0) = py;
        Rpi.at<double>(1,1) = 0;
        Rpi.at<double>(1,2) = py;

        Rpi.at<double>(2,0) = -pz;
        Rpi.at<double>(2,1) = -2*px;
        Rpi.at<double>(2,2) = pz;

        Mat E = (_eye - V[i])*(Rz*Rpi + opt_t);

        Mat e0(3,1,CV_64F);
        Mat e1(3,1,CV_64F);
        Mat e2(3,1,CV_64F);

        e0.at<double>(0,0) = E.at<double>(0,2);
        e0.at<double>(1,0) = E.at<double>(1,2);
        e0.at<double>(2,0) = E.at<double>(2,2);

        e1.at<double>(0,0) = E.at<double>(0,1);
        e1.at<double>(1,0) = E.at<double>(1,1);
        e1.at<double>(2,0) = E.at<double>(2,1);

        e2.at<double>(0,0) = E.at<double>(0,0);
        e2.at<double>(1,0) = E.at<double>(1,0);
        e2.at<double>(2,0) = E.at<double>(2,0);

/*
        printf("E\n");
        Print(E);

        printf("e2\n");
        Print(e2);
*/
        Mat sum1 = Sum(Sq(e2));
             //printf("e2\n");
        //Print(e2);

        Mat sum2 = Sum(2*Mul(e1,e2));

        //printf("e2\n");
        //Print(e2);

        Mat sum3 = Sum(2*Mul(e0,e2) + Sq(e1));

        //printf("e2\n");
        //Print(e2);

        Mat sum4 = Sum(2*Mul(e0,e1));
        Mat sum5 = Sum(Sq(e0));




        // E_2 =E_2+ sum([e2.^2 2.*e1.*e2 (2.*e0.*e2+e1.^2) 2.*e0.*e1 e0.^2]);
        E_2.at<double>(0,0) += sum1.at<double>(0,0);
        E_2.at<double>(0,1) += sum2.at<double>(0,0);
        E_2.at<double>(0,2) += sum3.at<double>(0,0);
        E_2.at<double>(0,3) += sum4.at<double>(0,0);
        E_2.at<double>(0,4) += sum5.at<double>(0,0);
    }

    double e4=E_2.at<double>(0,0);
    double e3=E_2.at<double>(0,1);
    double e2=E_2.at<double>(0,2);
    double e1=E_2.at<double>(0,3);
    double e0=E_2.at<double>(0,4);

    //printf("e0 to e4 = %f %f %f %f %f\n", e0, e1, e2, e3, e4);

    double a4=-e3;
    double a3=(4*e4-2*e2);
    double a2=(-3*e1+3*e3);
    double a1=(-4*e0+2*e2);
    double a0=e1;

    double coeffs[5];
//    double z[10];
/*
    // backwards in GSL
    coeffs[0] = a0;
    coeffs[1] = a1;
    coeffs[2] = a2;
    coeffs[3] = a3;
    coeffs[4] = a4;
*/
    coeffs[0] = a4;
    coeffs[1] = a3;
    coeffs[2] = a2;
    coeffs[3] = a1;
    coeffs[4] = a0;

    //printf("coeffs = %f %f %f %f\n", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
    int degrees = 4;
    double zero_real[5];
    double zero_imag[5];

    memset(zero_real, 0, sizeof(double)*5);
    memset(zero_imag, 0, sizeof(double)*5);

    rpoly_ak1(coeffs, &degrees, zero_real, zero_imag);

/*
    gsl_poly_complex_workspace *w = gsl_poly_complex_workspace_alloc(5);
    gsl_poly_complex_solve (coeffs, 5, w, z);
    gsl_poly_complex_workspace_free (w);
*/
    // get all valid solutions -> which are real zero

    at.clear();

    // Nghia - modified a bit here, if it fails use the original
    for(int i=0; i < 5; i++) {
        double _at = zero_real[i];

        double p1 = pow(1.0 + _at*_at, 3.0);

        if(fabs(p1) > 0.1 && zero_imag[i] == 0) {
            at.push_back(_at);
        }
    }

/*
    // check for valid solutions
    for(int i=0; i < 8; i+=2) {
        double _at = z[i];

        double p1 = pow(1.0 + _at*_at, 3.0);

        if(fabs(p1) > 0.1 && z[i+1] == 0)
            at.push_back(_at);
    }
*/
    //printf("at size: %d\n", at.size());

    al.resize(at.size());

    vector <double> al2, at2;

    for(unsigned int i=0; i < at.size(); i++) {
        double sa = (2.f*at[i]) / (1.f +at[i]*at[i]);
        double ca = (1.f - at[i]*at[i]) / (1.f + at[i]*at[i]);

        al[i] = atan2(sa,ca) * 180/M_PI;

        double tMaxMin = (4*a4*at[i]*at[i]*at[i] + 3*a3*at[i]*at[i] + 2*a2*at[i] + a1);

        if(tMaxMin > 0) {
            al2.push_back(al[i]);
            at2.push_back(at[i]);
        }
    }

    al = al2;
    at = at2;

    tnew = Mat(3,al.size(),CV_64F);

    for(unsigned int a=0; a < al.size(); a++) {
        Mat R = Rz*RpyMat(Vec3d(0, (al[a]*M_PI/180), 0));
        Mat t_opt = Mat::zeros(3,1,CV_64F);

        Mat pcol(3,1,CV_64F);

        for(int i=0; i < v.cols; i++) {
            pcol.at<double>(0,0) = p.at<double>(0,i);
            pcol.at<double>(1,0) = p.at<double>(1,i);
            pcol.at<double>(2,0) = p.at<double>(2,i);

            t_opt = t_opt + (V[i] - _eye)*R*pcol;
        }

        t_opt = G*t_opt;

        tnew.at<double>(0,a) = t_opt.at<double>(0,0);
        tnew.at<double>(1,a) = t_opt.at<double>(1,0);
        tnew.at<double>(2,a) = t_opt.at<double>(2,0);
    }
}

void Print(const Mat &m)
{
    for(int i=0; i < m.rows; i++) {
        for(int j=0; j < m.cols; j++) {
            printf("%4.6f ", m.at<double>(i,j));
        }
        printf("\n");
    }

    printf("\n");
}

void Print(const Quaternion &q)
{
    printf("vector = %f %f %f\n", q.vector[0], q.vector[1], q.vector[2]);
    printf("scalar = %f\n", q.scalar);
    printf("\n");
}


void rpoly_ak1(double op[MDP1], int* Degree, double zeror[MAXDEGREE], double zeroi[MAXDEGREE]){
    int i, j, jj, l, N, NM1, NN, NZ, zerok;

    double K[MDP1], p[MDP1], pt[MDP1], qp[MDP1], temp[MDP1];
    double bnd, df, dx, factor, ff, moduli_max, moduli_min, sc, x, xm;
    double aa, bb, cc, lzi, lzr, sr, szi, szr, t, u, xx, xxx, yy;

    const double RADFAC = 3.14159265358979323846/180; // Degrees-to-radians conversion factor = pi/180
    const double lb2 = log(2.0); // Dummy variable to avoid re-calculating this value in loop below
    const double lo = DBL_MIN/DBL_EPSILON;
    const double cosr = cos(94.0*RADFAC); // = -0.069756474
    const double sinr = sin(94.0*RADFAC); // = 0.99756405

    if ((*Degree) > MAXDEGREE){
        cout << "\nThe entered Degree is greater than MAXDEGREE. Exiting rpoly. No further action taken.\n";
        *Degree = -1;
        return;
    } // End ((*Degree) > MAXDEGREE)

    //Do a quick check to see if leading coefficient is 0
    if (op[0] != 0){

        N = *Degree;
        xx = sqrt(0.5); // = 0.70710678
        yy = -xx;

        // Remove zeros at the origin, if any
        j = 0;
        while (op[N] == 0){
            zeror[j] = zeroi[j] = 0.0;
            N--;
            j++;
        } // End while (op[N] == 0)

        NN = N + 1;

        // Make a copy of the coefficients
        for (i = 0; i < NN; i++)
            p[i] = op[i];

        while (N >= 1){ // Main loop
            // Start the algorithm for one zero
            if (N <= 2){
            // Calculate the final zero or pair of zeros
                if (N < 2){
                    zeror[(*Degree) - 1] = -(p[1]/p[0]);
                    zeroi[(*Degree) - 1] = 0.0;
                } // End if (N < 2)
                else { // else N == 2
                    Quad_ak1(p[0], p[1], p[2], &zeror[(*Degree) - 2], &zeroi[(*Degree) - 2], &zeror[(*Degree) - 1], &zeroi[(*Degree) - 1]);
                } // End else N == 2
                break;
            } // End if (N <= 2)

            // Find the largest and smallest moduli of the coefficients

            moduli_max = 0.0;
            moduli_min = DBL_MAX;

            for (i = 0; i < NN; i++){
                x = fabs(p[i]);
                if (x > moduli_max)   moduli_max = x;
                if ((x != 0) && (x < moduli_min))   moduli_min = x;
            } // End for i

            // Scale if there are large or very small coefficients
            // Computes a scale factor to multiply the coefficients of the polynomial. The scaling
            // is done to avoid overflow and to avoid undetected underflow interfering with the
            // convergence criterion.
            // The factor is a power of the base.

            sc = lo/moduli_min;

            if (((sc <= 1.0) && (moduli_max >= 10)) || ((sc > 1.0) && (DBL_MAX/sc >= moduli_max))){
                sc = ((sc == 0) ? DBL_MIN : sc);
                l = (int)(log(sc)/lb2 + 0.5);
                factor = pow(2.0, l);
                if (factor != 1.0){
                    for (i = 0; i < NN; i++)   p[i] *= factor;
                } // End if (factor != 1.0)
            } // End if (((sc <= 1.0) && (moduli_max >= 10)) || ((sc > 1.0) && (DBL_MAX/sc >= moduli_max)))

            // Compute lower bound on moduli of zeros

            for (i = 0; i < NN; i++)   pt[i] = fabs(p[i]);
            pt[N] = -(pt[N]);

            NM1 = N - 1;

            // Compute upper estimate of bound

            x = exp((log(-pt[N]) - log(pt[0]))/(double)N);

            if (pt[NM1] != 0) {
                // If Newton step at the origin is better, use it
                xm = -pt[N]/pt[NM1];
                x = ((xm < x) ? xm : x);
            } // End if (pt[NM1] != 0)

            // Chop the interval (0, x) until ff <= 0

            xm = x;
            do {
                x = xm;
                xm = 0.1*x;
                ff = pt[0];
                for (i = 1; i < NN; i++)   ff = ff *xm + pt[i];
            } while (ff > 0); // End do-while loop

            dx = x;

            // Do Newton iteration until x converges to two decimal places

            do {
                df = ff = pt[0];
                for (i = 1; i < N; i++){
                    ff = x*ff + pt[i];
                    df = x*df + ff;
                } // End for i
                ff = x*ff + pt[N];
                dx = ff/df;
                x -= dx;
            } while (fabs(dx/x) > 0.005); // End do-while loop

            bnd = x;

            // Compute the derivative as the initial K polynomial and do 5 steps with no shift

            for (i = 1; i < N; i++)   K[i] = (double)(N - i)*p[i]/((double)N);
            K[0] = p[0];

            aa = p[N];
            bb = p[NM1];
            zerok = ((K[NM1] == 0) ? 1 : 0);

            for (jj = 0; jj < 5; jj++) {
                cc = K[NM1];
                if (zerok){
                    // Use unscaled form of recurrence
                    for (i = 0; i < NM1; i++){
                        j = NM1 - i;
                        K[j] = K[j - 1];
                    } // End for i
                    K[0] = 0;
                   zerok = ((K[NM1] == 0) ? 1 : 0);
                } // End if (zerok)

                else { // else !zerok
                    // Used scaled form of recurrence if value of K at 0 is nonzero
                    t = -aa/cc;
                    for (i = 0; i < NM1; i++){
                        j = NM1 - i;
                        K[j] = t*K[j - 1] + p[j];
                    } // End for i
                    K[0] = p[0];
                    zerok = ((fabs(K[NM1]) <= fabs(bb)*DBL_EPSILON*10.0) ? 1 : 0);
                } // End else !zerok

            } // End for jj

            // Save K for restarts with new shifts
            for (i = 0; i < N; i++)   temp[i] = K[i];

            // Loop to select the quadratic corresponding to each new shift

            for (jj = 1; jj <= 20; jj++){

                // Quadratic corresponds to a double shift to a non-real point and its
                // complex conjugate. The point has modulus BND and amplitude rotated
                // by 94 degrees from the previous shift.

                xxx = -(sinr*yy) + cosr*xx;
                yy = sinr*xx + cosr*yy;
                xx = xxx;
                sr = bnd*xx;
                u = -(2.0*sr);

                // Second stage calculation, fixed quadratic

                Fxshfr_ak1(20*jj, &NZ, sr, bnd, K, N, p, NN, qp, u, &lzi, &lzr, &szi, &szr);

                if (NZ != 0){

                    // The second stage jumps directly to one of the third stage iterations and
                    // returns here if successful. Deflate the polynomial, store the zero or
                    // zeros, and return to the main algorithm.

                    j = (*Degree) - N;
                    zeror[j] = szr;
                    zeroi[j] = szi;
                    NN = NN - NZ;
                    N = NN - 1;
                    for (i = 0; i < NN; i++)   p[i] = qp[i];
                    if (NZ != 1){
                        zeror[j + 1] = lzr;
                        zeroi[j + 1] = lzi;
                    } // End if (NZ != 1)
                    break;
                } // End if (NZ != 0)
                else { // Else (NZ == 0)

                    // If the iteration is unsuccessful, another quadratic is chosen after restoring K
                    for (i = 0; i < N; i++)   K[i] = temp[i];
                } // End else (NZ == 0)

            } // End for jj

            // Return with failure if no convergence with 20 shifts

            if (jj > 20) {
                cout << "\nFailure. No convergence after 20 shifts. Program terminated.\n";
                *Degree -= N;
                break;
            } // End if (jj > 20)

        } // End while (N >= 1)
    } // End if op[0] != 0
    else { // else op[0] == 0
        cout << "\nThe leading coefficient is zero. No further action taken. Program terminated.\n";
        *Degree = 0;
    } // End else op[0] == 0

    return;
} // End rpoly_ak1

void Fxshfr_ak1(int L2, int* NZ, double sr, double v, double K[MDP1], int N, double p[MDP1], int NN, double qp[MDP1], double u, double* lzi, double* lzr, double* szi, double* szr)
{

    // Computes up to L2 fixed shift K-polynomials, testing for convergence in the linear or
    // quadratic case. Initiates one of the variable shift iterations and returns with the
    // number of zeros found.

    // L2 limit of fixed shift steps
    // NZ number of zeros found

    int fflag, i, iFlag = 1, j, spass, stry, tFlag, vpass, vtry;
    double a, a1, a3, a7, b, betas, betav, c, d, e, f, g, h, oss, ots=0, otv=0, ovv, s, ss, ts, tss, tv, tvv, ui, vi, vv;
    double qk[MDP1], svk[MDP1];

    *NZ = 0;
    betav = betas = 0.25;
    oss = sr;
    ovv = v;

    //Evaluate polynomial by synthetic division
    QuadSD_ak1(NN, u, v, p, qp, &a, &b);

    tFlag = calcSC_ak1(N, a, b, &a1, &a3, &a7, &c, &d, &e, &f, &g, &h, K, u, v, qk);

    for (j = 0; j < L2; j++){

        fflag = 1;
        //Calculate next K polynomial and estimate v
        nextK_ak1(N, tFlag, a, b, a1, &a3, &a7, K, qk, qp);
        tFlag = calcSC_ak1(N, a, b, &a1, &a3, &a7, &c, &d, &e, &f, &g, &h, K, u, v, qk);
        newest_ak1(tFlag, &ui, &vi, a, a1, a3, a7, b, c, d, f, g, h, u, v, K, N, p);

        vv = vi;

        // Estimate s

        ss = ((K[N - 1] != 0.0) ? -(p[N]/K[N - 1]) : 0.0);

        ts = tv = 1.0;

        if ((j != 0) && (tFlag != 3)){

           // Compute relative measures of convergence of s and v sequences

            tv = ((vv != 0.0) ? fabs((vv - ovv)/vv) : tv);
            ts = ((ss != 0.0) ? fabs((ss - oss)/ss) : ts);

            // If decreasing, multiply the two most recent convergence measures

            tvv = ((tv < otv) ? tv*otv : 1.0);
            tss = ((ts < ots) ? ts*ots : 1.0);

            // Compare with convergence criteria

            vpass = ((tvv < betav) ? 1 : 0);
            spass = ((tss < betas) ? 1 : 0);

            if ((spass) || (vpass)){

                // At least one sequence has passed the convergence test.
                // Store variables before iterating

                for (i = 0; i < N; i++)   svk[i] = K[i];

                s = ss;

                // Choose iteration according to the fastest converging sequence

                stry = vtry = 0;

                for ( ; ; ) {

                    if ((fflag && ((fflag = 0) == 0)) && ((spass) && (!vpass || (tss < tvv)))){
                        ; // Do nothing. Provides a quick "short circuit".
                    } // End if (fflag)

                    else { // else !fflag
                        QuadIT_ak1(N, NZ, ui, vi, szr, szi, lzr, lzi, qp, NN, &a, &b, p, qk, &a1, &a3, &a7, &c, &d, &e, &f, &g, &h, K);

                        if ((*NZ) > 0)   return;

                        // Quadratic iteration has failed. Flag that it has been tried and decrease the
                        // convergence criterion

                        iFlag = vtry = 1;
                        betav *= 0.25;

                        // Try linear iteration if it has not been tried and the s sequence is converging
                        if (stry || (!spass)){
                            iFlag = 0;
                        } // End if (stry || (!spass))
                        else {
                            for (i = 0; i < N; i++)   K[i] = svk[i];
                        } // End if (stry || !spass)

                    } // End else fflag

                    if (iFlag != 0){
                        RealIT_ak1(&iFlag, NZ, &s, N, p, NN, qp, szr, szi, K, qk);

                        if ((*NZ) > 0)   return;

                        // Linear iteration has failed. Flag that it has been tried and decrease the
                        // convergence criterion

                        stry = 1;
                        betas *= 0.25;

                        if (iFlag != 0){

                            // If linear iteration signals an almost double real zero, attempt quadratic iteration

                            ui = -(s + s);
                            vi = s*s;
                            continue;

                        } // End if (iFlag != 0)
                    } // End if (iFlag != 0)

                    // Restore variables
                    for (i = 0; i < N; i++)   K[i] = svk[i];

                    // Try quadratic iteration if it has not been tried and the v sequence is converging

                    if (!vpass || vtry)   break; // Break out of infinite for loop

                } // End infinite for loop

                // Re-compute qp and scalar values to continue the second stage

                QuadSD_ak1(NN, u, v, p, qp, &a, &b);
                tFlag = calcSC_ak1(N, a, b, &a1, &a3, &a7, &c, &d, &e, &f, &g, &h, K, u, v, qk);

            } // End if ((spass) || (vpass))

        } // End if ((j != 0) && (tFlag != 3))

        ovv = vv;
        oss = ss;
        otv = tv;
        ots = ts;
    } // End for j

    return;
} // End Fxshfr_ak1

void QuadSD_ak1(int NN, double u, double v, double p[MDP1], double q[MDP1], double* a, double* b)
{
    // Divides p by the quadratic 1, u, v placing the quotient in q and the remainder in a, b

    int i;

    q[0] = *b = p[0];
    q[1] = *a = -((*b)*u) + p[1];

    for (i = 2; i < NN; i++){
        q[i] = -((*a)*u + (*b)*v) + p[i];
        *b = (*a);
        *a = q[i];
    } // End for i

    return;
} // End QuadSD_ak1

int calcSC_ak1(int N, double a, double b, double* a1, double* a3, double* a7, double* c, double* d, double* e, double* f, double* g, double* h, double K[MDP1], double u, double v, double qk[MDP1])
{

    // This routine calculates scalar quantities used to compute the next K polynomial and
    // new estimates of the quadratic coefficients.

    // calcSC - integer variable set here indicating how the calculations are normalized
    // to avoid overflow.

    int dumFlag = 3; // TYPE = 3 indicates the quadratic is almost a factor of K

    // Synthetic division of K by the quadratic 1, u, v
    QuadSD_ak1(N, u, v, K, qk, c, d);

    if (fabs((*c)) <= (100.0*DBL_EPSILON*fabs(K[N - 1]))) {
        if (fabs((*d)) <= (100.0*DBL_EPSILON*fabs(K[N - 2])))   return dumFlag;
    } // End if (fabs(c) <= (100.0*DBL_EPSILON*fabs(K[N - 1])))

    *h = v*b;
    if (fabs((*d)) >= fabs((*c))){
        dumFlag = 2; // TYPE = 2 indicates that all formulas are divided by d
        *e = a/(*d);
        *f = (*c)/(*d);
        *g = u*b;
        *a3 = (*e)*((*g) + a) + (*h)*(b/(*d));
        *a1 = -a + (*f)*b;
        *a7 = (*h) + ((*f) + u)*a;
    } // End if(fabs(d) >= fabs(c))
    else {
        dumFlag = 1; // TYPE = 1 indicates that all formulas are divided by c;
        *e = a/(*c);
        *f = (*d)/(*c);
        *g = (*e)*u;
        *a3 = (*e)*a + ((*g) + (*h)/(*c))*b;
        *a1 = -(a*((*d)/(*c))) + b;
        *a7 = (*g)*(*d) + (*h)*(*f) + a;
    } // End else

    return dumFlag;
} // End calcSC_ak1

void nextK_ak1(int N, int tFlag, double a, double b, double a1, double* a3, double* a7, double K[MDP1], double qk[MDP1], double qp[MDP1])
{
    // Computes the next K polynomials using the scalars computed in calcSC_ak1

    int i;
    double temp;

    if (tFlag == 3){ // Use unscaled form of the recurrence
        K[1] = K[0] = 0.0;

        for (i = 2; i < N; i++)   K[i] = qk[i - 2];

        return;
    } // End if (tFlag == 3)

    temp = ((tFlag == 1) ? b : a);

    if (fabs(a1) > (10.0*DBL_EPSILON*fabs(temp))){
        // Use scaled form of the recurrence

        (*a7) /= a1;
        (*a3) /= a1;
        K[0] = qp[0];
        K[1] = -((*a7)*qp[0]) + qp[1];

        for (i = 2; i < N; i++)   K[i] = -((*a7)*qp[i - 1]) + (*a3)*qk[i - 2] + qp[i];

    } // End if (fabs(a1) > (10.0*DBL_EPSILON*fabs(temp)))
    else {
        // If a1 is nearly zero, then use a special form of the recurrence

        K[0] = 0.0;
        K[1] = -(*a7)*qp[0];

        for (i = 2; i < N; i++)   K[i] = -((*a7)*qp[i - 1]) + (*a3)*qk[i - 2];
    } // End else

    return;
} // End nextK_ak1

void newest_ak1(int tFlag, double* uu, double* vv, double a, double a1, double a3, double a7, double b, double c, double d, double f, double g, double h, double u, double v, double K[MDP1], int N, double p[MDP1])
{
    // Compute new estimates of the quadratic coefficients using the scalars computed in calcSC_ak1

    double a4, a5, b1, b2, c1, c2, c3, c4, temp;

    (*vv) = (*uu) = 0.0; // The quadratic is zeroed

    if (tFlag != 3){

        if (tFlag != 2){
            a4 = a + u*b + h*f;
            a5 = c + (u + v*f)*d;
        } // End if (tFlag != 2)
        else { // else tFlag == 2
            a4 = (a + g)*f + h;
            a5 = (f + u)*c + v*d;
        } // End else tFlag == 2

        // Evaluate new quadratic coefficients

        b1 = -K[N - 1]/p[N];
        b2 = -(K[N - 2] + b1*p[N - 1])/p[N];
        c1 = v*b2*a1;
        c2 = b1*a7;
        c3 = b1*b1*a3;
        c4 = -(c2 + c3) + c1;
        temp = -c4 + a5 + b1*a4;
        if (temp != 0.0) {
            *uu= -((u*(c3 + c2) + v*(b1*a1 + b2*a7))/temp) + u;
            *vv = v*(1.0 + c4/temp);
        } // End if (temp != 0)

    } // End if (tFlag != 3)

    return;
} // End newest_ak1

void QuadIT_ak1(int N, int* NZ, double uu, double vv, double* szr, double* szi, double* lzr, double* lzi, double qp[MDP1], int NN, double* a, double* b, double p[MDP1], double qk[MDP1], double* a1, double* a3, double* a7, double* c, double* d, double* e, double* f, double* g, double* h, double K[MDP1])
{
    // Variable-shift K-polynomial iteration for a quadratic factor converges only if the
    // zeros are equimodular or nearly so.

    int i, j = 0, tFlag, triedFlag = 0;
    double ee, mp, omp, relstp=0, t, u, ui, v, vi, zm;

    *NZ = 0; // Number of zeros found
    u = uu; // uu and vv are coefficients of the starting quadratic
    v = vv;

    do {
        Quad_ak1(1.0, u, v, szr, szi, lzr, lzi);

        // Return if roots of the quadratic are real and not close to multiple or nearly
        // equal and of opposite sign.

        if (fabs(fabs(*szr) - fabs(*lzr)) > 0.01*fabs(*lzr))   break;

        // Evaluate polynomial by quadratic synthetic division

        QuadSD_ak1(NN, u, v, p, qp, a, b);

        mp = fabs(-((*szr)*(*b)) + (*a)) + fabs((*szi)*(*b));

        // Compute a rigorous bound on the rounding error in evaluating p

        zm = sqrt(fabs(v));
        ee = 2.0*fabs(qp[0]);
        t = -((*szr)*(*b));

        for (i = 1; i < N; i++)   ee = ee*zm + fabs(qp[i]);

        ee = ee*zm + fabs((*a) + t);
        ee = (9.0*ee + 2.0*fabs(t) - 7.0*(fabs((*a) + t) + zm*fabs((*b))))*DBL_EPSILON;

        // Iteration has converged sufficiently if the polynomial value is less than 20 times this bound

        if (mp <= 20.0*ee){
            *NZ = 2;
            break;
        } // End if (mp <= 20.0*ee)

        j++;

        // Stop iteration after 20 steps
        if (j > 20)   break;

        if (j >= 2){
            if ((relstp <= 0.01) && (mp >= omp) && (!triedFlag)){
            // A cluster appears to be stalling the convergence. Five fixed shift
            // steps are taken with a u, v close to the cluster.

            relstp = ((relstp < DBL_EPSILON) ? sqrt(DBL_EPSILON) : sqrt(relstp));

            u -= u*relstp;
            v += v*relstp;

            QuadSD_ak1(NN, u, v, p, qp, a, b);

            for (i = 0; i < 5; i++){
                tFlag = calcSC_ak1(N, *a, *b, a1, a3, a7, c, d, e, f, g, h, K, u, v, qk);
                nextK_ak1(N, tFlag, *a, *b, *a1, a3, a7, K, qk, qp);
            } // End for i

            triedFlag = 1;
            j = 0;

            } // End if ((relstp <= 0.01) && (mp >= omp) && (!triedFlag))

        } // End if (j >= 2)

        omp = mp;

        // Calculate next K polynomial and new u and v

        tFlag = calcSC_ak1(N, *a, *b, a1, a3, a7, c, d, e, f, g, h, K, u, v, qk);
        nextK_ak1(N, tFlag, *a, *b, *a1, a3, a7, K, qk, qp);
        tFlag = calcSC_ak1(N, *a, *b, a1, a3, a7, c, d, e, f, g, h, K, u, v, qk);
        newest_ak1(tFlag, &ui, &vi, *a, *a1, *a3, *a7, *b, *c, *d, *f, *g, *h, u, v, K, N, p);

        // If vi is zero, the iteration is not converging
        if (vi != 0){
            relstp = fabs((-v + vi)/vi);
            u = ui;
            v = vi;
        } // End if (vi != 0)
    } while (vi != 0); // End do-while loop

    return;

} //End QuadIT_ak1

void RealIT_ak1(int* iFlag, int* NZ, double* sss, int N, double p[MDP1], int NN, double qp[MDP1], double* szr, double* szi, double K[MDP1], double qk[MDP1])
{

    // Variable-shift H-polynomial iteration for a real zero

    // sss - starting iterate
    // NZ - number of zeros found
    // iFlag - flag to indicate a pair of zeros near real axis

    int i, j = 0, nm1 = N - 1;
    double ee, kv, mp, ms, omp, pv, s, t=0;

    *iFlag = *NZ = 0;
    s = *sss;

    for ( ; ; ) {
        pv = p[0];

        // Evaluate p at s
        qp[0] = pv;
        for (i = 1; i < NN; i++)   qp[i] = pv = pv*s + p[i];

        mp = fabs(pv);

        // Compute a rigorous bound on the error in evaluating p

        ms = fabs(s);
        ee = 0.5*fabs(qp[0]);
        for (i = 1; i < NN; i++)   ee = ee*ms + fabs(qp[i]);

        // Iteration has converged sufficiently if the polynomial value is less than
        // 20 times this bound

        if (mp <= 20.0*DBL_EPSILON*(2.0*ee - mp)){
            *NZ = 1;
            *szr = s;
            *szi = 0.0;
            break;
        } // End if (mp <= 20.0*DBL_EPSILON*(2.0*ee - mp))

        j++;

        // Stop iteration after 10 steps

        if (j > 10)   break;

        if (j >= 2){
            if ((fabs(t) <= 0.001*fabs(-t + s)) && (mp > omp)){
                // A cluster of zeros near the real axis has been encountered;
                // Return with iFlag set to initiate a quadratic iteration

                *iFlag = 1;
                *sss = s;
                break;
            } // End if ((fabs(t) <= 0.001*fabs(s - t)) && (mp > omp))

        } //End if (j >= 2)

        // Return if the polynomial value has increased significantly

        omp = mp;

        // Compute t, the next polynomial and the new iterate
        qk[0] = kv = K[0];
        for (i = 1; i < N; i++)   qk[i] = kv = kv*s + K[i];

        if (fabs(kv) > fabs(K[nm1])*10.0*DBL_EPSILON){
            // Use the scaled form of the recurrence if the value of K at s is non-zero
            t = -(pv/kv);
            K[0] = qp[0];
            for (i = 1; i < N; i++)   K[i] = t*qk[i - 1] + qp[i];
        } // End if (fabs(kv) > fabs(K[nm1])*10.0*DBL_EPSILON)
        else { // else (fabs(kv) <= fabs(K[nm1])*10.0*DBL_EPSILON)
            // Use unscaled form
            K[0] = 0.0;
            for (i = 1; i < N; i++)   K[i] = qk[i - 1];
        } // End else (fabs(kv) <= fabs(K[nm1])*10.0*DBL_EPSILON)

        kv = K[0];
        for (i = 1; i < N; i++)   kv = kv*s + K[i];

        t = ((fabs(kv) > (fabs(K[nm1])*10.0*DBL_EPSILON)) ? -(pv/kv) : 0.0);

        s += t;

    } // End infinite for loop

    return;

} // End RealIT_ak1

void Quad_ak1(double a, double b1, double c, double* sr, double* si, double* lr, double* li)
{
    // Calculates the zeros of the quadratic a*Z^2 + b1*Z + c
    // The quadratic formula, modified to avoid overflow, is used to find the larger zero if the
    // zeros are real and both zeros are complex. The smaller real zero is found directly from
    // the product of the zeros c/a.

    double b, d, e;

    *sr = *si = *lr = *li = 0.0;

    if (a == 0) {
        *sr = ((b1 != 0) ? -(c/b1) : *sr);
        return;
    } // End if (a == 0))

    if (c == 0){
        *lr = -(b1/a);
        return;
    } // End if (c == 0)

    // Compute discriminant avoiding overflow

    b = b1/2.0;
    if (fabs(b) < fabs(c)){
        e = ((c >= 0) ? a : -a);
        e = -e + b*(b/fabs(c));
        d = sqrt(fabs(e))*sqrt(fabs(c));
    } // End if (fabs(b) < fabs(c))
    else { // Else (fabs(b) >= fabs(c))
        e = -((a/b)*(c/b)) + 1.0;
        d = sqrt(fabs(e))*(fabs(b));
    } // End else (fabs(b) >= fabs(c))

    if (e >= 0) {
        // Real zeros

        d = ((b >= 0) ? -d : d);
        *lr = (-b + d)/a;
        *sr = ((*lr != 0) ? (c/(*lr))/a : *sr);
    } // End if (e >= 0)
    else { // Else (e < 0)
        // Complex conjugate zeros

        *lr = *sr = -(b/a);
        *si = fabs(d/a);
        *li = -(*si);
    } // End else (e < 0)

    return;
} // End Quad_ak1

}
