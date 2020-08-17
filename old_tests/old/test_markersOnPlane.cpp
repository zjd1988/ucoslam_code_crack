#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "optimization/sparselevmarq.h"
#include "optimization/levmarq.h"
#include "utils.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
//#include <g2o/solvers/dense/linear_solver_dense.h>
//#include <g2o/types/types_seven_dof_expmap.h>
#include <g2o/core/base_binary_edge.h>
#include "basic_types.h"

using namespace std;template< typename T>
void   toStream__ts ( const  std::vector<T> &v,std::ostream &str ) {
    uint32_t s=v.size();
    str.write ( ( char* ) &s,sizeof ( s) );
    for(size_t i=0;i<v.size();i++) v[i].toStream( str);
}

template< typename T>
void   fromStream__ts (    std::vector<T> &v,std::istream &str ) {
    uint32_t s;
    str.read( ( char* ) &s,sizeof ( s) );
    v.resize(s);
    for(size_t i=0;i<v.size();i++) v[i].fromStream( str);
}

struct CamerasInfo{
    cv::Size camSize;
    cv::Mat K,dist;
    vector<ucoslam::Se3Transform> RT;

};

//struct Point2f{
//    float x,y;
//};
//struct Point3f{
//    float x,y,z;
//};

struct PointView{
    int camIndex;
    cv::Point2f p2d;

    void toStream(ostream &str)const{
        str.write((char*)&camIndex,sizeof(camIndex));
        str.write((char*)&p2d,sizeof(p2d));
    }
    void fromStream(istream &str){
        str.read((char*)&camIndex,sizeof(camIndex));
        str.read((char*)&p2d,sizeof(p2d));
          }
};

struct PointInfo{
    cv::Point3f p3d;
    vector<PointView> pointViews;
    void toStream(ostream &str)const{
        str.write((char*)&p3d,sizeof(p3d));
        toStream__ts(pointViews,str);
    }
    void fromStream(istream &str){
        str.read((char*)&p3d,sizeof(p3d));
        fromStream__ts(pointViews,str);
    }
};


struct BA_Data{

    BA_Data(){K=cv::Mat::eye(3,3,CV_32F);dist=cv::Mat::zeros(1,5,CV_32F);}
    cv::Size camSize;
    cv::Mat K,dist;
    vector<ucoslam::Se3Transform> cameraSetPoses;
    vector<PointInfo> pointSetInfo;


    void saveToFile(string path)const{
        std::ofstream file(path,ios::binary);
        if (!file) throw std::runtime_error("Could not open file for writing");
        uint64_t sig=23123;
        file.write((char*)&sig,sizeof(sig));
        toStream(file);
    }
    void readFromFile(string path){

        std::ifstream file(path,ios::binary);
        if (!file) throw std::runtime_error("Could not open file for reading");
        uint64_t sig=23123;
        file.read((char*)&sig,sizeof(sig));
        if (sig!=23123)throw std::runtime_error("Invalid file type");
        fromStream(file);
    }

    void toStream(ostream &str)const{
        str.write(K.ptr<char>(0),9*sizeof(float));
        str.write(dist.ptr<char>(0),5*sizeof(float));
        str.write((char*)&camSize,sizeof(camSize));

        toStream__ts(cameraSetPoses,str);
        toStream__ts(pointSetInfo,str);

    }
    void fromStream(istream &str){
        str.read(K.ptr<char>(0),9*sizeof(float));
        cout<<K<<endl;
        str.read(dist.ptr<char>(0),5*sizeof(float));
        cout<<dist<<endl;
        str.read((char*)&camSize,sizeof(camSize));
        fromStream__ts(cameraSetPoses,str);
        fromStream__ts(pointSetInfo,str);

    }
};

struct MarkerInfo{

    uint32_t id;//
    PointInfo pointSetInfo[4];
    ucoslam::Se3Transform pose;

    friend ostream &operator<<(ostream &str,const MarkerInfo &mi){
        str<<mi.id<<" ";
        for(int i=0;i<4;i++)
            str<<mi.pointSetInfo[i].p3d<<" ";
        for(int nv=0;nv<mi.pointSetInfo[0].pointViews.size();nv++)
            cout<< mi.pointSetInfo[0].pointViews[nv].camIndex<<" "<<mi.pointSetInfo[0].pointViews[nv].p2d<<" "<<mi.pointSetInfo[1].pointViews[nv].p2d<<" "<<mi.pointSetInfo[2].pointViews[nv].p2d<<" "<<mi.pointSetInfo[3].pointViews[nv].p2d<<endl;

        return str;
    }
};

struct CameraPointsProyected: vector<int>{
};

void loadCamerasFile(string pathToFile,BA_Data &BAData){
    ifstream file(pathToFile);
    if (!file)throw std::runtime_error("Could not open cameras file");

    string line;
    do{
        std::getline(file,line);
    }while(line[0]=='%');
    std::stringstream sline(line);

    sline>>BAData.camSize.width>>BAData.camSize.height;
    float fxy,cx,cy;
    sline>>fxy>>cx>>cy;
    BAData.K.at<float>(0,0)=fxy;
    BAData.K.at<float>(1,1)=fxy;
    BAData.K.at<float>(0,2)=cx;
    BAData.K.at<float>(1,2)=cy;

    float *pdist=BAData.dist.ptr<float>(0);
    for(int i=0;i<5;i++)
        sline>>pdist[i];


    //now, start reading cameras
    //    ci.RT.push_back(cv::Mat());//leave first element empty
    while(!file.eof()){
        std::getline(file,line);
        std::stringstream sline(line);
        //read first index
        int indx;
        if (!(sline>>indx)) break;
        indx--;//one less
        if (indx!=BAData.cameraSetPoses.size()) std::runtime_error("Error in camera index. They shhoudl start in 1 and be consecutive");

        //read the info
        cv::Mat T=cv::Mat::eye(4,4,CV_32F);
        cv::Mat R44=cv::Mat::eye(4,4,CV_32F);

        BAData.cameraSetPoses.push_back(ucoslam::Se3Transform());

        sline >>T.at<float>(0,3)>>T.at<float>(1,3)>>T.at<float>(2,3);
        for(int i=0;i<3;i++)
            T.at<float>(i,3)=-T.at<float>(i,3);
        cv::Mat R33=R44.rowRange(0,3).colRange(0,3);

        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                sline >>R33.at<float>(i,j);
        R33=R33.t();
        BAData.cameraSetPoses.back()=R44*T;

    }
}
void loadPointsInfo(string path,BA_Data &BADDATA){
    ifstream file(path);
    if (!file)throw std::runtime_error("Could not open points file");

    string line;
    do{
        std::getline(file,line);
    }while(line[0]=='%');

    vector<PointInfo> points;

    while(!file.eof()){
        std::stringstream sline(line);
        PointInfo pi;
        if (! (sline>>pi.p3d.x>>pi.p3d.y>>pi.p3d.z)) break;
        //now, read proyections
        int cam;cv::Point2f p2d;
        while(  sline>>cam>>p2d.x>>p2d.y ){
            pi.pointViews.push_back({cam-1,p2d});
        }
        BADDATA.pointSetInfo.push_back(pi);

        std::getline(file,line);
    };


}

vector<MarkerInfo> loadMarkerInfo(string path){
    ifstream file(path);
    if (!file)throw std::runtime_error("Could not open marker file");

    string line;
    do{
        std::getline(file,line);
    }while(line[0]=='%');

    vector<MarkerInfo> mivector;
    while(!file.eof()){
        std::stringstream sline(line);
        MarkerInfo mi;
        if (! (sline>>mi.id)) break;
        //      cout<<mi.id<<endl;
        //read 3d marker locations
        for(int i=0;i<4;i++)
            sline>>mi.pointSetInfo[i].p3d.x>>mi.pointSetInfo[i].p3d.y>>mi.pointSetInfo[i].p3d.z;

        //        cout<<mi.p3d[0]<<endl;
        //now, read proyections
        PointView pv;
        while(  sline>>pv.camIndex){
            pv.camIndex--;
            for(int i=0;i<4;i++) {
                sline>>pv.p2d.x>>pv.p2d.y;
                mi.pointSetInfo[i].pointViews.push_back(pv);
            }
        }


        cout<<mi<<endl;
        mivector.push_back(mi);

        std::getline(file,line);
    };
    return mivector;
}


inline cv::Point2f project(const cv::Point3f &p3d,const cv::Mat &cameraMatrix,const cv::Mat &RT,const cv::Mat & Distortion=cv::Mat()){


    assert(cameraMatrix.type()==CV_32F);
    assert(RT.type()==CV_32F);
    assert( (Distortion.type()==CV_32F &&  Distortion.total()>=5 )|| Distortion.total()==0);

    const float *cm=cameraMatrix.ptr<float>(0);
    const float *rt=RT.ptr<float>(0);

    //project point first
    float x= p3d.x* rt[0]+p3d.y* rt[1]+p3d.z* rt[2]+rt[3];
    float y= p3d.x* rt[4]+p3d.y* rt[5]+p3d.z* rt[6]+rt[7];
    float z= p3d.x* rt[8]+p3d.y* rt[9]+p3d.z* rt[10]+rt[11];




    float xx=x/z;
    float yy=y/z;

    if (Distortion.total()==5){//apply distortion //        k1,k2,p1,p2[,k3
        const float *k=Distortion.ptr<float>(0);
        float xx2=xx*xx;
        float yy2=yy*yy;
        float r2=xx2+yy2;
        float r4=r2*r2;
        float comm=1+k[0]*r2+k[1]*r4+k[4]*(r4*r2);
        float xxx = xx * comm + 2*k[2] *xx2+ k[3]*(r2+2*xx2);
        float yyy= yy*comm+ k[2]*(r2+2*yy2)+2*k[3]*xx*yy;
        xx=xxx;
        yy=yyy;
    }
    else if(Distortion.total()==8){// k1,k2,p1,p2,k3,k4,k5,k6
        const float *k=Distortion.ptr<float>(0);
        float xx2=xx*xx;
        float yy2=yy*yy;
        float r2=xx2+yy2;
        float r4=r2*r2;
        float num=1+k[0]*r2+k[1]*r4+k[4]*(r4*r2);
        float den=1+k[5]*r2+k[6]*r4+k[7]*(r4*r2);
        float comm=num/den;
        float xxx = xx * comm + 2*k[2] *xx2+ k[3]*(r2+2*xx2);
        float yyy= yy*comm+ k[2]*(r2+2*yy2)+2*k[3]*xx*yy;
        xx=xxx;
        yy=yyy;
    }
    return cv::Point2f((xx*cm[0])+cm[2],(yy*cm[4])+cm[5] );
}


vector<MarkerInfo> minfo;
vector<CameraPointsProyected> CamPointsProjected;

inline float hubber_weight(float x,float delta){
    if(x<0)x=-x;
    if (x<delta)return 1;
    else return sqrt(2*delta*x-delta*delta)/x;
}



void refineCameraParameters(BA_Data &TBAD){



    auto errorFunction_CameraParams=[&](const Eigen::Matrix<double,Eigen::Dynamic,1> &sol,Eigen::Matrix<double,Eigen::Dynamic,1> &err){
        //for each point
        float k[9]={sol(0),0,sol(1),0,sol(0),sol(2),0,0,1};
        cv::Mat K(3,3,CV_32F,k);
        float dist[5]={sol(3),sol(4),sol(5),sol(6),sol(7)};

        cv::Mat Dist(1,5,CV_32F,dist);
        int nerr=0;
        for( PointInfo &pi:TBAD.pointSetInfo)
            nerr+=pi.pointViews.size();
        nerr*=2;
        err.resize(nerr);
        //compute the number of errors
        int errIdx=0;
        for( PointInfo &pi:TBAD.pointSetInfo){
            for(auto cp:pi.pointViews){
                cv::Point2f reprj_err=project(pi.p3d,K,TBAD.cameraSetPoses[cp.camIndex],Dist)- cp.p2d;
                assert(!isnan( reprj_err.x) && !isnan( reprj_err.y));
                err(errIdx++)= (reprj_err.x);
                err(errIdx++)= (reprj_err.y);
            }
        }
    };

    ucoslam::LevMarq<double> optimizer;
    //    optimizer.setParams(1000,1e-3,1e-3);
    ucoslam::LevMarq<double>::eVector sol(8),err;
    sol(0)=TBAD.K.at<float>(0,0);//fx
    sol(1)=TBAD.K.at<float>(0,2);//cx
    sol(2)=TBAD.K.at<float>(1,2);//cy

    errorFunction_CameraParams(sol,err);
    cout<<"Initial err="<<err.cwiseProduct(err).sum()<<endl;

    for(int i=0;i<5;i++)
        sol(3+i)=TBAD.dist.ptr<float>()[i];
    cout<<sol<<endl;


    //now, optimize
    optimizer.verbose()=true;
    optimizer.solve(sol,std::bind(errorFunction_CameraParams,std::placeholders::_1,std::placeholders::_2));
    cout<<sol<<endl;

    TBAD.K.at<float>(0,0)=sol(0);
    TBAD.K.at<float>(1,1)=sol(0);//fy
    TBAD.K.at<float>(0,2)=sol(1);//cx
    TBAD.K.at<float>(1,2)=sol(2);//cy
    for(int i=0;i<5;i++)
        TBAD.dist.ptr<float>()[i]=sol(3+i);
}





void FullBA(BA_Data &TBAD,int niters){
    auto  toSE3Quat=[](const cv::Mat &cvT)
    {
        Eigen::Matrix<double,3,3> R;
        R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
                cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
                cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

        Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

        return g2o::SE3Quat(R,t);
    };
    g2o::SparseOptimizer Optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    Optimizer.setAlgorithm(solver);
    Optimizer.setVerbose( true);



    for(int i=0;i< TBAD.cameraSetPoses.size();i++){
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(toSE3Quat(TBAD.cameraSetPoses[i]));
        vSE3->setId(i);
        //       if (_params.fixed_frames.count( fid)  ) vSE3->setFixed(true);
        Optimizer.addVertex(vSE3);
    }
    float fx=TBAD.K.at<float>(0,0);
    float fy=TBAD.K.at<float>(1,1);
    float cx=TBAD.K.at<float>(0,2);
    float cy=TBAD.K.at<float>(1,2);
    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    Optimizer.addParameter( camera );



    //NOw points
    const float thHuber2D = sqrt(5.99);
    int pid=TBAD.cameraSetPoses.size();
    for(PointInfo pi:TBAD.pointSetInfo){

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        Eigen::Matrix<double,3,1> vp3d;
        vp3d << pi.p3d.x , pi.p3d.y ,pi.p3d.z;
        vPoint->setEstimate(vp3d);
        vPoint->setId( pid);
        vPoint->setMarginalized(true);
        Optimizer.addVertex(vPoint);
        //SET EDGES
        for( const auto  proj :pi.pointViews)
        {

            Eigen::Matrix<double,2,1> obs;
            obs << proj.p2d.x, proj.p2d.y;


            g2o::EdgeProjectXYZ2UV * e = new  g2o::EdgeProjectXYZ2UV();
            e->setParameterId(0, 0);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer.vertex(pid)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer.vertex(proj.camIndex)));
            e->setMeasurement(obs);
            e->setInformation(Eigen::Matrix2d::Identity()/**_InvScaleFactors[kpUn.octave]*/);

            if(true)
            {
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber2D);
            }
            Optimizer.addEdge(e);
        }
        pid++;
    }
    Optimizer.initializeOptimization();
    Optimizer.setVerbose(true);
    //    Optimizer->optimize(_params.nIters);
    Optimizer.optimize(niters);



    //move points and cameras back to the origin



    auto toCvMat=[](const g2o::SE3Quat &SE3)
    {
        Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
        cv::Mat cvMat(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0; j<4; j++)
                cvMat.at<float>(i,j)=eigMat(i,j);

        return cvMat.clone();

    };
    // Recover optimized data

    //Keyframes
    for(int i=0;i< TBAD.cameraSetPoses.size();i++)
    {
        g2o::SE3Quat SE3quat = static_cast<g2o::VertexSE3Expmap*>(Optimizer.vertex(i))->estimate();
        TBAD.cameraSetPoses[i]= toCvMat(SE3quat);
    }

    //Points
    pid=TBAD.cameraSetPoses.size();
    for(PointInfo &pi:TBAD.pointSetInfo){
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(Optimizer.vertex( pid++));
        auto est=vPoint->estimate();
        cv::Point3f np( est(0),est(1),est(2));
        pi.p3d=np;
    }
}


double computeReprjErr(BA_Data &TBAD){
    int n=0;double Err=0;
    for(auto pi: TBAD.pointSetInfo){
        for(auto cam_p:  pi.pointViews) {
            auto p2d=project(pi.p3d,TBAD.K,TBAD.cameraSetPoses[ cam_p.camIndex],TBAD.dist);
            auto err=p2d- cam_p.p2d;
            Err+=err.x*err.x+err.y*err.y;
            n++;
        }
    }
    return Err/double(n);

}
//get marker pose




//now, do full ba applying the restriction to the markers
void saveToPcd(BA_Data &MYDATA, string path);
int main(int argc,char **argv){
    BA_Data  MyBAData;

    try{

        if(argc!=4 && argc!=3){cerr<<" [camerasfile pointsfile markersfile] || [in.bad] "<<endl;return -1;}

        if (argc==4){
        cout<<"loadCamerasFile"<<endl;
        loadCamerasFile(argv[1],MyBAData);
        cout<<"loadPointsFile"<<endl;
        loadPointsInfo(argv[2],MyBAData);
        cout<<"loadMakers File"<<endl;
        minfo=loadMarkerInfo(argv[3]);

        //add the markers 3d points to the BADATA
        cout<<"SSSSSS=="<<MyBAData.pointSetInfo.size()<<endl;
        for(const MarkerInfo &marker:minfo){
            for(int i=0;i<4;i++)
                MyBAData.pointSetInfo.push_back(marker.pointSetInfo[i]);
        }

        cout<<"ERR="<<computeReprjErr(MyBAData)<<endl;
        cout<<"Refining camera params"<<endl;
        if(true)        refineCameraParameters(MyBAData);
        cout<<"ERR="<<computeReprjErr(MyBAData)<<endl;
        //check the reproj errors

        cout<<"remove point distortion"<<endl;
        float fx=MyBAData.K.at<float>(0,0);
        float fy=MyBAData.K.at<float>(1,1);
        float cx=MyBAData.K.at<float>(0,2);
        float cy=MyBAData.K.at<float>(1,2);
        //lets remove distortion
        for(auto &pi: MyBAData.pointSetInfo){
            vector< cv::Point2f > org_points(pi.pointViews.size());
            vector< cv::Point2f > und_points(pi.pointViews.size());
            for(int i=0;i< pi.pointViews.size();i++) org_points[i]=pi.pointViews[i].p2d;
            cv::undistortPoints	( org_points,und_points,MyBAData.K,MyBAData.dist);
            //copy back
            for(int i=0;i< pi.pointViews.size();i++) {
                und_points[i].x=und_points[i].x*fx+cx;
                und_points[i].y=und_points[i].y*fy+cy;
                pi.pointViews[i].p2d=und_points[i];
            }
        }


        MyBAData.dist.setTo(cv::Scalar::all(0));
        cout<<"ERR="<<computeReprjErr(MyBAData)<<endl;
        saveToPcd(MyBAData,"out.pcd");
        MyBAData.saveToFile("myba.bad");
        MyBAData.readFromFile("myba.bad");
        MyBAData.saveToFile("myba2.bad");
        FullBA(MyBAData,15);
        MyBAData.saveToFile("myba_fba.bad");

        saveToPcd(MyBAData,"out_fba.pcd");

        cout<<"ERR="<<computeReprjErr(MyBAData)<<endl;
        }
        else{
            MyBAData.readFromFile(argv[1]);
            minfo=loadMarkerInfo(argv[2]);

        }
        //cipy data back to markers

        int startMarkerPoints=  MyBAData.pointSetInfo.size()- minfo.size()*4;
        cout<<"SSSSSS222=="<<startMarkerPoints<<endl;
        for(  MarkerInfo &marker:minfo){
            for(int i=0;i<4;i++){
                marker.pointSetInfo[i]= MyBAData.pointSetInfo [ startMarkerPoints++];
            }
        }

        // computamos el angulo de cada marcador como el producto vectorial de 3 puntos
        vector<cv::Point3f> angles;
        for(auto &marker:minfo){
            cv::Point3f v1=marker.pointSetInfo[1].p3d-marker.pointSetInfo[0].p3d;
            cv::Point3f v2=marker.pointSetInfo[3].p3d-marker.pointSetInfo[0].p3d;
            cv::Point3f v3=marker.pointSetInfo[2].p3d-marker.pointSetInfo[1].p3d;
            cv::Point3f v4=marker.pointSetInfo[2].p3d-marker.pointSetInfo[3].p3d;
            auto angle1=v1.cross(v2) ;
            auto angle2=v4.cross(v3) ;
            auto meanangle=angle1+angle2;
            meanangle*=1./cv::norm(meanangle);
            cout<<angle1<<" "<<angle2<<endl;
            angles.push_back(meanangle);
        }

        //now, compute the angles
        for(int i=0;i<angles.size();i++)
            for(int j=i+1;j<angles.size();j++)
                cout<<  i<<" "<<j<<":"<<angles[i].dot(angles[j])<<" "<<acos( angles[i].dot(angles[j]))*180./M_PI<< endl;

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }

}
void saveToPcd(BA_Data &MYDATA,string path){

    ofstream filePCD ( path.c_str(),ios::binary );
    filePCD<<"# .PCD v.7 - Point Cloud Data file format"<<endl;
    filePCD<<"VERSION .7"<<endl;
    filePCD<<"FIELDS x y z   "<<endl;
    filePCD<<"SIZE 4 4 4 "<<endl;
    filePCD<<"TYPE F F F "<<endl;
    filePCD<<"COUNT 1 1 1 "<<endl;
    filePCD<<"VIEWPOINT 0 0 0 1 0 0 0"<<endl;
    filePCD<<"WIDTH "<<MYDATA.pointSetInfo.size() <<endl;
    filePCD<<"HEIGHT "<<1<<endl;
    filePCD<<"POINTS "<<MYDATA.pointSetInfo.size() <<endl;
    filePCD<<"DATA binary"<<endl;
    for(auto p:MYDATA.pointSetInfo)
        filePCD.write ( ( char* ) &(p.p3d),sizeof ( cv::Point3f) );


}
//home/salinas/Descargas/Prueba2/CameraInfo.txt /home/salinas/Descargas/Prueba2/sfmpoints.txt  /home/salinas/Descargas/Prueba2/gcpmarkers.txt
