#include <iostream>
#include <iomanip>
#include <fstream>
#include <aruco/markermap.h>
#include "optimization/sparselevmarq.h"
#include "stuff/utils.h"
#include "stuff/se3.h"
using namespace std;
using namespace ucoslam;
std::map<uint32_t,cv::Point3f> getRealLeicaCenters(string fp)throw(std::exception){
std::map<uint32_t,cv::Point3f> mset;
    ifstream file(fp);
    if (!file) throw std::runtime_error("NO intpu feile");

    while(!file.eof())    {
            int id;
            if(! (file>>id)) break;
             vector<cv::Point3f> points(4);
            cv::Point3f center(0,0,0);
            for(int i=0;i<4;i++){
                    file>>points[i].x>>points[i].y>>points[i].z;
                    center+=points[i];
            }
             center*=0.25;
            mset[id]=center;
    }
return mset;
}


cv::Point3f getCenter(const vector<cv::Point3f> &v){
cv::Point3f center(0,0,0);
for(auto p:v)center+=p;
return center*(1./double(v.size()));
}

vector<float> computeBestTransform(const std::map<uint32_t,cv::Point3f> &real,const aruco::MarkerMap &detected){

    //first, compute  horn to obtain initial estimation

    //get center of both
    vector<cv::Point3f> p3dreal,p3ddect;
    for(auto m:detected){
        if (real.count(m.id)){
            p3ddect.push_back(getCenter(m.points));
            p3dreal.push_back(real.at (m.id));
        }
    }

     auto RT=    rigidBodyTransformation_Horn1987(p3ddect,p3dreal,false);
     double avrg=0;
    {
        for(size_t i=0;i<p3dreal.size();i++){
            auto p=mult(RT,p3ddect[i]);
            avrg+=cv::norm(p-p3dreal[i]);
            cout<<cv::norm(p-p3dreal[i])<<" ";
        }

        cout<<" avrg="<<avrg/double(p3dreal.size())<<endl;
    }


    auto errFunct=[&](const SparseLevMarq<float>::eVector &sol,SparseLevMarq<float>::eVector &err){
        //Create the transform
        se3 rt(sol(0),sol(1),sol(2),sol(3),sol(4),sol(5));
        float scale=sol(6);
        //move the points first and then scale
        cv::Mat m=rt;
         err.resize(p3ddect.size());
        //compute err
        for(size_t i=0;i<p3dreal.size();i++){
            auto p=scale*mult(m,p3ddect[i]);
            err(i)= cv::norm(p - p3dreal[i]);
        }



    };

    se3 initSol(RT);
    SparseLevMarq<float>::eVector sol(7);
    for(int i=0;i<6;i++) sol(i)=initSol[i];
    sol(6)=1;
    SparseLevMarq<float> solver;
    SparseLevMarq<float>::Params p;
    p.verbose=true;
    p.maxIters=100;
    p.minError=1e-5;
    p.min_step_error_diff=1e-7;
    solver.setParams(p);
    cout<<sol.transpose()<<endl;
    solver.solve(sol,  std::bind(errFunct,std::placeholders::_1,std::placeholders::_2));
    cout<<sol.transpose()<<endl;
    //    ,std::bind(&MarkerSetLagragianOptimizer::jacobian,this,std::placeholders::_1,std::placeholders::_2));
    //final error
    {

        se3 rt(sol(0),sol(1),sol(2),sol(3),sol(4),sol(5));
        float scale=sol(6);
        //move the points first and then scale
        cv::Mat RT=rt;
        //compute err
        double avrg=0;
        for(size_t i=0;i<p3dreal.size();i++){
            auto p=scale*mult(RT,p3ddect[i]);
            avrg+=cv::norm(p-p3dreal[i]);
            cout<<cv::norm(p-p3dreal[i])<<" ";
        }

        cout<<" avrg="<<avrg/double(p3dreal.size())<<endl;
    }



}

int main(int argc,char **argv)
{
    if (argc!=3){cerr<<"in.txt markermap.yml"<<endl;return -1;}
    auto mc=getRealLeicaCenters(argv[1]);



    aruco::MarkerMap mmap;
    mmap.readFromFile(argv[2]);

    computeBestTransform(mc,mmap);

    return -1;

}
