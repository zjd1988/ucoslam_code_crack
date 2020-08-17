#include <iostream>
#include "picoflann.h"
#include "nanoflann.hpp"
#include <chrono>
#include <fstream>
#include <list>
using namespace std;

struct Timer{
    std::chrono::high_resolution_clock::time_point _s;
    Timer(){_s=std::chrono::high_resolution_clock::now();}
    double get( ){//returns time in ms
        auto e=std::chrono::high_resolution_clock::now();
        auto time=double(std::chrono::duration_cast<std::chrono::nanoseconds>(e-_s).count());
        return time/1e6;
    }

};

struct Point2f{
    Point2f(float X,float Y) { x=X;y=Y; }
    float x,y;
};


struct KdTreeAdaptor
{
    const std::vector<Point2f>* _points;

    KdTreeAdaptor(const std::vector<Point2f>*  data):_points(data)
    { }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return _points->size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline float kdtree_distance(const float *p1, const size_t idx_p2,size_t /*size*/) const
    {
        const auto &kpt=_points->at(idx_p2) ;
        float d0=p1[0]-kpt.x;
        float d1=p1[1]-kpt.y;
        return d0*d0+d1*d1;
    }
    inline float kdtree_get_pt(const size_t idx, int dim) const { return dim==0? _points->at(idx).x:_points->at(idx).y;}

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&  bb ) const { return false;}

};

typedef nanoflann::KDTreeSingleIndexAdaptor<
nanoflann::L2_Simple_Adaptor<float, KdTreeAdaptor  > ,
KdTreeAdaptor ,
2 /* dim */
> my_kd_tree_t;



std::vector<std::pair<uint32_t,double> >  getGTNN(const std::vector<Point2f> &data,Point2f &pref,int nn){
    vector<pair<uint32_t,double> > groundtruth;
    int index=0;
    for(const auto &p:data){
        double d0=pref.x-p.x;
        double d1=pref.y-p.y;
        groundtruth.push_back(  {index++,d0*d0+d1*d1 });
    }

    std::sort(groundtruth.begin(),groundtruth.end(),[](const pair<uint32_t,double>&a,const pair<uint32_t,double>&b){return a.second<b.second;});
    groundtruth.resize(nn);
    return groundtruth;
}

std::vector<std::pair<uint32_t,double> >  getGTRadiusSearch(const std::vector<Point2f> &data,Point2f &pref,double dist,int nn=-1){
    vector<pair<uint32_t,double> > groundtruth=getGTNN(data,pref,data.size());
    size_t n=0;
    double SqDist=dist*dist;
    for(size_t i=0;i<groundtruth.size();i++){
        if ( groundtruth[i].second<SqDist ) n++;
        else break;
    }
    groundtruth.resize(n);
    if (nn==-1){
        if (groundtruth.size()>size_t(nn)) groundtruth.resize(nn);
    }
    return groundtruth;
}
template<typename t1,typename t2,typename d1,typename d2>
bool checkAreEqual(const  std::vector<std::pair<t1,d1> > &a,const
                   std::vector<std::pair<t2,d2> >&b){

    if (a.size()!=b.size())return false;
    for(size_t i=0;i<a.size();i++)
        if (a[i].first!=b[i].first)return false;
    return true;
}

std::pair<float,float> testKnnPicoFlannRadiusSearch( vector<Point2f> &data,Point2f &pref, double dist,int nTimesSearch,int nn=-1){


    //pico flann adapter
    struct Point2fAdapter{
        inline   float operator( )(const Point2f &elem, int dim)const
        { return dim==0?elem.x:elem.y; }
    };

    std::pair<float,float> time_build_search;
    picoflann::KdTreeIndex<2,Point2fAdapter> kdtree;

    Timer timerBuild;
    kdtree.build(data);
    time_build_search.first=timerBuild.get();


   std::vector<std::pair<uint32_t,double> > res;
    Timer timerSearch;
    for(int i=0;i<nTimesSearch;i++){
      kdtree.radiusSearch(res,data,pref,dist,true);
    }
    time_build_search.second=timerSearch.get();

    auto groundtruth=getGTRadiusSearch(data,pref,dist,nn);
    if (!checkAreEqual(res,groundtruth))
        throw std::runtime_error("Error in picoflann radiusSearch");

    return time_build_search;
}



std::pair<float,float> testNanoFlannRadiusSearch(vector<Point2f> &data,Point2f &pref,double dist,int nTimesSearch,int nn=-1){
    std::pair<float,float> time_build_search;
    KdTreeAdaptor nano_adapt(&data);
    my_kd_tree_t nano_kdtree (2,  nano_adapt, nanoflann::KDTreeSingleIndexAdaptorParams(10));

    Timer timerBuild;
    nano_kdtree.buildIndex();
    time_build_search.first=timerBuild.get();


    std::vector<std::pair<size_t,float>>  idx_dist;
    nanoflann::SearchParams params;
    params.sorted = true;
    Timer timerSearch;
    for(int i=0;i<nTimesSearch;i++){
        nano_kdtree.radiusSearch((float*)&pref,dist*dist, idx_dist,params);
    }
    time_build_search.second=timerSearch.get();
    auto groundtruth=getGTRadiusSearch(data,pref,dist,nn);
    if (!checkAreEqual(idx_dist,groundtruth))
        throw std::runtime_error("Error in nanoflann radiusSearch");

    return time_build_search;

}
std::pair<float,float> testKnnPicoFlann( vector<Point2f> &data,Point2f &pref, int nn,int nTimesSearch){


    //pico flann adapter
    struct Point2fAdapter{
        inline   float operator( )(const Point2f &elem, int dim)const
        { return dim==0?elem.x:elem.y; }
    };

    std::pair<float,float> time_build_search;
    picoflann::KdTreeIndex<2,Point2fAdapter> kdtree;

    Timer timerBuild;
    kdtree.build(data);
    time_build_search.first=timerBuild.get();

    vector<pair<uint32_t,double> > res;
    Timer timerSearch;
    for(int i=0;i<nTimesSearch;i++)
        res=kdtree.searchKnn(data,pref,nn,true);
    time_build_search.second=timerSearch.get();

    auto groundtruth=getGTNN(data,pref,nn);
    if (!checkAreEqual(res,groundtruth))
        throw std::runtime_error("Error in picoflann knn");

    return time_build_search;
}

std::pair<float,float> testNanoFlann(vector<Point2f> &data,Point2f &pref, int nn,int nTimesSearch){
    std::pair<float,float> time_build_search;
    KdTreeAdaptor nano_adapt(&data);
    my_kd_tree_t nano_kdtree (2,  nano_adapt, nanoflann::KDTreeSingleIndexAdaptorParams(10));

    Timer timerBuild;
    nano_kdtree.buildIndex();
    time_build_search.first=timerBuild.get();


    Timer timerSearch;
    for(int i=0;i<nTimesSearch;i++){
        std::vector<size_t>   ret_index(nn);
        std::vector<float> out_dist_sqr(nn);
        nano_kdtree.knnSearch((float*)&pref, nn, &ret_index[0], &out_dist_sqr[0]);
    }
    time_build_search.second=timerSearch.get();

    return time_build_search;

}




int main(){
    try{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-1000.0,1000.0);
    generator.seed(0);
    size_t nData=10000;
    int nn=1000;
    int nTimesSearch=10000;
    double dist=40;
    Point2f pref(10,10);
    vector<Point2f> data;
    for(size_t i=0;i<nData;i++)
        data.push_back( Point2f ( distribution(generator),distribution(generator) ));


    std::pair<float,float> times;

      times=testKnnPicoFlannRadiusSearch(data,pref,dist,nTimesSearch,-1);
    cout<<"PicoFlann build= "<<times.first<<" radiusSearch="<<times.second<<endl;
    times=testNanoFlannRadiusSearch(data,pref,dist,nTimesSearch,-1);
    cout<<"NanoFlann build= "<<times.first<<" radiusSearch="<<times.second<<endl;


    //PICO FLANN AS VECTOR
   times=testKnnPicoFlann(data,pref,nn,nTimesSearch);
   cout<<"PicoFlann build= "<<times.first<<" KnnSearch="<<times.second<<endl;

     //NANO FLANN
    times=testNanoFlann(data,pref,nn,nTimesSearch);
    cout<<"NanoFlann build= "<<times.first<<" KnnSearch="<<times.second<<endl;



    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }


}
