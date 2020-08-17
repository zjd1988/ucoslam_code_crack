#include "ucoslam.h"
#include "optimization/globaloptimizer.h"
#include <aruco/markermap.h>
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

int main(int argc,char **argv){

//    CmdLineParser cml(argc,argv);
//    if (argc<4){cerr<<"Usage: wroldint worldout  type"<<endl;return -1;}

//    try{

//        ucoslam::debug::Debug::setLevel(10);
//        ucoslam::Slam Tworld;
//        Tworld.readFromFile(argv[1]);
//        //if (! Tworld.checkConsistency() ){cerr<<"NOT CONSISTENT"<<endl;return -1;}
//        //else{cerr<<"CONSISTENCY CHECK PASSED"<<endl;}



//        cout<<"globalReprojError="<<Tworld.TheMap->globalReprojChi2()<<endl;



////remove point with less than 3 observations

//        vector<uint32_t> pointsToRemove;
//        for(auto mp:Tworld.TheMap->map_points)
//            if (mp.frames.size()<3) pointsToRemove.push_back(mp.id);

//         for(auto pid_remove:pointsToRemove){
//            for(auto f_id:Tworld.TheMap->map_points [pid_remove].frames)
//                Tworld.TheMap->keyframes.at(f_id.first).ids[f_id.second]=std::numeric_limits<uint32_t>::max();
//            Tworld.TheMap->map_points.erase(pid_remove);
//        }



//        auto gopt=ucoslam::GlobalOptimizer::create(argv[3]);//creates default
//        ucoslam::GlobalOptimizer::ParamSet params(true);
//        params.minStepErr=1e-4;
//        params.fixFirstFrame=true;
//        params.nIters=10;

//        {
//        ucoslam::ScopeTimer  Timer("Total opt time");
//        gopt->optimize(Tworld.TheMap,params);
//        }

//        //        for(const auto &f:Tworld.TheFrameSet)    cout<<f.second.idx<<" "<<f.second.pose_f2g<<" d="<<cv::norm( Tworld.TheFrameSet.begin()->second.getCameraCenter()-f.second.getCameraCenter()  )<<" "<<f.second.getCameraCenter()<<endl;
//        //        cout<<endl;

//        cout<<"globalReprojError="<<Tworld.TheMap->globalReprojChi2()<<endl;


////        vector<float> chi2;
////        Tworld.globalReprojError(false,{},&chi2);
////        for(auto c:chi2)cout<<c<<" ";cout<<endl;

//        Tworld.saveToFile(argv[2]);
//      }catch(std::exception &ex){
//        cerr<<ex.what()<<endl;
//    }
}
