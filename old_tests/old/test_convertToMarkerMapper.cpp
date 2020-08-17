using namespace std;
#include "marker_mapper/mapper_types.h"
#include "marker_mapper/markermapper.h"
#include "marker_mapper/optimizers/fullsceneoptimizer.h"
#include "slam.h"
using namespace aruco_mm;

class ArucoMMConverter{
public:
  static  std::shared_ptr<MarkerMapper> convert(ucoslam::Slam &sl){

        //start processing
        std::shared_ptr<MarkerMapper> AMM;
        AMM=MarkerMapper::create( );
        AMM->setParams(sl.TheImageParams,sl.markerSize(),sl.TheMap.map_markers.begin()->first);


         convert(sl,AMM->getMarkerSet(),AMM->getFrameSet());
         return AMM;
    }

   static void convert(ucoslam::Slam &sl,MarkerSet &mset,FrameSet &fset){
        for(auto m:sl.TheMap.map_markers)
            mset[m.first]=MarkerInfo(m.first,m.second.pose_g2m,m.second.size);
        for(auto f:sl.TheFrameSet){
            FrameInfo  fi;
            fi.frame_idx=f.idx;
            fi. rt_c2g=f.pose_f2g;//4x4 matrix from global coordinates to camera
            fi.markers=f.markers;
            fset[f.idx]=fi;
        }

    }
};

int main(int argc,char **argv){
try{
    if(argc!=3){
        cerr<<"In Out"<<endl;return -1;
    }
    ucoslam::Slam sl;
    sl.readFromFile(argv[1]);
   auto mm=ArucoMMConverter::convert(sl);

    FullSceneOptimizer mopt;
    FullSceneOptimizer::Params params;
    params.max_iters=100;//stoi(cml("-it","100"));
    params.min_step_error=0.1;//stod(cml("-mste","0"));
    params.verbose=true;//cml["-v"];
    params.fix_camera_params=false;//cml["-fi"];
    params.fixedMarkers={mm->getOriginMarkerId()};
    mopt.optimize(mm->getMarkerSet(),mm->getFrameSet(),mm->getCameraParams(),params);
    mm->saveToPcd(argv[2]+string(".pcd"),true);
    mm->getMarkerMap().saveToFile(argv[2]+string(".yml"));

    }   catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }

}
