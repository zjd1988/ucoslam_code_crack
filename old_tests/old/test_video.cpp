#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "basic_types.h"
#include "utils.h"
#include "track_utils.h"
#include "debug.h"
#include "ORBextractor.h"
#include <cvba/bundler.h>
#include <aruco/markermap.h>
#include <aruco/posetracker.h>
#include <aruco/markerdetector.h>
#include "ippe.h"
#include <aruco/markerdetector.h>
#include "Initializer.h"
#include <random>
#include "globaloptimizer.h"
#include <slam.h>
#include <world.h>

using namespace std;
using namespace ucoslam;

//given a frame and a map, returns the set pose using the markers
//If not possible, return empty matrix
//the pose matrix returned is from Global 2 Frame
cv::Mat getPoseFromMarkersInMap(const Frame &frame,World &tworld){
    std::vector<uint32_t> validmarkers;//detected markers that are in the map

    //for each marker compute the set of poses
    vector<pair<cv::Mat,double> > pose_error;
    vector<cv::Point3f> markers_p3d;
    vector<cv::Point2f> markers_p2d;

    for(auto m:frame.markers){
        if (tworld.TheMap.map_markers.find(m.id)!=tworld.TheMap.map_markers.end()){
            ucoslam::Marker &mmarker=tworld.TheMap.map_markers[m.id];
            cv::Mat Mm_pose_g2m=mmarker.pose_g2m;
            //add the 3d points of the marker
            auto p3d=mmarker.get3DPoints();
            markers_p3d.insert(markers_p3d.end(),p3d.begin(),p3d.end());
            //and now its 2d projection
            markers_p2d.insert(markers_p2d.end(),m.begin(),m.end());

            auto poses_f2m=IPPE::solvePnP(tworld.markerSize,m,tworld.TheImageParams.CameraMatrix,tworld.TheImageParams.Distorsion);
            for(auto pose_f2m:poses_f2m)
                pose_error.push_back(   make_pair(pose_f2m * Mm_pose_g2m.inv(),-1));
        }
    }
    if (markers_p3d.size()==0)return cv::Mat();
    //now, check the reprojection error of each solution in all valid markers and take the best one


    for(auto &p_f2g:pose_error){
        vector<cv::Point2f> p2d_reprj;
        se3 pose_se3=p_f2g.first;
        cv::projectPoints(markers_p3d,pose_se3.getRvec(),pose_se3.getTvec(),tworld.TheImageParams.CameraMatrix,tworld.TheImageParams.Distorsion,p2d_reprj);
        p_f2g.second=0;
        for(size_t i=0;i<p2d_reprj.size();i++)
            p_f2g.second+= (p2d_reprj[i].x- markers_p2d[i].x)* (p2d_reprj[i].x- markers_p2d[i].x)+ (p2d_reprj[i].y- markers_p2d[i].y)* (p2d_reprj[i].y- markers_p2d[i].y);
    }

    //sort by error

    std::sort(pose_error.begin(),pose_error.end(),[](const pair<cv::Mat,double> &a,const pair<cv::Mat,double> &b){return a.second<b.second; });
//    for(auto p:pose_error)
//    cout<<"p:"<<p.first<<" "<<p.second<<endl;
    return pose_error[0].first;//return the one that minimizes the error
}


class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

int main(int argc,char **argv){

    ucoslam::debug::Debug::setLevel(10);
    CmdLineParser cml(argc,argv);
    if (argc<3 || cml["-h"]){
        cerr<<"Usage: video  camera_params.yml  [-f filter_method: 0 Homography, 1 Fundamental Matrix] [-d descriptor 0:orb 1:AKAZE] [-size markers_size] [-pcd out.pcd] [-dict <dictionary>] [-cvba out.cvba] [-ms markerset.yml]"<<endl;return -1;
    }
    //load images
    cv::VideoCapture vcap(argv[1]);
    if (!vcap.isOpened()) throw std::runtime_error("Video not opened");
    int nframes = 1;
    cv::Mat im1,im2;
    while(im1.empty()) vcap>>im1;

    for(int i=0;i<60;i++) vcap.grab();
    vcap.retrieve(im2);

_debug_msg("IMG-size="<< im2.rows << " " << im2.cols ,5);

    //read dictionary
    string aruco_dict=cml("-dic","ARUCO_MIP_36h12");
    //read image parameres
    ucoslam::ImageParams image_params;
    image_params.readFromXMLFile(argv[2]);
    image_params.resize(im1.size());
    //select the aruco parameters
    aruco::MarkerDetector::Params md_params;
    md_params._thresParam1_range=5;//enable multirange
    md_params._thresParam1=8;



    //select the feature detector
    cv::Ptr<cv::Feature2D> fdetector;
    if (stoi(cml("-d","0"))==0){
        auto orb=cv::ORB::create();
        orb->setMaxFeatures(2000);
        fdetector=orb;
    }
    else if (stoi(cml("-d","0"))==1){
        fdetector=cv::AKAZE::create();
    }
    else if (stoi(cml("-d","0"))==2){
        fdetector= new ucoslam::ORBextractor(2000,1.2,8,20,7);
    }

    //create the frame extractor
    ucoslam::FrameExtractor fextractor(image_params,fdetector,md_params,aruco_dict);
    fextractor.removeFromMarkers()=true;
    //extract keypoints and markers
    auto frame1=fextractor(im1);
    auto frame2=fextractor(im2);

    float markerSize=stof(cml("-size","1"));
    ucoslam::Initializer map_initializer;
    map_initializer.setParams(image_params,markerSize);
    World theWorld(image_params);

    auto matches=ucoslam::match_frames(frame1,frame2);
    if (map_initializer.initialize(frame1,frame2,matches,0.1,theWorld))
    {
        theWorld.updateCovisGraph(theWorld.TheFrameSet.rbegin()->first);
        GlobalOptimizer opt;
        GlobalOptimizer::Params goparams;
        goparams.fixed_frames={theWorld.TheFrameSet.begin()->second.idx};//set first frame as fixed
        goparams.verbose=false;
//        std::default_random_engine generator;
//        std::normal_distribution<double> distribution(0.0,1e-3);
//        for(auto &mp:theWorld.TheMap.map_points)
//            for(int i=0;i<3;i++)
//            mp.second.pose.rt[i+3]+=distribution(generator);

        //move the 3d points a bit to test
        opt.optimize(theWorld.TheMap,theWorld.TheFrameSet,image_params,markerSize,goparams);

        cout << "System has been initialized!" << endl;
    }
    else
    {
        cerr << "ERROR: system could not be initialized!" << endl;
        exit(1);
    }



    cv::namedWindow("image", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);    

    cv::imshow("image", im2);

    //====================================================================
    // PROCESS FOLLOWING FRAMES
    //====================================================================
    char key=0;
    uint refKF=theWorld.TheFrameSet.rbegin()->second.idx;//set as initial reference frame the last one inserted

    Frame prev_frame=frame2,curframe;
    while(vcap.grab() && key!=27){
        vcap.retrieve(im1);
        nframes++;
        _debug_msg("Nframes="<<nframes,5);

        // Get keypoints and markers
        curframe = fextractor(im1);

        // Init pose for current frame
        auto rt=getPoseFromMarkersInMap(curframe,theWorld);
        if (!rt.empty()) curframe.pose_f2g=rt;
        else curframe.pose_f2g=prev_frame.pose_f2g;

        // Match map points on current frame
        // ---------------------------------
        float distThr = 15; // Pixels
        std::vector<DMatch> map_matches;
        map_matches = matchMapPtsOnFrame(theWorld.TheMap, curframe, curframe.pose_f2g, theWorld.TheImageParams, distThr);
        if (ucoslam::debug::Debug::getLevel() > 1)
           cout << "* Found " << map_matches.size() << " map matches in current frame." << endl;

        // Get local map (i.e. related keyframes)
        std::vector<uint> localMap;
        localMap = getLocalMap(curframe, theWorld.TheFrameSet, theWorld.TheCVGraph, refKF);
      cout << "* Local map contains " << localMap.size() << " keyframes" << endl;

        // Add current frame to world
        // TODO: add condition to check if this could be a good keyframe
        //theWorld.TheFrameSet.addFrame(curframe);
        bool addedKF = theWorld.addKeyFrame(curframe, refKF);

        if (addedKF)
        {
            std::map<uint,MapPointStats> mapStats;
            mapStats = countVisMapPoints(theWorld.TheFrameSet);

            // Print info
if (0){
            cout << "Map stats: " << endl;
            for (auto m: mapStats)
            {
               cout << m.first << ": " << m.second.count << "(" << m.second.end << ")"<< endl;
            }
}
            // Recent Map Points Culling: Mur-Artal
            std::vector<uint> toBeRemoved;
            uint lastKF = theWorld.TheFrameSet.rbegin()->first;
            for(auto m: mapStats)
            {
               uint mapIdx = m.first;
               MapPointStats st = m.second;
               if ((st.end - st.start) < 3) // Must be checked
               {
                  if (st.end < lastKF &&  st.count < 3) // Observed from at least three keyframes?
                  {
                      toBeRemoved.push_back(mapIdx);
                  }
               }

            }
            cout << "--> " << toBeRemoved.size() << " map points to be removed." << endl;

            // TODO: loop over keyframes to remove the given map points
            // TODO: remove map point from set
            for (uint idx : toBeRemoved)
            {
                theWorld.removeMapPoint(idx);
            }
        }

        if (addedKF){ // DEBUG
            cout << "Added KF to set: " << curframe.idx << endl;
            cout << "Checking frames: " << endl;
            for (auto f_ : theWorld.TheFrameSet)
            {
               auto f = f_.second;
               cout << "F[" << f_.first << "]: " << f.idx << endl;
               cout << "\tPose: " << f.pose_f2g << endl;
            }
        }
        //cvWaitKey(0);

        // Optimize world with current set of frames
        if (addedKF){

       //     cout << "Covisibility graph: " << endl << theWorld.TheCVGraph << endl;
       //     cout << "Essential graph: " << endl << theWorld.TheCVGraph.getEG() << endl;

            GlobalOptimizer opt;
            GlobalOptimizer::Params goparams;
            goparams.fixed_frames={theWorld.TheFrameSet.begin()->second.idx};//set first frame as fixed
            goparams.verbose=false;

            theWorld.saveToFile("world.w");
            // Optimization
            opt.optimize(theWorld.TheMap,theWorld.TheFrameSet,image_params,markerSize,goparams);
            cout << "INFO: optimization done after adding new KF." << endl;

            theWorld.saveToPcd("world.pcd");

            // Find new candidate points
            // -------------------------
            // TODO: estudiar si hacer esto antes o despues de add keyframe
            int nadded = getNew3DMapPoints(theWorld.TheMap, theWorld.TheFrameSet, curframe, image_params);
            cout << "INFO: " << nadded << " new map points" << endl;

            theWorld.saveToPcd("world_after.pcd");
          //  cin.ignore();exit(0);

//            cv::imshow("image",im1);
//          cout << "PRESS A KEY TO CONTINUE..." << endl;
//            key=cv::waitKey(0);
        }

/*        if (addedKF){ // DEBUG
            //cout << "Added KF to set: " << curframe.idx << endl;
            cout << "-->Checking frames after optimization: " << endl;
            for (auto f_ : theWorld.TheFrameSet)
            {
               auto f = f_.second;
               cout << "F[" << f_.first << "]: " << f.idx << endl;
               cout << "\tPose: " << f.pose_f2g << endl;
            }
        }
*/

        // Local keyframe culling
        // -------------------------------------------------------
        bool deletedKF = false;
        if (addedKF)
        {
           uint idxRecentKF = theWorld.TheFrameSet.rbegin()->first;
           // Discard KF whose 90% of map points have been seen in at least other three KF
           deletedKF = cullLocalKeyframes(theWorld.TheFrameSet, idxRecentKF, theWorld.TheCVGraph);
           cout << "Removed KFs during culling? " << deletedKF << endl;
           cout << theWorld.TheFrameSet;
        }

        // Display and prepare for next iteration
        cout << theWorld;

        prev_frame=curframe;
    }

    cout << "End of program" << endl;

   return 0;
}



