#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <aruco/markerdetector.h>
#include <aruco/markerdetector.h>
#include "basic_types.h"
#include "utils.h"
#include "debug.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "optimization/globaloptimizer_simple.h"
#include "optimization/ippe.h"
#include "_slam.h"
#include "world.h"
#include "viewer.h"
#include <bitset>
using namespace std;
using namespace ucoslam;

string sigtostring(uint64_t sig){
    string sret;
    string alpha="qwertyuiopasdfghjklzxcvbnm1234567890QWERTYUIOPASDFGHJKLZXCVBNM";
    uchar * s=(uchar *)&sig;
    int n=sizeof(sig)/sizeof(uchar );
    for(int i=0;i<n;i++){
        sret.push_back(alpha[s[i]%alpha.size()]);
    }


    s[0]=0;

    return sret+":"+to_string(sig);
}

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

            auto poses_f2m=IPPE::solvePnP(tworld.markerSize(),m,tworld.TheImageParams.CameraMatrix,tworld.TheImageParams.Distorsion);
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


void drawMatches(cv::Mat &image,Frame & curFrame,const vector<cv::DMatch> &matches){
    for(auto m:matches){
        assert( m.queryIdx<curFrame.kpts.size());
        auto kp=curFrame.kpts[m.queryIdx].pt;
        cv::rectangle(image,kp-cv::Point2f(2,2),kp+cv::Point2f(2,2),cv::Scalar(0,255,0));
    }
}



class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};
int main(int argc,char **argv){
    Viewer TViewer;
    ucoslam::debug::Debug::setLevel(10);
    CmdLineParser cml(argc,argv);
    if (argc<3 || cml["-h"]){
        cerr<<"Usage: video  camera_params.yml  [-in world] [-f filter_method: 0 Homography, 1 Fundamental Matrix] [-d descriptor 0:orb 1:AKAZE] [-size markers_size] [-pcd out.pcd] [-dict <dictionary>] [-cvba out.cvba] [-ms markerset.yml] [-nomarkers]"<<endl;return -1;
    }

    cv::VideoCapture vcap(argv[1]);
    if (!vcap.isOpened()) throw std::runtime_error("Video not opened");
    cv::Mat in_image;
    while(in_image.empty()) vcap>>in_image;//read until valid data (for live cameras)

    World theWorld;
    //create the frame extractor
    string aruco_dict=cml("-dic","ARUCO_MIP_36h12");//read dictionary
    //read image parameres
    ucoslam::ImageParams image_params;
    image_params.readFromXMLFile(argv[2]);
    image_params.resize(in_image.size());
    //select the aruco parameters
    aruco::MarkerDetector::Params md_params;
    md_params._thresParam1_range=5;//enable multirange
    md_params._thresParam1=8;
    float markerSize=stof(cml("-size","1"));

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

    ucoslam::FrameExtractor fextractor(image_params,fdetector,md_params,aruco_dict);
    fextractor.removeFromMarkers()=true;
    fextractor.detectMarkers()=!cml["-nomarkers"];


    //if indicated, read status from file
    if (cml["-in"])
    {
        markerSize=theWorld.markerSize;
        theWorld.readFromFile(cml("-in"));
        //advance video until last frame processed
        while(int(vcap.get(CV_CAP_PROP_POS_FRAMES))< theWorld.nImagesProcessed)  vcap>>in_image;
        cout<<theWorld<<endl;
    }
    else{//initialize
        //extract keypoints and markers
        Frame  frame1=fextractor(in_image);
        ucoslam::Initializer map_initializer;
        map_initializer.setParams(image_params,markerSize);
        theWorld.setParams(image_params);//set the image params

        //        for(int i=0;i<48;i++) vcap.grab();
        bool isInitialized=false;
        while(!isInitialized){
            vcap>>in_image;
            Frame frame2=fextractor(in_image,vcap.get(CV_CAP_PROP_POS_FRAMES));
            cout<<"SIG="<<sigtostring(frame2.getSignature())<<endl;
            auto matches=ucoslam::match_frames(frame1,frame2,0.8);
            cout<<"NUOFMACHES="<<matches.size()<<endl;

            if (map_initializer.initialize(frame1,frame2,matches,0.05,theWorld))
            {
                theWorld.saveToFile("world-init.bin");

                isInitialized=true;
                GlobalOptimizerSimple opt;
                GlobalOptimizerSimple::Params params(true);
                params.fixed_frames={theWorld.TheFrameSet.begin()->first};
                opt.optimize(theWorld ,params);
                 cout << "System has been initialized!" << endl;
            }
            cv::imshow("image",in_image);
            cv::waitKey(10);
        }
        theWorld._curPose= theWorld.TheFrameSet.rbegin()->second.pose_f2g;
        theWorld._curKFRef=theWorld.TheFrameSet.rbegin()->second.idx;
    }
    theWorld.nImagesProcessed=int(vcap.get(CV_CAP_PROP_POS_FRAMES));

    theWorld.saveToFile("world.bin");
    theWorld.saveToPcd("world.pcd",0.1);

    World waux;
    waux.readFromFile("world.bin");
    _debug_msg_("Check signatures**********************");
    assert( waux.getSignature()==theWorld.getSignature());
    //    fextractor.detectMarkers()=false;
    cout<<"current pose="<<theWorld.TheFrameSet.rbegin()->second.pose_f2g<<endl;

    cv::namedWindow("image", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);

    cv::imshow("image", in_image);
    assert( theWorld.checkConsistency());
    TViewer.update(theWorld);
    TViewer.start();
    cv::waitKey(0);
    //cin.ignore();

    //====================================================================
    // PROCESS FOLLOWING FRAMES
    //====================================================================
    char key=0;

    Frame curframe;
    Frame prev_frame=theWorld.TheFrameSet.rbegin()->second;
    while(vcap.grab() && key!=27){
        _debug_msg("======================================================",5);
        _debug_msg("======================================================",5);
        theWorld.saveToFile("world-prev.bin");

        vcap.retrieve(in_image);

        // Get keypoints and markers
        curframe = fextractor(in_image,vcap.get(CV_CAP_PROP_POS_FRAMES));
        _debug_msg("======================== frame "<<vcap.get(CV_CAP_PROP_POS_FRAMES)<<" Fsig="<<sigtostring(curframe.getSignature())<<" Wsig="<<sigtostring(theWorld.getSignature()),5);

        // Init pose for current frame
        auto rt=getPoseFromMarkersInMap(curframe,theWorld);
        if (!rt.empty()) curframe.pose_f2g=rt;
        else curframe.pose_f2g=theWorld._curPose;

        // Match map points on current frame wrt previous reference KeyFrame
        // ---------------------------------
        //SECTION V.B of paper
        float distThr = 15; // Pixels
        std::vector<DMatch> map_matches;
        _debug_msg("*-------------1" ,5);
        map_matches = matchMapPtsOnFrames(theWorld ,{theWorld._curKFRef}, curframe,  curframe.pose_f2g ,{},theWorld._minDescDistance,distThr);
        //check
        _debug_msg("*-------------2" ,5);
        filter_ambiguous_query(map_matches); //remove ambigous matches
        _debug_msg("*-------------3" ,5);


        _debug_msg("* Found " << map_matches.size() << " map matches in current frame." ,5);
        drawMatches(in_image,curframe,map_matches);



        if (map_matches.size()<30){
            //need relocalization!!!!
            cerr<<"NNNNEDD RELOCALIZATION"<<endl;
            exit(0);
        }

        //now, refine pose using all available information (matches and markers)

        curframe.pose_f2g =refinePoseUsingMatches(curframe.pose_f2g ,theWorld  , curframe,map_matches ) ;

        // Get local map (i.e. related keyframes)
        auto localMap = getLocalMap(  theWorld , theWorld._curKFRef);
        //find now matches in all the frames of the lccal map
        map_matches = matchMapPtsOnFrames(theWorld ,localMap, curframe, curframe.pose_f2g, map_matches,theWorld._minDescDistance,distThr);
        _debug_msg("* Found " << map_matches.size() << " map matches in local map." ,5);

        //remove ambigous matches
        filter_ambiguous_query(map_matches);
        _debug_msg("* Found " << map_matches.size() << " map matches after removing ambiguous." ,5);

        //refine using all information found
        curframe.pose_f2g =refinePoseUsingMatches(curframe.pose_f2g ,theWorld  , curframe,map_matches ) ;
        _debug_msg("* refined pose :"<<curframe.pose_f2g,1);


        TViewer.updateCamerPose(curframe.pose_f2g);

        bool addedKF = theWorld.addKeyFrame(curframe,map_matches, theWorld._curKFRef);

        //        cerr<<"Press key"<<endl;
        //        cin.ignore();
        if (addedKF){
            theWorld._curKFRef=curframe.idx;
            assert( theWorld.checkConsistency());

            //now, must add new points using the connection graph and the unmatched keypoints from here
            _debug_msg("* keypoints added ",5);
            //            cerr<<"Press key"<<endl;
            //            cin.ignore();
        }
        else{
            //determine the kf with most tracked points
            std::unordered_map<uint32_t,uint32_t> frame_nmatchedpoints;
            for(auto match:map_matches){
                auto map_point=theWorld.TheMap.map_points.find(match.queryIdx);
                if (map_point==theWorld.TheMap.map_points.end())continue;
                for(auto frame:map_point->second.frames){
                    if (!frame_nmatchedpoints.count(frame.first)) frame_nmatchedpoints[frame.first]=0;
                    else frame_nmatchedpoints[frame.first]++;
                }
            }
            //take the maximum
            auto best_frame=frame_nmatchedpoints.begin()->first;
            auto best_frame_np=frame_nmatchedpoints.begin()->second;
            for(auto fm:frame_nmatchedpoints){
                if ( fm.second>best_frame_np){
                    best_frame=fm.first;
                    best_frame_np=fm.second;
                }
            }
            theWorld._curKFRef=best_frame;
        }
        cout<<"ADDEDKF="<<addedKF<<endl;
        _debug_msg("Current reference keyframe="<<theWorld._curKFRef,5);


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

        //        // Display and prepare for next iteration
        cout << theWorld;
        theWorld.nImagesProcessed=int(vcap.get(CV_CAP_PROP_POS_FRAMES));
        theWorld._curPose= curframe.pose_f2g;

        TViewer.update(theWorld);
        prev_frame=curframe;
        cv::imshow("image",in_image);
        theWorld.saveToFile("world.bin");
        theWorld.saveToPcd("world.pcd");
        {

            World waux;
            waux.readFromFile("world.bin");
            _debug_msg_("Check signatures**********************");
            assert( waux.getSignature()==theWorld.getSignature());
        }        char k=cv::waitKey(0);
        if (k=='o'){
            // Optimize world with current set of frames


            // Optimization
            theWorld.saveToPcd("world-po.pcd");
            theWorld.saveToFile("world-po.bin");

            GlobalOptimizerSimple opt;
            opt.optimize(theWorld,GlobalOptimizerSimple::Params(true));
            cout << "INFO: optimization done after adding new KF." << endl;
            theWorld.saveToPcd("world.pcd");
            theWorld.saveToFile("world.bin");


            TViewer.update(theWorld);
        }
        //        string number=to_string(int(vcap.get(CV_CAP_PROP_POS_FRAMES)));
        //        while(number.size()<4) number="0"+number;
        //       theWorld.saveToFile("world-"+number);
    }

    cout << "End of program" << endl;

    return 0;
}



