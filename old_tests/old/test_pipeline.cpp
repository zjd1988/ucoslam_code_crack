// Pipeline for SLAM

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
#include "globaloptimizer.h"

#include <marker_mapper/mapper_types.h>
#include <marker_mapper/optimizers/fullsceneoptimizer.h>
#include <marker_mapper/utils/utils3d.h>
using namespace std;
cvba::BundlerData<float> easyBundler(const ImageParams &ip, const Frame &f1,const Frame &f2,const std::vector<cv::DMatch> &matches,const vector<cv::Point3f>  &p3d=vector<cv::Point3f>() );
cvba::BundlerData<float> frameSetBundler(const ImageParams &ip, const FrameSet & fs, const vector<cv::Point3f>  &p3d  );


class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

int main(int argc,char **argv){

    ucoslam::debug::Debug::setLevel(9);
    CmdLineParser cml(argc,argv);
    if (argc<4 || cml["-h"]){
        cerr<<"Usage: video  camera_params.yml  [-f filter_method: 0 Homography, 1 Fundamental Matrix] [-d descriptor 0:orb 1:AKAZE] [-size markers_size] [-pcd out.pcd] [-dict <dictionary>] [-cvba out.cvba] [-ms markerset.yml]"<<endl;return -1;
    }
    //load images
    string vidname(argv[1]);
    //cv::Mat im1=cv::imread(argv[1],IMREAD_GRAYSCALE);
    // cv::Mat im2=cv::imread(argv[2],IMREAD_GRAYSCALE);
    cv::VideoCapture vid;
    vid.open(vidname);

    if (!vid.isOpened())
       throw std::runtime_error("Could not open video file");

    int nframes = 0;
    // Get first two frames
    cv::Mat im1, im2;
    vid.grab(); vid.retrieve(im1); nframes++;

for(int i=0;i<90;i++){ vid.grab(); nframes++;} // DEVELOP!!!
    vid.retrieve(im2);

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
        fdetector=cv:: AKAZE::create();
    }
    else if (stoi(cml("-d","0"))==2){
        fdetector= new ucoslam::ORBextractor(2000,1.2,8,20,7);
    }

    //====================================================================
    // SLAM INITIALIZATION
    //====================================================================

    //create the frame extractor
    ucoslam::FrameExtractor fextractor(image_params,fdetector,md_params,aruco_dict);
fextractor.removeFromMarkers()=false;
    //extract keypoints and markers
    auto frame1=fextractor(im1);
    auto frame2=fextractor(im2);

    ucoslam::Initializer map_initializer;
    float marker_size=stof(cml("-size","1"));
    map_initializer.setParams(image_params,marker_size);

    ucoslam::Map globalMap;
    //std::map<uint32_t,ucoslam::Frame> frameSet;
    ucoslam::FrameSet frameSet;

    auto matches=ucoslam::match_frames(frame1,frame2);
    //int lastKey = 0;

    auto rt=map_initializer.computeRt(frame1,frame2,matches);//pose from frame 1 to frame 2
cout<<"DIST="<<cv::norm(se3(rt).getTvec())<<endl;

    if (!rt.empty()){

        //set poses in frames
        frame1.pose_f2g=cv::Mat::eye(4,4,CV_32F);
        frame2.pose_f2g=rt;//transform moving from Global To Frame2


        //refine matches by reprojection error
        auto matches_filtered=filter_by_reprojection_error(image_params , frame1 ,frame2 ,matches,rt,2);

        //triangulate the filtered matches
        vector<cv::Point3f> p3d;
        triangulate(image_params,frame1,frame2,matches_filtered,p3d,rt);
        //print the reprj error just for the sake of curiosity
        cout<<"rerr="<<reprj_error(image_params,frame1,frame2,matches_filtered,p3d,0,rt)<<endl;



        //draw image with matches
        cv::Mat resImg;
        drawMatches(im1,frame1.kpts,im2,frame2.kpts, matches_filtered,resImg);
        cv::namedWindow("resImg", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
        cv::imshow("resImg",resImg);

        //save cvba if requested
        if( cml["-cvba"]){
            auto bdata=easyBundler(image_params,frame1,frame2,matches_filtered,p3d);
            bdata.saveToFile(cml("-cvba"));
            auto bundler=cvba::Bundler::create("pba sba uco_sba");
            cvba::Bundler::Params params;
            params.fixCameraExtrinsics(0);
            params.fixAllCameraIntrinsics=true;
            params.verbose=true;
            bundler->setParams(params);
            bundler->run(bdata);
            p3d=bdata.points;
            rt= se3(bdata.R[1],bdata.T[1]);
        }
        //save 3d points if requested
        if (cml["-pcd"])  savePointsToPCD(p3d, cml("-pcd"));

        // Add 3D points to map
        se3 tmp(rt);
        cv::Mat rtv = tmp.getRvec();
        cv::Mat p3d_descriptors;
        for (uint i = 0; i < matches_filtered.size(); i++)
           if (p3d_descriptors.empty())
                p3d_descriptors = frame1.desc.row(matches_filtered[i].queryIdx);
           else
               cv::vconcat(p3d_descriptors, frame1.desc.row(matches_filtered[i].queryIdx), p3d_descriptors);

        std::vector<uint32_t> ids = globalMap.addNewPoints(p3d, rtv, p3d_descriptors);

        // Init frame set
        frame1.idx = 1;
        for (uint i = 0; i < matches_filtered.size(); i++)
           frame1.ids[matches_filtered[i].queryIdx] = ids[i];  // Assign correct map point id

        //frameSet.insert(make_pair(1,frame1));
        frameSet.addFrame(frame1);

        // Prepare frame2
        frame2.idx = 2;
        for (uint i = 0; i < matches_filtered.size(); i++)
           frame2.ids[matches_filtered[i].trainIdx] = ids[i];  // Assign correct map point id
        // TODO: add descriptors from frame2
        frameSet.addFrame(frame2);


        cout << "Total map points: " << globalMap.size() << endl;
    }

    for(auto m:frame1.markers) m.draw(im1,cv::Scalar(0,255,0));
    for(auto m:frame2.markers) m.draw(im2,cv::Scalar(0,255,0));

    cv::namedWindow("im1", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    cv::namedWindow("im2", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    cv::imshow("im1",im1);
    cv::imshow("im2",im2);
    char key = cv::waitKey(0);

    //====================================================================
    // PROCESS FOLLOWING FRAMES
    //====================================================================
    // Go to next frame
    se3 prevPose = rt;
    cv::Mat imgFrame;
    while ( vid.grab() && key != 27)
    {
       vid.retrieve(imgFrame);
       nframes++;
       cout << "Processing frame: " <<  nframes << endl;

       // Match new keypoints with map points
       ucoslam::Frame curframe = fextractor(imgFrame);

       // Poyectar puntos con pose anterior
       std::vector<uint32_t> map_ids;
       std::vector<cv::Point3f> p3d = globalMap.getMapPoints(map_ids);

       cv::Mat projPoints;       
       cv::projectPoints(p3d, prevPose.getRvec(), prevPose.getTvec(), image_params.CameraMatrix, image_params.Distorsion, projPoints);

       std::vector<DMatch> map_matches; // [query=kpts; train=map]
       // Find matches map-points keypoints
       float distThr = 15; // Pixels
       for (uint mpix = 0; mpix < p3d.size(); mpix++)
       {
           // Find keypoints closer (spatial distance) to this map point
           // ----------------------------------------------------------
           cv::Point2f map_pt_proj(projPoints.at<float>(mpix,0), projPoints.at<float>(mpix,1));

           std::vector<int> candidate_kp;

           for (uint j = 0; j < curframe.kpts.size(); j++) // TODO: Preguntar a Rafa sobre undistorted kpts
           {
               auto kp = curframe.kpts[j];
               cv::Point2f kpt = kp.pt;

               float dist = distancePtsL2(map_pt_proj, kpt);
               if (dist <= distThr )
                  candidate_kp.push_back(j);
           }


           // Compute descriptor distance for selected keypoints
           // --------------------------------------------------
           if (candidate_kp.empty())
               continue;

           cv::Mat desc;
           for (uint j = 0; j < candidate_kp.size(); j++)
               if (desc.empty())
                   desc = curframe.desc.row(candidate_kp[j]).clone();
               else
                  cv::vconcat(desc, curframe.desc.row(candidate_kp[j]).clone(), desc);

           auto matches_kp_map = ucoslam::match_desc(globalMap.getMapPointDescriptor(mpix), desc);

           if (ucoslam::debug::Debug::getLevel() > 10)
           {
              cout << "\t Candidates for current map point: " << candidate_kp.size();
              cout << " Valid matches: " << matches_kp_map.size() << endl;

           }

           if (!matches_kp_map.empty())
           {
              cv::DMatch mt, tmp;
              tmp = matches_kp_map[0];
              mt.queryIdx = candidate_kp[tmp.trainIdx]; // kp
              mt.trainIdx = mpix;                       // map
              mt.distance = tmp.distance;
              map_matches.push_back(mt);
           }

       }
       if (ucoslam::debug::Debug::getLevel() > 1)
       {
          cout << "* Found " << map_matches.size() << " map matches in current frame." << endl;
       }

       // Optimize pose with cv::solvePnP
       // -------------------------------
       std::vector<cv::Point3f> p3d_sel;
       std::vector<cv::Point2f> p2d_sel;

       for (auto m : map_matches)
       {
          p3d_sel.push_back(p3d[m.trainIdx]);
          p2d_sel.push_back(curframe.kpts[m.queryIdx].pt);
       }

       auto rv = prevPose.getRvec();
       auto tv = prevPose.getTvec();       
       bool useRansac = false;
       if (useRansac)
          cv::solvePnPRansac(p3d_sel, p2d_sel, image_params.CameraMatrix, image_params.Distorsion, rv, tv, true); // RANSAC, VER DISTORTION
       else
          cv::solvePnP(p3d_sel, p2d_sel, image_params.CameraMatrix, image_params.Distorsion, rv, tv, true); // RANSAC, VER DISTORTION


       cv::Mat rv32, tv32;
       rv.convertTo(rv32, CV_32F);
       tv.convertTo(tv32, CV_32F);
       rt = ucoslam::se3(rv32, tv32);
       if (useRansac)
       {
           cout << "OLD-R-T: " << prevPose.getRvec() << " || " << prevPose.getTvec() << endl;
           cout << "NEW-R-T: " << rv32.t() << " || " << tv32.t() << endl;
           cout << "Distance: " << norm(prevPose.getTvec()-tv32.t()) << endl;
       }
       else
       {
           cout << "OLD-R-T: " << prevPose.getRvec() << " || " << prevPose.getTvec() << endl;
           cout << "NEW-R-T: " << rv32 << " || " << tv32 << endl;
           cout << "Distance: " << norm(prevPose.getTvec()-tv32) << endl;
       }


       // Update map matches on current frame
       for (auto m : map_matches)
       {
          curframe.ids[m.queryIdx] = map_ids[m.trainIdx];
       }
       curframe.pose_f2g = rt;

       // Reproject and filter by distance
       // --------------------------------------
       ucoslam::Frame otherFrame = frameSet.getLastFrame();
       auto matchesAfter = ucoslam::match_frames(otherFrame, curframe);
       cout << "Matches on previous frame: " << matchesAfter.size() << endl;
       cv::Mat CF2FS = rt * otherFrame.pose_f2g.inverse();

       auto matchesAfter_filt = filter_by_reprojection_error(image_params, otherFrame, curframe, matchesAfter, CF2FS, 2); // OJO al RT
       cout << "Matches on previous frame after filtering: " << matchesAfter_filt.size() << endl;
       // Triangulate on the filtered points
       vector<cv::Point3f> p3d_cf2fs;
       triangulate(image_params, otherFrame, curframe, matchesAfter_filt, p3d_cf2fs, CF2FS);

       // TODO: llevar puntos 3D al sistema de referencia global
       cv::Mat trans = otherFrame.pose_f2g.inverse();
       for (uint j = 0; j < p3d_cf2fs.size(); j++)
       {
          p3d_cf2fs[j] = mult<float,cv::Point3f>(trans, p3d_cf2fs[j]);
       }
       // cv::perspectiveTransform(p3d_cf2fs, p3d_cf2fs, trans);

       // TODO: add new triangulated matches to map?
       // Loop on matches, skip those kpts previously assigned to map points
       se3 rtSE3(rt);
       for (uint mix = 0; mix < matchesAfter_filt.size(); mix++)
       {
          auto m = matchesAfter_filt[mix];
          auto idxPrev = m.queryIdx;
          auto idxCur = m.trainIdx;
          if (otherFrame.ids[idxPrev] != INVALID_MAPPT && curframe.ids[idxCur] != INVALID_MAPPT)
          {
              auto pt3d = p3d_cf2fs[mix];
              auto thisId = globalMap.addNewPoint(pt3d, rtSE3.getRvec(), curframe.desc.row(idxCur));
              globalMap.map_points[thisId].addDescriptor(otherFrame.desc.row(idxPrev));

              otherFrame.ids[idxPrev] = thisId;
              curframe.ids[idxCur] = thisId;
          }
       }

       // Decision: Add new points, update Frame
       // --------------------------------------
       cout << "INFO::map::size() " << globalMap.size() << endl;
       if (cml["-pcd"]) // Save current map points for viz?
       {
           std::vector<uint32_t> trash;
           auto p3dTmp= globalMap.getMapPoints(trash);
           char foo[1024];
           sprintf(foo, "_F%03d.pcd", nframes);
           string outname = cml("-pcd") + string(foo);
           //savePointsToPCD(p3dTmp, outname);
           cv::Scalar color;
           if (nframes % 2)
               color = cv::Scalar(0, 5*nframes, 0);
           else
               color = cv::Scalar(0, 0, 5*nframes);
           savePointsToPCD(p3d_cf2fs, outname+"_part.pcd", color);
           savePointsToPCD(p3dTmp, outname, color);
       }

       // Add current frame to global set       
       frameSet.addFrame(curframe);

       cout << "INFO: FrameSet.size() " << frameSet.size() << endl;

       // Bundle adjustment       
     if(0)  {
           cout << "Running BA..." << endl;
           std::vector<uint32_t> trash;
           auto p3d = globalMap.getMapPoints(trash);
           auto bdata = frameSetBundler(image_params, frameSet, p3d);
           //bdata.saveToFile(cml("-cvba"));
           auto bundler=cvba::Bundler::create("pba sba uco_sba");
           cvba::Bundler::Params params;
           params.fixCameraExtrinsics(0);
           params.fixAllCameraIntrinsics=true;
           params.verbose=true;
           bundler->setParams(params);
           bundler->run(bdata);
           p3d=bdata.points;
           rt= se3(bdata.R[1],bdata.T[1]);
           cout << "Finished BA." << endl;
       }
     else{

         GlobalOptimizer::Params go_params;
         go_params.fixed_frames={ frameSet.begin()->second.idx };
         go_params.verbose=true;
         GlobalOptimizer go;
         go.optimize(globalMap,frameSet,image_params,marker_size,go_params);

         World w;
         w.setParams(marker_size);
         w.TheFrameSet = frameSet;
         w.TheMap = globalMap;
         w.TheImageParams = image_params;
         w.saveToFile("world.w");
         w.saveToPcd("world.pcd");
     }



       // Prepare next iteration
       prevPose = rt;

       // GUI wait
       cv::imshow("im2", imgFrame);
       key = cv::waitKey(0);
    }

    // Save the world!
    World theWorld;
    theWorld.TheFrameSet = frameSet;
    theWorld.TheMap = globalMap;
    theWorld.TheImageParams = image_params;

    cout << "Saving world to file...";
    theWorld.saveToFile("theWorld01.dat");
    cout << "done!" << endl;

    World newWorld;
    cout << "Loading and saving world to file...";
    newWorld.readFromFile("theWorld01.dat");
    newWorld.saveToFile("theWorld02.dat");
    cout << "done!" << endl;

    return 0;
}

cvba::BundlerData<float> frameSetBundler(const ImageParams &ip, const FrameSet & fs, const vector<cv::Point3f>  &p3d  )
{
    int nviews = fs.size();
    int npoints = p3d.size();

    cvba::BundlerData<float> Bdata(nviews, npoints); // <nviews, npoints>
    // 3D Points
    Bdata.points=p3d;

    cout << "Optimizing " << Bdata.points.size() << " points from " << nviews << " views." << endl;
/*
    for(uint i=0;i<matches.size();i++){
        auto m=matches[i];
        Bdata.imagePoints[ 0][i]=(f1.kpts[m.queryIdx].pt);
        Bdata.imagePoints[ 1][i]=(f2.kpts[m.trainIdx].pt);
    }
*/
    int vix = 0;
    for (auto frame : fs)
    {
       Bdata.R[vix] = frame.second.pose_f2g.getRvec();
       Bdata.T[vix] = frame.second.pose_f2g.getTvec();

       Bdata.cameraMatrix[vix] = ip.CameraMatrix.clone();
       Bdata.distCoeffs[vix] = ip.Distorsion.clone();

       for (uint i = 0; i < frame.second.ids.size(); i++)
          if (frame.second.ids[i] != INVALID_MAPPT)
          {
             Bdata.visibility[vix][i] = 1;
             Bdata.imagePoints[vix][i] = frame.second.kpts[i].pt;
          }

       vix++; // Next view
    }

    return Bdata;
}



cvba::BundlerData<float> easyBundler(const ImageParams &ip, const Frame &f1,const Frame &f2,const std::vector<cv::DMatch> &matches,const vector<cv::Point3f>  &p3d  ){

    cvba::BundlerData<float> Bdata(2,matches.size());
    if (p3d.size()==0) triangulate(ip,f1,f2,matches,Bdata.points);
    else Bdata.points=p3d;

    cout<<Bdata.points.size()<<" "<<matches.size()<<endl;
    for(uint i=0;i<matches.size();i++){
        auto m=matches[i];
        Bdata.imagePoints[ 0][i]=(f1.kpts[m.queryIdx].pt);
        Bdata.imagePoints[ 1][i]=(f2.kpts[m.trainIdx].pt);
    }
    Bdata.R[0]=f1.pose_f2g.getRvec();
    Bdata.T[0]=f1.pose_f2g.getTvec();
    Bdata.R[1]=f2.pose_f2g.getRvec();
    Bdata.T[1]=f2.pose_f2g.getTvec();
    Bdata.cameraMatrix[0]=ip.CameraMatrix.clone();
    Bdata.cameraMatrix[1]=ip.CameraMatrix.clone();
    Bdata.distCoeffs[0]=ip.Distorsion.clone();
    Bdata.distCoeffs[1]=ip.Distorsion.clone();
    Bdata.setFullVisibilityVector();
    return Bdata;
}

