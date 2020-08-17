#include <iostream>
#include "nanogo.h"
#include <opencv2/calib3d.hpp>
#include "ucoslam.h"
#include "sparselevmarq.h"
#include <cmath>
#include <random>
#include "nba.h"
using namespace std;
using namespace ucoslam;
using namespace nba;



int main(int argc,char **argv){
    try {
        if(argc<3)throw std::runtime_error("Usage: inmap outmap [iterations]");

        ucoslam::Map TheMap;
              cout<<"reading map"<<endl;
        TheMap.readFromFile(argv[1]);
        int niters=50;
        if(argc>=4)niters=stoi(argv[3]);

        nanogo::Graph<double> graph;
        //first, the cameras
        map<uint32_t, SE3Pose > camera_poses;
        CameraIntrinsics camparams;
        for(auto &kf:TheMap.keyframes){
            if (kf.isBad())continue;
            camera_poses[kf.idx].setParams(kf.pose_f2g.getRvec(),kf.pose_f2g.getTvec());
             if(!camparams.isSet())
                 camparams.setParams(kf.imageParams.CameraMatrix,kf.imageParams.Distorsion);
        }

        //now points and projections
        vector<MPoint> points;points.reserve(TheMap.map_points.size());
        list<ReprjError> errors;
        for(auto &p:TheMap.map_points){
            points.push_back(MPoint(p.getCoordinates()));
            for(auto of:p.getObservingFrames()){
                auto &kf=TheMap.keyframes[of.first];
                if ( kf.isBad() )continue;

                float weight=p.isBad() ?0:kf.scaleFactors[kf.und_kpts[of.second].octave];

                errors.push_back( ReprjError(&points.back(),&camparams,&camera_poses[of.first], kf.kpts[of.second],weight));
                graph.add(&errors.back());
            }
        }
        cout<<"POINTS="<<TheMap.map_points.size()<< " CAMS="<<camera_poses.size()<<endl;

        nanogo::SparseGraphSolver<double> SPGSolver;
        nanogo::SparseGraphSolver<double>::Params params;
        params.verbose=true;
        params.tau=0.0005;
        params.maxIters=niters;
        SPGSolver.solve(graph,params);

        //move data back to the map
        int ip=0;
        for(auto &mp:TheMap.map_points)
            mp.setCoordinates(  points[ip++].getPoint3d());

        //now, the cameras
        for(auto pose:camera_poses){
            TheMap.keyframes[pose.first].pose_f2g=ucoslam::se3(pose.second(0),pose.second(1),pose.second(2),pose.second(3),pose.second(4),pose.second(5));

            TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(0,0)=camparams(0);
            TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(1,1)=camparams(1);
            TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(0,2)=camparams(2);
            TheMap.keyframes[pose.first].imageParams.CameraMatrix.at<float>(1,2)=camparams(3);
            for(int p=0;p<5;p++)
                TheMap.keyframes[pose.first].imageParams.Distorsion.ptr<float>(0)[p]=camparams(4+p);
        }

        TheMap.saveToFile(argv[2]);
    } catch (const std::exception &ex) {
        cerr<<ex.what()<<endl;
    }


}
