#include <aruco/aruco.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "viewers/sgl.h"
using namespace std;



class DrawScene:public sgl::Drawer{
public:

     aruco::MarkerMap mmap;
     vector<cv::Mat> cameraPoses;

    void draw(sgl::Scene &scene){

        auto drawMarker=[](sgl::Scene &Scn, const aruco::Marker3DInfo &minf , int width=1){
            auto points=minf.points;
            Scn.drawLine((sgl::Point3*)&points[0],(sgl::Point3*)&points[1],{0,0,255},width);
            Scn.drawLine((sgl::Point3*)&points[1],(sgl::Point3*)&points[2],{0,255,0},width);
            Scn.drawLine((sgl::Point3*)&points[2],(sgl::Point3*)&points[3],{255,0,0},width);
            Scn.drawLine((sgl::Point3*)&points[3],(sgl::Point3*)&points[0],{155,0,155},width);
        };


        auto  drawPyramid=[](sgl::Scene &Scn,float w,float h,float z,const sgl::Color &color,int width=1){
            Scn.drawLine( {0,0,0}, {w,h,z},color,width);
            Scn.drawLine( {0,0,0}, {w,-h,z},color,width);
            Scn.drawLine( {0,0,0}, {-w,-h,z},color,width);
            Scn.drawLine( {0,0,0}, {-w,h,z},color,width);
            Scn.drawLine( {w,h,z}, {w,-h,z},color,width);
            Scn.drawLine( {-w,h,z}, {-w,-h,z},color,width);
            Scn.drawLine( {-w,h,z}, {w,h,z},color,width);
            Scn.drawLine( {-w,-h,z}, {w,-h,z},color,width);
        };

        auto drawAxis=[](sgl::Scene &Scn,float w, int width=1){

            Scn.drawLine( {0,0,0}, {0,0,w},{0,0,255},width);
            Scn.drawLine( {0,0,0}, {0,w,0},{0,255,0},width);
            Scn.drawLine( {0,0,0}, {w,0,0},{255,0,0},width);

        };


        scene.clear(sgl::Color(255,255,255));
        drawAxis(scene,2,2);
        //draw the markers of the map
        for(auto m:mmap)
            drawMarker(scene,m,2);
        float camsize=0.03;
        for(auto pose:cameraPoses)
        {
            scene.pushModelMatrix(sgl::Matrix44(pose.ptr<float>()));
            drawPyramid(scene,camsize,camsize*0.5,camsize*0.5,{255,0,255},2);
            scene.popModelMatrix();
        }
    }
};


int main(int argc,char **argv){
    try{

        if(argc<=3){cerr<<"Usage: markermap.yml cameraparams.yml im1 im2 ..."<<endl;return -1;}

        aruco::MarkerMap mmap;mmap.readFromFile(argv[1]);
        aruco::CameraParameters camparam;
        camparam.readFromXMLFile(argv[2]);
        aruco::MarkerDetector Mdetector;
//        Mdetector.getParams().enclosedMarker=true;
        Mdetector.setDictionary(mmap.getDictionary());
        aruco::CameraParameters cp;cp.readFromXMLFile(argv[2]);
        vector<cv::Mat> cameraPoses;
        vector<vector<aruco::Marker> > makerDetections;
        sgl::opencv::Viewer Viewer;
        auto drawer=std::make_shared<DrawScene>();
        Viewer.setParams(drawer,1,1280,960,"scene");
        drawer->mmap=mmap;
        for(int im=3;im<argc;im++){
            aruco::MarkerMapPoseTracker mpt;
            mpt.setParams(cp,mmap);
            cv::Mat image=cv::imread( argv[im]);
            auto markers=Mdetector.detect(image);
            cv::Mat camerapose;
            if (markers.size()==2){
                if (mpt.estimatePose(markers))
                    camerapose=mpt.getRTMatrix();
            }
            if (!camerapose.empty()){
                cameraPoses.push_back(camerapose);
                makerDetections.push_back(markers);
            }
        }
        drawer->cameraPoses=cameraPoses;
        Viewer.exec();
        //now, we have all the information to compare it with

        //for each pair of images with valid poses, compute the Essential Matrix and decompose
        double focal = (camparam.CameraMatrix.at<float>(0,0)+ camparam.CameraMatrix.at<float>(1,1))/2.;
        cv::Point2d pp( camparam.CameraMatrix.at<float>(0,2), camparam.CameraMatrix.at<float>(1,2));
        for(int i=0;i<cameraPoses.size();i++)
            for(int j=i+1;j<cameraPoses.size();j++){

                cv::Mat gt= cameraPoses[j]*cameraPoses[i].inv();

                vector<cv::Point2f> points1(8);
                vector<cv::Point2f> points2(8);

                // initialize the points here ... */
                int cp=0;
                for( int m = 0; m < 2; m++ ){
                    for( int p = 0; p < 4; p++,cp++ ){
                        points1[cp] =  makerDetections[i][m][p];
                        points2[cp] =  makerDetections[j][m][p];
                    }

                    cv::Mat E, R, t, mask,H,mask2;
                    E = cv::findEssentialMat(points1, points2, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
                    H=cv::findHomography(points1, points2,mask2,cv::RANSAC,1);
                    cout<<mask.t()<<endl;
                    if (!E.empty()){
                        recoverPose(E, points1, points2, R, t, focal, pp, mask);
                        cout<<"___________________"<<endl;
                        cout<<gt<<endl;
                        cout<<R<<" "<<endl<<t.t()<<endl;
                        cout<<"|||||||||||||||||||||||||"<<endl;
                    }
                }
            }


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }

}
