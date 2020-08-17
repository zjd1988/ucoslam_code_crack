#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include "viewers/sgl.h"
#include <aruco/aruco.h>
#include <iostream>
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

        auto drawStereo=[&](sgl::Scene &Scn,float w,float h,float z,const sgl::Color &color,int width=1){

            drawPyramid(Scn,w,h,z,color,width);
            sgl::Matrix44 m;m.translate({0.1,0,0});
            Scn.pushModelMatrix(m);
            drawPyramid(Scn,w,h,z, {255-color[0],255-color[1],255-color[2]},width);
            Scn.popModelMatrix();
            Scn.drawLine({0,0,0},{0.1,0,0},color,width);

        };


        scene.clear(sgl::Color(255,255,255));
        //draw the markers of the map
        for(auto m:mmap)
            drawMarker(scene,m,2);
        float camsize=0.03;
        for(auto pose:cameraPoses)
        {
            scene.pushModelMatrix(sgl::Matrix44(pose.ptr<float>()));
            drawStereo(scene,camsize,camsize*0.5,camsize*0.5,{255,0,255},2);
            scene.popModelMatrix();
        }
    }
};


int main(int argc,char **argv){

 try{
        if (argc<4){cerr<<"In_yml cameraParams imageout im1 im2..."<<endl;return -1;}

        aruco::MarkerMap mmap;mmap.readFromFile(argv[1]);

        aruco::MarkerDetector Mdetector;
//        Mdetector.getParams().enclosedMarker=true;
        Mdetector.setDictionary(mmap.getDictionary());
        aruco::CameraParameters cp;cp.readFromXMLFile(argv[2]);
        vector<cv::Mat> cameraPoses;

        for(int im=4;im<argc;im++){
            aruco::MarkerMapPoseTracker mpt;
            mpt.setParams(cp,mmap);
            cv::Mat image=cv::imread( argv[im]);
            auto markers=Mdetector.detect(image);
            //for(auto m:markers) m.draw(image);
            if (mpt.estimatePose(markers))
                cameraPoses.push_back(mpt.getRTMatrix());
        }


        auto drawer=std::make_shared<DrawScene>();
        drawer->mmap=mmap;
        drawer->cameraPoses=cameraPoses;
        sgl::opencv::Viewer Viewer;
        Viewer.setParams(drawer, 1 ,1280,960, "calobject");
        Viewer.exec();


    }   catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
