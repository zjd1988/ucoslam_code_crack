#include "cvni2.h"
#include <iostream>
using namespace  std;
int main(int argc,char **argv)
{
    try{

        if (argc!=2){
            cerr<<"Usage out.oni "<<endl;
            return -1;
        }

        CvNI2 camera;
        camera.open();
        camera.createRecorder(argv[1],false);

        while( camera.grab()){

            cv::Mat color;
            camera.retrieve(color);
            if (camera.isRecording())
                cv::circle(color,cv::Point(30,30),10,cv::Scalar(0,0,255),-1);

            cv::imshow("image",color);
            int k=cv::waitKey(10);
            if (k==27)break;
            if(k=='a')
                camera.setAutoExposureEnabled(!camera.getAutoExposureEnabled());
            if (k=='r'){
                if (!camera.isRecording())
                    camera.startRecording();
                else camera.stopRecording();
            }

        }

    }catch(std::exception &ex){

        cout<<ex.what()<<endl;

    }
}
