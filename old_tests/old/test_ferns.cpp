#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include "stuff/ferns.h"
#include "stuff/timers.h"
#include "featureextractors/ORBextractor.h"
#include <fbow/fbow.h>
#include "stuff/framedatabase.h"
using namespace std;

int  main(int argc,char **argv){
    try{
        if (argc!=3) throw std::runtime_error ("Usage:videofile bow");
        cv::VideoCapture vcap;
        vcap.open(argv[1]);
        fbow::Vocabulary voc;
        voc.readFromFile(argv[2]);
        ucoslam::ORBextractor extractor(2000,8,1.2);
        extractor.setNumberOfThreads(1);


        bool endP=false;
        cv::Mat inim,ingray,im,im2;
        ucoslam::FernKeyFrameDataBase DB;
        ucoslam::KPFrameDataBase KpDB;
        KpDB.loadFromFile(argv[2]);
        DB.setParams(0.05,10,4);
        uint32_t id=0;
        ucoslam::TimerAvrg TimerAdd(100),TimerQuery(100),TimerAddDB(100);
        float minV=std::numeric_limits<float>::max();
        vector<fbow::fBow> FBowDB;
         ucoslam::TimerAvrg Timer1(100),Timer2(100),Timer3(100);

        while(vcap.grab() &&!endP && id<5000){

            vcap.retrieve(inim);
            cv::cvtColor(inim,ingray,CV_BGR2GRAY);
            //resize
            float f= 128./float(inim.cols);

            vector<int> code;
            TimerAdd.start();
            cv::resize(ingray,im,cv::Size( 128, float(inim.rows)*f));
            cv::GaussianBlur(im,im2,cv::Size(7,7),5,5);
            code=DB.transform(im2);
            DB.add(code,id);
            TimerAdd.stop();
            TimerQuery.start();
            auto query_res=DB.query(code,true);
            TimerQuery.stop();


            for(auto id_dist:query_res){
                cout<<id_dist.second<<" ";
                minV=std::min(minV,id_dist.second);
            }
            cout<<endl<<endl;

            if (id%100==0){
            cout<<endl;
            cout<<"size="<<DB.size()<<endl;
            cout<<"distanceRange="<<(1-minV)<<endl;
            cout<<"Addition time "<<int(1.f/TimerAdd.getAvrg())<<" Hz"<<endl;
            cout<<"Query time    "<<int(1.f/TimerQuery.getAvrg())<<" Hz"<<endl;
            }

//            cv::Mat smaller;
//            cv::resize(ingray,smaller,cv::Size(640,480));
//            ucoslam::Frame frame;
//            frame.idx=id;
//            Timer1.start();
//            extractor.detectAndCompute(smaller,cv::Mat(),frame.und_kpts,frame.desc);
//            Timer1.stop();

//            Timer2.start();
//            KpDB.add(frame);
//            Timer2.stop();

//            vector<float> distances(FBowDB.size());
//            Timer3.start();
//            for(size_t i=0;i<FBowDB.size();i++)
//                distances[i]=fbow::fBow::score(FBowDB[i],FBowDB.back());
//            Timer3.stop();
//            if (id%100==0){

//            cout<<"BOW Kp time "<<int(1.f/Timer1.getAvrg())<<" Hz"<<endl;
//            cout<<"BOW Add time    "<<int(1.f/Timer2.getAvrg())<<" Hz"<<endl;
//            }

            id++;
        }
        ofstream file("out.bin",std::ios::binary);
        DB.toStream(file);
        file.close();
        ifstream filein("out.bin",std::ios::binary);
        DB.fromStream(filein);
        filein.close();
        file.open("out2.bin",std::ios::binary);
        DB.toStream(file);
        file.close();
    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
