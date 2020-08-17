#include <opencv2/highgui.hpp>

#include <iostream>
using namespace  std;
int main(int argc,char **argv){
    try{
        if (argc!=4){cerr<<"In1 In2 ... out"<<endl;return -1;}

        cv::VideoWriter vwriter;
        cv::VideoCapture vreader;
        vreader.open(argv[1]);
        cout<<"Open "<<argv[1]<<endl;
        if (!vreader.isOpened()) throw std::runtime_error("Could not open input video");

        cv::Mat frame;
        vreader>>frame;
        vwriter.open(argv[argc-1], CV_FOURCC('D','I','V','X'), 30, frame.size());
        if (!vwriter.isOpened()) throw std::runtime_error("Could not open output video");
        vwriter<<frame;

        int i=1;
        do{
            while(vreader.grab()){

                vreader.retrieve(frame);
                vwriter<<frame;
                cerr<<vreader.get(CV_CAP_PROP_POS_FRAMES)<< " ";
            }

            i++;
            if (i<=argc-2){
                vreader.release();
                cout<<"Open "<<argv[i]<<endl;
                vreader.open(argv[i]);

            }
        }while(i<=argc-2);

        vwriter.release();

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }
}
