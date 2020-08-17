#include <opencv2/highgui.hpp>
#include <iostream>
#include "ucoslam.h"
int main(int argc,char **argv){

    try {

        if(argc!=3) throw std::runtime_error("in.slam out.map");

        ucoslam::UcoSlam SLAM;
        SLAM.readFromFile(argv[1]);
        SLAM.getMap()->removeUnUsedKeyPoints();
        SLAM.getMap()->saveToFile(argv[2]);




    } catch (std::exception &ex) {
        std::cerr<<ex.what()<<std::endl;
    }


}
