#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

int main(int argc, char ** argv)
{
   CmdLineParser cmd(argc, argv);
    if (argc<2){cerr<<"Usage : out.avi [camera_index]"<<endl;return -1;}

   string vidname = argv[1];//cmd("-video", "capture.avi");

   int index=0;
   if (argc>=3) index=stoi(argv[2]);
   try
   {
      cv::VideoCapture vid;
      vid.open(index);

      if (!vid.isOpened())
         throw std::runtime_error("Could not open camera");

      cv::Mat frame;
        while(frame.empty()) vid>>frame;

      cv::VideoWriter vidout;

      char key = 0;

      int img_num=0;
      bool isrecording=false;
      while(vid.grab() && key != 27) // ESC
      {
         vid.retrieve(frame);

         cv::imshow("Frame", frame);

         key = cv::waitKey(5);
         if (key=='s') isrecording=!isrecording;
         if(key=='w'){
             string name=std::to_string(img_num++);
             while(name.size()!=4) name="0"+name;
             name+=".jpg";
             imwrite(name,frame);
         }
         if (isrecording)
         {
             if (!vidout.isOpened())
                 vidout.open(vidname, CV_FOURCC('D','I','V','X'), 30, frame.size());
             vidout << frame;
             cout<<"recording"<<endl;
         }


      }
      vidout.release();

   }
   catch(std::exception & exc)
   {cout << exc.what() << endl;}   

   return 0;
}
