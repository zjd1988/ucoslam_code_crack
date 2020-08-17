#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <aruco/aruco.h>
#include <ORBextractor.h>

#include <track.h>
#include <track_utils.h>
#include <utils.h>

using namespace std;
using namespace cv;
using namespace ucoslam;

#define MAXFRAMES 5
#define MKSIZE    0.039



class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

/* UTILS */


/*
void getLastTrackPoints(VTracks & vTracks, int frame, std::vector<cv::Point2f> & points, std::vector<int> & trIdx)
{
   int ntracks = vTracks.size();

   points.clear();
   trIdx.clear();

   // For each track
   for (int trix = 0; trix < ntracks; trix++)
   {
       cv::Point2f pt;
       int frameIdx;
       pt = vTracks[trix].getLastPoint(frameIdx);
       if (frameIdx == frame)
       {
          points.push_back(pt);
          trIdx.push_back(trix);
       }

   }
}
*/
/*
int findGoodMatches(const cv::Mat & descFrame, const cv::Mat & descNextFrame, std::vector< cv::DMatch > & goodMatches, int thrCte = 8)
{
    // For each keypoint, find its nearest neighbour
     //FlannBasedMatcher matcher;
     cv::BFMatcher matcher(NORM_HAMMING, true);
     std::vector< DMatch > matches;
     matcher.match(descFrame, descNextFrame, matches);
     cout << "Found " << matches.size() << " matches" << endl;

     int mindist, maxdist;
     mindist = maxdist = matches[0].distance;
     for (uint i = 0; i < matches.size(); i++)
     {
        if (matches[i].distance < mindist)
            mindist = matches[i].distance;
        if (matches[i].distance > maxdist)
            maxdist = matches[i].distance;
     }
     cout << "Distances interval: " << mindist << ", " << maxdist << endl;
     mindist = max(mindist, 1);

     goodMatches.clear(); // Just in case

     for (uint i = 0; i < matches.size(); i++)
     {
        if (matches[i].distance < mindist* thrCte) // DEVELOP!!!
            goodMatches.push_back(matches[i]);
     }

     return goodMatches.size();
}
*/

/* ============== MAIN ============= */
int main(int argc, char ** argv)
{
   CmdLineParser cmd(argc, argv);
   if (argc < 2 || cmd["-h"]) {
       cerr<<"Usage: -video <videoname> [-campars <camera_params.yml>] [-fm <0|1>] "<< endl;return -1;
   }
   string vidname = cmd("-video", "capture.avi");   

   string filterMethodS = cmd("-fm", "0");
   ucoslam::FILTER_METHOD filterMethod = ucoslam::FM_HOMOGRAPHY;
   if (stoi(filterMethodS) == 1) filterMethod = ucoslam::FM_FUNDAMENTAL;

   try
   {
      cv::VideoCapture vid;
      vid.open(vidname);

      if (!vid.isOpened())
         throw std::runtime_error("Could not open video file");

      Tracker tracker;
      int numKeyPoints = 1000;
      tracker.init(numKeyPoints);

      // Load camera parameters if available
      aruco::CameraParameters camParam;
      if (cmd["-campars"]) //(filecampars != "-1")
      {
         camParam.readFromXMLFile(cmd("-campars"));
      }

      int nframes = 0;
      aruco::MarkerDetector MKdetector;
      MKdetector.setDictionary("ARUCO_MIP_36h12"); // mjmarin: default type
      vector<aruco::Marker> Markers;

      cv::Mat frame, prevframe;

      std::vector<VMarkers> lMarkers;
      std::vector<cv::Mat> lDescriptors;
      std::vector< vector<cv::KeyPoint> > lKeypoints;

      //VKPTracks kpTracks;
      VTracks vTracks;

      TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03); // For optical flow
      Size winSize(15,15); //winSize(31,31);

      char key = 0;
      // Process video frames
      while(vid.grab() && key != 27) // ESC
      {
         // Get frame
         vid.retrieve(frame);
         nframes++;
         cout << "###Frame: " << nframes << endl;
/*
         // Detect markers
         if (camParam.isValid())
            MKdetector.detect(frame, Markers, camParam.CameraMatrix, camParam.Distorsion, MKSIZE);
         else
            MKdetector.detect(frame, Markers);

         cout << Markers.size() << endl;
*/
         // Detect keypoints
         tracker.process(frame);


         // Draw keypoints
         tracker.drawKeypoints(frame);

         tracker.drawTracks(frame);

         //cout << tracker.getTracks()[0] << endl;

         //cout << "Total tracks: " << tracker.size() << endl;
         cout << tracker;
/*
         // Optical flow based tracking
         vector<uchar> status;
         vector<float> err;
         std::vector<int> trIdx;
         std::vector<cv::Point2f> prevPoints, curPoints;
         if (nframes > 1)
         {
            getLastTrackPoints(vTracks, nframes, prevPoints, trIdx);
            calcOpticalFlowPyrLK(prevframe, frame, prevPoints, curPoints, status, err, winSize,
                              3, termcrit, 0, 0.001);

            // Draw flow
            for (uint trix = 0; trix < prevPoints.size(); trix++)
            {
               line(frame, prevPoints[trix], curPoints[trix], Scalar(0,0,255), 2);
            }
         }

         if (nframes >= MAXFRAMES)
         {
             //cout << "End of tracking" << endl;

             // Print track coordinates
             for (uint trix = 0; trix < vTracks.size(); trix++)
             {
                cout << endl <<"Track " << trix << ": ";
                for (uint pix = 0; pix < vTracks[trix].vPoints.size(); pix++)
                   cout << "(" << vTracks[trix].vPoints[pix].pt.x << ", " <<  vTracks[trix].vPoints[pix].pt.y <<  ") ";
             }

         }

*/

         // Generate 3D points
         if (tracker.getCurrentFrame() > 5)
         {
             std::vector<cv::Point2f> pts1, pts2, undistP01, undistP02;
             //undistortTrackPoints(tracker.getTracks(), 1, camParam, undistP01);
             //undistortTrackPoints(tracker.getTracks(), 5, camParam, undistP02);

             vector<cv::DMatch> wMatches, tmpMatches;
             getPointsPairFromTracks(tracker.getTracks(), tracker.getCurrentFrame()-5, tracker.getCurrentFrame(), pts1, pts2, tmpMatches);

             if (pts1.size() > 4) // We need at least four points
             {
                 undistortPointSet(pts1, camParam, undistP01);
                 undistortPointSet(pts2, camParam, undistP02);

                 cv::Mat transMat;
                 wMatches = filter_matches(undistP01, undistP02, tmpMatches, filterMethod, transMat, 2.5f);

                 if (wMatches.size() >= 4) // Can it be computed?
                 {
                     // Use the filtered points to compute the rt info
                     vector<cv::Point2f> points1_und,points2_und;
                     points1_und.reserve(wMatches.size());
                     points2_und.reserve(wMatches.size());
                     for (auto m: wMatches ){
                         points1_und.push_back(undistP01[m.queryIdx]);
                         points2_und.push_back(undistP02[m.queryIdx]);
                     }

                     // Triangulate
                     std::vector<cv::Point3f > points3D;

                     std::vector<RTSolution> rtSol;
                     rtSol = getRtFromMatrix(filterMethod, transMat, camParam.CameraMatrix, points1_und, camParam.CameraMatrix, points2_und);

                     if (rtSol[0].positivePointsRatio > 0.9) // DEVELOP!!! [0]
                     {

                         // Try bundle adjustment
                         // TODO: cvba

                         points3D = rtSol[0].points3d;
                         char pcdfile[1024];
                         sprintf(pcdfile, "reconstructionFr%02d_fm%d.pcd", tracker.getCurrentFrame(), filterMethod);
                         savePointsToPCD(points3D, pcdfile);
                         cout << "3D reconstruction with " << points3D.size() << " points." << endl;
                     }
                 }
             }
         }

         // Display
         cv::imshow("Frame", frame);
         key = cv::waitKey(0);

         cv::swap(prevframe, frame); //frame.copyTo(prevframe);

      } // while grab

   }
   catch(std::exception & exc)
   {cout << exc.what() << endl;}

   return 0;
}

