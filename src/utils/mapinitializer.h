#ifndef _MAPINITIALIZER_H
#define _MAPINITIALIZER_H
#include <opencv2/opencv.hpp>
#include <utility>
#include "map.h"
#include "utils/framematcher.h"

namespace ucoslam
{

class MapInitializer {
    typedef std::pair<int,int> Match;
    Frame _refFrame;
    int _nAttempts = 0;
    FrameMatcher fmatcher;
public:
    enum MODE { ARUCO,KEYPOINTS,BOTH,NONE };
    struct Params {
        float minDistance = 0.1;
        float minDescDistance = -1;
        float markerSize = 0;
        float minNumMatches = 50;
        float aruco_minerrratio_valid = 2.5;
        bool  allowArucoOneFrame = false;
        float max_makr_rep_err = 2.5;
        float nn_match_ratio = 0.8;
        MODE mode = BOTH;
        int  maxAttemptsRestart = 150;
    };
    Params _params;
public:
    MapInitializer(float sigma = 1.0, int iterations = 200);
    void setParams(Params &p);
    void reset();
    bool process(const Frame &f, std::shared_ptr<Map> map);
    void setReferenceFrame(const Frame &frame);
    bool initialize(const Frame &frame2, std::shared_ptr<Map> map);

private:
    bool initialize_(const Frame &frame2, std::shared_ptr<Map> map);
    vector<cv::DMatch > _9860761537440310106;
    vector<cv::Point3f> _11999208601973379867;
    bool _7274126694617365277( const Frame &_46082543180066935, std::shared_ptr<Map> _11093822290287);
    std::pair<cv::Mat,MODE> _9813592252344743680(const Frame &_3005401603918369712, 
        const Frame &_3005401603918369727, vector <cv::DMatch> &_6807036698572949990, float _1686524438688096954 = 0);

    bool _11671783024730682148(const cv::Mat &_1646854292500902885,const std::vector<cv::KeyPoint> & _594645163826321276,
        const std::vector<cv::KeyPoint> &_16987994083097500267, vector<cv::DMatch> &_5507076421631549640,
        cv::Mat &_11093821901461, cv::Mat &_11093822381707, vector<cv::Point3f> &_706246337618936, 
        vector<bool> &_1883142761634074017);
    void _5668215658172178667(vector<bool> &_17271253899155467930, float &_46082543172245582, cv::Mat &_11093822009278 );

    void _11788074806349018216(vector<bool> &_1527043420153744413, float &_46082543172245582, cv::Mat &_11093821994570);
    cv::Mat _5082483593203424965(const vector<cv::Point2f> &_11093821939251, const vector<cv::Point2f> &_11093821939250);
    cv::Mat _5082483593203429753(const vector<cv::Point2f> &_11093821939251, const vector<cv::Point2f> &_11093821939250);
    float _17785459191918434301(const cv::Mat &_11093822009278, const cv::Mat &_11093822009342, 
        vector<bool> &_17271253899155467930, float _46082543171087264 );
    float _9516949822686382330(const cv::Mat &_11093821994570, vector<bool> &_17271253899155467930, float _46082543171087264);
    bool _1187546224459158354(vector<bool> &_17271253899155467930, cv::Mat &_11093821994570, cv::Mat &_2654435844,
        cv::Mat &_11093821901461, cv::Mat &_11093822381707, vector<cv::Point3f> &_706246337618936, 
        vector<bool> &_1883142761634074017, float _1686011036669914721, int _3782192360946256634);

    bool _1187546224459158348(vector<bool> &_17271253899155467930, cv::Mat &_11093822009278, cv::Mat &_2654435844,
        cv::Mat &_11093821901461, cv::Mat &_11093822381707, vector<cv::Point3f>&_706246337618936, 
        vector<bool> &_1883142761634074017, float _1686011036669914721, int _3782192360946256634 );

    void _13188689179917490056(const cv::KeyPoint &_11093822348761, const cv::KeyPoint &_11093822348742, 
        const cv::Mat &_175247761573, const cv::Mat &_175247761572, cv::Mat &_11093821939030 );

    void _12337435833092867029(const vector<cv::KeyPoint> &_46082576192198777, vector<cv::Point2f> &_11959346835625416039, 
        cv::Mat &_2654435853);

    int _993392631849970936(const cv::Mat &_2654435851, const cv::Mat &_2654435885, 
        const vector<cv::KeyPoint>&_3005399810348660273, const vector<cv::KeyPoint>&_3005399810348660272,
        const vector<Match> &_5507076421631549640, vector<bool> &_1527043420153744413,
        const cv::Mat &_2654435844, vector<cv::Point3f> &_706246337618936, float _11093822376282, 
        vector<bool> &_3005399809928654743, float &_16937213055926717083);


    void _11668855431419298303(const cv::Mat &_2654435838, cv::Mat &_175247761703, cv::Mat &_175247761702, cv::Mat &_2654435885);
    vector<cv::KeyPoint> _994360233184819249;
    vector<cv::KeyPoint> _994360233184819248;
    vector<Match> _7917477704619030428;
    cv::Mat _8168161081211572021;
    float _10193178724179165899, _994360216124235670;
    int _3513368080762767352;
    vector<vector<size_t>> _10193178724467558310;
    map <uint32_t,se3> _4498230092506100729 ;
};
}

#endif