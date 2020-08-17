/**
Copyright 2014 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
***/
#ifndef manualscanner_impl_CvNI2_Header_
#define manualscanner_impl_CvNI2_Header_
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <OpenNI.h>
#include <iostream>
#include <string>
#include <memory>
#include <fstream>
#include <map>
#ifndef __func__
# define __func__ __FUNCTION__
#endif


class CvNI2  {

public:
    inline CvNI2(bool useColor=true) ;
    inline ~CvNI2() ;
    std::string getName() throw ( std::exception ) {return "ni2";}
    //returns a vector of connected devices
    inline static bool isDevicePlugged ( ) throw ( std::exception );
    //opens the device (or an oni file)
    inline void open ( std::string onifilepath="" ) throw ( std::exception );
    //closes the device
    inline void close()throw ( std::exception );

    //grans an image
    inline  bool grab(int timeout=-1) throw ( std::exception );
    //retrieves data to user space
    inline  void retrieve ( cv::Mat  &color,cv::Mat &depth ) throw ( std::exception );
    inline  void retrieve ( cv::Mat  &color ) throw ( std::exception );
    //releases the device
    inline  void release() throw ( std::exception );
    //indicates if the device is opened
    inline bool isOpen() const {return _isOpen;}

    inline bool isOniFile() const {return _isOniFile;}

    //enables saving the input to an output file
    //call after open
    inline void createRecorder ( std::string path ,bool start=false ) throw ( std::exception );

    inline bool isRecorderCreated (  ) {return !_recorder.empty();}
    inline void destroyRecorder (  ) throw ( std::exception );

    inline void startRecording( ) throw ( std::exception );
    inline void stopRecording( ) throw ( std::exception );
    inline bool isRecording( ) const {return _isRecording; }//&& !_recorder.empty();} <- unused now.

    inline void setAutoExposureEnabled ( bool enabled ) throw ( std::exception );
    inline void setAutoWhiteBalanceEnabled ( bool enabled ) throw ( std::exception );

    inline bool getAutoExposureEnabled()   throw ( std::exception );
    inline bool getAutoWhiteBalanceEnabled()   throw ( std::exception );

    inline bool setGain ( int gain ) throw ( std::exception );
    inline bool setExposure ( int exposure ) throw ( std::exception );
    inline int getGain() throw ( std::exception );
    inline int getExposure() throw ( std::exception );
    inline double getTimeStamp()const {return _ntimes_grabbed;}

    bool isMyFileType(std::string filepath) {return filepath.find(".oni")!=std::string::npos;}

private:
    inline uint64_t getSignature(const cv::Mat &c,const cv::Mat &d);

    bool _isOpen;
    bool _isOniFile;
    bool _useColor;
    openni::Device _device;
    openni::VideoStream _depth_stream,_color_stream;
    bool _isColorReady,_isDepthReady;
    openni::VideoFrameRef _depth_frame , _color_frame;
    cv::Ptr<openni::Recorder> _recorder;
    bool _isRecording;
    openni::PixelFormat _depthFormat;
    uint64_t _ntimes_grabbed;

    // Added manual management for saving files, to avoid loop reproduction of the OpenNI2.
    std::ifstream binaryfilestream;
    std::ofstream binarysave;

    bool isInputFromBinary = false;
    uint64_t _FirstFrameSig=std::numeric_limits<uint64_t>::min();
    std::map<uint64_t,uint64_t> signatures_frame;

};

bool CvNI2::isDevicePlugged ( ) throw ( std::exception ) {
    //INIT
    if ( openni::OpenNI::initialize() != openni::STATUS_OK )
        throw std::runtime_error ( "OpenNI initilization error"  );
    openni::Device device;
    bool res=  device.open ( openni::ANY_DEVICE ) == openni::STATUS_OK ;
    device.close();
    return res;

}

CvNI2::CvNI2(bool useColor) {

    _isRecording=_isOniFile=_isOpen=false;
    _useColor=useColor;
}
CvNI2::~CvNI2() {
    release();
}

void CvNI2::close()throw ( std::exception ){
    _device.close();

    if(binaryfilestream.is_open()) binaryfilestream.close();
    if(binarysave.is_open()) binarysave.close();
    _isOpen=false;
}

void CvNI2::open ( std::string onifilepath ) throw ( std::exception ) {

    _ntimes_grabbed=0;
    //INIT
    if ( openni::OpenNI::initialize() != openni::STATUS_OK )
        throw cv::Exception ( -1,"OpenNI initilization error",__func__,__FILE__,__LINE__ );

    //OPEN DEVICE
    if ( onifilepath.empty() ) {
        if ( _device.open ( openni::ANY_DEVICE ) != openni::STATUS_OK )
            throw  std::runtime_error ("OpenNI could not open device " );
        if ( _device.setDepthColorSyncEnabled ( true ) != openni::STATUS_OK )
            std::cerr<<__func__<<" : WARN: Can't set setDepthColorSyncEnabled:\n";
    } else {
        _isOniFile=true;

        /* OLD METHOD WITH THE ORIGINAL OPENNI2 DRIVER.*/
         if ( _device.open ( onifilepath.c_str() ) != openni::STATUS_OK ){
            throw  std::runtime_error ("OpenNI could not open path :"+onifilepath  );
        }

    }
    if (isInputFromBinary == true) {
        _isOpen=true;
    }

    //CHECK THAT THERE IS DEPTH SENSOR
    if ( _device.getSensorInfo ( openni::SENSOR_DEPTH ) == NULL )
        throw  std::runtime_error ("OpenNI Couldn't find depth sensor" );

    //OPEN DEPTH SENSOR
    if ( _depth_stream.create ( _device, openni::SENSOR_DEPTH ) != openni::STATUS_OK )
        throw   std::runtime_error (std::string ( "OpenNI Couldn't create depth stream:" ) +openni::OpenNI::getExtendedError() );

    //CHECK THAT THERE IS COLOR SENSOR
    if (_useColor)
        if ( _device.getSensorInfo ( openni::SENSOR_COLOR ) == NULL )
            _useColor=false;
    //           throw  std::runtime_error ("OpenNI Couldn't find color sensor" );

    if (_useColor){
        //OPEN COLOR SENSOR
        if ( _color_stream.create ( _device, openni::SENSOR_COLOR ) != openni::STATUS_OK )
            throw  std::runtime_error (std::string ( "OpenNI Couldn't create color stream:" ) +openni::OpenNI::getExtendedError() );
    }

    //Set depth registration, this is optional in many applications
    if ( onifilepath.empty() ) {
        _depth_stream.setMirroringEnabled ( false );
        if (_useColor) _color_stream.setMirroringEnabled ( false );
        if ( ! _device.isImageRegistrationModeSupported ( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
            throw  std::runtime_error ( "Device does not support registration " );
        if ( _device.setImageRegistrationMode ( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) != openni::STATUS_OK )
            throw  std::runtime_error ( "Could not set depth registration " );

    }

    //SET DEPTH SENSOR MODE 640x480 1mm
    const openni::SensorInfo& si = _depth_stream.getSensorInfo();
    int idx1mm=-1,idx100um=-1;
    for ( int i=0; i<si.getSupportedVideoModes().getSize(); i++ ){
        if ( si.getSupportedVideoModes() [i].getResolutionX() ==640 &&
             si.getSupportedVideoModes() [i].getResolutionY() ==480 &&
             si.getSupportedVideoModes() [i].getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM ) {
            idx1mm=i;
        }
        //        if ( si.getSupportedVideoModes() [i].getResolutionX() ==640 &&
        //                si.getSupportedVideoModes() [i].getResolutionY() ==480 &&
        //                si.getSupportedVideoModes() [i].getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_100_UM ) {
        //            idx100um=i;
        //        }
    }

    if ( idx1mm==-1 && idx100um==-1){
        throw  std::runtime_error ("OpenNI Couldn't find mode 640x480:1mm nor 640x480:100um" );
    }
    if (idx1mm!=-1){
        _depth_stream.setVideoMode ( si.getSupportedVideoModes() [idx1mm] );
        _depthFormat=openni::PIXEL_FORMAT_DEPTH_1_MM;
    }
    else{
        _depth_stream.setVideoMode ( si.getSupportedVideoModes() [idx100um] );
        _depthFormat=openni::PIXEL_FORMAT_DEPTH_100_UM;
    }

    //SET COLOR SENSOR MODE 640x480 RGB888
    if (_useColor){
        int idx=-1;
        const openni::SensorInfo& si_color= _color_stream.getSensorInfo();
        for ( int i=0; i<si_color.getSupportedVideoModes().getSize(); i++ )
            if ( si_color.getSupportedVideoModes() [i].getResolutionX() ==640 &&
                 si_color.getSupportedVideoModes() [i].getResolutionY() ==480 &&
                 si_color.getSupportedVideoModes() [i].getPixelFormat() == openni::PIXEL_FORMAT_YUV422 ) {
                idx=i;
                break;

            }
        if ( idx==-1 )
            throw  std::runtime_error ("OpenNI Couldn't find mode 640x480:PIXEL_FORMAT_YUV422 " );

        _color_stream.setVideoMode ( si_color.getSupportedVideoModes() [idx] );
    }
    //set frame by frame    if reading from file
    if ( !onifilepath.empty() ) {
        if ( _device.getPlaybackControl()->setSpeed ( -1 ) != openni::STATUS_OK )
            throw  std::runtime_error (std::string ( "OpenNI Couldn't set -1 speed in playback " ) +openni::OpenNI::getExtendedError() );
    }


    if ( _depth_stream.start() != openni::STATUS_OK )
        throw  std::runtime_error (std::string ( "OpenNI Couldn't start depth stream:" ) +openni::OpenNI::getExtendedError() );
    if (_useColor)
        if ( _color_stream.start() != openni::STATUS_OK )
            throw  std::runtime_error (std::string ( "OpenNI Couldn't start depth stream:" ) +openni::OpenNI::getExtendedError() );

    _isOpen=true;
}

//
//
void CvNI2::createRecorder ( std::string fpath,bool start ) throw ( std::exception ) {

    destroyRecorder();
    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
    if ( !_recorder.empty() )
        throw  std::runtime_error ("already recording" );
    _recorder=new openni::Recorder();
    if ( _recorder->create ( fpath.c_str() ) != openni::STATUS_OK )
        throw  std::runtime_error ("error opnening oni file for writing:"+fpath );
    //now, attatch the streams
    if (_useColor)
        if ( _recorder->attach ( _color_stream ) != openni::STATUS_OK )
            throw  std::runtime_error ("error attatching color stream" );
    if ( _recorder->attach ( _depth_stream ) != openni::STATUS_OK )
        throw  std::runtime_error ("error attatching depth stream" );
    _isRecording=false;
    if ( start ) startRecording();
}

void CvNI2::destroyRecorder (  ) throw ( std::exception ){
    _isRecording=false;
    if ( _recorder.empty() ) return;
    if ( _isRecording ) _recorder->stop();
    _recorder.release();
}

void CvNI2::startRecording( ) throw ( std::exception ) {
    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
    if ( _recorder.empty() )
        throw  std::runtime_error ("call  saveToOniFile first" );
    if ( !_isRecording )
        if ( _recorder->start() != openni::STATUS_OK )
            throw  std::runtime_error ("error starting to record" );
    _isRecording=true;
}
void CvNI2::stopRecording( ) throw ( std::exception ) {
    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
    if ( _recorder.empty() )
        throw  std::runtime_error ("call  saveToOniFile first" );
    if ( !_isRecording )
        throw  std::runtime_error ("not recording" );
    _recorder->stop();
    _isRecording=false;

}


uint64_t CvNI2::getSignature(const cv::Mat &color,const  cv::Mat &depth){
    uint64_t seed = 0;
    assert(depth.type()==CV_8UC2||depth.type()==CV_16UC1);
    for(int y=0;y<color.rows;y++){
        const cv::Vec2b* colorPtr=color.ptr<cv::Vec2b>(y);
        const uint16_t *depthPtr=depth.ptr<uint16_t>(y);
        for(int x=0;x<color.cols;x++){
            int ic=0;
            char *icc=(char*)&ic;
            memcpy(icc,(const char*)& (colorPtr[x][0]),2*sizeof(char));
            memcpy(icc+2,(const char*)&depthPtr[x],2*sizeof(char));
            seed^= ic +   0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
    }
    return seed;
}


bool CvNI2::grab(int timeOut ) throw ( std::exception ) {
    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
        int changedStreamDummy;
        openni::VideoStream  *pStream [2]= { &_depth_stream,&_color_stream};
        _isColorReady=_isDepthReady=false;
        int nStreams=2;
        if (!_useColor){
            _isColorReady=true;
            nStreams=1;
        }
        //wait for two streams to arrive
        while ( ( _isColorReady & _isDepthReady ) == false) {
            if (timeOut==-1) timeOut=openni::TIMEOUT_FOREVER;

            if ( openni::OpenNI::waitForAnyStream ( pStream, nStreams, &changedStreamDummy, timeOut ) !=openni::STATUS_OK ) {
                std::cerr<< "Wait failed! (timeout is    ) "<<  openni::OpenNI::getExtendedError() <<std::endl;
                return false;
            }
            //is depth frame ready??
            if ( !_depth_stream.readFrame ( &_depth_frame ) ==openni::STATUS_OK )
                throw  std::runtime_error ("Depth Stream error, exiting." );
            if ( _useColor)
                if ( !_color_stream.readFrame ( &_color_frame ) ==openni::STATUS_OK )
                    throw  std::runtime_error ("Color Stream error, exiting." );
            _isColorReady=true;
            _isDepthReady = true;
        }

        cv::Mat cimage(_color_frame.getHeight() ,_color_frame.getWidth(), CV_8UC2,(char*)_color_frame.getData());
        cv::Mat dimage(_depth_frame.getHeight(),_depth_frame.getWidth(),CV_16UC1,(uint16_t*)_depth_frame.getData());
        auto sig=getSignature(cimage,dimage);;
        try{
            auto &frameIdx=signatures_frame.at(sig);
            if (fabs(frameIdx-_ntimes_grabbed)>10) return false;
        }catch (std::exception &ex){
            signatures_frame.insert({sig,_ntimes_grabbed});

        }


        if (_ntimes_grabbed==0){
            _FirstFrameSig=getSignature(cimage,dimage);
        }
        else if ( getSignature(cimage,dimage)==_FirstFrameSig)return false;

        _ntimes_grabbed++;


    _isColorReady=true;
    _isDepthReady=true;

    return _isDepthReady&_isColorReady;
}


void CvNI2::retrieve ( cv::Mat  &color,cv::Mat &depth ) throw ( std::exception ) {

    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
    if ( ! ( _isDepthReady&_isColorReady ) )
        throw  std::runtime_error ("No frame to be retrieved." );
     if (_useColor){
        cv::Mat yuyv(_color_frame.getHeight() ,_color_frame.getWidth(), CV_8UC2,(char*)_color_frame.getData());
        cv::cvtColor(yuyv, color, CV_YUV2BGR_Y422);
    }
    depth.create ( _depth_frame.getHeight(),_depth_frame.getWidth(),CV_16UC1 );
    memcpy ( depth.ptr<char> ( 0 ),_depth_frame.getData(),_depth_frame.getWidth() *_depth_frame.getHeight() *2 );

}

void CvNI2::retrieve ( cv::Mat  &color ) throw ( std::exception ) {
        if (!_useColor) {std::cerr<<"Not using color but need it: "<<__FILE__ <<std::endl;return;}
        if ( !_isOpen )
            throw  std::runtime_error ("Not opened " );
        if ( ! ( _isDepthReady&_isColorReady ) )
            throw  std::runtime_error ("No frame to be retrieved." );

        cv::Mat yuyv(_color_frame.getHeight() ,_color_frame.getWidth(), CV_8UC2,(char*)_color_frame.getData());
        cv::cvtColor(yuyv, color, CV_YUV2BGR_Y422);



}

void CvNI2::release() throw ( std::exception ) {

    if ( !_recorder.empty() ) _recorder->stop();
    _depth_stream.stop();
    _depth_stream.destroy();
    if (_useColor){
        _color_stream.stop();
        _color_stream.destroy();
    }
    _device.close();
    openni::OpenNI::shutdown();
    _isOpen=false;
}


void CvNI2::setAutoExposureEnabled ( bool enabled ) throw ( std::exception ) {
    if (!_useColor) return ;
    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
    openni::CameraSettings *set=_color_stream.getCameraSettings();
    if ( set==NULL )
        throw  std::runtime_error ("Invalid camera settings" );
    if ( set->setAutoExposureEnabled ( enabled ) !=openni::STATUS_OK )
        throw  std::runtime_error ("setAutoExposureEnabled error" );

}
void CvNI2::setAutoWhiteBalanceEnabled ( bool enabled ) throw ( std::exception ) {
    if (!_useColor) return ;
    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
    openni::CameraSettings *set=_color_stream.getCameraSettings();
    if ( set==NULL )
        throw  std::runtime_error ("Invalid camera settings" );
    if ( set->setAutoWhiteBalanceEnabled ( enabled ) !=openni::STATUS_OK )
        throw  std::runtime_error ("setAutoWhiteBalanceEnabled error" );
}

bool CvNI2::getAutoExposureEnabled()   throw ( std::exception ) {
    if (!_useColor) return false;
    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
    openni::CameraSettings *set=_color_stream.getCameraSettings();
    if ( set==NULL )
        throw  std::runtime_error ("Invalid camera settings" );
    return set->getAutoExposureEnabled();


}

bool CvNI2::getAutoWhiteBalanceEnabled()  throw ( std::exception ) {
    if (!_useColor) return false;
    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
    openni::CameraSettings *set=_color_stream.getCameraSettings();
    if ( set==NULL )
        throw  std::runtime_error ("Invalid camera settings" );
    return set->getAutoWhiteBalanceEnabled();

}

bool CvNI2::setGain ( int gain ) throw ( std::exception ) {
    if (!_useColor) return false;

    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
    openni::CameraSettings *set=_color_stream.getCameraSettings();
    if ( set==NULL )
        throw  std::runtime_error ("Invalid camera settings" );
    return set->setGain ( gain ) ==openni::STATUS_OK ;


}
bool CvNI2::setExposure ( int exposure ) throw ( std::exception ) {
    if (!_useColor) return false;
    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
    openni::CameraSettings *set=_color_stream.getCameraSettings();
    if ( set==NULL )
        throw  std::runtime_error ("Invalid camera settings" );
    return set->setExposure ( exposure ) ==openni::STATUS_OK ;

}
int CvNI2::getGain() throw ( std::exception ) {
    if (!_useColor) return -1;
    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
    openni::CameraSettings *set=_color_stream.getCameraSettings();
    if ( set==NULL )
        throw  std::runtime_error ("Invalid camera settings" );
    return set->getGain();

}
int CvNI2::getExposure() throw ( std::exception ) {
    if (!_useColor) return -1;
    if ( !_isOpen )
        throw  std::runtime_error ("Not opened " );
    openni::CameraSettings *set=_color_stream.getCameraSettings();
    if ( set==NULL )
        throw  std::runtime_error ("Invalid camera settings" );
    return set->getExposure();

}

#endif
