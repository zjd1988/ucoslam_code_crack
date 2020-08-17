/**
* This file is part of  UCOSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* UCOSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* UCOSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with UCOSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/
#include "imageparams.h"
#include "basictypes/io_utils.h"
#include "basictypes/hash.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
namespace ucoslam {

ImageParams::ImageParams(){
    CamSize=cv::Size(-1,-1);
}
ImageParams& ImageParams::operator=(const ImageParams& IP){
    IP.CameraMatrix.copyTo(CameraMatrix);
    IP.Distorsion.copyTo(Distorsion);
    CamSize=IP.CamSize;

    bl=IP.bl;//stereo camera base line
    rgb_depthscale=IP.rgb_depthscale;//scale to obtain depth from the rgbd values
    return *this;
}



ImageParams::ImageParams(const ImageParams &IP){
    (*this)=IP;

}
bool ImageParams::isValid() const
{
    return CameraMatrix.rows != 0 && CameraMatrix.cols != 0 && Distorsion.rows != 0 && Distorsion.cols != 0
            && CamSize.width != -1 && CamSize.height != -1;
}
/**Adjust the parameters to the size of the image indicated
 */
void ImageParams::resize(cv::Size size)
{
    if (!isValid())
        throw cv::Exception(9007, "invalid object", "CameraParameters::resize", __FILE__, __LINE__);
    if (size == CamSize)
        return;
    // now, read the camera size
    // resize the camera parameters to fit this image size
    float AxFactor = float(size.width) / float(CamSize.width);
    float AyFactor = float(size.height) / float(CamSize.height);
    CameraMatrix.at<float>(0, 0) *= AxFactor;
    CameraMatrix.at<float>(0, 2) *= AxFactor;
    CameraMatrix.at<float>(1, 1) *= AyFactor;
    CameraMatrix.at<float>(1, 2) *= AyFactor;
    CamSize = size;
}

void ImageParams::undistortPoints(std::vector<cv::Point2f> &points_io, std::vector<cv::Point2f> *out){

    std::vector<cv::Point2f> pout;
    if (out==0) out=&pout;

    assert(CameraMatrix.type()==CV_32F);
    if(!fisheye_model)
        cv::undistortPoints ( points_io, *out, CameraMatrix,  Distorsion);//results are here normalized. i.e., in range [-1,1]
    else{
        cv::fisheye::undistortPoints(points_io, *out, CameraMatrix,  Distorsion);
    }

    float fx=CameraMatrix.at<float> ( 0,0 );
    float fy=CameraMatrix.at<float> ( 1,1 );
    float cx=CameraMatrix.at<float> ( 0,2 );
    float cy= CameraMatrix.at<float> ( 1,2 );

    if (out==&pout){
        for ( size_t i=0; i<pout.size(); i++ ) {
            points_io[i].x=pout[i].x*fx+cx;
            points_io[i].y=pout[i].y*fy+cy;
        }
    }
    else{
        for ( size_t i=0; i<pout.size(); i++ ) {
            (*out)[i].x=(*out)[i].x*fx+cx;
            (*out)[i].y=(*out)[i].y*fy+cy;
        }
    }
}


uint64_t ImageParams::getSignature()const{
    Hash Sig;
    Sig+=CameraMatrix;
    Sig+=Distorsion;
    Sig.add(CamSize);
    Sig.add(bl);
    Sig.add(rgb_depthscale);
    return Sig;
}


/****
 *
 *
 *
 *
 */
void ImageParams::readFromXMLFile(std::string filePath)
{
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    if(!fs.isOpened()) throw std::runtime_error(std::string(__FILE__)+" could not open file:"+filePath);


    int w = -1, h = -1;
    cv::Mat MCamera, MDist;
    fs["image_width"] >> w;
    fs["image_height"] >> h;
    fs["distortion_coefficients"] >> MDist;
    fs["camera_matrix"] >> MCamera;
    fisheye_model=0;
    fs["fisheye_model"] >> fisheye_model;

    fs["baseline"]>>bl;
    float ds=0;
    fs["rgb_depthscale"]>>ds;

    if(ds==0) rgb_depthscale=1;
    else rgb_depthscale=ds;

    if (MCamera.cols == 0 || MCamera.rows == 0){
        fs["Camera_Matrix"] >> MCamera;
        if (MCamera.cols == 0 || MCamera.rows == 0)
            throw std::runtime_error(std::string(__FILE__)+" File :" + filePath + " does not contains valid camera matrix");
    }

    if (w == -1 || h == 0){
        fs["image_Width"] >> w;
        fs["image_Height"] >> h;
        if (w == -1 || h == 0)
           throw std::runtime_error(std::string(__FILE__)+  "File :" + filePath + " does not contains valid camera dimensions");
    }
    if (MCamera.type() != CV_32FC1)
        MCamera.convertTo(CameraMatrix, CV_32FC1);
    else
        CameraMatrix = MCamera;

    if (MDist.total() < 4 && !fisheye_model){
        fs["Distortion_Coefficients"] >> MDist;
        if (MDist.total() < 4)
             throw std::runtime_error(std::string(__FILE__)+   "File :" + filePath + " does not contains valid distortion_coefficients" );
    }

    // convert to 32 and get the first elements only
    cv::Mat mdist32;
    MDist.convertTo(mdist32, CV_32FC1);

    Distorsion=cv::Mat::zeros(1, 5, CV_32FC1);
    for (int i = 0; i < mdist32.total(); i++)
        Distorsion.ptr<float>(0)[i] = mdist32.ptr<float>(0)[i];

    CamSize.width = w;
    CamSize.height = h;

}

std::ostream & operator<<(std::ostream &str,const ImageParams &ip){
    str<<ip.CameraMatrix<<std::endl<<ip.Distorsion<<std::endl<<ip.CamSize<<std::endl<<"bl="<<ip.bl<<" rgb_depthscale="<<ip.rgb_depthscale<<" fisheye_model="<<ip.fisheye_model<< std::endl;
    return str;
}
/**Saves this to a file
  */
void ImageParams::saveToXMLFile(std::string path )
{
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "image_width" << CamSize.width;
        fs << "image_height" << CamSize.height;
        fs << "camera_matrix" << CameraMatrix;
        fs << "distortion_coefficients" << Distorsion;
        fs << "baseline"<<bl;
        fs << "rgb_depthscale"<<rgb_depthscale;
        fs << "fisheye_model"<<fisheye_model;
}

void ImageParams::toStream(std::ostream &str) const {

    toStream__(CameraMatrix,str);
    toStream__(Distorsion,str);
    str.write((char*)&CamSize,sizeof(CamSize));
    str.write((char*)&bl,sizeof(bl));
    str.write((char*)&rgb_depthscale,sizeof(rgb_depthscale));
    str.write((char*)&fisheye_model,sizeof(fisheye_model));

}

void ImageParams::fromStream(std::istream &str) {
    fromStream__(CameraMatrix,str);
    fromStream__(Distorsion,str);
    str.read((char*)&CamSize,sizeof(CamSize));
    str.read((char*)&bl,sizeof(bl));
    str.read((char*)&rgb_depthscale,sizeof(rgb_depthscale));
    str.read((char*)&fisheye_model,sizeof(fisheye_model));

}
}
