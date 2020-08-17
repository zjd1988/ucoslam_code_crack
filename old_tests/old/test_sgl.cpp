/*****************************
Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

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
********************************/
//IF YOU USE THIS CODE, PLEASE CITE
//"Automatic generation and detection of highly reliable fiducial markers under occlusion"
//http://www.sciencedirect.com/science/article/pii/S0031320314000235
//AND
//"Generation of fiducial marker dictionaries using mixed integer linear programming"
//http://www.sciencedirect.com/science/article/pii/S0031320315003544

#include <memory>
#include <cstring>
#include <vector>
#include <cmath>
#include <iostream>

//Simplest graphics library
//Author. Rafael Muñoz Salinas (rmsalinas@uco.es) 2017

//Extremmely simple yet efficient device independent graphic library for points and lines
namespace sgl{
struct Point3{
    Point3( ){ }
    Point3(float X,float Y,float Z){x=X;y=Y;z=Z;}
    float x,y,z;

    friend std::ostream & operator<<(std::ostream &str, const Point3 &p){
        str<<"["<<p.x<<" "<<p.y<<" "<<p.z<<"]";
        return str;
    }
};
struct Point2{
    Point2( ){ }
    Point2(float X,float Y){x=X;y=Y;}
    float x,y;

    friend std::ostream & operator<<(std::ostream &str, const Point2 &p){
        str<<"["<<p.x<<" "<<p.y<<"]";
        return str;
    }
};
struct Matrix44{

    inline Matrix44 (){float m[16]={1,0,0,0, 0,1,0,0 ,0,0,1,0 ,0,0,0,1};std::memcpy(m44,m,16*sizeof(float));}
    inline Matrix44 (float m[16]){ std::memcpy(m44,m,16*sizeof(float));}
    inline const Matrix44&operator=(const Matrix44&M){std::memcpy(m44,M.m44,16*sizeof(float));return *this;}
    inline   void  rotateX(float rads){
        float c=cos(rads),s=sin(rads);
        float m[16]={1,0,0,0, 0,c,-s,0, 0,s,c,0, 0,0,0,1};
        (*this)= Matrix44(m)*(*this);
    }
    inline   void  rotateY(float rads){
        float c=cos(rads),s=sin(rads);
        float m[16]={c,0,s,0, 0,1,0,0 ,-s,0,c,0 ,0,0,0,1};
        (*this)= Matrix44(m)*(*this);
    }

    inline   void  rotateZ(float rads){
        float c=cos(rads),s=sin(rads);
        float m[16]={c,-s,0,0, s,c,0,0 ,0,0,1,0 ,0,0,0,1};
        (*this)= Matrix44(m)*(*this);
    }
    inline   void  translate(const Point3 &p){
        float m[16]={1,0,0,p.x, 0,1,0,p.y ,0,0,1,p.z ,0,0,0,1};
        (*this)= Matrix44(m)*(*this);
     }
    inline   Point3 operator*(const Point3 &p){    return Point3( p.x*m44[0]+ p.y*m44[1]+p.z*m44[2]+m44[3], p.x*m44[4]+ p.y*m44[5]+p.z*m44[6]+m44[7],p.x*m44[8]+ p.y*m44[9]+p.z*m44[10]+m44[11] );}
    inline  Matrix44 operator*(const Matrix44 &M){
        Matrix44 res;
        for(int i=0;i<3;i++)
            for(int j=0;j<4;j++){
                res(i,j)=0;
                for(int k=0;k<4;k++)
                    res(i,j)+= at(i,k)*M(k,j);
            }
        return res;
    }


    //access to elements
    inline float & operator()(int r,int c){return m44[r*4+c];}
    inline float   operator()(int r,int c)const{return m44[r*4+c];}
    inline float & at(int r,int c){return m44[r*4+c];}


    friend std::ostream & operator<<(std::ostream &str, const Matrix44 &m){
        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++)
                str<<m(i,j)<<" ";
            str<<std::endl;
        }
        return str;
    }

    float m44[16];
};



struct Color{
    Color(){}
    Color(unsigned char  r,unsigned char  g,unsigned char b){rgb[0]=r;rgb[1]=g;rgb[2]=b;}
    inline unsigned char &operator[](int i){return rgb[i];}
    unsigned char rgb[3];
};


class Scene{

    Matrix44 _viewMatrix;
    std::vector<Matrix44> modelMatrices;
    Matrix44 _cameraMatrix;
    Matrix44 TM;//transform from global coordinates to camera ones
    Color *colorbuffer=0;

    float _focal;
    int _width,_height;

public:
    Scene(){
        modelMatrices.resize(1);
        modelMatrices[0]= Matrix44();//set identity
        TM=_viewMatrix;
    }

    ~Scene(){if (colorbuffer) delete colorbuffer;}

    inline void setCameraParams(float focal,int width,int height){
        _focal=focal;
        _width=width;
        _height=height;
        _cameraMatrix(0,0)=_focal*float(_width);
        _cameraMatrix(1,1)=_focal*float(_height);
        _cameraMatrix(0,2)=float(_width)/2.;
        _cameraMatrix(1,2)=float(_height)/2.;
        if (colorbuffer!=0) delete colorbuffer;
        colorbuffer=new Color[_width*_height*3];
    }
    //this will erase the transform matrix
    inline void setViewMatrix(const Matrix44&vp){_viewMatrix=vp; TM=_viewMatrix*modelMatrices.back();}

    inline void setModelMatrix(const Matrix44 &M=Matrix44()){
        modelMatrices.resize(1);
        modelMatrices[0]=M;
        TM=_viewMatrix*modelMatrices.back();
    }
    inline void pushModelMatrix(const Matrix44 &M=Matrix44()){
        modelMatrices.push_back(modelMatrices.back()*M);
        TM=_viewMatrix*modelMatrices.back();
    }
    inline void popModelMatrix(){
        if (modelMatrices.size()>1){
            modelMatrices.pop_back();
            TM=_viewMatrix*modelMatrices.back();
        }
    }

    inline void clearTransformMatrix(){TM=_viewMatrix; }


    inline void clear(const Color &backgroundColor){
        const int size=_width*_height;
        for(int i=0;i<size;i++) colorbuffer[i]=backgroundColor;
      }
    //returns current view point
    inline Matrix44& getViewMatrix(){return _viewMatrix;}
    inline Matrix44& getModelMatrix(){return modelMatrices.back();}

    //draws a 3d point
    inline void drawPoint(const Point3 &p,const Color &c,int size=1){  drawPoint(&p,c,size);}

    inline void drawPoint(const Point3 *p,const Color &c,int size=1){
        Point2 pres;
        if ( project(*p,pres)){
            //draw the point (size 1 at this moment)
            colorbuffer[int(pres.y*_width)+int(pres.x)]=c;
        }
            (void)size;
    }

    inline void drawLine(const Point3 &p1,const Point3 &p2,const Color &color,int size=1){drawLine(&p1,&p2,color,size);}
    inline void drawLine(const Point3 *p1,const Point3 *p2,const Color &color,int size=1){
        Point3 p1t=TM*(*p1); if ( p1t.z<0 ) return;
        Point3 p2t=TM*(*p2); if(p2t.z<0) return;//check that both are in front of camera(otherwise, do not draw)
        Point2 pr1,pr2;
        if(! project(p1t,pr1))return;
        if(! project(p2t,pr2))return;
        drawline(pr1,pr2,color,size);//project line bweten projected points
    }

    //returns the internal frame buffer
    inline unsigned char* getBuffer()const{return (unsigned char*)colorbuffer;}

    inline int getWidth()const{return _width;}
    inline int getHeight()const{return _height;}
private:


    inline bool project(const Point3 &p,Point2 &res){
        Point3 pres=_cameraMatrix*(p);
        if(fabs(pres.z)<1e-9) return false;
        res.x=pres.x/pres.z;
        res.y=pres.y/pres.z;
        return true;
    }

    //Bresenham's algorithm

    void drawline(Point2 start, Point2 end,  const Color& color ,int size=1)
    {
        (void)size;
        int x0=start.x,y0=start.y,x1=end.x,y1=end.y;
        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        while (true)
        {
            if(y0>=0 && x0>=0 && y0<_height && x0<_width)   colorbuffer[y0*_width+x0]=color;
            if (x0 == x1 && y0 == y1) return;
            int e2 = (err << 1);
            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }
};

}




///---------------------------------------------------------------------------------------------



#include <opencv2/highgui.hpp>
#include <string>
#include <thread>
//Class using an opencv window to render and manipulate a sgl scene
class sgl_OpenCV_Viewer{
    sgl::Scene _Scene;
    std::string _wname;
    std::thread _thread;
public:


    void setParams(float f,int width,int height,std::string wname){
        _Scene.setCameraParams(f,width,height);
        _wname=wname;
    }

    void show(){
        _thread=std::thread(&sgl_OpenCV_Viewer::start_thread,this);
    }

    void join(){
        _thread.join();
    }

protected:
    //must reimplement this
    virtual void drawScene(sgl::Scene &scn) {}


private:

    void start_thread(){
        cv::namedWindow(_wname,1);
        cv::setMouseCallback(_wname, &sgl_OpenCV_Viewer::mouseCallBackFunc , this);
        _internal_drawScene();

    }

    void _internal_drawScene(){
        drawScene(_Scene);
        cv::imshow(_wname,cv::Mat(_Scene.getHeight(),_Scene.getWidth(),CV_8UC3,_Scene.getBuffer()));
        while( ((int)cv::waitKey(0))!=27 ) ;
    }

    struct mouseInfo{
        sgl::Point2 pos;
         bool isTranslating=false,isZooming=false,isRotating=false;
    }mi;


    static   void mouseCallBackFunc(int event, int x, int y, int flags, void* userdata){
        sgl_OpenCV_Viewer *Sv=(sgl_OpenCV_Viewer*)userdata;
        bool redraw=false;
        if  ( event == cv::EVENT_LBUTTONDOWN ){
            Sv->mi.isRotating=Sv->mi.isTranslating=Sv->mi.isZooming=false;
            if ( flags&cv::EVENT_FLAG_CTRLKEY)
                Sv->mi.isZooming=true;
            else if ( flags&cv::EVENT_FLAG_SHIFTKEY) Sv->mi.isTranslating=true;
            else Sv->mi.isRotating=true;
        }
        else if  ( event == cv::EVENT_MBUTTONDOWN ) Sv->mi.isTranslating=true;
        else if ( event == cv::EVENT_LBUTTONUP ) {              Sv->mi.isRotating=Sv->mi.isTranslating=Sv->mi.isZooming=false;
        }
        else if ( event == cv::EVENT_MBUTTONUP ) Sv->mi.isTranslating=false;
        else if ( event == cv::EVENT_MOUSEMOVE )
        {
            sgl::Point2  dif(Sv->    mi.pos.x-x,Sv->   mi.pos.y-y);
            sgl::Matrix44 tm;//=Sv->_Scene.getTransformMatrix();

            if (Sv->mi.isRotating){
                tm.rotateX(-float(dif.y)/100);
                tm.rotateY(float(dif.x)/100);
            }
            else if (Sv->mi.isZooming){
                auto vp=Sv->_Scene.getViewMatrix();
                vp.translate({0,0, -dif.y*0.01});
                Sv->_Scene.setViewMatrix(vp);
                redraw=true;
            }
            else if (Sv->mi.isTranslating){
                auto vp=Sv->_Scene.getViewMatrix();
                vp.translate(sgl::Point3(float(-dif.x)/100, float(-dif.y)/100,0.f));
                Sv->_Scene.setViewMatrix(vp);
                redraw=true;
            }
            if (Sv->mi.isRotating||Sv->mi.isZooming ||Sv->mi.isTranslating)  {
                sgl::Matrix44 res= tm*Sv->_Scene.getModelMatrix() ;
                Sv->_Scene.setModelMatrix(res);
                redraw=true;
            }
        }
        Sv->mi.pos=sgl::Point2(x,y);
        if (redraw)             Sv->_internal_drawScene();
    }
};


//---------------------------------------------------------
//A simple instance
#include <iostream>
using namespace std;
class SimpleSceneViewer:public sgl_OpenCV_Viewer{

    public:
    void drawScene(sgl::Scene &Scn) {
        Scn.clear(sgl::Color(125,125,125));
        drawAxis(Scn,1);
        sgl::Matrix44 Tm;

        Tm.translate({0,0,1});
        Scn.pushModelMatrix(Tm);
        drawRectangle(Scn,0.25,sgl::Color(125,0,125),0);
       Scn.popModelMatrix();
         Tm.translate({0,0,1});
        Scn.pushModelMatrix(Tm);
        drawRectangle(Scn,0.25,sgl::Color(125,255,125),0);
        Scn.popModelMatrix();

    }

private:
    void drawAxis(sgl::Scene &Scn,float size){

        Scn.drawLine({0,0,0},{size,0,0},{255,0,0});
        Scn.drawLine({0,0,0},{0,size,0},{0,255,0});
        Scn.drawLine({0,0,0},{0,0,size},{0,0,255});
    }

    void drawRectangle(sgl::Scene &Scn,float size,sgl::Color color,float t=0,int width=1){

        float s2=size/2.;
         sgl::Point3 c[4]={ {-s2,s2,t}, {s2,s2,t},{s2,-s2,t},{-s2,-s2,t}};
        Scn.drawLine(c[0],c[1],color,width);
        Scn.drawLine(c[1],c[2],color,width);
        Scn.drawLine(c[2],c[3],color,width);
        Scn.drawLine(c[3],c[0],color,width);
    }
};



int main(int argc,char **argv){
    try{

        SimpleSceneViewer Scv;
        Scv.setParams(0.3,800,600,"simple");
        Scv.show();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        Scv.join();



    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }

}
