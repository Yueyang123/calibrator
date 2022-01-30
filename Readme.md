# 基于opencv的图像校准

具体程序参考
[源码]()

## 计算相机内参

### 采集图像
我这里因为不是uvc摄像头，是不支持UVC协议的摄像头，采用了自己写的摄像头接口进行的
采集，正常直接用opencv自带的VideoCapture直接采集就可以了
```cpp
/*
 * @Description: 
 * @Autor: YURI
 * @Date: 2022-01-29 20:44:37
 * @LastEditors: YURI
 * @LastEditTime: 2022-01-30 03:02:58
 */
#include <framebuffer.h>
#include <iostream>
#include <signal.h>
#include "ffmpeg_sws.h"
#include "opencv_mat.h"
#include "framebuffer.h"
#include <string.h>
#include "camera.h"
#include "camera_uvc/camera_uvc.h"
#include "camera_gc2053/camera_gc2053.h"
#define VIDEO_INDEX 0 
#define FB_PATH     "/dev/fb0"
#define CAM_PLAT
typedef struct 
{
    int width;
    int height;
    unsigned char* rawframe;    //原始数据
    unsigned char* showframe;   //需要显示的数据
}vip_mutilchannel_app;


using namespace std;
using namespace cv;
framebuffer *fb;
Mat frame;
char path[10];
char text[20];
int takeindex=0;

camera*               plat_capture; 
opencv_mat*           mat_convert;    //opencv图像转换
ffmpeg_sws_interface* ff_sws;         //ffmpeg转换接口
vip_mutilchannel_app cap_plat;
framebuffer* p; //绘制图像


void take_index_picture(int sig)
{
    takeindex++;
    printf("save the %d picture \r\n",takeindex);
    sprintf(text,"%s/%d.bmp",path,takeindex);
    imwrite(text,frame);
    if(takeindex>=30)printf("FINISH END \r\n");
}
int main(int argc ,void ** argv)
{
    signal(SIGINT,take_index_picture);
    sprintf(path,"%s",argv[1]);
 #ifdef CAM_PLAT   
    cap_plat.width=1920;
    cap_plat.height=1080;
    mat_convert=new opencv_mat(cap_plat.width,cap_plat.height);
    ff_sws=new ffmpeg_sws_interface(cap_plat.width,cap_plat.height,AV_PIX_FMT_NV21,AV_PIX_FMT_BGR24); 
    plat_capture=new camera_gc2053(cap_plat.width,cap_plat.height,V4L2_PIX_FMT_NV21,VIDEO_INDEX);
    plat_capture->camera_open();
 #endif
 #ifdef CAM_UVC  
    cap_plat.width=640;
    cap_plat.height=480;
    mat_convert=new opencv_mat(cap_plat.width,cap_plat.height);
    ff_sws=new ffmpeg_sws_interface(cap_plat.width,cap_plat.height,AV_PIX_FMT_YUYV422,AV_PIX_FMT_RGB24);
    plat_capture=new camera_uvc(cap_plat.width,cap_plat.height,V4L2_PIX_FMT_YUYV,VIDEO_INDEX);
    plat_capture->camera_open();
 #endif

    cap_plat.showframe=(unsigned char*)malloc(cap_plat.width*cap_plat.height*3+1000);
    p=new framebuffer(FB_PATH);
    p->print_info();
    while (1)
    {
        plat_capture->camera_get_fps(0);
        cap_plat.rawframe=plat_capture->read_frame();
        ff_sws->ffmpeg_sws_convert(cap_plat.rawframe,cap_plat.showframe);
        p->show_rgbbuffer(cap_plat.showframe,0,0,cap_plat.width,cap_plat.height,1);
        frame=mat_convert->opencv_convert(cap_plat.showframe);
        printf("FPS: %lf \r\n" ,plat_capture->camera_get_fps(1) ) ;        
    }
    return 0;
}
 

```

opencv_cap_picture 放在板端运行
采集出30张图像考下来备用

### Matlab计算内参和畸变

直接而在MATLAB的Command Window里面输入cameraCalibrator即可调用标定应用。 

计算出的结果中
里面的RadialDistortion对应k1，k2，k3设置为0了。 TangentialDistortion对应p1，p2。 
IntrinsicMatrix对应内参，注意这个和OpenCV中是转置的关系，注意不要搞错。

## 手动找到像素点何其对应的世界坐标

用下面的程序找到两两对应的坐标

```cpp

/*
 * @Description: 这个程序主要是为了找匹配点的
 * 将世界坐标和对应像素点记录下来
 * @Autor: YURI
 * @Date: 2022-01-30 01:28:32
 * @LastEditors: YURI
 * @LastEditTime: 2022-01-30 02:52:21
 */


    #include <opencv2/core/core.hpp>  
    #include <opencv2/highgui/highgui.hpp>  
    #include <stdio.h>  
      
    using namespace cv;  
      
    cv::Mat img;  
    void on_mouse(int event,int x,int y,int flags,void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号  
    {  
        printf("[%d,%d]\r\n",x,y); 
    }  
    int main(int argc,void** argv)  
    {  
        img = imread((char*)argv[1]);  
        namedWindow("img");//定义一个img窗口  
        setMouseCallback("img",on_mouse,0);//调用回调函数  
        imshow("img",img);  
        cv::waitKey(0);  
        return 1;
    }  


```

## 通过前面得到的内参和对应坐标计算外参

```cpp

/**
 * 640x680 UVC 参数
 * 内参矩阵
 *  927  0   0         927  0   378 
 *  0   922  0          0   922 181
 *  378 181  1  --》     0   0   1
 * 
 * 畸变参数：0.0814 1.059  0  -0.0249  0.035 
 * 
 * 参数点
 * （20,20,-20） （161, 183）
 * （-20,20,-20）（352 ,170）
 * （0,40,-20）   （250, 80）
 * （20,40,-20）  （155 ,80）
 * （-20,40,-20） （350 ,77）
 * 
 * 
 *  内参 ：
    [927, 0, 378;
    0, 922, 181;
    0, 0, 1]
    外参R ：
    [-0.983137867886935, -0.0269591431329599, -0.1808677343482473;
    0.05541797587717625, -0.986485208380106, -0.1541939739319782;
    -0.1742664071942963, -0.1616172585115694, 0.9713449856126276]
    外参T ：
    [-28.32676932096033;
    15.3704959647409;
    218.1288004110183]
    畸变 ：
    [0.0814;
    0;
    0;
    0;
    0]
 * 
*/

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
using namespace std;
using namespace cv;
int main() {
    //相机内参矩阵
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 927;
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 378;
    cameraMatrix.at<double>(1, 1) = 922;
    cameraMatrix.at<double>(1, 2) = 181;
    cameraMatrix.at<double>(2, 2) = 1;
    //相机畸变系数
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = 0.0814;
    distCoeffs.at<double>(1, 0) = 0;
    distCoeffs.at<double>(2, 0) = 0;
    distCoeffs.at<double>(3, 0) = 0;
    distCoeffs.at<double>(4, 0) = 0;

    //将控制点在世界坐标系的坐标压入容器
    vector<Point3f> objP;
    objP.clear();
    objP.push_back(Point3f(20,20,-20));
    objP.push_back(Point3f(-20,20,-20));
    objP.push_back(Point3f(0,40,-20));
    objP.push_back(Point3f(20,40,-20));
    objP.push_back(Point3f(-20,40,-20));

    //将之前已经检测到的角点的坐标压入容器
    std::vector<Point2f> points;
    points.clear();
    points.push_back(Point2f(161, 183));
    points.push_back(Point2f(352 ,170));
    points.push_back(Point2f(250, 80));
    points.push_back(Point2f(155 ,80));
    points.push_back(Point2f(350 ,77));

    //创建旋转矩阵和平移矩阵
    Mat rvecs = Mat::zeros(3, 1, CV_64FC1);
    Mat tvecs = Mat::zeros(3, 1, CV_64FC1);

    //求解pnp
    //solvePnP(objP, points, cameraMatrix, distCoeffs, rvecs, tvecs);
    solvePnPRansac(objP, points, cameraMatrix, distCoeffs, rvecs, tvecs);
    Mat rotM = Mat::eye(3, 3, CV_64F);
    Mat rotT = Mat::eye(3, 3, CV_64F);
    Rodrigues(rvecs, rotM);  //将旋转向量变换成旋转矩阵
    Rodrigues(tvecs, rotT);
    Mat inner=Mat::zeros(3, 4, CV_64F);
    Mat outter=Mat::zeros(4, 4, CV_64F);
    inner.at<double>(0,0)=cameraMatrix.at<double>(0,0);
    inner.at<double>(0,1)=cameraMatrix.at<double>(0,1);
    inner.at<double>(0,2)=cameraMatrix.at<double>(0,2);
    inner.at<double>(1,0)=cameraMatrix.at<double>(1,0);
    inner.at<double>(1,1)=cameraMatrix.at<double>(1,1);
    inner.at<double>(1,2)=cameraMatrix.at<double>(1,2);
    inner.at<double>(2,0)=cameraMatrix.at<double>(2,0);
    inner.at<double>(2,1)=cameraMatrix.at<double>(2,1);
    inner.at<double>(2,2)=cameraMatrix.at<double>(2,2);

    outter.at<double>(0,0)=rotM.at<double>(0,0);
    outter.at<double>(0,1)=rotM.at<double>(0,1);
    outter.at<double>(0,2)=rotM.at<double>(0,2);
    outter.at<double>(1,0)=rotM.at<double>(1,0);
    outter.at<double>(1,1)=rotM.at<double>(1,1);
    outter.at<double>(1,2)=rotM.at<double>(1,2);
    outter.at<double>(2,0)=rotM.at<double>(2,0);
    outter.at<double>(2,1)=rotM.at<double>(2,1);
    outter.at<double>(2,2)=rotM.at<double>(2,2);
    outter.at<double>(3,3)=1;
    outter.at<double>(0,3)=tvecs.at<double>(0,0);
    outter.at<double>(1,3)=tvecs.at<double>(1,0);
    outter.at<double>(2,3)=tvecs.at<double>(2,0);   
    Mat res=inner*outter;
    cout<<"内参 ："<<endl;
    cout<<inner<<endl;
    cout<<"外参 ："<<endl;
    cout<<outter<<endl;
    cout<<"畸变 ："<<endl;
    cout<<distCoeffs<<endl;
    cout<<"运算结果 ："<<endl;
    cout<<res<<endl;

    int x,y,z;
    Mat world_point=Mat::zeros(4, 1, CV_64F);
    cout<<"输入世界坐标:"<<endl;
    while (cin>>x>>y>>z)
    {
        cout<<x<<" "<<y<<" "<<z<<" "<<endl;
        world_point.at<double>(0,0)=x;
        world_point.at<double>(1,0)=y;
        world_point.at<double>(2,0)=z;
        world_point.at<double>(3,0)=1;
        Mat pix=res*world_point;
        float res_x,res_y;
        res_x=pix.at<double>(0,0)/pix.at<double>(2,0);
        res_y=pix.at<double>(1,0)/pix.at<double>(2,0);
        
        cout<<"像素坐标:"<< pix <<endl<<res_x<<" "<<res_y<<endl;
    }
    return 0;
}

```

最后可以通过输入世界坐标，程序输出像素坐标，看一看标定是否成功，建议不要将畸变参数完全输入
输入前一个或前两个就可以了，不然似乎会出问题。