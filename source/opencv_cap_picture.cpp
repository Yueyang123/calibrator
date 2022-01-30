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
 