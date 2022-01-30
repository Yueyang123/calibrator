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

