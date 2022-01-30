/*
 * @Description: 
 * @Autor: YURI
 * @Date: 2022-01-30 00:29:35
 * @LastEditors: YURI
 * @LastEditTime: 2022-01-30 03:50:52
 */
/**
 * 1920x1080 CSI 参数
 * 1710  0      0       1710 0    1101
 * 0    1731    0       0    1731 560
 * 1101 560     1 --》  0    0    1
 * 
 * 畸变参数：-0.5895 0.9057 0 -0.0045  -0.0116 
 * 
 * 
 * 40,20,60     742,385
 * 40,20,40     747,533
 * 20,20,20     900,682
 * -20,20,20    1200,673
 * 0,20,40      1055,520
 * 40,38,60     867,476
 * 40,38,40     867,554
 * 20,38,20     951,635
 * -20,38,20    1117,633
 * 0,38,40      1031,552
 * 100,38,140   643,192
 * -80,38,140   1320,161
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
    cameraMatrix.at<double>(0, 0) = 1710;
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 1101;
    cameraMatrix.at<double>(1, 1) = 1731;
    cameraMatrix.at<double>(1, 2) = 560;
    cameraMatrix.at<double>(2, 2) = 1;
    //相机畸变系数
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = 0;
    distCoeffs.at<double>(1, 0) = 0;
    distCoeffs.at<double>(2, 0) = 0;
    distCoeffs.at<double>(3, 0) = 0;
    distCoeffs.at<double>(4, 0) = 0 ;

    //将控制点在世界坐标系的坐标压入容器
    vector<Point3f> objP;
    objP.clear();
    objP.push_back(Point3f(40,20,60));
    objP.push_back(Point3f(40,20,40));
    objP.push_back(Point3f(20,20,20));
    objP.push_back(Point3f(-20,20,20));
    objP.push_back(Point3f(0,20,40));

    objP.push_back(Point3f(40,38,60));
    objP.push_back(Point3f(40,38,40));
    objP.push_back(Point3f(20,38,20));
    objP.push_back(Point3f(-20,38,20));
    objP.push_back(Point3f(0,38,40 ));

    objP.push_back(Point3f(100,38,140));
    objP.push_back(Point3f(-80,38,140));


    //将之前已经检测到的角点的坐标压入容器
    std::vector<Point2f> points;
    points.clear();
    points.push_back(Point2f(742,385));
    points.push_back(Point2f(747,533));
    points.push_back(Point2f(900,682));
    points.push_back(Point2f(1200,673));
    points.push_back(Point2f(1055,520));
    points.push_back(Point2f( 867,476));
    points.push_back(Point2f(867,554));
    points.push_back(Point2f(951,635));
    points.push_back(Point2f(1117,633));
    points.push_back(Point2f(1031,552));
    points.push_back(Point2f(643,192));
    points.push_back(Point2f(1320,161));

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
    
    //计算相机旋转角
    // double theta_x, theta_y, theta_z;
    // double PI = 3.1415926;
    // theta_x = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2));
    // theta_y = atan2(-rotM.at<double>(2, 0),
    //     sqrt(rotM.at<double>(2, 1) * rotM.at<double>(2, 1) + rotM.at<double>(2, 2) * rotM.at<double>(2, 2)));
    // theta_z = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0));
    // theta_x = theta_x * (180 / PI);
    // theta_y = theta_y * (180 / PI);
    // theta_z = theta_z * (180 / PI);

    // //计算深度
    // Mat P;
    // P = (rotM.t()) * tvecs;

    // //输出
    // cout << "角度" << endl;
    // cout << theta_x << endl;
    // cout << theta_y << endl;
    // cout << theta_z << endl;
    // cout << P << endl;

    return 0;
}
