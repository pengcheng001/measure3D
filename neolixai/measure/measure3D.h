#ifndef __MEASURE3D_H__
#define __MEASURE3D_H__

#include<opencv2/core/core.hpp>
namespace neolix {
/*
目前的算法只能检测在规定区域内一个物体的体积，也就是在规定区域内只能有一个物体，如果有多个物体，那么将会将这些物体看做是一个物体计算其外接体积。
一个可以测量规定区域内有多个物体体积的想法：
	1.获得将待测物体旋转到与摄像头X-Y平面平行的旋转矩阵
	2.根据旋转矩阵，将深度图中在Z值替换成旋转后的z值，
	3.将变换后的深度图，转为伪彩色，根据伪彩色图寻找物体的外接轮廓
	4.这时候，如果在伪彩色中有多个物体，就会找到过个轮廓
	5.根据轮廓再找到相应的外接矩形，如果这时候有多个轮廓就会找到多个外接矩形
	6.根据各个外接矩形的区域得到在原始点云中相应的点云
	7.根据旋转矩阵，将相应的点云变换
	8.根据变换得到点云就就可以获得体积
	ps：每个外接矩形都可以得到相应的物体体积

*/
  class measureVol3D
  {
  public:
    measureVol3D();
    measureVol3D(float *data, size_t size);
    void setPlaneDepth(double depth);
    void setMinAndMaxRange(double gap);
    void setSteps(int steps);
    bool measure(cv::Mat rotateMatrix);
    bool measure(cv::Mat rotateMatrix,double &length, double &width, double &height);
    void setSeleteStep(int selectStep);
    void isUsedTruc(bool used);

  private:
    bool parseObject(cv::Mat rotateMatrix);
    void histAdjustRange();
    static double getDistance(cv::Point2f point1, cv::Point2f point2);

    cv::Mat objectPoints;
    double minVal;
    double maxVal;//the depth of plane
    double minAndMaxrange;
    double truncationVal;
    short  invalid;
    bool isSetPlaneDepth;
    //short min_x,max_x,min_y,max_y;///the variable is used for futrue;Which cam map pointcloud to plane
    int  steps;
    int selectStep;
    double length, width, height;
    bool usedTruck;
  };

}

#endif
