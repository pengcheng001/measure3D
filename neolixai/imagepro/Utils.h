#ifndef  _NEOLIX_UTILS__H__
#define  _NEOLIX_UTILS__H__

#include<opencv2/core/core.hpp>
#include<fstream>
#include<iostream>
#include<vector>

#include "../driverdecorate/base.h"
#include"../pointcloud/pointcloud.hpp"

using namespace std;
using namespace  cv;

namespace neolix{

template <typename T1, typename T2>
struct index_value
{
    T1 index;
    T2 value;
    index_value(T1 idx, T2 val)
    {
        index = idx;
        value = val;
    }
    index_value(){}

};

void LaplasSharp(const cv::Mat &src, cv::Mat &dest);
void GammaConver(const cv::Mat &src, cv::Mat &dest, double gamma);
void ExtractObject(const cv::Mat &depthImage, cv::Mat &BinaryImage);
void readDepthData(cv::Mat &dest, const size_t rows, const size_t cols,const string RawPath);
void onMouse(int event, int x ,int y, int flags, void* img);
NEOLIX_STATUS_LIST  PopRang(cv::Mat &srcImg,cv::vector<cv::Point>& contour,float &PixLength,float &PixWidth,cv::vector<cv::Point2f> &point);
NEOLIX_STATUS_LIST calBoundingRectangle(cv::Mat srcImage, cv::Vector<cv::Point> &contuor, float &PixXLenght, float &PixYLength,cv::Point2f point[]);
void seg(cv::Mat &img,cv::Mat &img1,cv::Mat &img2,const int &rows,const int &cols);
void Getxy(const float PixLength,const float PixWidth,short distance,float &Length,float &Width);
void Getxyz2(cv::vector<cv::Point2f> point,short distance,float &Length,float &Width);
unsigned short calculateDepthFromDepthImagInRangeCountour(cv::Mat &depthIamge, cv::vector<cv::Point> &contour,double &confidence,unsigned short &minDepth);
unsigned short calculateDepthFromDepthImagOutRangeCountour(cv::Mat &depthIamge, cv::vector<cv::Point> &contour,double &confidence);
void getBigerRect(cv::Rect rect, cv::Point point);
//校准相机到测量平台的距离
bool adjustSystem( cv::Mat depthIamgeRoi,  cv::Mat mask, unsigned short &distance);
#ifdef TY_SDK
//录制RGB视频,采集训练数据
void recordVideo(const string videopath);
#endif
void padDepthMask(const cv::Mat colorDepthImage, cv::Mat &mask);

void distancesFromCamToPad(std::vector<short> &distances,std::vector<index_value<int, cv::Point2i> > &centerPoints, cv::Mat PadDepthImage);

float calDisCam2Pad(std::vector<float> &distances, std::vector<index_value<int,cv::Point2i> > &centerPoints, cv::Point2f point);


void calCoutousCenter(cv::vector<cv::Point> &contour, cv::Point2f &center);
///获得最小二乘的方差矩阵
int  leastSquareEquation(const cv::Mat depthImage, cv::Mat &dest);
//从拟合平面获得深度
void Getavepaddis(cv::vector<cv::Point2f> point ,const double a[],double &paddis);

//查表变换函数 dst(src(i)) = lut(src(i))

void LUT( cv::Mat depth1, ushort table[], cv::Mat &depth2);
///this funtion use for pointclouds fitness plane
int leastSquareEquationForPointCloud(pointcloud<double> *point,cv::Mat &dest);
////this funtion use for depthData finness palne which will use the biase of depth data(u ,v)
//int leastSquareEquationFordepthCloud(pointcloud<double> *depthPoints, cv::Mat &dest);
///this function wil select those depth points that inside maxRect and outside minRect in depth image
int selectedDepthPointFromDepthImage(pointcloud<double> &depthPoints, cv::Mat &depthImage, cv::Rect maxRect, cv::Rect minRect);

}
#endif
