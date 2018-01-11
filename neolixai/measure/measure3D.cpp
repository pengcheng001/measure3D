#include"measure3D.h"
#include<map>
#include<vector>
#include<iostream>
#include<opencv2/imgproc/imgproc.hpp>
#include<fstream>
#include<string>
//#define DEBUDSAVEDATA

namespace neolix {

  double measureVol3D::getDistance(cv::Point2f point1, cv::Point2f point2)
  {
    double distances;
    distances = pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2);
    return sqrt(distances);
  }
  measureVol3D::measureVol3D():minVal(0),maxVal(0),minAndMaxrange(0),truncationVal(0),invalid(0),isSetPlaneDepth(false),steps(2),selectStep(0),length(0),width(0),height(0),usedTruck(false)
  {}
  measureVol3D::measureVol3D(float *data, size_t size):minVal(0),maxVal(0),minAndMaxrange(0),truncationVal(0),invalid(0),isSetPlaneDepth(false),steps(2),selectStep(0),length(0),width(0),height(0),usedTruck(false)
  {
    cv::Mat t = cv::Mat(size,3,CV_32FC1,data);
    this->objectPoints = t.clone();
#ifdef DEBUDSAVEDATA
    std::ofstream ofile;
    ofile.open("measure3DOrigindata.txt");
    ofile<<this->objectPoints<<std::endl;
    ofile.close();

#endif

  }

  bool measureVol3D::parseObject(cv::Mat rotateMatrix)
  {
    if(3 != rotateMatrix.rows || 3 != rotateMatrix.cols) return false;
   // std::cout<<"cv::determinant(rotateMatrix)"<<abs(1-cv::determinant(rotateMatrix))<<std::endl;
    if(abs(1-cv::determinant(rotateMatrix)) > 0.00001) return false;

    if(3 != this->objectPoints.cols) return false;
    if(this->objectPoints.rows <= 3) return false;

   // std::cout<<this->objectPoints<<std::endl;
    //std::cout<<rotateMatrix<<std::endl;


    this->objectPoints =(rotateMatrix*this->objectPoints.t()).t();

#ifdef DEBUDSAVEDATA
    std::ofstream ofile;
    ofile.open("measure3DparaseData.txt");
    ofile<<this->objectPoints<<std::endl;
    ofile.close();
#endif

   /// cv::minMaxLoc(this->objectPoints,&minVal);
    return true;
  }

  void measureVol3D::histAdjustRange()
  {
    std::map<float, int> hist;
    int sz = this->objectPoints.rows;
    const float* ptr = objectPoints.ptr<float>();
    size_t totalNum = 0;
    for(int idx = sz; idx != 0; idx--,ptr += 3)
      {
        if(invalid == ptr[2]) continue;
        totalNum++;
        if(hist.find(ptr[2]) != hist.end())
          {
            hist[ptr[2]]++;
          }else{
            hist.insert(std::make_pair(ptr[2],1));
          }
      }
    if(hist.empty()){
        minVal = 0;
        maxVal = 999999;
        return;
      }
    if(!this->isSetPlaneDepth) maxVal = hist.rbegin()->first;
    minVal = hist.begin()->first;
    size_t sum = 0;
    const size_t delta_min = static_cast<size_t>(totalNum*0.2);
    for(std::map<float, int>::iterator it = hist.begin(); it != hist.end(); it++)
      {
        if( minAndMaxrange > (maxVal - it->first)) break;
        sum += it->second;
        if(sum > delta_min)
          {
            truncationVal = it->first;
            break;
          }
      }
    if(maxVal - truncationVal < minAndMaxrange)
      {
        int mid = (maxVal + truncationVal)/2;
        maxVal = mid + minAndMaxrange/2;
        truncationVal = mid + minAndMaxrange/2;
        if(truncationVal < 0) truncationVal =0;
      }

  }

  bool measureVol3D::measure(cv::Mat rotateMatrix)
  {
    if(!this->parseObject(rotateMatrix)) return false;
    if( (maxVal - truncationVal) < minAndMaxrange )
      {
        std::cout<<"Minimum measurement height is"<<minAndMaxrange<<", but height detected by the program is "<<maxVal - truncationVal<<"!"<<std::endl;
        return false;
      }
    histAdjustRange();
    cv::Point2f point;
    std::vector<cv::Point2f> points;

    int sz = this->objectPoints.rows;
    float* ptr =objectPoints.ptr<float>();
    float temp;
    int numPoinrIN = 0;
#ifdef DEBUDSAVEDATA
    std::ofstream ofile;
    ofile.open("Selectpoint.txt");

#endif
    for(int id = 0; id < sz;id++)
    {
//      temp = (ptr[id*3+2] - minVal)*steps/(truncationVal - minVal);

      if(this->usedTruck)
      {
          if(ptr[id*3+2] < truncationVal)
          {


              point.x = ptr[id*3];
              point.y = ptr[id*3+1];
              points.push_back(point);
#ifdef DEBUDSAVEDATA
              ofile<<ptr[id*3]<<" "<<ptr[id*3 +1 ]<<" "<<ptr[id*3 + 2]<<std::endl;
#endif
              numPoinrIN++;

          }
      }else
      {
          temp = (ptr[id*3+2] - minVal)*steps/(maxVal - minVal);

       //   std::cout<<"depth: "<<ptr[id*3+2]<<"  step:"<<temp<<std::endl;
          if(temp < 0) temp = 0;
          if(temp > steps) temp = steps;
         // std::cout<<"depth: "<<ptr[id*3+2]<<"  step:"<<temp<<std::endl;
          if(temp <= selectStep)
          {
            point.x = ptr[id*3];
            point.y = ptr[id*3+1];
       //     std::cout<<"x: "<< point.x<<"  y:"<<point.y<<std::endl;
            points.push_back(point);
#ifdef DEBUDSAVEDATA
              ofile<<ptr[id*3]<<" "<<ptr[id*3 +1 ]<<" "<<ptr[id*3 + 2]<<std::endl;
#endif
            numPoinrIN++;
          }

      }

    }
#ifdef DEBUDSAVEDATA
    ofile.close();
#endif

    if(points.size() < 5)
      {
       std::cout<<"Can not find target object"<<std::endl;
        return false;
      }

    points.resize(numPoinrIN);
 //   this->findObject = cv::Mat(points);
   // std::cout<<"ttt: "<<numPoinrIN<<"  vector size:"<<points.size()<<" mat rows:"<<findObject.rows<<std::endl;
  //  std::cout<<cv::Mat(points)<<std::endl;
    cv::RotatedRect box = cv::minAreaRect(cv::Mat(points));
    cv::Point2f vertex[4];
    box.points(vertex);

   // std::cout<<"detected external rectangle vertices is: "<<vertex[0]<<","<<vertex[1]<<","<<vertex[2]<<std::endl;

    length = measureVol3D::getDistance(vertex[0], vertex[1]);
    width  = measureVol3D::getDistance(vertex[1], vertex[2]);
    height = maxVal - minVal;

    return true;
  }
 bool measureVol3D::measure(cv::Mat rotateMatrix, double &length, double &width, double &height)
 {
     if(false == measure(rotateMatrix)) return false;
     length = this->length;
     width = this->width;
     height = this->height;
     return true;
 }
  void measureVol3D::setPlaneDepth(double depth)
  {
    this->maxVal = depth;
    this->isSetPlaneDepth = true;
  }
  void measureVol3D::setMinAndMaxRange(double gap)
  {
    this->minAndMaxrange = gap;
  }
  void measureVol3D::setSteps(int steps)
  {
    this->steps = steps;
  }
  void measureVol3D::setSeleteStep(int selectStep)
  {
      this->selectStep = selectStep;
  }
  void measureVol3D::isUsedTruc(bool used)
  {
      this->usedTruck = used;
  }
}
