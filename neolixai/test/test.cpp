#include"test.h"
#include <fstream>
#include<vector>
#include<string>
#include <fstream>
#include"../../include/libTof.h"
#include"../neolixMV.h"
#include"../pointcloud/pointcloud.hpp"
#include"../measure/measure3D.h"
#include<iostream>


namespace neolix {
  std::vector<std::string> split(const std::string &s, const std::string &seperator){
    std::vector<std::string> result;
    typedef std::string::size_type string_size;
    string_size i = 0;

    while(i != s.size()){
      //找到字符串中首个不等于分隔符的字母；
      int flag = 0;
      while(i != s.size() && flag == 0){
        flag = 1;
        for(string_size x = 0; x < seperator.size(); ++x)
        if(s[i] == seperator[x]){
          ++i;
          flag = 0;
          break;
        }
      }

      //找到又一个分隔符，将两个分隔符之间的字符串取出；
      flag = 0;
      string_size j = i;
      while(j != s.size() && flag == 0){
        for(string_size x = 0; x < seperator.size(); ++x)
        if(s[j] == seperator[x]){
          flag = 1;
          break;
        }
        if(flag == 0)
        ++j;
      }
      if(i != j){
        result.push_back(s.substr(i, j-i));
        i = j;
      }
    }
    return result;
  }



  void test_rotatePlane()
  {
    ///read data from data
    std::ifstream ifile;
    std::vector<std::string> result;
    ifile.open("/home/neolix/mydata/point0.txt");
    std::string str;
    int i =0;
    point3Df *p = new point3Df[39000];
    while(std::getline(ifile,str))
      {
        result = split(str," ");
        p[i].x = std::atof(result[0].c_str());
        p[i].y = std::atof(result[1].c_str());
        p[i].z = std::atof(result[2].c_str());
      //  std::cout<<p[i].x<<" , " <<p[i].y<<" , " <<p[i].z <<std::endl;
        i++;
      }
    ifile.close();
    while(1)
    {
        rotatePlane rp;
        rp.setFitnessMethod(1);
        rp.setData(p,i);
        rp.CalculatedRotationMatrix();
        rp.printPara();


    }
  }

  void test_measure3D()
  {
      ///read data from file
      ///
      std::cout<<"test test_measure3D"<<std::endl;
      std::ifstream ifile;
      std::vector<std::string> result;
      ifile.open("/home/neolix/neolixProject/measureVol2.0/measureVol/neolixmeasure/point0.txt");
      std::string str;
      std::getline(ifile, str);
      result = split(str," ");
      int h = std::atoi(result[0].c_str());
      std::getline(ifile, str);
      result = split(str," ");
      int w = std::atoi(result[0].c_str());
      int maxrect[4];
      int minrect[4];
      //std::cout<<h<<"  "<<w<<std::endl;
      std::getline(ifile, str);
      result = split(str," ");
      for(int i =0; i < 4; i++)
      {
            maxrect[i] = std::atoi(result[i].c_str());
           // std::cout<<maxrect[i]<<std::endl;

       }
      std::getline(ifile, str);
      result = split(str," ");
      for(int i =0; i < 4; i++){
          minrect[i] = std::atoi(result[i].c_str());
         // std::cout<<maxrect[i]<<std::endl;
      }
      void *data = malloc(sizeof(point3Df) *h*w);
      int i =0;
      while(std::getline(ifile, str))
      {
          result = split(str, " ");
          ((point3Df*)data)[i].x = std::atof(result[0].c_str())/1000.0f;
          ((point3Df*)data)[i].y = std::atof(result[1].c_str())/1000.0f;
          ((point3Df*)data)[i].z = std::atof(result[2].c_str())/1000.0f;
       //   std::cout<<((point3Df*)data)[i].x<<" , "<<((point3Df*)data)[i].y<<" , "<<((point3Df*)data)[i].z<<std::endl;
          i++;
      }
       cv::Rect maxRect;
       cv::Rect minRect;
       maxRect.x = maxrect[0];
       maxRect.y = maxrect[1];
       maxRect.width = maxrect[2];
       maxRect.height = maxrect[3];

       minRect.x = minrect[0];
       minRect.y = minrect[1];
       minRect.width = minrect[2];
       minRect.height = minrect[3];
//       std::cout<<maxRect<<std::endl;
//       std::cout<<minRect<<std::endl;

       while(1)
       {
           std::cout<<"test test_measure3D "<<std::endl;

           pointcloudData pcd;
           pcd.data = data;
           pcd.height = h;
           pcd.width = w;

           rotatePlane rp;
           rp.setFitnessMethod(1);
           rp.setData(pcd,maxRect, minRect);
           rp.CalculatedRotationMatrix();
           double rotatePanleDepth ;
           if(!rp.getPlaneDepth(rotatePanleDepth))
           {
               std::cout<<"can not get plane depth"<<std::endl;
               continue;
           }
           double *rotationMatrix = const_cast<double*>(rp.getRotationMatrix());

           cv::Mat rotationMat_ (3,3,CV_64FC1,rotationMatrix);
           cv::Mat rotationMat;
           rotationMat_.convertTo(rotationMat,CV_32FC1);
          // rp.printPara();

           size_t size;
           float *objectData = const_cast<float*>(rp.getInsidePointCloud(size));
           measureVol3D mv3d(objectData,size);
           mv3d.setMinAndMaxRange(30.0);
           mv3d.setPlaneDepth(rotatePanleDepth);
           mv3d.setSteps(6);
           mv3d.setSeleteStep(2);
           mv3d.measure(rotationMat);
       }

/*       cv::Mat rotateObject = mv3d.getObject();
       cv::Mat findObject = mv3d.getfindObjects();
   //    std::cout<<findObject<<std::endl;
       std::ofstream rojct;
       rojct.open("rotateObject.txt");
       for(int row = 0; row < rotateObject.rows; row++ )
       {
           float *ptr = rotateObject.ptr<float>(row);
           for(int col = 0; col < rotateObject.cols; col++)
           {
               rojct<<ptr[col]<<" ";
           }
           rojct<<std::endl;
       }


       std::ofstream frojct;
       frojct.open("frotateObject.txt");
       for(int row = 0; row < findObject.rows; row++ )
       {
           float *ptr = findObject.ptr<float>(row);
           for(int col = 0; col < findObject.cols; col++)
           {
               frojct<<ptr[col]<<" ";
           }
           frojct<<std::endl;
       }
       frojct.close();
       rojct.close()*/;





//       pointcloud<double>* plane = rp.getplanePoints();
//       std::ofstream ofilePlane;
//       ofilePlane.open("planePoint.txt");
//       std::ofstream ofileObject;
//       ofileObject.open("objectPoint.txt");

//       for(int i = 0; i < plane->getSize(); i++)
//       {
//           ofilePlane<<plane->data[i][0]<<" "<<plane->data[i][1]<<" "<<plane->data[i][2]<<std::endl;
//       }
//       size_t size;
//       const float *object = rp.getInsidePointCloud(size);
//       for(size_t i = 0; i < size; i++)
//       {
//           ofileObject<<object[i*3]<<" "<<object[i*3 + 1]<<" "<<object[i*3 + 2]<<std::endl;
//       }

//       ofileObject.close();
//       ofilePlane.close();


       ifile.close();
       free(data);

  }

}
