#include<opencv2/core/core.hpp>
#include<vector>
#include"../imagepro/Utils.h"
#include"../imagepro/CalDepth.h"
#include<string>
#include"config.h"
#include"../measure/measure3D.h"
#include"../neolixMV.h"
#include"../driverdecorate/base.h"
#include"../measure/measure.h"
#include"../pointcloud/pointcloud.hpp"
#include"../pointcloud/ransac.h"
#include"../common/DepthRender.hpp"
#include"../buildmodel/backgroundReconstruction.h"
#include"../pointcloud/rotatePlane.h"
namespace neolix{

     cv::Mat g_rect_para; //大区域
     cv::Mat g_plane_para_least;  //最小二乘拟合获得的平面参数
     cv::Mat g_plane_ransca;//随机一致性抽样算法获得的平面参数
     cv::Mat g_cammer_intrinic;//内参
     cv::Mat g_plane_rect;//小区域
     short g_fix_length = 0;
     short g_fix_width  = 0;
     short g_fix_height = 0;


 void dajustVol(short fix_lenght, short fix_width, short fix_height)
 {
     g_fix_length = fix_lenght;
     g_fix_height = fix_height;
     g_fix_width  = fix_width;
 }
bool measureVol3(pointcloudData pointCloud, vol &v, int method)
{
    cv::Rect minRect(g_plane_rect.at<int>(0,0),g_plane_rect.at<int>(1,0),g_plane_rect.at<int>(2,0),g_plane_rect.at<int>(3,0));
    cv::Rect maxRect(g_rect_para.at<int>(0,0),g_rect_para.at<int>(1,0),g_rect_para.at<int>(2,0),g_rect_para.at<int>(3,0));
    rotatePlane rp;
    double rotatePanleDepth;//旋转后平面到摄像头的距离
    rp.setFitnessMethod(method);
    rp.setData(pointCloud,maxRect,minRect);
    rp.CalculatedRotationMatrix();
    if(!rp.getPlaneDepth(rotatePanleDepth))
    {
        std::cout<<"can not get plane depth"<<std::endl;
        return false;
    }
    double *rotationMatrix = const_cast<double*>(rp.getRotationMatrix());

    cv::Mat rotationMat_ (3,3,CV_64FC1,rotationMatrix);
    cv::Mat rotationMat;
    rotationMat_.convertTo(rotationMat,CV_32FC1);
    size_t size;
    float *objectData = const_cast<float*>(rp.getInsidePointCloud(size));
    measureVol3D mv3d(objectData,size);
    mv3d.setMinAndMaxRange(30.0);
    mv3d.setPlaneDepth(rotatePanleDepth);
    mv3d.setSteps(6);
    mv3d.setSeleteStep(2);
    mv3d.measure(rotationMat, v.length, v.width, v.height);
    v.length += g_fix_length ;
    v.width += g_fix_width;
    v.height += g_fix_height;
    return true;

}

 bool measureVol2(depthData depth_, vol &v, int method)
 {
   void* data = malloc(depth_.width*depth_.height*sizeof(short));
   memcpy(data,(void*)depth_.data,depth_.width*depth_.height*sizeof(short));
   cv::Mat d = cv::Mat(depth_.height,depth_.width,CV_16UC1,data);
   if(NEOLIX_FALSE == reconstructionPlane(d,method)) return false;

   double plane_para[3] = {0};
   float length, width,height;
    if( 1 == method)
   {
       plane_para[0] = g_plane_ransca.at<double>(0,0);
       plane_para[1] = g_plane_ransca.at<double>(1,0);
       plane_para[2] = g_plane_ransca.at<double>(2,0);
   }else
   {
       plane_para[0] = g_plane_para_least.at<double>(0,0);
       plane_para[1] = g_plane_para_least.at<double>(1,0);
       plane_para[2] = g_plane_para_least.at<double>(2,0);
   }
   height = -1;

   if( NEOLIX_FALSE ==  measureBox(d,plane_para,length,width,height) || height == -1)
   {
       return false;
   }else{
 //  std::cout<<"length: "<<length<<"width: "<<width<<"height: "<<height<<std::endl;
   v.length = length + g_fix_length ;
   v.width = width + g_fix_width;
   v.height = height + g_fix_height;
   return true;
   }
 }

bool measureVol(depthData depth_,vol &v, int method)
{
    void* data = malloc(depth_.width*depth_.height*sizeof(short));
    memcpy(data,(void*)depth_.data,depth_.width*depth_.height*sizeof(short));
    cv::Mat d = cv::Mat(depth_.height,depth_.width,CV_16UC1,data);
    double plane_para[3] = {0};
    float length, width,height;
     if( 1 == method)
    {
        plane_para[0] = g_plane_ransca.at<double>(0,0);
        plane_para[1] = g_plane_ransca.at<double>(1,0);
        plane_para[2] = g_plane_ransca.at<double>(2,0);
    }else
    {
        plane_para[0] = g_plane_para_least.at<double>(0,0);
        plane_para[1] = g_plane_para_least.at<double>(1,0);
        plane_para[2] = g_plane_para_least.at<double>(2,0);
    }
    height = -1;

    if( NEOLIX_FALSE ==  measureBox(d,plane_para,length,width,height) || height == -1)
    {
        return false;
    }else{
  //  std::cout<<"length: "<<length<<"width: "<<width<<"height: "<<height<<std::endl;
    v.length = length + g_fix_length ;
    v.width = width + g_fix_width;
    v.height = height + g_fix_height;
    return true;
    }
}

void setParameter(IN__ double backgroundParameter[], int method)
{
    if(0 == method)
    {
        g_plane_para_least = (cv::Mat_<double>(3,1)<<backgroundParameter[0],backgroundParameter[1],backgroundParameter[2]);
    }else
    {
        g_plane_ransca = (cv::Mat_<double>(3,1)<<backgroundParameter[0],backgroundParameter[1],backgroundParameter[2]);
    }
}

bool backgroundReconstruction(depthData depth_,double parameter[],int &n, int method)
{

    if(n < 3) return false;
    void* data = malloc(depth_.width*depth_.height*sizeof(short));
    memcpy(data,(void*)depth_.data,depth_.width*depth_.height*sizeof(short));
    cv::Mat d = cv::Mat(depth_.height,depth_.width,CV_16UC1,data);
  //  double  fu = g_cammer_intrinic.at<double>(0,0);
   // double  u0 = g_cammer_intrinic.at<double>(2,0);
    //double  v0 = g_cammer_intrinic.at<double>(3,0);
    //double  fv = g_cammer_intrinic.at<double>(1,0);
    cv::Rect rects = cv::Rect(g_plane_rect.at<int>(0,0), g_plane_rect.at<int>(1,0), g_plane_rect.at<int>(2,0), g_plane_rect.at<int>(3,0));
    cv::Mat depthRoi = d(rects).clone();
    cv::Mat para;

    if(0 == method)
    {
         if(-1 == leastSquareEquation(depthRoi, para))
        {
            std::cout<<"Failed reconstruction background"<<std::endl;
            free(data);
            return false;
        }
        double *a = para.ptr<double>();
        parameter[0] = a[0];
        parameter[1] = a[1];
        parameter[2] = a[2];
        g_plane_para_least = (cv::Mat_<double>(3,1)<<a[0],a[1],a[2]);
        free(data);
        return true;
    }
    else if(1 == method)
    {
        pointcloud<double,1,1> ptcld;
        const double *model;
        ptcld.converToPointCloud(depthRoi);
        ransac rs(&ptcld);
        if(NEOLIX_SUCCESS == rs.fittingPlane(0.995,0.37,8))
        {
            std::cout<<"rancas fiting success"<<std::endl;
            model = rs.getModel();
            parameter[0] = model[0];
            parameter[1] = model[1];
            parameter[2] = model[2];
            g_plane_ransca = (cv::Mat_<double>(3,1)<<model[0],model[1],model[2]);
            free(data);
            return true;
        }else
        {
            std::cout<<"rancas fiting failed"<<std::endl;
            free(data);
            return false;
        }

    }else
    {   free(data);
        return false;
    }
}

bool setArea(rect safeZone, rect planeZone, double internalCoefficient[], int n)
{
    if(n < 4) return false;
    g_rect_para = (cv::Mat_<int>(4,1)<<safeZone.left_x,safeZone.left_y,safeZone.width,safeZone.height);
    g_plane_rect = (cv::Mat_<int>(4,1)<<planeZone.left_x,planeZone.left_y,planeZone.width,planeZone.height);
    g_cammer_intrinic = (cv::Mat_<double>(4,1)<<internalCoefficient[0], internalCoefficient[1], internalCoefficient[2], internalCoefficient[3]);
    return true;
}

bool setMaxMinArea(rect maxZone, rect minZone)
{
    if(maxZone.height <= minZone.height || maxZone.width <= minZone.width) return false;

    g_rect_para = (cv::Mat_<int>(4,1)<<maxZone.left_x,maxZone.left_y,maxZone.width,maxZone.height);
    g_plane_rect = (cv::Mat_<int>(4,1)<<minZone.left_x,minZone.left_y,minZone.width,minZone.height);
    return true;
}

NEOLIX_STATUS_LIST config()
{
     cv::FileStorage fr("./configFile/config.yml",cv::FileStorage::READ);
     if(!fr.isOpened())
     {
         std::cout<<"can not open config.yml"<<std::endl;
         fr.release();
         return NEOLIX_FALSE;
     }
     fr[configNames[NEOLIX_RECT]]>>g_rect_para;
     fr[configNames[NEOLIX_PLANE_FITTING_LESRT]] >> g_plane_para_least;
     fr[configNames[NEOLIX_PLANE_FITTING_RACSCA]] >> g_plane_ransca;
     fr[configNames[NEOLIX_MEASURE_FLAT]]>>g_plane_rect;
     #ifdef  TY_SDK
     fr[configNames[NEOLIX_CAMMER_INTRINSIC_TY]] >> g_cammer_intrinic;
     #endif // TY_SDK
     #ifdef  ADI_TOF_SDK
     fr[configNames[NEOLIX_CAMMER_INTRINSIC_ADI_TOF]] >> g_cammer_intrinic;
     #endif // ADI_TOF_SDK

     fr.release();
     return NEOLIX_SUCCESS;

}

///½«²âÁ¿Æ½Ì¨µÈ·Ö³Én*n¸öÇøÓò£¬·µ»ØµÄ¸÷ÇøÓòµÄÉî¶ÈºÍÖÐÐÄµã
void splitNareaFromPad(cv::Mat DepthImage, int n,  std::vector<neolix::index_value<int,cv::Point2i> > &centerPoints, std::vector<short> &distances)
{
    int rows = DepthImage.rows;
    int cols = DepthImage.cols;
    double confidence;
    if(rows < n || cols < n) throw "the padDepthImage cols and rows must more than n!";
    cv::Mat shortImage;
    if(DepthImage.type() != CV_16SC1)
    {
        DepthImage.convertTo(shortImage,CV_16SC1);
    }else
    {
        shortImage = DepthImage;

    }
    std::vector<int> rowPoints;
    std::vector<int> colPoints;
    std::vector<short> depthPoints;
    int rectWidth, rectHeight;
    int ptindex = 0;
    cv::Mat roi;
    rowPoints.resize(n*2);
    colPoints.resize(n*2);
    rowPoints[0]   =  0;
    rowPoints[2*n-1] =  rows-1;
    colPoints[0]   =  0;
    colPoints[2*n-1] =  cols-1;

    for(int pt = 1; pt < 2*n-2;pt +=2)
    {
        rowPoints[pt]   = (pt / 2 + 1)*(rows + 1)/n -1;
        rowPoints[pt+1] = rowPoints[pt] + 1;

        colPoints[pt]   = (pt /2 + 1)*(cols + 1)/ n -1;
        colPoints[pt+1] = colPoints[pt] + 1;
    }
    for(int colPoint = 0; colPoint < 2*n; colPoint += 2)
    {
        for(int rowPoint = 0; rowPoint < 2*n; rowPoint += 2)
        {
            rectWidth  = colPoints[colPoint+1] - colPoints[colPoint] + 1;
            rectHeight = rowPoints[rowPoint+1] - rowPoints[rowPoint] + 1;
            cv::Rect box(colPoints[colPoint], rowPoints[rowPoint],
                         rectWidth, rectHeight);
            cv::Point2i center(colPoints[colPoint] + rectWidth/2, rowPoints[rowPoint] + rectHeight/2);
            neolix::index_value<int, cv::Point2i> iv ;
            iv.index = ptindex++;
            iv.value = center;
            centerPoints.push_back(iv);
            //¼ÆËãÉãÏñÍ·µ½¸ÃÇøÓòµÄ¾àÀë
            roi = shortImage(box);
            depthPoints.clear();
            short* ptr = roi.ptr < short>();
            for (int index = 0; index < roi.size().area(); index++)
            {
                depthPoints.push_back(ptr[index]);
            }
            caldepth depthHist(&depthPoints);
            depthHist.calDepthHist();
            unsigned short dis = depthHist.getdepth(confidence);
            distances.push_back(dis);

        }
    }
}

void stormToXML(const std::vector<float> &distances, std::vector<neolix::index_value<int,cv::Point2i> > &centerPoints, std::string filename )
{
    cv::FileStorage fw(filename,cv::FileStorage::WRITE);
    if(!fw.isOpened()) std::cout<<"can not open xml file:"<<filename<<std::endl;
    std::string index("index");
    fw<<"size"<<(int)distances.size();
    fw<<"distances"<<"[";
    for(auto val : distances) fw<<val;
    fw<<"]";

    fw<<"centerPoints"<<"{";
    for(auto val :centerPoints )
    {
        std::stringstream sstr;
        sstr<<index<<val.index;
        fw<<sstr.str()<<val.value;
    }
    fw<<"}";
    fw.release();

}


void readFromXml(std::vector<float> &distances, std::vector<neolix::index_value<int, cv::Point2i>> &centerPoints, std::string filename)
{
    cv::FileStorage fr(filename, cv::FileStorage::READ);
    if(!fr.isOpened()) std::cout<<"can not open xml file:"<<filename<<std::endl;
    cv::FileNode node = fr["distances"];
    if(node.type()!= FileNode::SEQ)
    {
      cerr<<"distances is not a sequence! FAIL"<<endl;
    }
    distances.clear();
    for(cv::FileNodeIterator iter = node.begin(); iter != node.end(); iter++)
    {
        distances.push_back((float)(*iter));
    }
    node = fr["centerPoints"];
    int size = fr["size"];
    std::string index("index");
    centerPoints.clear();
    cv::Point2i point ;
    for(int i = 1; i <= size; i++)
    {
        std::stringstream sstr;
        sstr<<index<<i;
        node[sstr.str()]>>point;
        neolix::index_value<int, cv::Point2i> iv(i,point);
        centerPoints.push_back(iv);
    }
    fr.release();

}
void initEmptyMatConfigFile()
{
  cv::Mat data;
  cv::FileStorage fw("./configFile/config.yml",cv::FileStorage::WRITE);
  if(!fw.isOpened())return;
  for(int i=0; i < flagNunber; i++) fw<<configNames[i]<<data;
  fw.release();

}

void UptateComfigToXml(cv::Mat &data, STORM_FLAG flag)
{
    std::vector<cv::Mat> mats ;
    cv::FileStorage fr("./configFile/config.yml",cv::FileStorage::READ);
    for(int i = 0; i < flagNunber; i++)
    {
        Mat tMat ;
        if(flag != i)fr[configNames[i]]>>tMat;
        else tMat = data;
        mats.push_back(tMat);
    }
    fr.release();
    cv::FileStorage fwf("./configFile/config.yml",cv::FileStorage::WRITE);
    for(int i = 0;  i < flagNunber; i++)
    {
        fwf<<configNames[i]<<mats[i];
    }
    fwf.release();
}

void getDepthColor(depthData depth_,void *buff)
{
    void* data = malloc(depth_.width*depth_.height*sizeof(short));
    memcpy(data, (void*)depth_.data, depth_.width*depth_.height*sizeof(short));
    cv::Mat d = cv::Mat(depth_.height,depth_.width,CV_16UC1,data);
    DepthRender render ;
    cv::Mat depthColorMat = render.Compute(d);
    memcpy(buff, (void*)depthColorMat.data, depthColorMat.rows*depthColorMat.cols*3*sizeof(uchar));
    free(data);

}

}
