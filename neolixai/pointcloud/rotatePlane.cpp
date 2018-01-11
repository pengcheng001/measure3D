#include"rotatePlane.h"
#include<opencv2/core/core.hpp>
#include"../imagepro/Utils.h"
#include"../pointcloud/ransac.h"
#include<opencv2/contrib/contrib.hpp>
#include<cmath>
#include<fstream>
//#define SAVEPOINT
namespace neolix {

  rotatePlane::rotatePlane():planePoints(nullptr),size(0),in_size_(0),method(1),planeParameter(nullptr),normalVector(nullptr),inMinRectData(nullptr),RotationMatrix(nullptr)
  {
  }

  bool rotatePlane::setData(pointcloudData pcd, cv::Rect maxRect, cv::Rect minRect)
  {

#ifdef SAVEPOINT
      std::ofstream innerPointFile;
      std::ofstream outerPointFile;

      innerPointFile.open("innerPoint.txt");
      outerPointFile.open("outerPoint.txt");

#endif
    if(maxRect.area() <= minRect.area()) return false;

    size_t cloudPointWidth = pcd.width;
   // FrameData_t *pointdata = (FrameData_t*)data;
    void* pointdata = malloc(pcd.height * pcd.width * sizeof(point3Df));
    memcpy(pointdata,pcd.data, pcd.height * pcd.width * sizeof(point3Df));
    cv::Point point;
    size_t count_points = 0;//for between MaxRect and MinRect count points
    size_t in_count_points = 0;//for the count of points in MinRect Arae

    if(nullptr != planePoints) delete planePoints;
    this->planePoints = new pointcloud<double>();

    this->planePoints->resize(maxRect.area(),maxRect.area());

    if( nullptr != this->inMinRectData) delete[] this->inMinRectData;
    this->inMinRectData = new float[maxRect.area() *3 ];
//    delete[] inMinRectData;
//    inMinRectData = nullptr;


    for(int row = maxRect.y; row < maxRect.height; row++)
      {
        for(int col = maxRect.x; col < maxRect.width; col++ )
          {
            point.x = col;
            point.y = row;


          if( maxRect.contains(point) && !minRect.contains(point) )
           //if( !((col <minRect.x+minRect.width  && minRect.x<col) && !( row > minRect.y && row < minRect.y + minRect.height)))///inn

           // if(true)
              {
              // std::cout<<"col:"<<col<<"row:"<<row<<std::endl;
             //  std::cout<<"minRect.x:"<<minRect.x<<"  minRect.x+minRect.width:"<<minRect.x+minRect.width<<std::endl;
                if( ((point3Df*)pointdata)[row*cloudPointWidth + col].z > 30)
                {
                   this->planePoints->data[count_points][0] = static_cast<double>(((point3Df*)pointdata)[row*cloudPointWidth + col].x ); ///The raw data is in meters, converted to millimeters
                   this->planePoints->data[count_points][1] = static_cast<double>(((point3Df*)pointdata)[row*cloudPointWidth + col].y );
                   this->planePoints->data[count_points][2] = static_cast<double>(((point3Df*)pointdata)[row*cloudPointWidth + col].z );
                   // this->planePoints->data[count_points][2] = 11.2;
                  //  std::cout<<pcld->data[0][2]<<std::endl;
                 //  std::cout<<this->planePoints->data[count_points][2]<<std::endl;
#ifdef SAVEPOINT
                    outerPointFile<<this->planePoints->data[count_points][0]<<" "<<this->planePoints->data[count_points][1]<<" "<<this->planePoints->data[count_points][2]<<std::endl;
#endif
                   count_points++;
                }
              }else
              {
                if(((point3Df*)pointdata)[row*cloudPointWidth + col].z > 30)
                {
                 //  float tt    = static_cast<double>(((point3Df*)pointdata)[row*cloudPointWidth + col].x * 1000.0f);
                  inMinRectData[3*in_count_points]     = static_cast<float>(((point3Df*)pointdata)[row*cloudPointWidth + col].x );
                  inMinRectData[3*in_count_points + 1] = static_cast<double>(((point3Df*)pointdata)[row*cloudPointWidth + col].y );
                  inMinRectData[3*in_count_points + 2] = static_cast<double>(((point3Df*)pointdata)[row*cloudPointWidth + col].z );
                //  std::cout<<inMinRectData[3*in_count_points]<<" , "<<inMinRectData[3*in_count_points+1]<<" , "<<inMinRectData[3*in_count_points + 2]<<std::endl;
#ifdef SAVEPOINT
                  innerPointFile<<inMinRectData[3*in_count_points]<<" "<<inMinRectData[3*in_count_points + 1]<<" "<<inMinRectData[3*in_count_points + 2]<<std::endl;
#endif
                  in_count_points++;

                }
              }
          }
      }

#ifdef SAVEPOINT
    innerPointFile.close();
    outerPointFile.close();
#endif
    this->planePoints->onlyModifySzie(count_points);
  //  std::cout<<" count_points================  "<<count_points<<std::endl;
    this->size = count_points;
    this->in_size_= in_count_points;

    free(pointdata);
    return true;

  }

  bool rotatePlane::setData(void *data, size_t size)
  {
    point3Df *pointdata = (point3Df*)data;
    if(0 != planePoints) delete planePoints;
    this->planePoints = new pointcloud<double>();
    this->planePoints->resize(size,size);
    for(size_t i = 0; i < size; i++)
      {
        this->planePoints->data[i][0] = static_cast<double>(pointdata[i].x);
        this->planePoints->data[i][1] = static_cast<double>(pointdata[i].y);
        this->planePoints->data[i][2] = static_cast<double>(pointdata[i].z);
      }
    this->planePoints->onlyModifySzie(static_cast<unsigned long>( size) );
    this->size = size;
    return true;
  }

  pointcloud<double>* rotatePlane::getplanePoints()
  {
      return planePoints;
  }

  rotatePlane::~rotatePlane()
  {

    if(nullptr != planePoints) { delete planePoints; planePoints = nullptr; }
    if(nullptr != planeParameter) {delete[] planeParameter; planeParameter = nullptr;}
    if(nullptr != inMinRectData) {delete[] inMinRectData; inMinRectData = nullptr;}
    if(nullptr != normalVector){delete[] normalVector; normalVector = nullptr;}
    if(nullptr != RotationMatrix) {delete[] RotationMatrix; RotationMatrix = nullptr;}
  }

  void rotatePlane::setFitnessMethod(int method)
  {
    this->method = method;
  }

  bool rotatePlane::fitness()
  {
    if(nullptr != planeParameter)  delete[] planeParameter;
    if(nullptr != normalVector)    delete[] normalVector;
    planeParameter = new double[3];
    normalVector   = new double[3];

    if(0 == this->method)
      {
        cv::Mat para;
        if(-1 == leastSquareEquationForPointCloud(this->planePoints, para)) return false;
        double *a = para.ptr<double>();
        this->planeParameter[0] = a[0];
        this->planeParameter[1] = a[1];
        this->planeParameter[2] = a[2];

        this->normalVector[0]  = -a[0];
        this->normalVector[1]  = -a[1];
        this->normalVector[2]  = 1;

        return true;
      }
    else if(1 == this->method)
      {
        ransac rs(this->planePoints);
        rs.isFitDepthData(false);
        if( NEOLIX_SUCCESS == rs.fittingPlane(0.995,0.37,5))
          {
            const double *a ;
            a =rs.getModel();
          //  rs.printPara();
            planeParameter[0] = a[0];
            planeParameter[1] = a[1];
            planeParameter[2] = a[2];


            this->normalVector[0]  = -a[0];
            this->normalVector[1]  = -a[1];
            this->normalVector[2]  = 1.0;
            return true;
          }
        else return false;
      }
    else   return false;

  }

  void rotatePlane::printPara()
  {
    if(nullptr != planeParameter)
      {
        std::cout<<"plane parameter is "<<planeParameter[0]<<" , "<<planeParameter[1]<<" , "<<-1<<" ,"<<planeParameter[2]<<" , "<<std::endl;
        std::cout<<"normalVector of plane parameter is "<<normalVector[0]<<" , "<<normalVector[1]<<" , "<<normalVector[2]<<" , "<<std::endl;

      }
  }

  bool rotatePlane::CalculatedRotationMatrix()
  {
    if(nullptr == this->planePoints) {std::cout<<"no planePoints"<<std::endl;return false;}
    if(false == this->fitness()){std::cout<<"can not fitness"<<std::endl ;return false;}
    Point3D<double> normal(this->normalVector);//
    Point3D<double> zVector(0,0,1);

    ///Calculated rotation Angle
    ///
    double arcCos = normal.dot(zVector)/(normal.model()*zVector.model());
    double angle = 0;
    if(normal[0] > 0)
      angle = - std::acos(arcCos);
    else
      angle = std::acos(arcCos);

    ///Calculated rotation vector
    double z = (zVector[1] - zVector[0]*normal[1]/normal[0]) / (zVector[0]*normal[2]/normal[0] - zVector[2]);
    double x = 0 - (normal[1] + normal[2] * z) / normal[0];
    double y = 1.0;

    double norm_ = pow(z,2) + pow(x,2) + pow(y,2);
    norm_ = sqrt(norm_);

    cv::Mat rotationVector_t = ( cv::Mat_<double>(3,1)<<(x/norm_*angle),(y/norm_*angle),(z/norm_*angle) );
    cv::Mat rotationMatrix;
    cv::Rodrigues(rotationVector_t,rotationMatrix);
    if(nullptr != this->RotationMatrix) delete[] this->RotationMatrix;
    this->RotationMatrix = new double[9];
    RotationMatrix[0] = rotationMatrix.at<double>(0,0);
    RotationMatrix[1] = rotationMatrix.at<double>(0,1);
    RotationMatrix[2] = rotationMatrix.at<double>(0,2);
    RotationMatrix[3] = rotationMatrix.at<double>(1,0);
    RotationMatrix[4] = rotationMatrix.at<double>(1,1);
    RotationMatrix[5] = rotationMatrix.at<double>(1,2);
    RotationMatrix[6] = rotationMatrix.at<double>(2,0);
    RotationMatrix[7] = rotationMatrix.at<double>(2,1);
    RotationMatrix[8] = rotationMatrix.at<double>(2,2);
   //std::cout<<"rotationMatrix:\n"<<rotationMatrix<<std::endl;
    return true;

  }

  const float* rotatePlane::getInsidePointCloud(size_t &size_)
  {
    size_ = this->in_size_;
    return this->inMinRectData;
  }

  const double* rotatePlane::getRotationMatrix()
  {
    return this->RotationMatrix;
  }

  bool rotatePlane::getPlaneDepth(double &depth)
  {
    if(nullptr == this->RotationMatrix) return false;
    if(nullptr == this->planeParameter) return false;
    depth = RotationMatrix[8]*planeParameter[2];
    return true;
  }
}
