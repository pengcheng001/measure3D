#ifndef ROTATE_PLANE_H__
#define ROTATE_PLANE_H__

#include"pointcloud.hpp"
#include<opencv2/core/core.hpp>
#include<vector>
#include"../neolixMV.h"

namespace neolix {


  class rotatePlane{
  public:
    rotatePlane();
    virtual ~rotatePlane();
    bool setData(pointcloudData pcd ,cv::Rect maxRect, cv::Rect minRect);
    bool setData(void *data,size_t size);
    void setFitnessMethod(int method);
    void printPara();
    bool CalculatedRotationMatrix();
    const float* getInsidePointCloud(size_t &size_);
    pointcloud<double>* getplanePoints();
    const double* getRotationMatrix();
    bool getPlaneDepth(double &depth);


  private:
    bool fitness();

    pointcloud<double> *planePoints;
    size_t size;//for between MaxRect and MinRect count points
    size_t in_size_;//for the count of points in MinRect Arae
    int method;//for fitness plane method: 0 is eastSquare, 1 is random unit sample
    double *planeParameter;//Parameter of plane
    double *normalVector;//The normal vector of the plane
    float *inMinRectData;//The memory area is stored in the small rectangular box, which corresponds to the Mat format of opencv.Format is: x, y, z;X, y, z;......
    double *RotationMatrix;//Rotation matrix, consisting of 9 double components (3X3)

  };
}

#endif
