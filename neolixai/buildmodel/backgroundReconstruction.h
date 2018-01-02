#ifndef BACK_GROUND_RECONSTRUCTION__H
#define BACK_GROUND_RECONSTRUCTION__H

#include"../imagepro/Utils.h"
#include"../pointcloud/pointcloud.hpp"
#include"../pointcloud/ransac.h"
#include"../driverdecorate/base.h"
namespace neolix {



    neolix_status reconstructionPlane(cv::Mat &depthImag, int method = 0);



}

#endif
