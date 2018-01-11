#include"backgroundReconstruction.h"

namespace neolix {

    extern cv::Mat g_rect_para;
    extern cv::Mat g_plane_para_least;
    extern cv::Mat g_plane_ransca;
    extern cv::Mat g_cammer_intrinic;
    extern cv::Mat g_plane_rect;

    neolix_status reconstructionPlane(cv::Mat &depthImag, int method)
    {

        pointcloud<double> points;

        cv::Rect maxRect(g_rect_para.at<int>(0,0), g_rect_para.at<int>(1,0),
                         g_rect_para.at<int>(2,0),g_rect_para.at<int>(3,0));
        cv::Rect minRect(g_plane_rect.at<int>(0,0), g_plane_rect.at<int>(1,0),
                         g_plane_rect.at<int>(2,0),g_plane_rect.at<int>(3,0));

        if(selectedDepthPointFromDepthImage(points,depthImag,maxRect,minRect) < 0) return NEOLIX_FALSE;

        if(0 == method)
        {
            cv::Mat plane;
            double *model;
            if(leastSquareEquationForPointCloud(&points, plane) < 0)return NEOLIX_FALSE;
            model = plane.ptr<double>();
            g_plane_para_least = (cv::Mat_<double>(3,1)<<model[0],model[1],model[2]);


        }
        else if (1 == method) {

            const double *model;
            ransac rs(&points);
            rs.isFitDepthData(false);
            if(NEOLIX_SUCCESS == rs.fittingPlane(0.995,0.37,3))
            {
                model = rs.getModel();
               g_plane_ransca = (cv::Mat_<double>(3,1)<<model[0],model[1],model[2]);
              }else {
                return NEOLIX_FALSE;
              }

        }

        return NEOLIX_SUCCESS;

    }

}
