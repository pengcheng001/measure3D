#include"measure.h"
#include"../common/DepthRender.hpp"
#include"../imagepro/Utils.h"

namespace neolix
{
  extern cv::Mat g_rect_para;
  extern cv::Mat g_plane_para_least;
  extern cv::Mat g_plane_ransca;
  extern cv::Mat g_cammer_intrinic;
  extern cv::Mat g_plane_rect;
  static DepthRender render ;
  static cv::vector<cv::Point2f> points(10);
  static float pixLength, pix_width;
  static double paddis,confidence;
  static unsigned short boxDistance;
  static cv::Point2f objCenter;
NEOLIX_STATUS_LIST measureBox(IN_ const cv::Mat depthImage,
				IN_ const double  planePar[3],
				OUT_ float &length,
				OUT_ float &width,
				OUT_ float &height,
				IN_  bool  visual,
				IN_  bool  useRect)
{
	cv::Mat depthRoi, colorRoi;
	cv::vector<cv::Point> contours;
	if(true == useRect)
    {
//        cv::Rect rects  = cv::Rect(g_rect_para.at<long>(0,0), g_rect_para.at<long>(1,0),
//				 g_rect_para.at<long>(2,0), g_rect_para.at<long>(3,0));
	    cv::Rect rects  = cv::Rect(g_plane_rect.at<long>(0,0), g_plane_rect.at<long>(1,0),
				     g_plane_rect.at<long>(2,0), g_plane_rect.at<long>(3,0));
        depthRoi = depthImage(rects);
    }else depthRoi = depthImage;
	colorRoi = depthRoi.clone();
	colorRoi = render.Compute(colorRoi);
	if(true == visual)
    {
        cv::imshow("measureDisplayRect",colorRoi);
        cv::Mat colorFull ;
        colorFull = render.Compute(depthImage);
        cv::imshow("measureDisplayFull",colorFull);

    }
	points.clear();
	NEOLIX_STATUS_LIST status = PopRang(colorRoi,contours,pixLength,pix_width,points);
	if(NEOLIX_CANOT_FIND_CONTOURS == status) return NEOLIX_CANOT_FIND_CONTOURS;
	if(NEOLIX_FALSE == status) return NEOLIX_FALSE;

	calCoutousCenter(contours, objCenter);
	points.push_back(objCenter);
	Getavepaddis(points,planePar,paddis);
    unsigned short minDepth = 9999;//set a bigger number
    boxDistance = calculateDepthFromDepthImagInRangeCountour(depthRoi,contours,confidence,minDepth);
	Getxyz2(points, boxDistance,length,width);
    height = static_cast<float>(paddis - minDepth);

	return NEOLIX_SUCCESS;
}

}
