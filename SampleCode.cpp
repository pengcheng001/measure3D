// SampleCode.cpp : ¶¨Òå¿ØÖÆÌ¨Ó¦ÓÃ³ÌÐòµÄÈë¿Úµã¡£
//

#include <stdio.h>
#include<opencv2/opencv.hpp>
//#include "pthread.h"
//#include "libTof.h"
#include "include/libTof.h"
#include "neolixai/neolixMV.h"
#include "neolixai/common/DepthRender.hpp"
#include "neolixai/imagepro/Utils.h"
#include "neolixai/config/hardDebug.h"
#include <fstream>
#include<string>
#define BYTES_PER_POINT			2


#include"neolixai/test/test.h"

#define ROUND_UP(x, align) (x+(align-1))&(~(align-1))
//»ñÈ¡Ïà»úµÄYV12Êý¾Ý
short * g_Video_data;
#define CHAR_BUFFERSIZE 20
//#define CAPPOINTCLOUD
#define MEASURE
bool ismeasure = false;
bool issave    = false;
#ifdef MEASURE
static int DEPTHVIDEO_FRAME_SIZE;
#endif
void exitMain(bool &exit_main)
{

    int key = cv::waitKey(30);
        switch (key & 0xff)
        {
          case'q':
            exit_main = true;
            destroyAllWindows();
            break;
          case'm':
            ismeasure = true;
            break;
          case's':
            issave = true;
            break;
          default:
             break;
        }
}
int main()
{
    int rec[8] = { 0 };
    int rv;
    static int DEPTHMAP_W;
    static int DEPTHMAP_H;
    static int DEPTHVIDEO_W;
    static int DEPTHVIDEO_H;
    static int DEPTHVIDEO_FRAME_SIZE;

    FrameData_t* frame_data;
    FrameDataRgb_t* frame_data_Rgb;
    //unsigned char	*PColorbuffer_s;


	CLibTof tof;
	rv = tof.LibTOF_Connect();
	if (rv<0)
	{
		printf(" LibTOF_Connect Failed\n");;
		return 0;
	}
	
	tof.LibTOF_SetUseCase(MODE_1_5FPS_1200);

	DeviceInfo_t *deviceinfo = new DeviceInfo_t;//(DeviceInfo_t *)malloc(sizeof(DeviceInfo_t));
	memset(deviceinfo,0,sizeof(DeviceInfo_t));
	rv = tof.LibTOF_GetDeviceInfo(deviceinfo);
	if (rv<0)
	{
		printf(" LibTOF_GetDeviceInfo Failed\n");
		return 0;
	}
	else
	{
		DEPTHMAP_W = deviceinfo->DepthFrameWidth;
		DEPTHMAP_H = deviceinfo->DepthFrameHeight-1;//¼õÈ¥Ò»ÐÐÍ·³¤¶È
		DEPTHVIDEO_W = deviceinfo->VisibleFrameWidth;
		DEPTHVIDEO_H = deviceinfo->VisibleFrameHeight;

		

         DEPTHVIDEO_FRAME_SIZE = ROUND_UP(DEPTHVIDEO_W*DEPTHVIDEO_H*3/2+DEPTHVIDEO_W,1024);
	}

	frame_data = (FrameData_t*)malloc(sizeof(FrameData_t)*DEPTHMAP_W*DEPTHMAP_H + DEPTHMAP_W);
	memset(frame_data, 0, sizeof(FrameData_t)*DEPTHMAP_W*DEPTHMAP_H + DEPTHMAP_W);
	frame_data_Rgb = (FrameDataRgb_t*)malloc(sizeof(FrameDataRgb_t)*DEPTHMAP_W*DEPTHMAP_H + DEPTHMAP_W);
	memset(frame_data_Rgb, 0, sizeof(FrameDataRgb_t)*DEPTHMAP_W*DEPTHMAP_H + DEPTHMAP_W);

//    if (nullptr == PColorbuffer_s)
//	{
//		PColorbuffer_s = (unsigned char *)malloc(DEPTHMAP_W*DEPTHMAP_H * 4);
//	}

    g_Video_data = (short *)malloc(DEPTHVIDEO_FRAME_SIZE);


	//Camera_Parame_t camparam;
	//tof.LibTOF_GetCameraParam(camparam);
	//double innercoefficient[9] = { 0 };

	//for (int k = 0; k < 9; k++)
	//{
	//	innercoefficient[k] = camparam.Mtof[k];
	//}
	//std::cout << "Mtof:[\n" << camparam.Mtof[0] << " ," << camparam.Mtof[1] << " ," << camparam.Mtof[2] << " ,"
	//	<< std::endl << camparam.Mtof[3] << " ," << camparam.Mtof[4] << " ," << camparam.Mtof[5] << " ,"
	//	<< std::endl<< camparam.Mtof[6] << " ," << camparam.Mtof[7] << " ," << camparam.Mtof[8] << "\n]"<<std::endl;
//
//	//ÉãÏñ»ú²ÎÊýÉè¶¨
//	TofRgb mTofRgb;
//	double CamIntrinsic[9] = { 896.562034933577, 0, 678.0007388623873,
//		0, 895.7353155364204, 381.3408219828144,
//		0, 0, 1 };
#ifdef MEASURE
	double innercoefficient[4] = { 216.366592, 216.764084, 113.697975, 86.6220627 };
#endif
	bool check = true;
//	double Rotate[9] = { 0.999719, 0.022053, -0.008680,
//		-0.021973, 0.999716, 0.009222,
//		0.008881, -0.009028, 0.999920 };
//	double TranSalation[3] = { -13.6877*0.001, -7.60232*0.001, -4.00786*0.001 };
//

	bool exit_main = false;

#ifdef CAPPOINTCLOUD
	int fileindex = 0;
#endif
    int fileindex = 0;
	while (!exit_main)
	{
		FrameData_t* frame_data_p;
		FrameDataRgb_t* frame_data_Rgb_tmp;
		frame_data_p = frame_data;
		frame_data_Rgb_tmp = frame_data_Rgb;//MRAS04Éè±¸£¬»Ò¶ÈÄ£Ê½Ê±£¬´æ·ÅµÄÊÇ»Ò¶ÈÊý¾Ý£¬Ã¿¸öÏñËØÕ¼2×Ö½Ú

		DEPTHVIDEO_FRAME_SIZE = ROUND_UP(DEPTHVIDEO_W*DEPTHVIDEO_H*3/2+DEPTHVIDEO_W,1024);//????????

		short *g_Video_data = (short *)malloc(DEPTHVIDEO_FRAME_SIZE);
		int rs = tof.LibTOF_RcvDepthFrame2(frame_data_p, frame_data_Rgb_tmp, DEPTHMAP_W, DEPTHMAP_H);
		tof.LibTOF_RcvVideoFrame(g_Video_data, DEPTHVIDEO_FRAME_SIZE);

		cv::Mat mtVideoData(DEPTHVIDEO_H * 3 / 2, DEPTHVIDEO_W, CV_8UC1, (void*)g_Video_data);
		cv::Mat mtOutData(DEPTHVIDEO_H, DEPTHVIDEO_W, CV_8UC3);
		cv::cvtColor(mtVideoData, mtOutData, cv::COLOR_YUV420p2RGB);
		cv::imshow("rgb",mtOutData);


		if (rs != LTOF_SUCCESS)
			return -1;

		/*
		* get depthdata format is dpeth1,depth2....depthn which is rows and cols
		* author:PengCheng / Email:pengcheng@neolix.com
		*/

		short * depths = new short[DEPTHMAP_W * DEPTHMAP_H ];
        neolix::point3Df *cloud = new neolix::point3Df[DEPTHMAP_W * DEPTHMAP_H ];
		for (int i = 0; i < DEPTHMAP_H * DEPTHMAP_W ; i++)

		{
		depths[i] = static_cast<short> (frame_data_p[i].z * 1000.0f);
        cloud[i].x = frame_data_p[i].x * 1000.0f;
        cloud[i].y = frame_data_p[i].y * 1000.0f;
        cloud[i].z = frame_data_p[i].z * 1000.0f;
      //  std::cout<<cloud[i].x<<"  , "<<cloud[i].y<<"  , "<<cloud[i].z<<std::endl;
		}

		neolix::depthData pd;
		pd.data = depths;
		pd.width = DEPTHMAP_W;
		pd.height = DEPTHMAP_H;

        neolix::pointcloudData pointscloud;
        pointscloud.width = DEPTHMAP_W;
        pointscloud.height = DEPTHMAP_H;
        pointscloud.data = cloud;
		void* buff = malloc(DEPTHMAP_H * DEPTHMAP_W * 3 * sizeof(uchar));

		neolix::getDepthColor(pd, buff);
		cv::Mat mycolor(DEPTHMAP_H, DEPTHMAP_W, CV_8UC3, buff);
		cv::Rect rectangle = cv::Rect(rec[0], rec[1], rec[2], rec[3]);
		cv::Rect rectangle1 = cv::Rect(rec[4], rec[5], rec[6], rec[7]);
		cv::rectangle(mycolor, rectangle, cv::Scalar(128, 255, 25));
		cv::rectangle(mycolor, rectangle1, cv::Scalar(128, 255, 25));

		cv::resize(mycolor,mycolor,cv::Size(mycolor.cols*3,mycolor.rows*3),0,0,INTER_CUBIC);
		cv::imshow("mycolor", mycolor);
		 exitMain(exit_main);

		if (check)
		{
			
			neolix::hardebug(pd, rec);
			neolix::rect planeZone = { rec[4], rec[5], rec[6], rec[7] };
			neolix::rect safeZone = { rec[0], rec[1], rec[2], rec[3] };
            neolix::setArea(safeZone, planeZone, innercoefficient, 4);
//			double planpar[3] = { 0 };
			int n = 3;
//			neolix::backgroundReconstruction(pd, planpar, n, 0);
//			neolix::setParameter(planpar,0);
			std::cout << rec[0] << "," << rec[1] << "," << rec[2] << "," << rec[3] << "," << std::endl;
			std::cout << rec[4] << "," << rec[5] << "," << rec[6] << "," << rec[7] << "," << std::endl;

			check = false;
			continue;
		}
#ifdef MEASURE
		if(ismeasure)
		  {
            std::string point("point");
            std::stringstream ss;
            ss<<fileindex;
            point +=ss.str();
            point += ".txt";

            std::string depth("depth");
            depth +=ss.str();
            depth += ".txt";
            fileindex++;

            std::ofstream ofile;
            std::ofstream ofile2;
            ofile.open(point);
            ofile2.open(depth);



            for (int i = 0; i < DEPTHMAP_H * DEPTHMAP_W ; i++)

            {
          //  depths[i] = static_cast<short> (frame_data_p[i].z * 1000.0f);
             ofile2<<frame_data_p[i].z*1000.0f<<std::endl;
          ///   if(frame_data_p[i].z > 0) filter invial data
             ofile<<frame_data_p[i].x*1000.0f<<"  "<<frame_data_p[i].y*1000.0f<<"  "<<frame_data_p[i].z*1000.0f<<std::endl;
 //            ofile<<cloud[i].x<<"  "<<cloud[i].y<<"  "<<cloud[i].z<<std::endl;

            }
            ofile.close();
            ofile2.close();
            std::cout<<"Save pointCloud in "<<point<<std::endl;
            std::cout<<"Save depth in "<<depth<<std::endl;
		     ismeasure = false;
		      neolix::vol v;
           //  if (neolix::measureVol2(pd, v, 1));
             if(neolix::measureVol3(pointscloud,v,1))

		      std::cout << "length: " << v.length << "width: " << v.width << "height: " << v.height << std::endl;
             else std::cout<<"can not measure VOL"<<std::endl;
		  }
#endif

#ifdef CAPPOINTCLOUD
//      if(issave)
//          {
//          std::string point("point");
//          std::stringstream ss;
//          ss<<fileindex;
//          point +=ss.str();
//          point += ".txt";

//          std::string depth("depth");
//          depth +=ss.str();
//          depth += ".txt";
//          fileindex++;

//          std::ofstream ofile;
//          std::ofstream ofile2;
//          ofile.open(point);
//          ofile2.open(depth);
//          ofile2<<DEPTHMAP_H<<std::endl;
//          ofile2<<DEPTHMAP_W<<std::endl;

//          ofile<<DEPTHMAP_H<<std::endl;
//          ofile<<DEPTHMAP_W<<std::endl;

//          ofile<<rec[0]<<" "<<rec[1]<<" "<<rec[2]<<" "<<rec[3]<<std::endl;
//          ofile<<rec[4]<<" "<<rec[5]<<" "<<rec[6]<<" "<<rec[7]<<std::endl;


//          for (int i = 0; i < DEPTHMAP_H * DEPTHMAP_W ; i++)

//          {
//        //  depths[i] = static_cast<short> (frame_data_p[i].z * 1000.0f);
//           ofile2<<frame_data_p[i].z*1000.0f<<std::endl;
//        ///   if(frame_data_p[i].z > 0) filter invial data
//           ofile<<frame_data_p[i].x*1000.0f<<"  "<<frame_data_p[i].y*1000.0f<<"  "<<frame_data_p[i].z*1000.0f<<std::endl;

//          }
//          ofile.close();
//          ofile2.close();
//          std::cout<<"Save pointCloud in "<<point<<std::endl;
//          std::cout<<"Save depth in "<<depth<<std::endl;
//          issave = false;
//        }
#endif
      delete[] depths;
      delete[] cloud;

	}

	tof.LibTOF_DisConnect();
	return 0;

}

int main0()
{

  std::cout<<"=========== TEST  test_rotatePlane() begin========================"<<std::endl;

  //neolix::test_rotatePlane();

  std::cout<<"=========== TEST  test_rotatePlane() end  ========================"<<std::endl<<std::endl;

  std::cout<<"=========== TEST  test_measure3D()   begin ========================"<<std::endl;



  neolix::test_measure3D();
  std::cout<<"=========== TEST  test_measure3D()    end  ========================"<<std::endl<<std::endl;
 // neolix::testCloud();

  return 0;

}

