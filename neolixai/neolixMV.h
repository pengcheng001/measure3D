#ifndef __MEOLIX_MV_H__
#define __MEOLIX_MV_H__

namespace neolix
{

#define OUT__
#define IN__


   typedef  struct RECT
   {
       int left_x;
       int left_y;
       int width;
       int height;
   }rect;

   typedef struct VOLUME
   {
       double length;
       double width;
       double height;
   }vol;
    typedef struct tag_POINT3DF
    {
        float        x;
        float        y;
        float        z;
    }point3Df;
   typedef struct DEPTHDADA
   {
       void* data;//(short*)
       int width;
       int height;
   }depthData;

  typedef struct POINTCLOUDDATA
  {
       void *data;//(point3Df*)
       int width;
       int height;
  }pointcloudData;

#ifdef __cplusplus

extern "C"
{
#endif // __cplusplus
   bool setArea(rect safeZone, rect planeZone,IN__ double internalCoefficient[],int n);
   bool backgroundReconstruction(depthData depth,OUT__ double parameter[],int &n, int method = 0);
   void setParameter(IN__ double backgroundParameter[], int method = 0);
   bool measureVol(depthData depth,vol &v, int method = 0);
   void getDepthColor(depthData depth,void *buff);
   //Î¢µ÷³¤¿í¸ß
   void dajustVol(short fix_lenght = 0, short fix_width = 0, short fix_height = 0);
   bool measureVol2(depthData depth_, vol &v, int method = 0);
   bool measureVol3(pointcloudData pointCloud, vol &v, int method = 1);
   bool setMaxMinArea(rect maxZone, rect minZone);
#ifdef __cplusplus
}
#endif // __cplusplus
}
#endif // __MEOLIX_MV_H__
