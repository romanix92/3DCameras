/*==============================================================================

@file
   fastcv.inl

@brief
   Inline call to function table for public API.

Copyright (c) 2010 QUALCOMM Incorporated.
All Rights Reserved Qualcomm Proprietary

Export of this technology or software is regulated by the U.S.
Government. Diversion contrary to U.S. law prohibited.

All ideas, data and information contained in or disclosed by
this document are confidential and proprietary information of
QUALCOMM Incorporated and all rights therein are expressly reserved.
By accepting this material the recipient agrees that this material
and the information contained therein are held in confidence and in
trust and will not be used, copied, reproduced in whole or in part,
nor its contents revealed in any manner to others without the express
written permission of QUALCOMM Incorporated.

==============================================================================*/

//==============================================================================
// Include Files
//==============================================================================

#include <stdio.h>
#include <stdlib.h>

//==============================================================================
// Defines
//==============================================================================

#ifdef ANDROID
   #include <android/log.h>
   #define FASTCV_ERROR( FMT, ... ) \
      __android_log_print( ANDROID_LOG_ERROR, "", FMT, __VA_ARGS__ );
#else
   #define FASTCV_ERROR( FMT, ... ) \
      fprintf( stderr, FMT, __VA_ARGS__ );
#endif

#ifndef fcvAssertHook
   #define fcvAssertHook( FASTCV_ASSERT_HOOK_IN, \
                          FASTCV_ASSERT_HOOK_FILE, \
                          FASTCV_ASSERT_HOOK_LINE ) \
                 FASTCV_ERROR( "%s@%d: %s\n", \
                               FASTCV_ASSERT_HOOK_FILE, \
                               FASTCV_ASSERT_HOOK_LINE, \
                               #FASTCV_ASSERT_HOOK_IN )
#endif

#ifndef NDEBUG
   #define fcvAssert( FASTCV_ASSERT_IN ) \
      if( !(FASTCV_ASSERT_IN) ) \
         fcvAssertHook( FASTCV_ASSERT_IN, __FILE__, __LINE__ );
#else
   #define fcvAssert( FASTCV_ASSERT_IN ) ((void)0)
#endif


//==============================================================================
// Declarations
//==============================================================================

#ifdef _MSC_VER
   #pragma warning(disable:4311)
#endif

extern void
(**ppfcvFilterMedian3x3u8_v2)
( 
   const uint8_t* __restrict src,
   unsigned int srcWidth,
   unsigned int srcHeight,
   unsigned int srcStride,
   uint8_t* __restrict dst,
   unsigned int dstStride 
);

extern void
(**ppfcvFilterGaussian3x3u8_v2)
( 
   const uint8_t* __restrict srcImg,
   unsigned int srcWidth,
   unsigned int srcHeight,
   unsigned int srcStride,
   uint8_t* __restrict dstImg,
   unsigned int dstStride,
   int border 
);

extern void
(**ppfcvFilterGaussian5x5u8_v2)
( 
   const uint8_t* __restrict srcImg,
   unsigned int srcWidth,
   unsigned int srcHeight,
   unsigned int srcStride,
   uint8_t* __restrict dstImg,
   unsigned int dstStride,
   int blurBorder 
);

extern void
(**ppfcvFilterGaussian11x11u8_v2)
( 
   const uint8_t* __restrict srcImg,
   unsigned int srcWidth,
   unsigned int srcHeight,
   unsigned int srcStride,
   uint8_t* __restrict dstImg,
   unsigned int dstStride,
   int blurBorder 
);

extern void
(**ppfcvColorYCrCb420PseudoPlanarToRGB8888u8)
( const uint8_t* __restrict src,
  unsigned int              srcWidth,
  unsigned int              srcHeight,
  unsigned int              srcYStride,
  unsigned int              srcCStride,
  uint32_t* __restrict      dst,
  unsigned int              dstStride
);

extern void
(**ppfcvColorYUV420toRGB565u8)
(
  const  uint8_t* __restrict yuv420,
   unsigned int                       width,
   unsigned int                       height,
   uint32_t* __restrict rgb565
);

extern void
(**ppfcvColorYCrCbH1V1toRGB888u8)
(
   const uint8_t* __restrict crcb,
   unsigned int                       width,
   unsigned int                       height,
   uint8_t* __restrict rgb888
);

extern void
(**ppfcvColorYCrCbH2V2toRGB888u8)
(
   const uint8_t* __restrict y_src,
   unsigned int                       width,
   unsigned int                       height,
   uint8_t* __restrict rgb888
);

extern void
(**ppfcvColorYCrCbH2V1toRGB888u8)
(
   const uint8_t* __restrict ysrc,
   unsigned int                       width,
   unsigned int                       height,
   uint8_t* __restrict dst
);

extern void
(**ppfcvColorYCrCbH1V2toRGB888u8)
(
   const uint8_t* __restrict ysrc,
   unsigned int                       width,
   unsigned int                       height,
   uint8_t* __restrict dst
);

extern void
(**ppfcvColorRGB888toYCrCbu8_v2)
(
   const uint8_t* __restrict src,
   unsigned int              srcWidth,
   unsigned int              srcHeight,
   unsigned int              srcStride,
   uint8_t* __restrict       dst,
   unsigned int              dstStride
);

extern int
(**ppfcvDescriptor17x17u8To36s8)
(
   const uint8_t* __restrict patch,
   int8_t* __restrict descriptorChar,
   int32_t*  __restrict descriptorNormSq
);


extern int32_t
(**ppfcvDotProducts8)
(
   const int8_t* __restrict a,
   const int8_t* __restrict b,
   unsigned int abSize
);

extern int32_t
(**ppfcvDotProductu8)
(
   const uint8_t* __restrict  a,
   const uint8_t* __restrict  b,
   unsigned int         abSize
);

extern int32_t
(**ppfcvDotProduct36x1s8)( const int8_t* __restrict a,
                          const int8_t* __restrict b );

extern void
(**ppfcvDotProduct36x4s8)( const int8_t* __restrict A,
                          const int8_t* __restrict B,
                          const int8_t* __restrict C,
                          const int8_t* __restrict D,
                          const int8_t* __restrict E,
                          int32_t dotProducts[4] );

extern void
(**ppfcvDotProductNorm36x4s8)( const int8_t* __restrict A,
                              float                         invLengthA,
                              const int8_t* __restrict vB0,
                              const int8_t* __restrict vB1,
                              const int8_t* __restrict vB2,
                              const int8_t* __restrict vB3,
                              float* __restrict             invLengthsB,
                              float* __restrict             dotProducts  );
extern int32_t
(**ppfcvDotProduct36x1u8)( const uint8_t* __restrict a,
                      const uint8_t* __restrict b );

extern void
(**ppfcvDotProduct36x4u8)
(
   const uint8_t* __restrict A,
   const uint8_t* __restrict B,
   const uint8_t* __restrict C,
   const uint8_t* __restrict D,
   const uint8_t* __restrict E,
   uint32_t                  dotProducts[4]
);

extern void
(**ppfcvDotProductNorm36x4u8)( const uint8_t* __restrict  A,
                              float                            invLengthA,
                              const uint8_t* __restrict  vB0,
                              const uint8_t* __restrict  vB1,
                              const uint8_t* __restrict  vB2,
                              const uint8_t* __restrict  vB3,
                              float* __restrict                invLengthsB,
                              float* __restrict                dotProducts );

extern int32_t
(**ppfcvDotProduct64x1s8)( const int8_t* __restrict a,
                           const int8_t* __restrict b );

extern void
(**ppfcvDotProduct64x4s8)
(
   const int8_t* __restrict A,
   const int8_t* __restrict B,
   const int8_t* __restrict C,
   const int8_t* __restrict D,
   const int8_t* __restrict E,
   int32_t dotProducts[4]
);

extern void
(**ppfcvDotProductNorm64x4s8)( const int8_t* __restrict A,
                              float                         invLengthA,
                              const int8_t* __restrict vB0,
                              const int8_t* __restrict vB1,
                              const int8_t* __restrict vB2,
                              const int8_t* __restrict vB3,
                              float* __restrict             invLengthsB,
                              float* __restrict             dotProducts  );

extern uint32_t
(**ppfcvDotProduct64x1u8)
(
   const uint8_t* __restrict a,
   const uint8_t* __restrict b
);

extern void
(**ppfcvDotProduct64x4u8)
(
   const uint8_t* __restrict A,
   const uint8_t* __restrict B,
   const uint8_t* __restrict C,
   const uint8_t* __restrict D,
   const uint8_t* __restrict E,
   uint32_t                  dotProducts[4]
);

extern void
(**ppfcvDotProductNorm64x4u8)( const uint8_t* __restrict  A,
                              float                            invLengthA,
                              const uint8_t* __restrict  vB0,
                              const uint8_t* __restrict  vB1,
                              const uint8_t* __restrict  vB2,
                              const uint8_t* __restrict  vB3,
                              float* __restrict                invLengthsB,
                              float* __restrict                dotProducts );

extern int32_t
(**ppfcvDotProduct128x1s8)( const int8_t* __restrict a,
                            const int8_t* __restrict b );

extern void
(**ppfcvDotProduct128x4s8)( const int8_t* __restrict A,
                             const int8_t* __restrict B,
                             const int8_t* __restrict C,
                             const int8_t* __restrict D,
                             const int8_t* __restrict E,
                             int32_t dotProducts[4] );

extern void
(**ppfcvDotProductNorm128x4s8)
(
   const int8_t* __restrict A,
   float                invLengthA,
   const int8_t* __restrict vB0,
   const int8_t* __restrict vB1,
   const int8_t* __restrict vB2,
   const int8_t* __restrict vB3,
   float* __restrict    invLengthsB,
   float* __restrict    dotProducts
);

extern uint32_t
(**ppfcvDotProduct128x1u8)
(
   const uint8_t* __restrict a,
   const uint8_t* __restrict b
);

extern void
(**ppfcvDotProduct128x4u8)
(
   const uint8_t* __restrict A,
   const uint8_t* __restrict B,
   const uint8_t* __restrict C,
   const uint8_t* __restrict D,
   const uint8_t* __restrict E,
   uint32_t                  dotProducts[4]
);

extern void
(**ppfcvDotProductNorm128x4u8)(const uint8_t* __restrict  A,
                              float                  invLengthA,
                              const uint8_t* __restrict  vB0,
                              const uint8_t* __restrict  vB1,
                              const uint8_t* __restrict  vB2,
                              const uint8_t* __restrict  vB3,
                              float* __restrict      invLengthsB,
                              float* __restrict      dotProducts );


extern void
(**ppfcvDotProduct8x8u8)( const uint8_t* ptch, const uint8_t* img,
                         unsigned short imgW, unsigned short imgH, int nX,
                         int nY, unsigned int nNum, int32_t* nProducts );

extern void
(**ppfcvDotProduct11x12u8)( const uint8_t* __restrict ptch,
                           const uint8_t* __restrict img,
                           unsigned short imgW, unsigned short imgH, int iX,
                           int iY, int32_t* __restrict dotProducts );

extern void
(**ppfcvFilterSobel3x3u8_v2)
( const uint8_t* __restrict src,
  unsigned int              srcWidth,
  unsigned int              srcHeight,
  unsigned int              srcStride,
  uint8_t* __restrict       dst,
  unsigned int              dstStride 
);

extern void
(**ppfcvFilterCanny3x3u8_v2)
( const uint8_t* __restrict srcImg,
  unsigned int              srcWidth,
  unsigned int              srcHeight,
  unsigned int              srcStride,
  uint8_t* __restrict       dstImg,
  unsigned int              dstStride,
  int                       low,
  int                       high 
);

extern void
(**ppfcvImageDiffu8_v2)
( const uint8_t* __restrict src1,
  const uint8_t* __restrict src2,
  unsigned int              srcWidth,
  unsigned int              srcHeight,
  unsigned int              srcStride,
  uint8_t* __restrict       dst,
  unsigned int              dstStride
);

extern void
(**ppfcvImageDiffs16_v2)
( const int16_t* __restrict src1,
  const int16_t* __restrict src2,
   unsigned int             srcWidth, 
   unsigned int             srcHeight, 
   unsigned int             srcStride, 
        int16_t* __restrict dst,
   unsigned int             dstStride 
);

extern void
(**ppfcvImageDifff32_v2) 
(  const float* __restrict src1,
   const float* __restrict src2,
  unsigned int             srcWidth, 
  unsigned int             srcHeight, 
  unsigned int             srcStride, 
         float* __restrict dst,
  unsigned int             dstStride
);

extern void
(**ppfcvImageDiffu8f32_v2) 
( const uint8_t* __restrict src1,
  const uint8_t* __restrict src2,
   unsigned int             srcWidth, 
   unsigned int             srcHeight, 
   unsigned int             srcStride, 
          float* __restrict dst,
   unsigned int             dstStrde 
);


extern void
(**ppfcvImageDiffu8s8_v2) 
( const uint8_t* __restrict src1,
  const uint8_t* __restrict src2,
   unsigned int             srcWidth, 
   unsigned int             srcHeight, 
   unsigned int             srcStride, 
         int8_t* __restrict dst,
   unsigned int             dstStride 
);

extern void
(**ppfcvImageGradientInterleaveds16_v2)
( const uint8_t* __restrict src,
  unsigned int              srcWidth,
  unsigned int              srcHeight,
  unsigned int              srcStride,
  int16_t* __restrict       gradients,
  unsigned int              gradStride
);

extern void
(**ppfcvImageGradientInterleavedf32_v2)
( const uint8_t* __restrict src,
  unsigned int              srcWidth,
  unsigned int              srcHeight,
  unsigned int              srcStride,
  float* __restrict         gradients,
  unsigned int              gradStride
);

extern void
(**ppfcvImageGradientPlanars16_v2)
( const uint8_t* __restrict src,
  unsigned int              srcWidth,
  unsigned int              srcHeight,
  unsigned int              srcStride,
  int16_t* __restrict       dx,
  int16_t* __restrict       dy,
  unsigned int              dxyStride 
);

extern void
(**ppfcvImageGradientPlanarf32_v2)
( const uint8_t* __restrict src,
  unsigned int              srcWidth,
  unsigned int              srcHeight,
  unsigned int              srcStride,
  float* __restrict         dx,
  float* __restrict         dy,
  unsigned int              dxyStride 
);

extern void
(**ppfcvImageGradientSobelInterleaveds16_v2)
( const uint8_t* __restrict  src,
  unsigned int               srcWidth,
  unsigned int               srcHeight,
  unsigned int               srcStride,
  int16_t* __restrict        gradients,
  unsigned int               gradStride 
);

extern void
(**ppfcvImageGradientSobelInterleavedf32_v2)
( const uint8_t* __restrict src,
  unsigned int              srcWidth,
  unsigned int              srcHeight,
  unsigned int              srcStride,
  float* __restrict         gradients,
  unsigned int              gradStride
);

extern void
(**ppfcvImageGradientSobelPlanars16_v2)
( const uint8_t* __restrict  src,
  unsigned int               srcWidth,
  unsigned int               srcHeight,
  unsigned int               srcStride,
  int16_t* __restrict        dx,
  int16_t* __restrict        dy,
  unsigned int               dxyStride
);

extern void
(**ppfcvImageGradientSobelPlanarf32_v2)
( const uint8_t* __restrict  src,
  unsigned int               srcWidth,
  unsigned int               srcHeight,
  unsigned int               srcStride,
  float*                     dx,
  float*                     dy,
  unsigned int               dxyStride
);

extern void
(**ppfcvImageGradientSobelPlanarf32f32_v2)
( const float * __restrict  src,
  unsigned int              srcWidth,
  unsigned int              srcHeight,
  unsigned int              srcStride,
  float*                    dx,
  float*                    dy,
  unsigned int              dxyStride
);

extern int
(**ppfcvClusterEuclideanf32)( const float* __restrict  points,
                           int                      numPoints,  // actually not used but helpful
                           int                      dim,
                           int                      pointStride,
                           const size_t* __restrict indices,
                           int                      numIndices,
                           int                      numClusters,
                           float* __restrict        clusterCenters,
                           int                      clusterCenterStride,
                           float* __restrict        newClusterCenters,
                           size_t* __restrict       clusterMemberCounts,
                           size_t* __restrict       clusterBindings,
                           float*                   sumOfClusterDistances );


extern int
(**ppfcvClusterEuclideanNormedf32)( const float* __restrict  points,
                              int                      numPoints,
                              int                      dim,
                              int                      pointStride,
                              const size_t* __restrict indices,
                              int                      numIndices,
                              int                      numClusters,
                              float* __restrict        clusterCenters,
                              int                      clusterCenterStride,
                              float* __restrict        newClusterCenters,
                              size_t* __restrict       clusterMemberCounts,
                              size_t* __restrict       clusterBindings,
                              float*                   sumOfClusterDistances ) ;


extern int
(**ppfcvClusterEuclideanNormed36f32)( const float* __restrict  points,
                                    int                      numPoints,
                                    int                      pointStride,
                                    const size_t* __restrict indices,
                                    int                      numIndices,
                                    int                      numClusters,
                                    float* __restrict        clusterCenters,
                                    int                      clusterCenterStride,
                                    float* __restrict        newClusterCenters,
                                    size_t* __restrict       clusterMemberCounts,
                                    size_t* __restrict       clusterBindings,
                                    float*                   sumOfClusterDistances );

extern void
(**ppfcvImageGradientSobelPlanars8_v2)
( const uint8_t* __restrict src,
  unsigned int              srcWidth,
  unsigned int              srcHeight,
  unsigned int              srcStride,
  int8_t* __restrict        dx,
  int8_t* __restrict        dy,
  unsigned int              dxyStride 
);

extern void
(**ppfcvCornerFast9u8)
(
   const uint8_t*im,
   unsigned int xsize,
   unsigned int ysize,
   int barrier,
   unsigned int border,
   uint32_t* xy,
   unsigned int maxnumcorners,
   uint32_t* numcorners,
   unsigned int stride
);

extern void
(**ppfcvCornerFast9InMasku8)
(
   const uint8_t* __restrict im,
   unsigned int xsize,
   unsigned int ysize,
   int barrier,
   unsigned int border,
   uint32_t* __restrict xy,
   unsigned int maxnumcorners,
   uint32_t* __restrict numcorners,
   unsigned int stride,
   const uint8_t* __restrict bitMask,
   unsigned int maskWidth,
   unsigned int maskHeight
);

extern void
(**ppfcvCornerHarrisu8)
(
   const uint8_t* __restrict srcImg,
   unsigned int width,
   unsigned int height,
   unsigned int stride,
   unsigned int border,
   uint32_t* __restrict xy,
   unsigned int maxnumcorners,
   uint32_t* __restrict numcorners,
   int threshold
);

extern unsigned int
(**ppfcvLocalHarrisMaxu8)
(
   const uint8_t* __restrict src,
   unsigned int              srcWidth,
   unsigned int              srcHeight,
   unsigned int              srcStride,
   unsigned int              posX,
   unsigned int              posY,
   unsigned int             *maxX,
   unsigned int             *maxY,
   int                      *maxScore
);

extern void
(**ppfcvCornerHarrisInMasku8)
(
   const uint8_t* __restrict srcImg,
   unsigned int width,
   unsigned int height,
   unsigned int stride,
   unsigned int border,
   uint32_t* __restrict xy,
   unsigned int maxnumcorners,
   uint32_t* __restrict numcorners,
   int threshold,
   const uint8_t* __restrict bitMask,
   int maskWidth,
   int maskHeight
);

extern void
(**ppfcvGeomAffineFitf32)( const fcvCorrespondences* __restrict corrs,
                          float* __restrict affine );

extern int
(**ppfcvGeomAffineEvaluatef32)( const fcvCorrespondences* __restrict corrs,
                               float* __restrict affine,
                               float maxsqerr,
                               uint16_t* __restrict inliers,
                               int32_t* numinliers );

extern void
(**ppfcvGeomHomographyFitf32)( const fcvCorrespondences* __restrict corrs,
                              float* __restrict homography );

extern int
(**ppfcvGeomHomographyEvaluatef32)( const fcvCorrespondences* __restrict corrs,
                                   float* __restrict homography,
                                   float maxsqerr,
                                   uint16_t* __restrict inliers,
                                   int32_t* numinliers );

extern float
(**ppfcvGeomPoseRefineGNf32) ( const fcvCorrespondences* __restrict corrs,
                              short minIterations,
                              short maxIterations,
                              float stopCriteria,
                              float* initpose,
                              float* refinedpose );

extern int
(**ppfcvGeomPoseUpdatef32) (
   const float* __restrict projected,
   const float* __restrict reprojErr,
   const float* __restrict invz,
   const float* __restrict reprojVariance,
   unsigned int                numpts,
   float*       __restrict pose );

extern int
(**ppfcvGeomPoseOptimizeGNf32) (
   const float* __restrict projected,
   const float* __restrict reprojErr,
   const float* __restrict invz,
   const float* __restrict reprojVariance,
   unsigned int                numpts,
   float*       __restrict pose );

extern float
(**ppfcvGeomPoseEvaluateErrorf32) (
   const fcvCorrespondences* __restrict corrs,
   const float*              __restrict pose,
   float*                    __restrict projected,
   float*                    __restrict reprojErr,
   float*                    __restrict invz,
   float*                    __restrict reprojVariance );

extern int
(**ppfcvGeomPoseEvaluatef32) ( const fcvCorrespondences* __restrict corrs,
                              const float* pose,
                              float maxSquErr,
                              uint16_t* __restrict inliers,
                              uint32_t* numInliers );

extern void
(**ppfcvGeom3PointPoseEstimatef32) ( const fcvCorrespondences* __restrict corrs,
                                     float* pose,
                                     int32_t* numPoses );

extern void
(**ppfcvFilterCorr3x3s8_v2)
(
const int8_t* __restrict mask,
const uint8_t* __restrict srcImg, 
unsigned int srcWidth, 
unsigned int srcHeight,
unsigned int srcStride,
uint8_t* __restrict dstImg,
unsigned int dstStride
);

extern void
(**ppfcvFilterCorrSep9x9s16_v2)
(
   const int16_t* __restrict kernel,
   const int16_t* __restrict srcImg,
   unsigned int srcWidth, 
   unsigned int srcHeight, 
   unsigned int srcStride,
   int16_t* __restrict tmpImg,
   int16_t* __restrict dstImg, 
   unsigned int dstStride 
);

extern void
(**ppfcvFilterCorrSep13x13s16)
(
   const int16_t* __restrict knl,
   const int16_t* __restrict srcimg, unsigned int w, unsigned int h,
   int16_t* __restrict tmpimg,
   int16_t* __restrict dstimg
);


extern void
(**ppfcvFilterCorrSep11x11s16_v2)
(
   const int16_t* __restrict kernel,
   const int16_t* __restrict srcImg,
   unsigned int srcWidth, 
   unsigned int srcHeight, 
   unsigned int srcStride,
   int16_t* __restrict tmpImg,
   int16_t* __restrict dstImg, 
   unsigned int dstStride 
);

extern void
(**ppfcvFilterCorrSep13x13s16_v2)
(
   const int16_t* __restrict kernel,
   const int16_t* __restrict srcImg,
   unsigned int srcWidth, 
   unsigned int srcHeight, 
   unsigned int srcStride,
   int16_t* __restrict tmpImg,
   int16_t* __restrict dstImg, 
   unsigned int dstStride 
);

extern void
(**ppfcvFilterCorrSep15x15s16_v2)
(
   const int16_t* __restrict kernel,
   const int16_t* __restrict srcImg,
   unsigned int srcWidth, 
   unsigned int srcHeight, 
   unsigned int srcStride,
   int16_t* __restrict tmpImg,
   int16_t* __restrict dstImg, 
   unsigned int dstStride 
);

extern void
(**ppfcvFilterCorrSep17x17s16_v2)
(
   const int16_t* __restrict kernel,
   const int16_t* __restrict srcImg,
   unsigned int srcWidth, 
   unsigned int srcHeight, 
   unsigned int srcStride,
   int16_t* __restrict tmpImg,
   int16_t* __restrict dstImg, 
   unsigned int dstStride 
);

extern int
( **ppfcvScaleDownBy2u8_v2)
( const uint8_t* __restrict imgSrc,
  unsigned int width, 
  unsigned int height,
  unsigned int srcStride,
  uint8_t* __restrict imgDst,
  unsigned int dstStride
);

extern int
( **ppfcvScaleDownBy4u8_v2)
( const uint8_t* __restrict imgSrc,
  unsigned int srcWidth, 
  unsigned int srcHeight,
  unsigned int srcStride,
  uint8_t* __restrict imgDst,
  unsigned int dstStride
);

extern int
( **ppfcvScaleDown3To2u8)
( 
   const uint8_t* __restrict src,
   unsigned int             srcWidth,
   unsigned int             srcHeight,
   unsigned int             srcStride,
   uint8_t* __restrict      dst, 
   unsigned int             dstStride
);

extern void
(**ppfcvScaleDownu8_v2)
( const uint8_t* __restrict srcImg,
  unsigned int srcWidth, 
  unsigned int srcHeight,
  unsigned int srcStride,
  uint8_t* __restrict dstImg,
  unsigned int dstWidth, 
  unsigned int dstHeight,
  unsigned int dstStride
);

extern int
(**ppfcvScaleDownNNu8)
( 
   const uint8_t* __restrict src,
   unsigned int              srcWidth,
   unsigned int              srcHeight,
   unsigned int              srcStride,
   uint8_t* __restrict       dst,
   unsigned int              dstWidth,
   unsigned int              dstHeight,
   unsigned int              dstStride 
);

extern void
(**ppfcvScaleUpBy2Gaussian5x5u8_v2)
( const uint8_t* __restrict src,
  unsigned int srcWidth,
  unsigned int srcHeight,
  unsigned int srcStride,
  uint8_t* __restrict dst,
  unsigned int dstStride
);

extern void
(**ppfcvScaleDownBy2Gaussian5x5u8)
( const uint8_t* __restrict src,
  unsigned int width,
  unsigned int height,
  uint8_t* __restrict dst
);

extern void
(**ppfcvScaleDownBy2Gaussian5x5u8_v2)
( const uint8_t* __restrict src,
  unsigned int srcWidth,
  unsigned int srcHeight,
  unsigned int srcStride,
  uint8_t* __restrict dst,
  unsigned int dstStride
);

extern void
(**ppfcvImageIntensityStats) ( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             int                       xBegin,
                             int                       yBegin,
                             unsigned int              recWidth,
                             unsigned int              recHeight,
                             float*                    mean,
                             float*                    variance );

extern void
(**ppfcvImageIntensityHistogram)( const uint8_t* __restrict src,
                                unsigned int              srcWidth,
                                int                       xBegin,
                                int                       yBegin,
                                unsigned int              recWidth,
                                unsigned int              recHeight,
                                int32_t*                  histogram );

extern void
(**ppfcvIntegrateImageu8_v2)
( const uint8_t* __restrict imageIn,
  unsigned int imageWidth,
  unsigned int imageHeight,
  unsigned int imageStride,
  uint32_t* __restrict integralImageOut,
  unsigned int integralImageStride 
);

extern void
(**ppfcvIntegratePatchu8_v2)
( const uint8_t* __restrict imageIn,
  unsigned int imageWidth,
  unsigned int imageHeight,
  unsigned int imageStride,
  int patchX, 
  int patchY,
  unsigned int patchW, 
  unsigned int patchH,
  uint32_t* __restrict intgrlImgOut,
  uint32_t* __restrict intgrlSqrdImgOut 
);

extern void
(**ppfcvIntegratePatch12x12u8_v2)
( const uint8_t* __restrict imageIn,
  unsigned int imageWidth,
  unsigned int imageHeight,
  unsigned int imageStride,
  int patchX, 
  int patchY,
  uint32_t* __restrict intgrlImgOut,
  uint32_t* __restrict intgrlSqrdImgOut 
);

extern void
(**ppfcvIntegratePatch18x18u8_v2)
( const uint8_t* __restrict imageIn,
  unsigned int imageWidth,
  unsigned int imageHeight,
  unsigned int imageStride,
  int patchX, 
  int patchY,
  uint32_t* __restrict intgrlImgOut,
  uint32_t* __restrict intgrlSqrdImgOut 
);

extern void
(**ppfcvIntegrateImageLineu8)
(
   const uint8_t* __restrict imageIn,
   unsigned short numPxls,
   uint32_t* intgrl,
   uint32_t* intgrlSqrd
);

extern void
(**ppfcvIntegrateImageLine64u8)
(
   const uint8_t* __restrict imageIn,
   uint16_t* intgrl,
   uint32_t* intgrlSqrd
);

extern int
(**ppfcvNCCPatchOnCircle8x8u8_v2)
( const uint8_t* __restrict patch_pixels,
  const uint8_t* __restrict image_pixels,
  unsigned short            image_w,
  unsigned short            image_h,
  unsigned short            search_center_x,
  unsigned short            search_center_y,
  unsigned short            search_radius,
  int                       filterLowVariance,
  uint16_t*                 best_x,
  uint16_t*                 best_y,
  uint32_t*                 bestNCC,
  int                       doSubPixel,
  float*                    subX,
  float*                    subY );


extern int
(**ppfcvNCCPatchOnSquare8x8u8_v2)
( const uint8_t* __restrict patch_pixels,
  const uint8_t* __restrict image_pixels,
  unsigned short            image_w,
  unsigned short            image_h,
  unsigned short            search_center_x,
  unsigned short            search_center_y,
  unsigned short            search_w,
  int                       filterLowVariance,
  uint16_t*                 best_x,
  uint16_t*                 best_y,
  uint32_t*                 bestNCC,
  int                       doSubPixel,
  float*                    subX,
  float*                    subY );

extern void
(**ppfcvSumOfAbsoluteDiffs8x8u8_v2)
( const uint8_t* __restrict patch,
  unsigned int patchStride,
  const uint8_t* __restrict src,
  unsigned int srcWidth, 
  unsigned int srcHeight,
  unsigned int srcStride,
  uint16_t* __restrict dst,
  unsigned int dstStride
);

extern void
(**ppfcvSumOfSquaredDiffs36x4s8)( const int8_t* __restrict A,
                                   float invLenA,
                                   const int8_t* __restrict B,
                                   const int8_t* __restrict C,
                                   const int8_t* __restrict D,
                                   const int8_t* __restrict E,
                                   const float invLenB[4],
                                   float distances[4] );

extern void
(**ppfcvSumOfSquaredDiffs36xNs8)( const int8_t* __restrict A,
                             float invLenA,
                             const int8_t* const * __restrict B,
                             const float* __restrict invLenB,
                             unsigned int numB,
                             float* __restrict distances );

extern void
(**ppfcvSort8Scoresf32)( float* __restrict inScores,
                         float* __restrict outScores );

extern void
(**ppfcvFilterThresholdu8_v2)
( const uint8_t* __restrict src,
  unsigned int srcWidth,
  unsigned int srcHeight,
  unsigned int srcStride,
  uint8_t* __restrict dst,
  unsigned int dstStride,
  unsigned int threshold 
);

extern void
(**ppfcvFilterDilate3x3u8_v2)
( const uint8_t* __restrict src,
  unsigned int srcWidth,
  unsigned int srcHeight,
  unsigned int srcStride,
  uint8_t* __restrict dst,
  unsigned int dstStride 
);

extern void
(**ppfcvFilterErode3x3u8_v2)
( const uint8_t* __restrict src,
  unsigned int srcWidth,
  unsigned int srcHeight,
  unsigned int srcStride,
  uint8_t* __restrict dst,
  unsigned int dstStride 
);

extern int
(**ppfcvTransformAffine8x8u8_v2)
(
   const uint8_t* __restrict   nImage,
   unsigned int imageWidth,
   unsigned int imageHeight,
   unsigned int imageStride,
   const int32_t* __restrict nPos,
   const int32_t* __restrict nAffine,
   uint8_t* __restrict patch,
   unsigned int patchStride
);

extern void
(**ppfcvWarpPerspectiveu8)
(
   const uint8_t* __restrict src,
   unsigned int srcwidth,
   unsigned int srcheight,
   uint8_t* __restrict dst,
   unsigned int dstwidth,
   unsigned int dstheight,
   float* __restrict kernel
);

extern void
(**ppfcvWarpPerspectiveu8_v2)
(
   const uint8_t* __restrict src,
   unsigned int srcWidth,
   unsigned int srcHeight,
   unsigned int srcStride,
   uint8_t* __restrict dst,
   unsigned int dstWidth,
   unsigned int dstHeight,
   unsigned int dstStride,
   float* __restrict kernel
);

extern void
(**ppfcv3ChannelWarpPerspectiveu8)
(
   const uint8_t* __restrict src,
   unsigned int srcwidth,
   unsigned int srcheight,
   uint8_t* __restrict dst,
   unsigned int dstwidth,
   unsigned int dstheight,
   float* __restrict kernel
);

extern void
(**ppfcv3ChannelWarpPerspectiveu8_v2)
(
   const uint8_t* __restrict src,
   unsigned int srcWidth,
   unsigned int srcHeight,
   unsigned int srcStride,
   uint8_t* __restrict dst,
   unsigned int dstWidth,
   unsigned int dstHeight,
   unsigned int dstStride,
   float* __restrict kernel
);

extern void
(**ppfcvFilterGaussian5x5s16_v2)
(
   const int16_t* __restrict srcImg,
   unsigned int srcWidth,
   unsigned int srcHeight,
   unsigned int srcStride,
   int16_t* __restrict dstImg,
   unsigned int dstStride,
   int blurBorder
);

extern void
(**ppfcvFilterGaussian5x5s32_v2)
(
   const int32_t* __restrict srcImg,
   unsigned int srcWidth,
   unsigned int srcHeight,
   unsigned int srcStride,
   int32_t* __restrict dstImg,
   unsigned int dstStride,
   int blurBorder
);

extern int
(**ppfcvTransformAffineu8)
(
   const uint8_t* __restrict  nImage,
   unsigned int imageWidth,
   unsigned int imageHeight,
   const float nPos[ 2 ],
   const float nAffine[ 4 ],
   uint8_t* __restrict nPatch,
   unsigned int patchWidth,
   unsigned int patchHeight
);

extern int
(**ppfcvTransformAffineu8_v2)
(
   const uint8_t* __restrict  nImage,
   unsigned int imageWidth,
   unsigned int imageHeight,
   unsigned int imageStride,
   const float nPos[ 2 ],
   const float nAffine[ 4 ],
   uint8_t* __restrict nPatch,
   unsigned int patchWidth,
   unsigned int patchHeight,
   unsigned int patchStride
);

extern void
(**ppfcvCopyRotated17x17u8)
(
   const uint8_t*region,
   uint8_t*patch,
   int nOri
 );

extern void
(**ppfcvCornerFast9Scoreu8)
(
   const uint8_t*im,
   unsigned int xsize,
   unsigned int ysize,
   int barrier,
   unsigned int border,
   uint32_t* xy,
   uint32_t* __restrict scores,
   unsigned int maxnumcorners,
   uint32_t* numcorners,
   unsigned int stride
);

extern void
(**ppfcvCornerFast9InMaskScoreu8)
(
   const uint8_t* __restrict im,
   unsigned int xsize,
   unsigned int ysize,
   int barrier,
   unsigned int border,
   uint32_t* __restrict xy,
   uint32_t* __restrict scores,
   unsigned int maxnumcorners,
   uint32_t* __restrict numcorners,
   unsigned int stride,const uint8_t* __restrict bitMask,
   unsigned int maskWidth,
   unsigned int maskHeight
);

extern void
(**ppfcvTrackLKOpticalFlowu8)
(
   const uint8_t* __restrict   src1,
   const uint8_t* __restrict   src2,
   unsigned int                               width,
   unsigned int                               height,
   const fcvPyramidLevel                 *src1Pyr,
   const fcvPyramidLevel                 *scr2Pyr,
   const fcvPyramidLevel                 *dx1Pyr,
   const fcvPyramidLevel                 *dy1Pyr,
   const float*                      featureXY,
   float*                            featureXY_out,
   int32_t*                              featureStatus,
   int                               featureLen,
   int                               windowWidth,
   int                               windowHeight,
   int                               maxIterations,
   int                               nPyramidLevels,
   float                             maxResidue,
   float                             minDisplacement,
   float                             minEigenvalue,
   int                               lightingNormalized
);

extern void
(**ppfcvTrackLKOpticalFlowf32)
(
   const uint8_t* __restrict   src1,
   const uint8_t* __restrict   src2,
   unsigned int                               width,
   unsigned int                               height,
   const fcvPyramidLevel                 *src1Pyr,
   const fcvPyramidLevel                 *scr2Pyr,
   const fcvPyramidLevel                 *dx1Pyr,
   const fcvPyramidLevel                 *dy1Pyr,
   const float*                      featureXY,
   float*                            featureXY_out,
   int32_t*                              featureStatus,
   int                               featureLen,
   int                               windowWidth,
   int                               windowHeight,
   int                               maxIterations,
   int                               nPyramidLevels,
   float                             maxResidue,
   float                             minDisplacement,
   float                             minEigenvalue,
   int                               lightingNormalized
);

extern int
(**ppfcvPyramidCreatef32)
(
   const float* __restrict base,
   unsigned int baseWidth,
   unsigned int baseHeight,
   unsigned int numLevels,
   fcvPyramidLevel* pyramid
);

extern int
(**ppfcvPyramidCreateu8)
(
   const uint8_t* __restrict base,
   unsigned int baseWidth,
   unsigned int baseHeight,
   unsigned int numLevels,
   fcvPyramidLevel * pyramid
);

extern int
(**ppfcvPyramidSobelGradientCreatei16)
(
   const fcvPyramidLevel * imgPyr,
   fcvPyramidLevel * dxPyr,
   fcvPyramidLevel * dyPyr,
   unsigned int numLevels
);

extern int
(**ppfcvPyramidSobelGradientCreatei8)
(
   const fcvPyramidLevel * imgPyr,
   fcvPyramidLevel * dxPyr,
   fcvPyramidLevel * dyPyr, unsigned int numLevels
);

extern int
(**ppfcvPyramidSobelGradientCreatef32)
(
   const fcvPyramidLevel * imgPyr,
   fcvPyramidLevel * dxPyr,
   fcvPyramidLevel * dyPyr,
   unsigned int numLevels
);

extern uint32_t
(**ppfcvBitCountu8)
(
   const uint8_t* __restrict src,
   unsigned int len
);

extern uint32_t
(**ppfcvBitCount32x1u8)
(
   const uint8_t* __restrict src
);

extern void
(**ppfcvBitCount32x4u8)
(
   const uint8_t* __restrict A,
   const uint8_t* __restrict B,
   const uint8_t* __restrict C,
   const uint8_t* __restrict D,
   uint32_t* __restrict  count
);

extern uint32_t
(**ppfcvBitCount64x1u8)
(
   const uint8_t* __restrict src
);

extern void
(**ppfcvBitCount64x4u8)
(
  const uint8_t* __restrict A,
  const uint8_t* __restrict B,
  const uint8_t* __restrict C,
  const uint8_t* __restrict D,
  uint32_t count[4]
);

extern uint32_t
(**ppfcvBitCountu32)
(
   const uint32_t* __restrict src,
   unsigned int len
);

extern uint32_t
(**ppfcvHammingDistanceu8)
(
   const uint8_t* __restrict a,
   const uint8_t* __restrict b,
   unsigned int len
);

extern uint32_t
(**ppfcvHammingDistance32x1u8a4)
(
   const uint8_t* __restrict a,
   const uint8_t* __restrict b
);

extern uint32_t
(**ppfcvHammingDistance64x1u8a4)
(
    const uint8_t* __restrict a,
    const uint8_t* __restrict b
);

extern uint32_t
(**ppfcvHammingDistance32x1u8)
(
    const uint8_t* __restrict a,
    const uint8_t* __restrict b
);

extern uint32_t
(**ppfcvHammingDistance64x1u8)
(
    const uint8_t* __restrict a,
    const uint8_t* __restrict b
);

extern void
(**ppfcvHammingDistance32x4u8a4)
(
    const uint8_t* __restrict A,
    const uint8_t* __restrict B,
    const uint8_t* __restrict C,
    const uint8_t* __restrict D,
    const uint8_t* __restrict E,
    uint32_t HamminDistances[4]
);

extern void
(**ppfcvHammingDistance64x4u8a4)
(
    const uint8_t* __restrict A,
    const uint8_t* __restrict B,
    const uint8_t* __restrict C,
    const uint8_t* __restrict D,
    const uint8_t* __restrict E,
    uint32_t HamminDistances[4]
);

extern void
(**ppfcvHammingDistance64x4u8)
(
    const uint8_t* __restrict A,
    const uint8_t* __restrict B,
    const uint8_t* __restrict C,
    const uint8_t* __restrict D,
    const uint8_t* __restrict E,
    uint32_t HamminDistances[4]
);

extern int
(**ppfcvTrackBMOpticalFlow16x16u8)
(
   const uint8_t* __restrict   src1,
   const uint8_t* __restrict   src2,
   uint32_t                    srcWidth,
   uint32_t                    srcHeight,
   uint32_t                    srcStride,
   uint32_t                    roiLeft,
   uint32_t                    roiTop,
   uint32_t                    roiRight,
   uint32_t                    roiBottom,
   uint32_t                    shiftSize,
   uint32_t                    searchWidth,
   uint32_t                    searchHeight,
   uint32_t                    searchStep,
   uint32_t                    usePrevious,
   uint32_t *                  numMv,
   uint32_t *                  locX,
   uint32_t *                  locY,
   uint32_t *                  mvX,
   uint32_t *                  mvY
);

extern int
(**ppfcvMserInit)
(
  const unsigned int width,
  const unsigned int height, 
  unsigned int delta, 
  unsigned int minArea ,
  unsigned int maxArea , 
  float maxVariation ,
  float minDiversity , void ** mserHandle 
);

extern void
(**ppfcvMserRelease) (void *mserHandle);

extern void 
(**ppfcvMseru8) 
( 
  void *mserHandle,
  const uint8_t* __restrict srcPtr,unsigned int  srcWidth, 
  unsigned int srcHeight, unsigned int srcStride, 
  unsigned int maxContours,
  unsigned int * __restrict numContours, unsigned int * __restrict numPointsInContour,
unsigned int pointsArraySize,
unsigned int* __restrict pointsArray 
);

extern void
(**ppfcvMserExtu8)( void *mserHandle,
                    const uint8_t* __restrict srcPtr,unsigned int srcWidth, 
                    unsigned int srcHeight, unsigned int srcStride, 
                    unsigned int maxContours,
                    unsigned int * __restrict numContours, unsigned int * __restrict numPointsInContour   ,
                    unsigned int* __restrict pointsArray, unsigned int pointsArraySize,
                    unsigned int * __restrict contourVariation,
                    int * __restrict contourPolarity,
                    unsigned int * __restrict contourNodeId,
                    unsigned int * __restrict contourNodeCounter
                  );


extern void
(**ppfcvBoundingRectangle)  
(
const uint32_t * __restrict xy, uint32_t numPoints, 
  uint32_t * rectTopLeftX, uint32_t * rectTopLeftY,
  uint32_t * rectWidth, uint32_t *rectHeight
);

extern void
(**ppfcvUpsampleVerticalu8)  
( 
  const uint8_t* __restrict src,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvUpsampleHorizontalu8)  
( 
  const uint8_t* __restrict src,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvUpsample2Du8) 
( 
  const uint8_t* __restrict src,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvUpsampleVerticalInterleavedu8)  
( 
  const uint8_t* __restrict src,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvUpsampleHorizontalInterleavedu8) 
( 
  const uint8_t* __restrict src,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvUpsample2DInterleavedu8) 
( 
  const uint8_t* __restrict src,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvColorYCbCr444PlanarToRGB565u8)
( 
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcCb,
  const uint8_t* __restrict srcCr,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCbStride,
  uint32_t                  srcCrStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvColorYCbCr444PlanarToRGB888u8)
( 
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcCb,
  const uint8_t* __restrict srcCr,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCbStride,
  uint32_t                  srcCrStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvColorYCbCr444PlanarToRGBA8888u8)
( 
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcCb,
  const uint8_t* __restrict srcCr,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCbStride,
  uint32_t                  srcCrStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvColorYCbCr422PlanarToRGB565u8)
( 
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcCb,
  const uint8_t* __restrict srcCr,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCbStride,
  uint32_t                  srcCrStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvColorYCbCr422PlanarToRGB888u8)
( 
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcCb,
  const uint8_t* __restrict srcCr,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCbStride,
  uint32_t                  srcCrStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvColorYCbCr422PlanarToRGBA8888u8)
( 
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcCb,
  const uint8_t* __restrict srcCr,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCbStride,
  uint32_t                  srcCrStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvColorYCbCr420PlanarToRGB565u8)
( 
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcCb,
  const uint8_t* __restrict srcCr,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCbStride,
  uint32_t                  srcCrStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvColorYCbCr420PlanarToRGB888u8)
( 
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcCb,
  const uint8_t* __restrict srcCr,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCbStride,
  uint32_t                  srcCrStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvColorYCbCr420PlanarToRGBA8888u8)
( 
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcCb,
  const uint8_t* __restrict srcCr,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCbStride,
  uint32_t                  srcCrStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvColorYCbCr444PseudoPlanarToRGB565u8)
(
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcC,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride
);

extern void
(**ppfcvColorYCbCr444PseudoPlanarToRGB888u8)
(
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcC,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride
);

extern void
(**ppfcvColorYCbCr444PseudoPlanarToRGBA8888u8)
(
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcC,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride
);

extern void
(**ppfcvColorYCbCr422PseudoPlanarToRGB565u8)
(
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcC,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride
);

extern void
(**ppfcvColorYCbCr422PseudoPlanarToRGB888u8)
(
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcC,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride
);

extern void
(**ppfcvColorYCbCr422PseudoPlanarToRGBA8888u8)
(
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcC,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride
);

extern void
(**ppfcvColorYCbCr420PseudoPlanarToRGB565u8)
(
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcC,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride
);

extern void
(**ppfcvColorYCbCr420PseudoPlanarToRGB888u8)
(
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcC,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride
);

extern void
(**ppfcvColorYCbCr420PseudoPlanarToRGBA8888u8)
(
  const uint8_t* __restrict srcY,
  const uint8_t* __restrict srcC,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcYStride,
  uint32_t                  srcCStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride
);

extern void
(**ppfcvEdgeWeightings16)
( 
  int16_t* __restrict edgeMap,        
  const uint32_t      edgeMapWidth,          
  const uint32_t      edgeMapHeight,         
  const uint32_t      edgeMapStride,        
  const uint32_t      weight,         
  const uint32_t      edge_limit,     
  const uint32_t      hl_threshold,   
  const uint32_t      hh_threshold,   
  const uint32_t      edge_denoise_factor 
);

extern void
(**ppfcvDeinterleaveu8)
( 
  const uint8_t* __restrict src,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  uint8_t* __restrict       dst0,
  uint32_t                  dst0Stride,
  uint8_t* __restrict       dst1,
  uint32_t                  dst1Stride 
);

extern void
(**ppfcvInterleaveu8)
(
  const uint8_t* __restrict src0,
  const uint8_t* __restrict src1,
  uint32_t                  imageWidth,
  uint32_t                  imageHeight,
  uint32_t                  src0Stride,
  uint32_t                  src1Stride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvDWTHaarTransposeu8)
(
  const uint8_t* __restrict src,                       
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  int16_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvDWT53TabTransposes16)
( 
  const int16_t* __restrict src,                       
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  int16_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvIDWT53TabTransposes16)
( 
  const int16_t*   __restrict src,                       
  uint32_t                    srcWidth,
  uint32_t                    srcHeight,
  uint32_t                    srcStride,
  int16_t* __restrict         dst,
  uint32_t                    dstStride 
);

extern void
(**ppfcvIDWTHaarTransposes16)
( 
  const int16_t* __restrict src,                       
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstStride 
);

extern void
(**ppfcvScaleUpPolyu8)
(
  const uint8_t* __restrict src,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstWidth,
  uint32_t                  dstHeight,
  uint32_t                  dstStride 
);   

extern void
(**ppfcvScaleUpPolyInterleaveu8)
( 
  const uint8_t* __restrict src,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstWidth,
  uint32_t                  dstHeight,
  uint32_t                  dstStride 
);  

extern void
(**ppfcvScaleDownMNu8)
( 
  const uint8_t* __restrict src,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstWidth,
  uint32_t                  dstHeight,
  uint32_t                  dstStride 
);

extern void
(**ppfcvScaleDownMNInterleaveu8)
(
  const uint8_t* __restrict src,
  uint32_t                  srcWidth,
  uint32_t                  srcHeight,
  uint32_t                  srcStride,
  uint8_t* __restrict       dst,
  uint32_t                  dstWidth,
  uint32_t                  dstHeight,
  uint32_t                  dstStride 
);

extern uint32_t
(**ppfcvKMeansTreeSearch36x10s8)
(
   const   int8_t* __restrict nodeChildrenCenter,
   const uint32_t* __restrict nodeChildrenInvLenQ32,
   const uint32_t* __restrict nodeChildrenIndex,
   const  uint8_t* __restrict nodeNumChildren,
         uint32_t             numNodes,
   const   int8_t* __restrict key
);

extern void
(**ppfcvLinearSearch8x36s8)
(
   const uint32_t * __restrict dbLUT,
   uint32_t                    numDBLUT,
   const int8_t   * __restrict descDB,
   const uint32_t * __restrict descDBInvLenQ38,
   const uint16_t * __restrict descDBTargetId,
   uint32_t                    numDescDB,
   const int8_t   * __restrict srcDesc,
   const uint32_t * __restrict srcDescInvLenQ38,
   const uint32_t * __restrict srcDescIdx,
   uint32_t                    numSrcDesc,
   const uint16_t * __restrict targetsToIgnore,
   uint32_t                    numTargetsToIgnore,
   uint32_t                    maxDistanceQ31,
   uint32_t       * __restrict correspondenceDBIdx,
   uint32_t       * __restrict correspondencSrcDescIdx,
   uint32_t       * __restrict correspondenceDistanceQ31,
   uint32_t                    maxNumCorrespondences,
   uint32_t       * __restrict numCorrespondences
);

extern int
(**ppfcvLinearSearchPrepare8x36s8_v2)
(
   uint32_t * __restrict dbLUT,
   uint32_t              numDBLUT,
   int8_t   * __restrict descDB,
   uint32_t * __restrict descDBInvLenQ38,
   uint16_t * __restrict descDBTargetId,
   uint32_t * __restrict idxLUT,
   uint32_t              numDescDB
);

extern void
(**ppfcvFindContoursExternalu8) ( uint8_t* __restrict   src,
                           uint32_t              srcWidth,
                           uint32_t              srcHeight,
                           uint32_t              srcStride,
                           uint32_t              maxNumContours,
                           uint32_t* __restrict  numContours,
                           uint32_t* __restrict  numContourPoints,
                           uint32_t** __restrict contourStartPoints,
                           uint32_t* __restrict  pointBuffer,
                           uint32_t              pointBufferSize,
                           int32_t               hierarchy[][4],
                           void*                 contourHandle );

extern void
(**ppfcvFindContoursListu8) ( uint8_t* __restrict   src,
                       uint32_t              srcWidth,
                       uint32_t              srcHeight,
                       uint32_t              srcStride,
                       uint32_t              maxNumContours,
                       uint32_t*__restrict   numContours,
                       uint32_t* __restrict  numContourPoints,
                       uint32_t** __restrict contourStartPoints,
                       uint32_t* __restrict  pointBuffer,
                       uint32_t              pointBufferSize,
                       void*                 contourHandle );

extern void
(**ppfcvFindContoursCcompu8)( uint8_t* __restrict   src,
                        uint32_t              srcWidth,
                        uint32_t              srcHeight,
                        uint32_t              srcStride,
                        uint32_t              maxNumContours,
                        uint32_t*__restrict   numContours,
                        uint32_t* __restrict  holeFlag,
                        uint32_t* __restrict  numContourPoints,
                        uint32_t** __restrict contourStartPoints,
                        uint32_t* __restrict  pointBuffer,
                        uint32_t              pointBufferSize,
                        int32_t               hierarchy[][4],
                        void*                 contourHandle );

extern void 
(**ppfcvFindContoursTreeu8) ( uint8_t* __restrict   src,
                       uint32_t              srcWidth,
                       uint32_t              srcHeight,
                       uint32_t              srcStride,
                       uint32_t              maxNumContours,
                       uint32_t* __restrict  numContours,
                       uint32_t* __restrict  holeFlag,
                       uint32_t* __restrict  numContourPoints,
                       uint32_t** __restrict contourStartPoints,
                       uint32_t* __restrict  pointBuffer,
                       uint32_t              pointBufferSize,
                       int32_t               hierarchy[][4],
                       void*                 contourHandle );

extern void *
(**ppfcvFindContoursAllocate) ( uint32_t srcStride );

extern void
(**ppfcvFindContoursDelete) ( void* contourHandle );

extern void 
(**ppfcvSolvef32) (const float32_t * __restrict A, 
            int32_t numRows, 
            int32_t numCols, 
            const float32_t * __restrict b, 
            float32_t * __restrict x);

extern void 
(**ppfcvGetPerspectiveTransformf32)( const float32_t* __restrict src,
                            const float32_t* __restrict dst,
                            float32_t* __restrict       transformCoefficient );


extern void
(**ppfcvSetElementsu8)(   uint8_t * __restrict src,
                           uint32_t             srcWidth, 
                           uint32_t             srcHeight,
                           uint32_t             srcStride, 
                           uint8_t              value,
                     const uint8_t * __restrict mask ,
                           uint32_t             maskStride  
                    );

extern void
(**ppfcvSetElementss32)(   int32_t * __restrict src, 
                            uint32_t             srcWidth, 
                            uint32_t             srcHeight,
                            uint32_t             srcStride, 
                            int32_t              value,
                      const uint8_t * __restrict mask ,
                            uint32_t             maskStride
                     );

extern  void
(**ppfcvSetElementsf32)(   float32_t * __restrict src, 
                            uint32_t               srcWidth, 
                            uint32_t               srcHeight,
                            uint32_t               srcStride, 
                            float32_t              value,
                      const uint8_t   * __restrict mask,
                            uint32_t               maskStride
                     );

extern  void
(**ppfcvSetElementsc4u8)(  uint8_t * __restrict src,
                           uint32_t             srcWidth, 
                           uint32_t             srcHeight,
                           uint32_t             srcStride, 
                           uint8_t              value1,
                           uint8_t              value2,
                           uint8_t              value3,
                           uint8_t              value4,
                     const uint8_t * __restrict mask,
                           uint32_t             maskStride
                    );

extern void
(**ppfcvSetElementsc4s32)(  int32_t * __restrict src, 
                            uint32_t             srcWidth, 
                            uint32_t             srcHeight,
                            uint32_t             srcStride, 
                            int32_t              value1,
                            int32_t              value2,
                            int32_t              value3,
                            int32_t              value4,
                      const uint8_t * __restrict mask,
                            uint32_t             maskStride 
                     );

extern void
(**ppfcvSetElementsc4f32)(  float32_t * __restrict src, 
                            uint32_t               srcWidth, 
                            uint32_t               srcHeight,
                            uint32_t               srcStride, 
                            float32_t              value1,
                            float32_t              value2,
                            float32_t              value3,
                            float32_t              value4,
                      const uint8_t   * __restrict mask,
                            uint32_t               maskStride
                     );
extern  void
(**ppfcvSetElementsc3u8)(  uint8_t * __restrict src,
                           uint32_t             srcWidth, 
                           uint32_t             srcHeight,
                           uint32_t             srcStride, 
                           uint8_t              value1,
                           uint8_t              value2,
                           uint8_t              value3,
                     const uint8_t * __restrict mask,
                           uint32_t             maskStride
                    );

extern void
(**ppfcvSetElementsc3s32)(  int32_t * __restrict src, 
                            uint32_t             srcWidth, 
                            uint32_t             srcHeight,
                            uint32_t             srcStride, 
                            int32_t              value1,
                            int32_t              value2,
                            int32_t              value3,
                      const uint8_t * __restrict mask,
                            uint32_t             maskStride 
                     );

extern void
(**ppfcvSetElementsc3f32)(  float32_t * __restrict src, 
                            uint32_t               srcWidth, 
                            uint32_t               srcHeight,
                            uint32_t               srcStride, 
                            float32_t              value1,
                            float32_t              value2,
                            float32_t              value3,
                      const uint8_t   * __restrict mask,
                            uint32_t               maskStride
                     );


extern void
(**ppfcvAdaptiveThresholdGaussian3x3u8)( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );
extern void
(**ppfcvAdaptiveThresholdGaussian5x5u8)( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );

extern void
(**ppfcvAdaptiveThresholdGaussian11x11u8)( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );

extern void
(**ppfcvAdaptiveThresholdMean3x3u8)( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );


extern void
(**ppfcvAdaptiveThresholdMean5x5u8)( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );

extern void
(**ppfcvAdaptiveThresholdMean11x11u8)( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride ); 
 

extern void 
(**ppfcvBoxFilter3x3u8)( const uint8_t* __restrict src, 
                         uint32_t            srcWidth,
                         uint32_t            srcHeight,
                         uint32_t            srcStride, 
                         uint8_t* __restrict dst, 
                         uint32_t            dstStride 
                   );



extern void 
(**ppfcvBoxFilter5x5u8)( const uint8_t* __restrict src, 
                         uint32_t            srcWidth,
                         uint32_t            srcHeight,
                         uint32_t            srcStride, 
                         uint8_t* __restrict dst, 
                         uint32_t            dstStride 
                   );


extern void 
(**ppfcvBoxFilter11x11u8)(const uint8_t* __restrict src, 
                          uint32_t            srcWidth,
                          uint32_t            srcHeight,
                          uint32_t            srcStride, 
                          uint8_t* __restrict dst, 
                          uint32_t            dstStride 
                   );
 
extern void 
(**ppfcvBilateralFilter5x5u8)(const uint8_t* __restrict src, 
                               uint32_t            srcWidth,
                               uint32_t            srcHeight,
                               uint32_t            srcStride, 
                               uint8_t* __restrict dst, 
                               uint32_t            dstStride 
                        );



extern void 
(**ppfcvBilateralFilter7x7u8)(const uint8_t* __restrict src, 
                               uint32_t            srcWidth,
                               uint32_t            srcHeight,
                               uint32_t            srcStride, 
                               uint8_t* __restrict dst, 
                               uint32_t            dstStride 
                    );


extern void 
(**ppfcvBilateralFilter9x9u8)(const uint8_t* __restrict src, 
                               uint32_t            srcWidth,
                               uint32_t            srcHeight,
                               uint32_t            srcStride, 
                               uint8_t* __restrict dst, 
                               uint32_t            dstStride 
                    );

extern void
(**ppfcvSegmentFGMasku8)(uint8_t* __restrict    src,
                        uint32_t               srcWidth,
                        uint32_t               srcHeight,
                        uint32_t               srcStride,
                        uint8_t                Polygonal,
                        uint32_t              perimScale);


extern void
(**ppfcvAbsDiffu8)(const uint8_t * __restrict src1, 
                   const uint8_t * __restrict src2,   
                   uint32_t             srcWidth,
                   uint32_t             srcHeight,
                   uint32_t             srcStride,
                   uint8_t * __restrict dst,
                   uint32_t             dstStride );



extern void
(**ppfcvAbsDiffs32)(const int32_t * __restrict  src1, 
              const int32_t * __restrict  src2,   
                    uint32_t              srcWidth,
                    uint32_t              srcHeight,
                    uint32_t              srcStride,
                    int32_t * __restrict  dst,
                    uint32_t              dstStride );



extern void
(**ppfcvAbsDifff32)(const float32_t * __restrict  src1, 
              const float32_t * __restrict  src2,    
                    uint32_t                srcWidth,
                    uint32_t                srcHeight,
                    uint32_t                srcStride,
                    float32_t * __restrict  dst,
                    uint32_t                dstStride );


extern void
(**ppfcvAbsDiffVu8)(const uint8_t * __restrict src, 
                    uint8_t              value,   
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
                    uint8_t * __restrict dst,
                    uint32_t             dstStride );


extern void
(**ppfcvAbsDiffVs32)(const int32_t * __restrict src, 
                     int32_t              value, 
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
                     int32_t * __restrict dst,
                     uint32_t             dstStride );

extern void
(**ppfcvAbsDiffVf32)(const float32_t * __restrict src, 
                     float32_t              value,   
                     uint32_t               srcWidth,
                     uint32_t               srcHeight,
                     uint32_t               srcStride,
                     float32_t * __restrict dst,
                     uint32_t               dstStride );


extern void
(**ppfcvAbsDiffVc4u8)(const uint8_t * __restrict src, 
                    uint8_t              value1,
                    uint8_t              value2,
                    uint8_t              value3,
                    uint8_t              value4,
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
                    uint8_t * __restrict dst,
                    uint32_t             dstStride );


extern void
(**ppfcvAbsDiffVc4s32)(const int32_t * __restrict src, 
                     int32_t              value1,
                     int32_t              value2,
                     int32_t              value3,
                     int32_t              value4,
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
                     int32_t * __restrict dst,
                     uint32_t             dstStride );

extern void
(**ppfcvAbsDiffVc4f32)(const float32_t * __restrict src, 
                     float32_t              value1,   
                     float32_t              value2,
                     float32_t              value3,
                     float32_t              value4,
                     uint32_t               srcWidth,
                     uint32_t               srcHeight,
                     uint32_t               srcStride,
                     float32_t * __restrict dst,
                     uint32_t               dstStride);

extern void
(**ppfcvAbsDiffVc3u8)(const uint8_t * __restrict src, 
                    uint8_t              value1,
                    uint8_t              value2,
                    uint8_t              value3,
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
                    uint8_t * __restrict dst,
                    uint32_t             dstStride );

extern void
(**ppfcvAbsDiffVc3s32)(const int32_t * __restrict src, 
                     int32_t              value1,
                     int32_t              value2,
                     int32_t              value3,
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
                     int32_t * __restrict dst,
                     uint32_t             dstStride );

extern void
(**ppfcvAbsDiffVc3f32)(const float32_t * __restrict src, 
                     float32_t              value1,   
                     float32_t              value2,
                     float32_t              value3,
                     uint32_t               srcWidth,
                     uint32_t               srcHeight,
                     uint32_t               srcStride,
                     float32_t * __restrict dst,
                     uint32_t               dstStride);

extern
int (**ppfcvKDTreeCreate36s8f32)
( const        int8_t*  __restrict vectors,
  const     float32_t*  __restrict invLengths,
                  int              numVectors,
   fcvKDTreeDatas8f32**            kdtrees 
);

extern
int (**ppfcvKDTreeDestroy36s8f32)
( fcvKDTreeDatas8f32* kdtrees 
);

extern
int (**ppfcvKDTreeQuery36s8f32)
( fcvKDTreeDatas8f32*       kdtrees,
       const  int8_t* __restrict query,
           float32_t             queryInvLen,
                 int             maxNNs,
           float32_t             maxDist,
                 int             maxChecks,
       const uint8_t* __restrict mask,
             int32_t*             numNNsFound,
             int32_t* __restrict NNInds,
           float32_t* __restrict NNDists 
);

extern void (**ppfcvBitwiseOru8)
(
 	const uint8_t* __restrict src1, 
	const uint8_t* __restrict src2,  
	uint32_t                  srcWidth,
	uint32_t                  srcHeight,
	uint32_t                  srcStride,
	uint8_t * __restrict      dst,
	uint32_t                  dstStride,
	uint8_t * __restrict      mask,
	uint32_t                  maskStride 
);

extern void 
(**ppfcvBitwiseOrs32)
(
 	const int32_t* __restrict src1, 
	const int32_t* __restrict src2,  
	uint32_t                  srcWidth,
	uint32_t                  srcHeight,
	uint32_t                  srcStride,
	int32_t * __restrict      dst,
	uint32_t                  dstStride,
	uint8_t * __restrict      mask,
	uint32_t                  maskStride
);

extern void 
(**ppfcvColorRGB888ToGrayu8)
( 
 	const uint8_t* __restrict src,
	uint32_t 			 srcWidth,
	uint32_t 			srcHeight,
	uint32_t 			srcStride,
	uint8_t* 	   __restrict dst,
	uint32_t  			dstStride
);

extern void 
(**ppfcvTiltedIntegralu8s32)
( 
 	const uint8_t* __restrict src,
	uint32_t 			 srcWidth,
	uint32_t 			srcHeight,
	uint32_t 			srcStride,
	int32_t* __restrict 	  dst,
	uint32_t 			dstStride
);

extern void 
(**ppfcvConvValids16)
( 
	const int16_t* __restrict src1,
	uint32_t 			  src1Width,
	uint32_t 			 src1Height,
	uint32_t            src1Stride,
	const int16_t* __restrict src2,
	uint32_t 			  src2Width,
	uint32_t 			 src2Height,
	uint32_t 			 src2Stride,
	int32_t* __restrict 		dst,
	uint32_t 			  dstStride
);


//==============================================================================
// Function Definitions
//==============================================================================


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterMedian3x3u8( const uint8_t* __restrict src,
                       unsigned int width,
                       unsigned int height,
                       uint8_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0x7) == 0 );     // 64-bit alignment
   fcvAssert( ((int)dst & 0x7) == 0 );     // 64-bit alignment
   fcvAssert( (width & 0x7) == 0 );        // multiple of 8
#endif

   return (**ppfcvFilterMedian3x3u8_v2)( src, width, height, width, dst, width );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterMedian3x3u8_v2( const uint8_t* __restrict src,
                         unsigned int srcWidth,
                         unsigned int srcHeight,
                         unsigned int srcStride,
                         uint8_t* __restrict dst,
                         unsigned int dstStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);
   
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );   // Stride is at least as much as Width
#endif

   return (**ppfcvFilterMedian3x3u8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterGaussian3x3u8( const uint8_t* __restrict src,
                         unsigned int width,
                         unsigned int height,
                         uint8_t* __restrict dst,
                         int border )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );           // multiple of 8
#endif
   fcvAssert( ((width - 2)/6)*6 <= (width - 2) &&
              ((width - 2)/6)*6 >= ((width - 2) - 5) );

   return (**ppfcvFilterGaussian3x3u8_v2)( src, width, height, width, dst, 
                                        width, border );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterGaussian3x3u8_v2( const uint8_t* __restrict src,
                           unsigned int srcWidth,
                           unsigned int srcHeight,
                           unsigned int srcStride,
                           uint8_t* __restrict dst,
                           unsigned int dstStride,
                           int border )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src  & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst  & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth  & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );   // Stride is at least as much as Width
#endif
   fcvAssert( ((srcWidth - 2)/6)*6 <= (srcWidth - 2) &&
              ((srcWidth - 2)/6)*6 >= ((srcWidth - 2) - 5) );

   return (**ppfcvFilterGaussian3x3u8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride, border );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterGaussian5x5u8( const uint8_t* __restrict src,
                        unsigned int width,
                        unsigned int height,
                        uint8_t* __restrict dst,
                        int border )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );           // multiple of 8
#endif

   return (**ppfcvFilterGaussian5x5u8_v2)( src, width, height, width, dst, width, 
                                        border );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterGaussian5x5u8_v2( const uint8_t* __restrict src,
                           unsigned int srcWidth,
                           unsigned int srcHeight,
                           unsigned int srcStride,
                           uint8_t* __restrict dst,
                           unsigned int dstStride,
                           int border )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );   // Stride is at least as much as Width
#endif

   return (**ppfcvFilterGaussian5x5u8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride, border );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterGaussian11x11u8( const uint8_t* __restrict src,
                          unsigned int width,
                          unsigned int height,
                          uint8_t* __restrict dst,
                          int blurBorder )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );           // multiple of 8
#endif

   return (**ppfcvFilterGaussian11x11u8_v2)( src, width, height, width, dst, 
                                          width, blurBorder );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterGaussian11x11u8_v2( const uint8_t* __restrict src,
                             unsigned int srcWidth,
                             unsigned int srcHeight,
                             unsigned int srcStride,
                             uint8_t* __restrict dst,
                             unsigned int dstStride,
                             int blurBorder )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );   // Stride is at least as much as Width
#endif

   return(**ppfcvFilterGaussian11x11u8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride, blurBorder );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYUV420toRGB8888u8
(
   const uint8_t* __restrict src,
   unsigned int                       width,
   unsigned int                       height,
   uint32_t* __restrict dst
)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );           // multiple of 8
#endif

   return (**ppfcvColorYCrCb420PseudoPlanarToRGB8888u8)( src, width, height, width, width, dst, width * sizeof(uint32_t) );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCrCb420PseudoPlanarToRGB8888u8( const uint8_t* __restrict src,
                                         unsigned int              srcWidth,
                                         unsigned int              srcHeight,
                                         unsigned int              srcYStride,
                                         unsigned int              srcCStride,
                                         uint32_t* __restrict      dst,
                                         unsigned int              dstStride )
{
   srcYStride = (srcYStride==0 ? srcWidth   : srcYStride);
   srcCStride = (srcCStride==0 ? srcWidth   : srcCStride);
   dstStride  = (dstStride==0  ? srcWidth*4 : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcYStride & 0x7) == 0 );   // multiple of 8
   fcvAssert( (srcCStride & 0x7) == 0 );   // multiple of 4
   fcvAssert( (dstStride & 31) == 0 );     // multiple of 32
   fcvAssert( (srcYStride >= srcWidth) );  // Y-stride is at least as much as srcWidth
   fcvAssert( (srcCStride >= srcWidth) );  // Y-stride is at least as much as srcWidth/2
   fcvAssert( (dstStride >= srcWidth*4) ); // Dst-stride is at least as much as srcWidth*3
#endif

   return (**ppfcvColorYCrCb420PseudoPlanarToRGB8888u8)( src, srcWidth, srcHeight, 
                                                         srcYStride, srcCStride,
                                                         dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYUV420toRGB565u8
(
   const uint8_t* __restrict src,
   unsigned int                       width,
   unsigned int                       height,
   uint32_t* __restrict dst
)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x3) == 0 );           // multiple of 4
#endif

   return (**ppfcvColorYUV420toRGB565u8)( src, width, height, dst );
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvColorYCrCbH1V1toRGB888u8
(
   const uint8_t* __restrict src,
   unsigned int                       width,
   unsigned int                       height,
   uint8_t* __restrict dst
)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );  // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );  // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );     // multiple of 8
#endif

   return (**ppfcvColorYCrCbH1V1toRGB888u8)( src, width, height, dst );
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvColorYCrCbH2V2toRGB888u8
(
   const uint8_t* __restrict y_src,
   unsigned int                       width,
   unsigned int                       height,
   uint8_t* __restrict dst
)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)y_src & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );        // multiple of 8
#endif

   return (**ppfcvColorYCrCbH2V2toRGB888u8)( y_src, width, height, dst );
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvColorRGB888toYCrCbu8_v2
(
   const uint8_t* __restrict src,
   unsigned int              srcWidth,
   unsigned int              srcHeight,
   unsigned int              srcStride,
   uint8_t* __restrict       dst,
   unsigned int              dstStride
)
{
   srcStride = (srcStride==0 ? srcWidth*3 : srcStride);
   dstStride = (dstStride==0 ? srcWidth*3 : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride >= srcWidth*3) ); // multiple of 8
   fcvAssert( (dstStride >= srcWidth*3) ); // multiple of 8
#endif

   return (**ppfcvColorRGB888toYCrCbu8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvColorRGB888toYCrCbu8
(
   const uint8_t* __restrict src,
   unsigned int                       width,
   unsigned int                       height,
   uint8_t* __restrict dst
)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );           // multiple of 8
#endif

   return (**ppfcvColorRGB888toYCrCbu8_v2)( src, width, height, width*3, dst, width*3 );
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvColorYCrCbH2V1toRGB888u8
(
   const uint8_t* __restrict src,
   unsigned int                       width,
   unsigned int                       height,
   uint8_t* __restrict dst
)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );           // multiple of 8
#endif

   return (**ppfcvColorYCrCbH2V1toRGB888u8)( src, width, height, dst );
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvColorYCrCbH1V2toRGB888u8
(
   const uint8_t* __restrict ysrc,
   unsigned int                       width,
   unsigned int                       height,
   uint8_t* __restrict dst
)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT

   fcvAssert( ((int)ysrc & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );        // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );        // multiple of 8
#endif

   return (**ppfcvColorYCrCbH1V2toRGB888u8)( ysrc, width, height, dst );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvDescriptor17x17u8To36s8( const uint8_t* __restrict patch,
                             int8_t* __restrict descriptorChar,
                             int32_t*  __restrict descriptorNormSq )
{
   return (**ppfcvDescriptor17x17u8To36s8)(patch, descriptorChar, descriptorNormSq);
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int32_t
fcvDotProducts8( const int8_t* __restrict a,
                 const int8_t* __restrict b,
                 unsigned int abSize )
{
   return (**ppfcvDotProducts8)( a, b, abSize );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline uint32_t
fcvDotProductu8( const uint8_t* __restrict a,
                  const uint8_t* __restrict b,
                  unsigned int abSize )
{
   return (**ppfcvDotProductu8)( a, b, abSize );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int32_t
fcvDotProduct36x1s8( const int8_t* __restrict a,
                      const int8_t* __restrict b )
{
   return (**ppfcvDotProduct36x1s8)( a, b );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProduct36x4s8( const int8_t* __restrict A,
                      const int8_t* __restrict B,
                      const int8_t* __restrict C,
                      const int8_t* __restrict D,
                      const int8_t* __restrict E,
                      int32_t* __restrict dotProducts )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
#endif

   (**ppfcvDotProduct36x4s8)( A, B, C, D, E, dotProducts );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProductNorm36x4s8(  const int8_t* __restrict A,
                           float                    invLengthA,
                           const int8_t* __restrict vB0,
                           const int8_t* __restrict vB1,
                           const int8_t* __restrict vB2,
                           const int8_t* __restrict vB3,
                           float* __restrict        invLengthsB,
                           float* __restrict        dotProducts  )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)invLengthsB & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( sizeof(*invLengthsB) == 4 );
   fcvAssert( sizeof(*dotProducts) == 4 );
#endif

   (**ppfcvDotProductNorm36x4s8)( A, invLengthA, vB0, vB1, vB2, vB3, invLengthsB,
                              dotProducts );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline uint32_t
fcvDotProduct36x1u8( const uint8_t* __restrict a,
                      const uint8_t* __restrict b )
{
   return (**ppfcvDotProduct36x1u8)( a, b );
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProduct36x4u8( const uint8_t* __restrict A,
                      const uint8_t* __restrict B,
                      const uint8_t* __restrict C,
                      const uint8_t* __restrict D,
                      const uint8_t* __restrict E,
                      uint32_t* __restrict      dotProducts )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
#endif

   (**ppfcvDotProduct36x4u8)( A, B, C, D, E, dotProducts );
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProductNorm36x4u8(  const uint8_t* __restrict  A,
                           float                      invLengthA,
                           const uint8_t* __restrict  vB0,
                           const uint8_t* __restrict  vB1,
                           const uint8_t* __restrict  vB2,
                           const uint8_t* __restrict  vB3,
                           float* __restrict          invLengthsB,
                           float* __restrict          dotProducts )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)invLengthsB & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( sizeof(*invLengthsB) == 4 );
   fcvAssert( sizeof(*dotProducts) == 4 );
#endif

   (**ppfcvDotProductNorm36x4u8)( A, invLengthA, vB0, vB1, vB2, vB3, invLengthsB,
                              dotProducts );
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int32_t
fcvDotProduct64x1s8( const int8_t* __restrict a,
                      const int8_t* __restrict b )
{
   return (**ppfcvDotProduct64x1s8)( a, b );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProduct64x4s8( const int8_t* __restrict A,
                      const int8_t* __restrict B,
                      const int8_t* __restrict C,
                      const int8_t* __restrict D,
                      const int8_t* __restrict E,
                      int32_t* __restrict      dotProducts )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
#endif

   (**ppfcvDotProduct64x4s8)( A, B, C, D, E, dotProducts );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProductNorm64x4s8(  const int8_t* __restrict A,
                           float                    invLengthA,
                           const int8_t* __restrict vB0,
                           const int8_t* __restrict vB1,
                           const int8_t* __restrict vB2,
                           const int8_t* __restrict vB3,
                           float* __restrict        invLengthsB,
                           float* __restrict        dotProducts  )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)invLengthsB & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( sizeof(*invLengthsB) == 4 );
   fcvAssert( sizeof(*dotProducts) == 4 );
#endif

   (**ppfcvDotProductNorm64x4s8)( A, invLengthA,
                              vB0,
                              vB1,
                              vB2,
                              vB3, invLengthsB,
                              dotProducts );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline uint32_t
fcvDotProduct64x1u8( const uint8_t* __restrict a,
                      const uint8_t* __restrict b )
{
   return (**ppfcvDotProduct64x1u8)( a, b );
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProduct64x4u8( const uint8_t* __restrict A,
                      const uint8_t* __restrict B,
                      const uint8_t* __restrict C,
                      const uint8_t* __restrict D,
                      const uint8_t* __restrict E,
                      uint32_t* __restrict      dotProducts )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
#endif

   (**ppfcvDotProduct64x4u8)( A, B, C, D, E, dotProducts );
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProductNorm64x4u8(  const uint8_t* __restrict  A,
                           float                  invLengthA,
                           const uint8_t* __restrict  vB0,
                           const uint8_t* __restrict  vB1,
                           const uint8_t* __restrict  vB2,
                           const uint8_t* __restrict  vB3,
                           float* __restrict      invLengthsB,
                           float* __restrict      dotProducts )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)invLengthsB & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( sizeof(*invLengthsB) == 4 );
   fcvAssert( sizeof(*dotProducts) == 4 );
#endif

   (**ppfcvDotProductNorm64x4u8)( A, invLengthA,
                              vB0,
                              vB1,
                              vB2,
                              vB3, invLengthsB,
                              dotProducts );
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int32_t
fcvDotProduct128x1s8( const int8_t* __restrict a,
                       const int8_t* __restrict b )
{
   return (**ppfcvDotProduct128x1s8)( a, b );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProduct128x4s8( const int8_t* __restrict A,
                       const int8_t* __restrict B,
                       const int8_t* __restrict C,
                       const int8_t* __restrict D,
                       const int8_t* __restrict E,
                       int32_t* __restrict      dotProducts )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
#endif

   (**ppfcvDotProduct128x4s8)( A, B, C, D, E, dotProducts );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProductNorm128x4s8(  const int8_t* __restrict A,
                            float                invLengthA,
                            const int8_t* __restrict vB0,
                            const int8_t* __restrict vB1,
                            const int8_t* __restrict vB2,
                            const int8_t* __restrict vB3,
                            float* __restrict    invLengthsB,
                            float* __restrict    dotProducts  )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)invLengthsB & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( sizeof(*invLengthsB) == 4 );
   fcvAssert( sizeof(*dotProducts) == 4 );
#endif

   (**ppfcvDotProductNorm128x4s8)( A, invLengthA, vB0, vB1, vB2, vB3, invLengthsB,
                              dotProducts );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline uint32_t
fcvDotProduct128x1u8( const uint8_t* __restrict a,
                       const uint8_t* __restrict b )
{
   return (**ppfcvDotProduct128x1u8)( a, b );
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProduct128x4u8( const uint8_t* __restrict A,
                       const uint8_t* __restrict B,
                       const uint8_t* __restrict C,
                       const uint8_t* __restrict D,
                       const uint8_t* __restrict E,
                       uint32_t* __restrict      dotProducts )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
#endif

   (**ppfcvDotProduct128x4u8)( A, B, C, D, E, dotProducts );
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProductNorm128x4u8(  const uint8_t* __restrict  A,
                            float                  invLengthA,
                            const uint8_t* __restrict  vB0,
                            const uint8_t* __restrict  vB1,
                            const uint8_t* __restrict  vB2,
                            const uint8_t* __restrict  vB3,
                            float* __restrict      invLengthsB,
                            float* __restrict      dotProducts )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)invLengthsB & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( sizeof(*invLengthsB) == 4 );
   fcvAssert( sizeof(*dotProducts) == 4 );
#endif

   (**ppfcvDotProductNorm128x4u8)( A, invLengthA, vB0, vB1, vB2, vB3, invLengthsB,
                              dotProducts );
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProduct8x8u8( const uint8_t* __restrict ptch,
                     const uint8_t* __restrict img,
                     unsigned short imgW, unsigned short imgH,
                     int nX, int nY, unsigned int nNum, int32_t* __restrict dotProducts )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
#endif

   (**ppfcvDotProduct8x8u8)( ptch, img, imgW, imgH, nX, nY, nNum, dotProducts );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDotProduct11x12u8( const uint8_t* __restrict ptch,
                       const uint8_t* __restrict img,
                       unsigned short imgW, unsigned short imgH,
                       int iX, int iY,
                       int32_t* __restrict dotProducts )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)dotProducts & 0xF) == 0 );     // 128-bit alignment
#endif

   (**ppfcvDotProduct11x12u8)( ptch, img, imgW, imgH, iX, iY, dotProducts );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterSobel3x3u8( const uint8_t* __restrict src,
                      unsigned int width,
                      unsigned int height,
                      uint8_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );           // multiple of 8
#endif

   return (**ppfcvFilterSobel3x3u8_v2)( src, width, height, width, dst, width );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterSobel3x3u8_v2( const uint8_t* __restrict src,
                        unsigned int srcWidth,
                        unsigned int srcHeight,
                        unsigned int srcStride,
                        uint8_t* __restrict dst,
                        unsigned int dstStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );   // Stride is at least as much as Width
#endif

   return (**ppfcvFilterSobel3x3u8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCanny3x3u8( const uint8_t* __restrict src,
                      unsigned int width,
                      unsigned int height,
                      uint8_t* __restrict dst,
                      int low,
                      int high )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );           // multiple of 8
#endif
   fcvAssert( (low >=0 ) && (low <= 255) );
   fcvAssert( (high >=0 ) && (high <= 255) );
   fcvAssert( low <= high );

   return (**ppfcvFilterCanny3x3u8_v2)( src, width, height, width, dst, width, low, high );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCanny3x3u8_v2( const uint8_t* __restrict src,
                        unsigned int srcWidth,
                        unsigned int srcHeight,
                        unsigned int srcStride,
                        uint8_t* __restrict dst,
                        unsigned int dstStride,
                        int low,
                        int high )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // at least as much as width
#endif
   fcvAssert( (low >=0 ) && (low <= 255) );
   fcvAssert( (high >=0 ) && (high <= 255) );
   fcvAssert( low <= high );

   return (**ppfcvFilterCanny3x3u8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride, low, high );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageDiffu8( const uint8_t* __restrict src1,
                const uint8_t* __restrict src2,
                 unsigned int             srcWidth,
                 unsigned int             srcHeight,
                      uint8_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src1 & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)src2 & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   return(**ppfcvImageDiffu8_v2)( src1, src2, srcWidth, srcHeight, srcWidth, dst,
                               srcWidth );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageDiffu8_v2( const uint8_t* __restrict src1,
                   const uint8_t* __restrict src2,
                   unsigned int              srcWidth,
                   unsigned int              srcHeight,
                   unsigned int              srcStride,
                   uint8_t* __restrict       dst,
                   unsigned int              dstStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src1 & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)src2 & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );   // Stride is at least as much as Width
#endif

   return(**ppfcvImageDiffu8_v2)( src1, src2, srcWidth, srcHeight, srcStride, dst, dstStride );
}

inline void 
fcvImageDiffs16( const int16_t* __restrict src1,
                 const int16_t* __restrict src2,
                  unsigned int             srcWidth, 
                  unsigned int             srcHeight, 
                  unsigned int             srcStride, 
                       int16_t* __restrict dst,
                  unsigned int             dstStride )
{
   srcStride = (srcStride==0 ? (srcWidth * sizeof (int16_t)) : srcStride);
   dstStride = (dstStride==0 ? (srcWidth * sizeof (int16_t)) : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src1 & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)src2 & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );   // Stride is at least as much as Width
#endif

   return(**ppfcvImageDiffs16_v2)( src1, src2, srcWidth, srcHeight, srcStride, dst, dstStride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void 
fcvImageDifff32(  const float* __restrict src1,
                  const float* __restrict src2,
                 unsigned int             srcWidth, 
                 unsigned int             srcHeight, 
                 unsigned int             srcStride, 
                        float* __restrict dst,
                 unsigned int             dstStride )
{
   srcStride = (srcStride==0 ? (srcWidth * sizeof (float)) : srcStride);
   dstStride = (dstStride==0 ? (srcWidth * sizeof (float)) : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src1 & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)src2 & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );    // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );    // Stride is at least as much as Width
#endif

  return (**ppfcvImageDifff32_v2)( src1, src2, srcWidth, srcHeight, srcStride, 
                                   dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void 
fcvImageDiffu8f32( const uint8_t* __restrict src1,
                   const uint8_t* __restrict src2,
                    unsigned int             srcWidth, 
                    unsigned int             srcHeight, 
                    unsigned int             srcStride, 
                           float* __restrict dst,
                    unsigned int             dstStride )
{

   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src1 & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)src2 & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );    // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );    // Stride is at least as much as Width
#endif

  return (**ppfcvImageDiffu8f32_v2)( src1, src2, srcWidth, srcHeight, srcStride, 
                                     dst, dstStride );

}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void 
fcvImageDiffu8s8( const uint8_t* __restrict src1,
                  const uint8_t* __restrict src2,
                   unsigned int             srcWidth, 
                   unsigned int             srcHeight, 
                   unsigned int             srcStride, 
                         int8_t* __restrict dst,
                   unsigned int             dstStride )
{

   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src1 & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)src2 & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );    // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );    // Stride is at least as much as Width
#endif

  return (**ppfcvImageDiffu8s8_v2)( src1, src2, srcWidth, srcHeight, srcStride, 
                                    dst, dstStride );

}



//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientInterleaveds16( const uint8_t* __restrict src,
                                unsigned int              srcWidth,
                                unsigned int              srcHeight,
                                unsigned int              srcStride,
                                int16_t* __restrict       gradients
                              )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)gradients & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
#endif

   return(**ppfcvImageGradientInterleaveds16_v2)(src,srcWidth,srcHeight,srcStride,gradients,(srcWidth-2)*2*sizeof(int16_t));
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientInterleaveds16_v2( const uint8_t* __restrict src,
                                   unsigned int              srcWidth,
                                   unsigned int              srcHeight,
                                   unsigned int              srcStride,
                                   int16_t* __restrict       gradients,
                                   unsigned int              gradStride )
{
   srcStride  = (srcStride==0 ? srcWidth : srcStride);
   gradStride = (gradStride==0 ? srcWidth*2*sizeof(int16_t) : gradStride); //4*width because 2 grad values, 2 bytes each.

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );          // 128-bit alignment
   fcvAssert( ((int)gradients & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );          // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );         // multiple of 8
   fcvAssert( (gradStride & 15) == 0 );         // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );        // at least as much as width
   fcvAssert( (gradStride >= (srcWidth*2*sizeof(int16_t))) );   // at least as much as 4*width
#endif

   return(**ppfcvImageGradientInterleaveds16_v2)(src,srcWidth,srcHeight,srcStride,gradients,gradStride);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientInterleavedf32( const uint8_t* __restrict src,
                                unsigned int              srcWidth,
                                unsigned int              srcHeight,
                                unsigned int              srcStride,
                                float* __restrict         gradients )

{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)gradients & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
   fcvAssert( sizeof(*gradients) == 4 );
#endif

   return(**ppfcvImageGradientInterleavedf32_v2)(src,srcWidth,srcHeight,srcStride,gradients,(srcWidth-2)*2*sizeof(float));
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientInterleavedf32_v2( const uint8_t* __restrict src,
                                   unsigned int              srcWidth,
                                   unsigned int              srcHeight,
                                   unsigned int              srcStride,
                                   float* __restrict         gradients,
                                   unsigned int              gradStride )

{
   srcStride  = (srcStride==0 ? srcWidth : srcStride);
   gradStride = (gradStride==0 ? srcWidth*2*sizeof(float) : gradStride); 

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );          // 128-bit alignment
   fcvAssert( ((int)gradients & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (srcWidth & 7) == 0 );            // multiple of 8
   fcvAssert( (srcStride & 7) == 0 );           // multiple of 8
   fcvAssert( (gradStride & 31) == 0 );         // multiple of 32
   fcvAssert( (srcStride >= srcWidth) );        // at least as much as width
   fcvAssert( (gradStride >= (srcWidth*2*sizeof(float))) );   // at least as much as 8*width
#endif

   return(**ppfcvImageGradientInterleavedf32_v2)(src,srcWidth,srcHeight,srcStride,gradients,gradStride);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientPlanars16( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           unsigned int              srcStride,
                           int16_t* __restrict       dx,
                           int16_t* __restrict       dy )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dx & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dy & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
#endif

   return(**ppfcvImageGradientPlanars16_v2)(src,srcWidth,srcHeight,srcStride,dx,dy,srcWidth*sizeof(int16_t));
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientPlanars16_v2( const uint8_t* __restrict src,
                              unsigned int              srcWidth,
                              unsigned int              srcHeight,
                              unsigned int              srcStride,
                              int16_t* __restrict       dx,
                              int16_t* __restrict       dy,
                              unsigned int              dxyStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dxyStride = (dxyStride==0 ? (srcWidth<<1) : dxyStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );        // 128-bit alignment
   fcvAssert( ((int)dx & 0xF) == 0 );         // 128-bit alignment
   fcvAssert( ((int)dy & 0xF) == 0 );         // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );       // multiple of 8
   fcvAssert( (dxyStride & 0xF) == 0 );       // multiple of 16
   fcvAssert( (srcStride >= srcWidth) );      // srcStride should be at least as much as srcWidth
   fcvAssert( (dxyStride >= (srcWidth<<1)) ); // dxyStride should be at least twice as much as srcWidth
#endif

   return(**ppfcvImageGradientPlanars16_v2)(src,srcWidth,srcHeight,srcStride,dx,dy,dxyStride);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientPlanarf32( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           unsigned int              srcStride,
                           float* __restrict         dx,
                           float* __restrict         dy )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dx & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dy & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
   fcvAssert( sizeof(*dx) == 4 );
   fcvAssert( sizeof(*dy) == 4 );
#endif

   return(**ppfcvImageGradientPlanarf32_v2)(src,srcWidth,srcHeight,srcStride,dx,dy,srcWidth*sizeof(float));
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientPlanarf32_v2( const uint8_t* __restrict src,
                              unsigned int              srcWidth,
                              unsigned int              srcHeight,
                              unsigned int              srcStride,
                              float* __restrict         dx,
                              float* __restrict         dy,
                              unsigned int              dxyStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dxyStride = (dxyStride==0 ? srcStride*4 : dxyStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dx & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dy & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride & 15) == 0 );    // multiple of 8
   fcvAssert( (dxyStride & 31) == 0 );    // multiple of (8 * 4 bytes)
   fcvAssert( srcStride >= srcWidth );    // at least as much as width
   fcvAssert( dxyStride >= srcWidth*4 );  // at least as much as 4*width
#endif

   return(**ppfcvImageGradientPlanarf32_v2)(src,srcWidth,srcHeight,srcStride,dx,dy,dxyStride);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientSobelInterleaveds16( const uint8_t* __restrict  src,
                                     unsigned int               srcWidth,
                                     unsigned int               srcHeight,
                                     unsigned int               srcStride,
                                     int16_t* __restrict        gradients )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)gradients & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
#endif

   return(**ppfcvImageGradientSobelInterleaveds16_v2)(src,srcWidth,srcHeight,srcStride,gradients,(srcWidth-2)*2*sizeof(int16_t));
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientSobelInterleaveds16_v2( const uint8_t* __restrict  src,
                                        unsigned int               srcWidth,
                                        unsigned int               srcHeight,
                                        unsigned int               srcStride,
                                        int16_t* __restrict        gradients,
                                        unsigned int               gradStride )
{
   srcStride  = (srcStride==0 ? srcWidth : srcStride);
   gradStride = (gradStride==0 ? srcWidth*2*sizeof(int16_t) : gradStride); 

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );          // 128-bit alignment
   fcvAssert( ((int)gradients & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );          // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );         // multiple of 8
   fcvAssert( (gradStride & 15) == 0 );         // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );        // at least as much as width
   fcvAssert( (gradStride >= (srcWidth*2*sizeof(int16_t))) );   // at least as much as 4*width
#endif

   return(**ppfcvImageGradientSobelInterleaveds16_v2)(src,srcWidth,srcHeight,srcStride,gradients,gradStride);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientSobelInterleavedf32( const uint8_t* __restrict src,
                                     unsigned int              srcWidth,
                                     unsigned int              srcHeight,
                                     unsigned int              srcStride,
                                     float* __restrict         gradients)
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)gradients & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
   fcvAssert( sizeof(*gradients) == 4 );
#endif

   return (**ppfcvImageGradientSobelInterleavedf32_v2)(src,srcWidth,srcHeight,srcStride,gradients,(srcWidth-2)*2*sizeof(float));
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientSobelInterleavedf32_v2( const uint8_t* __restrict src,
                                        unsigned int              srcWidth,
                                        unsigned int              srcHeight,
                                        unsigned int              srcStride,
                                        float* __restrict         gradients,
                                        unsigned int              gradStride )
{
   srcStride  = (srcStride==0 ? srcWidth : srcStride);
   gradStride = (gradStride==0 ? srcWidth*2*sizeof(float) : gradStride); 

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );          // 128-bit alignment
   fcvAssert( ((int)gradients & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (srcWidth & 7) == 0 );            // multiple of 8
   fcvAssert( (srcStride & 7) == 0 );           // multiple of 8
   fcvAssert( (gradStride & 31) == 0 );         // multiple of 32
   fcvAssert( (srcStride >= srcWidth) );        // at least as much as width
   fcvAssert( (gradStride >= (srcWidth*2*sizeof(float))) );   // at least as much as 8*width
#endif

   return (**ppfcvImageGradientSobelInterleavedf32_v2)(src,srcWidth,srcHeight,srcStride,gradients,gradStride);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientSobelPlanars16( const uint8_t* __restrict  src,
                                unsigned int               srcWidth,
                                unsigned int               srcHeight,
                                unsigned int               srcStride,
                                int16_t* __restrict        dx,
                                int16_t* __restrict        dy)
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dx & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dy & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
#endif

   return (**ppfcvImageGradientSobelPlanars16_v2)(src,srcWidth,srcHeight,srcStride,dx,dy,srcWidth*sizeof(int16_t));
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientSobelPlanars16_v2( const uint8_t* __restrict  src,
                                   unsigned int               srcWidth,
                                   unsigned int               srcHeight,
                                   unsigned int               srcStride,
                                   int16_t* __restrict        dx,
                                   int16_t* __restrict        dy,
                                   unsigned int               dxyStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dxyStride = (dxyStride==0 ? (srcWidth*sizeof(int16_t)) : dxyStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dx & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( ((int)dy & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (dxyStride & 15 ) == 0 );     // multiple of 16
   fcvAssert( (srcStride >= srcWidth) );    // Stride is at least as much as Width
   fcvAssert( (dxyStride >= (srcWidth*sizeof(int16_t))) );    // Stride is at least as much as Width
#endif

   return (**ppfcvImageGradientSobelPlanars16_v2)(src,srcWidth,srcHeight,srcStride,dx,dy,dxyStride);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientSobelPlanars8( const uint8_t* __restrict src,
                               unsigned int              srcWidth,
                               unsigned int              srcHeight,
                               unsigned int              srcStride,
                               int8_t* __restrict        dx,
                               int8_t* __restrict        dy)
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dx & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dy & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
#endif

   return (**ppfcvImageGradientSobelPlanars8_v2)(src,srcWidth,srcHeight,srcStride,dx,dy,srcWidth);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientSobelPlanars8_v2( const uint8_t* __restrict src,
                                  unsigned int              srcWidth,
                                  unsigned int              srcHeight,
                                  unsigned int              srcStride,
                                  int8_t* __restrict        dx,
                                  int8_t* __restrict        dy,
                                  unsigned int              dxyStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dxyStride = (dxyStride==0 ? srcWidth : dxyStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dx & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( ((int)dy & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (dxyStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );    // Stride is at least as much as Width
   fcvAssert( (dxyStride >= srcWidth) );    // Stride is at least as much as Width
#endif

   return (**ppfcvImageGradientSobelPlanars8_v2)(src,srcWidth,srcHeight,srcStride,dx,dy,dxyStride);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientSobelPlanarf32( const uint8_t* __restrict  src,
                                unsigned int               srcWidth,
                                unsigned int               srcHeight,
                                unsigned int               srcStride,
                                float*                     dx,
                                float*                     dy)
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dx & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dy & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
#endif

   return (**ppfcvImageGradientSobelPlanarf32_v2)(src,srcWidth,srcHeight,srcStride,dx,dy,srcWidth*sizeof(float));
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientSobelPlanarf32_v2( const uint8_t* __restrict  src,
                                   unsigned int               srcWidth,
                                   unsigned int               srcHeight,
                                   unsigned int               srcStride,
                                   float*                     dx,
                                   float*                     dy,
                                   unsigned int               dxyStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dxyStride = (dxyStride==0 ? (srcWidth*sizeof(float)) : dxyStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dx & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( ((int)dy & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );     // multiple of 8
   fcvAssert( (dxyStride & 31 ) == 0 );     // multiple of 16
   fcvAssert( (srcStride >= srcWidth) );    // Stride is at least as much as Width
   fcvAssert( (dxyStride >= (srcWidth*sizeof(float))) );    // Stride is at least as much as Width*sizeof each value.
#endif

   return (**ppfcvImageGradientSobelPlanarf32_v2)(src,srcWidth,srcHeight,srcStride,dx,dy,dxyStride);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientSobelPlanarf32f32( const float * __restrict  src,
                                   unsigned int              srcWidth,
                                   unsigned int              srcHeight,
                                   unsigned int              srcStride,
                                   float*                    dx,
                                   float*                    dy)
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dx & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dy & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
   fcvAssert( sizeof(*dx) == 4 );
   fcvAssert( sizeof(*dy) == 4 );
#endif

   return (**ppfcvImageGradientSobelPlanarf32f32_v2)(src,srcWidth,srcHeight,srcStride*sizeof(float),dx,dy,srcWidth*sizeof(float));
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvImageGradientSobelPlanarf32f32_v2( const float * __restrict  src,
                                      unsigned int              srcWidth,
                                      unsigned int              srcHeight,
                                      unsigned int              srcStride,
                                      float*                    dx,
                                      float*                    dy,
                                      unsigned int              dxyStride )
{
   srcStride = (srcStride==0 ? (srcWidth*sizeof(float)) : srcStride);
   dxyStride = (dxyStride==0 ? (srcWidth*sizeof(float)) : dxyStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dx & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( ((int)dy & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (srcStride & 31) == 0 );      // multiple of (8 * 4 bytes)
   fcvAssert( (dxyStride & 31 ) == 0 );     // multiple of (8 * 4 bytes)
   fcvAssert( (srcStride >= (srcWidth*sizeof(float))) );    // Stride is at least as much as Width*sizeof each value.
   fcvAssert( (dxyStride >= (srcWidth*sizeof(float))) );    // Stride is at least as much as Width*sizeof each value.
#endif

   return (**ppfcvImageGradientSobelPlanarf32f32_v2)(src,srcWidth,srcHeight,srcStride,dx,dy,dxyStride);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline  int
fcvClusterEuclideanNormed36f32( const float* __restrict  points,
                                int                      numPoints,
                                int                      pointStride,
                                const size_t* __restrict indices,
                                int                      numIndices,
                                int                      numClusters,
                                float* __restrict        clusterCenters,
                                int                      clusterCenterStride,
                                float* __restrict        newClusterCenters,
                                size_t* __restrict       clusterMemberCounts,
                                size_t* __restrict       clusterBindings,
                                float*                   sumOfClusterDistances )
{
    return  (**ppfcvClusterEuclideanNormed36f32)(points,numPoints,pointStride,indices,numIndices,numClusters,clusterCenters,
                                           clusterCenterStride,newClusterCenters,clusterMemberCounts,
                                           clusterBindings,sumOfClusterDistances);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvClusterEuclideanf32( const float* __restrict  points,
                        int                      numPoints,  // actually not used but helpful
                        int                      dim,
                        int                      pointStride,
                        const size_t* __restrict indices,
                        int                      numIndices,
                        int                      numClusters,
                        float* __restrict        clusterCenters,
                        int                      clusterCenterStride,
                        float* __restrict        newClusterCenters,
                        size_t* __restrict       clusterMemberCounts,
                        size_t* __restrict       clusterBindings,
                        float*                   sumOfClusterDistances )
{
    return (**ppfcvClusterEuclideanf32)(points,numPoints,dim,pointStride,indices,numIndices,numClusters,
                                  clusterCenters,clusterCenterStride,
                                  newClusterCenters,clusterMemberCounts,clusterBindings,
                                  sumOfClusterDistances);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvClusterEuclideanNormedf32( const float* __restrict  points,
                              int                      numPoints,
                              int                      dim,
                              int                      pointStride,
                              const size_t* __restrict indices,
                              int                      numIndices,
                              int                      numClusters,
                              float* __restrict        clusterCenters,
                              int                      clusterCenterStride,
                              float* __restrict        newClusterCenters,
                              size_t* __restrict       clusterMemberCounts,
                              size_t* __restrict       clusterBindings,
                              float*                   sumOfClusterDistances )
{
    return (**ppfcvClusterEuclideanNormedf32)(points,numPoints,dim,pointStride,indices,numIndices,numClusters,clusterCenters,
                                        clusterCenterStride,newClusterCenters,clusterMemberCounts,
                                        clusterBindings,sumOfClusterDistances);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvCornerFast9u8( const uint8_t* __restrict src,
                  unsigned int width,
                  unsigned int height,
                  unsigned int stride,
                  int barrier,
                  unsigned int border,
                  uint32_t* __restrict xy,
                  unsigned int maxnumcorners,
                  uint32_t* __restrict numcorners )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)xy & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );           // multiple of 8
#endif
   fcvAssert( width <= 2048 );

   (**ppfcvCornerFast9u8)( src, width, height, barrier,border, xy,
                      maxnumcorners, numcorners, stride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvCornerFast9InMasku8( const uint8_t* __restrict src,
                        unsigned int width,
                        unsigned int height,
                        unsigned int stride,
                        int barrier,
                        unsigned int border,
                        uint32_t* __restrict xy,
                        unsigned int maxnumcorners,
                        uint32_t* __restrict numcorners,
                        const uint8_t* __restrict bitMask,
                        unsigned int maskWidth,
                        unsigned int maskHeight )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( ((int)xy & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );      // multiple of 8
#endif
   fcvAssert( width <= 2048 );

   (**ppfcvCornerFast9InMasku8)( src, width, height, barrier, border, xy,
                             maxnumcorners, numcorners, stride,
                             bitMask, maskWidth, maskHeight );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void
fcvCornerHarrisu8( const uint8_t* __restrict src,
                    unsigned int width,
                    unsigned int height,
                    unsigned int stride,
                    unsigned int border,
                    uint32_t* __restrict xy,
                    unsigned int maxnumcorners,
                    uint32_t* __restrict numcorners,
                    int threshold )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( ((int)xy & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvCornerHarrisu8)( src, width, height, stride, border, xy,
                        maxnumcorners, numcorners, threshold );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline unsigned int
fcvLocalHarrisMaxu8 (const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     unsigned int              srcStride,
                     unsigned int              posX,
                     unsigned int              posY,
                     unsigned int             *maxX,
                     unsigned int             *maxY,
                     int                      *maxScore )

{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   return (**ppfcvLocalHarrisMaxu8)
              ( src, srcWidth, srcHeight, srcStride, posX,
                posY, maxX, maxY, maxScore);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void
fcvCornerHarrisInMasku8( const uint8_t* __restrict srcImg,
                          unsigned int width,
                          unsigned int height,
                          unsigned int stride,
                          unsigned int border,
                          uint32_t* __restrict xy,
                          unsigned int maxnumcorners,
                          uint32_t* __restrict numcorners,
                          int threshold,
                          const uint8_t* __restrict bitMask,
                          unsigned int maskWidth,
                          unsigned int maskHeight )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcImg & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( ((int)xy & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvCornerHarrisInMasku8)( srcImg, width, height, stride, border, xy, maxnumcorners,
                              numcorners, threshold, bitMask, maskWidth,
                              maskHeight );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void
fcvGeomAffineFitf32( const fcvCorrespondences* __restrict corrs,
                      float* __restrict affine )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*affine) == 4 );
#endif

   (**ppfcvGeomAffineFitf32)( corrs, affine );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline int
fcvGeomAffineEvaluatef32( const fcvCorrespondences* __restrict corrs,
                           float* __restrict affine,
                           float maxsqerr,
                           uint16_t* __restrict inliers,
                           int32_t* numinliers )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*affine) == 4 );
#endif

   return (**ppfcvGeomAffineEvaluatef32)( corrs, affine, maxsqerr, inliers,
                                      numinliers );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void
fcvGeomHomographyFitf32( const fcvCorrespondences* __restrict corrs,
                          float* __restrict homography )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*homography) == 4 );
#endif

   (**ppfcvGeomHomographyFitf32)( corrs, homography );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline int
fcvGeomHomographyEvaluatef32( const fcvCorrespondences* __restrict corrs,
                               float* __restrict homography,
                               float maxsqerr,
                               uint16_t* __restrict inliers,
                               int32_t* numinliers )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*homography) == 4 );
#endif

   return (**ppfcvGeomHomographyEvaluatef32)( corrs, homography, maxsqerr, inliers,
                                          numinliers );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline float
fcvGeomPoseRefineGNf32( const fcvCorrespondences* __restrict corrs,
                         short minIterations,
                         short maxIterations,
                         float stopCriteria,
                         float* initpose,
                         float* refinedpose )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*initpose) == 4 );
   fcvAssert( sizeof(*refinedpose) == 4 );
#endif

   return (**ppfcvGeomPoseRefineGNf32)( corrs, minIterations, maxIterations, stopCriteria,
                                    initpose, refinedpose );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline int
fcvGeomPoseUpdatef32(
   const float* __restrict projected,
   const float* __restrict reprojErr,
   const float* __restrict invz,
   const float* __restrict reprojVariance,
   unsigned int                numpts,
   float*       __restrict pose )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*pose) == 4 );
#endif

   return (**ppfcvGeomPoseUpdatef32)( projected, reprojErr, invz,
                                 reprojVariance, numpts, pose );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline int
fcvGeomPoseOptimizeGNf32(
   const float* __restrict projected,
   const float* __restrict reprojErr,
   const float* __restrict invz,
   const float* __restrict reprojVariance,
   unsigned int                numpts,
   float*       __restrict pose )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*pose) == 4 );
#endif

   return (**ppfcvGeomPoseOptimizeGNf32)( projected, reprojErr, invz,
                                     reprojVariance, numpts, pose );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline float
fcvGeomPoseEvaluateErrorf32(
   const fcvCorrespondences* __restrict corrs,
   const float*              __restrict pose,
   float*                    __restrict projected,
   float*                    __restrict reprojErr,
   float*                    __restrict invz,
   float*                    __restrict reprojVariance )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*pose) == 4 );
#endif

   return (**ppfcvGeomPoseEvaluateErrorf32)( corrs, pose, projected, reprojErr,
                                        invz, reprojVariance );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline int
fcvGeomPoseEvaluatef32( const fcvCorrespondences* __restrict corrs,
                         const float* pose,
                         float maxSquErr,
                         uint16_t* __restrict inliers,
                         uint32_t* numInliers )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*pose) == 4 );
#endif

   return (**ppfcvGeomPoseEvaluatef32)( corrs, pose, maxSquErr, inliers, numInliers );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void
fcvGeom3PointPoseEstimatef32( const fcvCorrespondences* __restrict corrs,
                              float* pose,
                              int32_t* numPoses )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*pose) == 4 );
#endif

   return (**ppfcvGeom3PointPoseEstimatef32)( corrs, pose, numPoses );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCorr3x3s8( const int8_t* __restrict kernel,
                     const uint8_t* __restrict src, unsigned int w, unsigned int h,
                     uint8_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );  // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );  // 128-bit alignment
   fcvAssert( (w & 0x1) == 0 );         // even
   fcvAssert( (h & 0x1) == 0 );         // even
#endif

   (**ppfcvFilterCorr3x3s8_v2)( kernel, src, w, h, w, dst, w );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCorr3x3s8_v2( const int8_t* __restrict kernel,
                       const uint8_t* __restrict src, 
                       unsigned int srcWidth, 
                       unsigned int srcHeight,
                       unsigned int srcStride,
                       uint8_t* __restrict dst,
                       unsigned int dstStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x1) == 0 );     // even
   fcvAssert( (srcHeight & 0x1) == 0 );    // even
   fcvAssert( (srcStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );   // Stride is at least as much as Width
#endif

   (**ppfcvFilterCorr3x3s8_v2)( kernel, src, srcWidth, srcHeight, srcStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCorrSep9x9s16( const int16_t* __restrict knl,
                         const int16_t* __restrict src, unsigned int w, unsigned int h,
                         int16_t* __restrict tmp,
                         int16_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)knl & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)tmp & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (w & 0x7) == 0 );          // multiple of 8
#endif
   fcvAssert( w >= 8 );

   (**ppfcvFilterCorrSep9x9s16_v2)( knl, src, w, h, w, tmp, dst, w );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCorrSep11x11s16( const int16_t* __restrict knl,
                           const int16_t* __restrict src, unsigned int w, unsigned int h,
                           int16_t* __restrict tmp, int16_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)knl & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)tmp & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (w & 0x7) == 0 );          // multiple of 8
#endif
   fcvAssert( w >= 8 );

   (**ppfcvFilterCorrSep11x11s16_v2)( knl, src, w, h, w, tmp, dst, w );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCorrSep13x13s16( const int16_t* __restrict knl,
                           const int16_t* __restrict src, unsigned int w, unsigned int h,
                           int16_t* __restrict tmp,
                           int16_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)knl & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)tmp & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (w & 0x7) == 0 );          // multiple of 8
#endif
   fcvAssert( w >= 8 );

   (**ppfcvFilterCorrSep13x13s16)( knl, src, w, h, tmp, dst );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCorrSep15x15s16( const int16_t* __restrict knl,
                           const int16_t* __restrict src, unsigned int w, unsigned int h,
                           int16_t* __restrict tmp,
                           int16_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)knl & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)tmp & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (w & 0x7) == 0 );          // multiple of 8
#endif
   fcvAssert( w >= 8 );

   (**ppfcvFilterCorrSep15x15s16_v2)( knl, src, w, h, w, tmp, dst, w );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCorrSep17x17s16( const int16_t* __restrict knl,
                           const int16_t* __restrict src, unsigned int w, unsigned int h,
                           int16_t* __restrict tmp,
                           int16_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)knl & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)tmp & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (w & 0x7) == 0 );          // multiple of 8
#endif
   fcvAssert( w >= 8 );

   (**ppfcvFilterCorrSep17x17s16_v2)( knl, src, w, h, w, tmp, dst, w );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCorrSep9x9s16_v2( const int16_t* __restrict kernel,
                           const int16_t* __restrict srcImg,
                           unsigned int w, unsigned int h, unsigned int srcStride,
                           int16_t* __restrict tmpImg,
                           int16_t* __restrict dstImg, unsigned int dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)kernel & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)srcImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dstImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)tmpImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (w & 0x7) == 0 );               // multiple of 8
#endif
   fcvAssert( w >= 8 );

   return (**ppfcvFilterCorrSep9x9s16_v2) (kernel, srcImg, w, h, srcStride, tmpImg, dstImg, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCorrSep11x11s16_v2( const int16_t* __restrict kernel,
                             const int16_t* __restrict srcImg,
                             unsigned int w, unsigned int h, unsigned int srcStride,
                             int16_t* __restrict tmpImg,
                             int16_t* __restrict dstImg, unsigned int dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)kernel & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)srcImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dstImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)tmpImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (w & 0x7) == 0 );               // multiple of 8
#endif
   fcvAssert( w >= 8 );

   return (**ppfcvFilterCorrSep11x11s16_v2) (kernel, srcImg, w, h, srcStride, tmpImg, dstImg, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCorrSep13x13s16_v2( const int16_t* __restrict kernel,
                             const int16_t* __restrict srcImg,
                             unsigned int w, unsigned int h, unsigned int srcStride,
                             int16_t* __restrict tmpImg,
                             int16_t* __restrict dstImg, unsigned int dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)kernel & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)srcImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dstImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)tmpImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (w & 0x7) == 0 );               // multiple of 8
#endif
   fcvAssert( w >= 8 );

   return (**ppfcvFilterCorrSep13x13s16_v2) (kernel, srcImg, w, h, srcStride, tmpImg, dstImg, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCorrSep15x15s16_v2( const int16_t* __restrict kernel,
                             const int16_t* __restrict srcImg,
                             unsigned int w, unsigned int h, unsigned int srcStride,
                             int16_t* __restrict tmpImg,
                             int16_t* __restrict dstImg, unsigned int dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)kernel & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)srcImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dstImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)tmpImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (w & 0x7) == 0 );               // multiple of 8
#endif
   fcvAssert( w >= 8 );

   return (**ppfcvFilterCorrSep15x15s16_v2) (kernel, srcImg, w, h, srcStride, tmpImg, dstImg, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterCorrSep17x17s16_v2( const int16_t* __restrict kernel,
                             const int16_t* __restrict srcImg,
                             unsigned int w, unsigned int h, unsigned int srcStride,
                             int16_t* __restrict tmpImg,
                             int16_t* __restrict dstImg, unsigned int dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)kernel & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)srcImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dstImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)tmpImg & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (w & 0x7) == 0 );               // multiple of 8
#endif
   fcvAssert( w >= 8 );

   return (**ppfcvFilterCorrSep17x17s16_v2) (kernel, srcImg, w, h, srcStride, tmpImg, dstImg, dstStride );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
inline void
fcvImageIntensityStats( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        int                       xBegin,
                        int                       yBegin,
                        unsigned int              recWidth,
                        unsigned int              recHeight,
                        float*                    mean,
                        float*                    variance )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
   fcvAssert( sizeof(*mean) == 4 );
   fcvAssert( sizeof(*variance) == 4 );
#endif
   fcvAssert( (recHeight * recWidth ) > 1 );

   (**ppfcvImageIntensityStats)( src, srcWidth, xBegin, yBegin, recWidth, recHeight, mean,
                            variance );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
inline void
fcvImageIntensityHistogram(  const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             int                       xBegin,
                             int                       yBegin,
                             unsigned int              recWidth,
                             unsigned int              recHeight,
                             int32_t*                  histogram )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );           // 128-bit alignment
   fcvAssert( ((int)histogram & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );           // multiple of 8
#endif

   (**ppfcvImageIntensityHistogram)( src, srcWidth, xBegin, yBegin, recWidth, recHeight,
                                histogram );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvIntegrateImageu8( const uint8_t* __restrict src,
                      unsigned int width, unsigned int height,
                      uint32_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );        // multiple of 8
   fcvAssert( height < 2048 );             
#endif

   (**ppfcvIntegrateImageu8_v2)( src, width, height, width, dst, (width+1)*sizeof(uint32_t) );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvIntegrateImageu8_v2( const uint8_t* __restrict src,
                        unsigned int srcWidth, 
                        unsigned int srcHeight,
                        unsigned int srcStride,
                        uint32_t* __restrict dst,
                        unsigned int dstStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? ((srcWidth+1)*sizeof(uint32_t)) : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );         // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );         // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );         // multiple of 8
   fcvAssert( (srcStride & 31) == 0 );         // multiple of 8
   fcvAssert( (dstStride & 31) == 0 );         // multiple of 32
   fcvAssert( (srcStride >= srcWidth) );       // Stride is at least as much as Width
   fcvAssert( (dstStride >= ((srcWidth+1)*sizeof(uint32_t))) ); // Stride is at least as much as Width*4 (in bytes)
   fcvAssert( (srcHeight) < 2048 );  
#endif

   (**ppfcvIntegrateImageu8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvIntegratePatchu8( const uint8_t* __restrict src,
                      unsigned int width, unsigned int height, int patchX, int patchY,
                      unsigned int patchW, unsigned int patchH,
                      uint32_t* __restrict intgrlImgOut,
                      uint32_t* __restrict intgrlSqrdImgOut )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );        // multiple of 8
   fcvAssert( (patchW*patchH) < 66051 );   // to avoid overflow
#endif
   fcvAssert( height < 2048 );

   (**ppfcvIntegratePatchu8_v2)( src, width, height, width, patchX, patchY, patchW, patchH, 
                                 intgrlImgOut, intgrlSqrdImgOut );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvIntegratePatchu8_v2( const uint8_t* __restrict src,
                        unsigned int srcWidth, 
                        unsigned int srcHeight, 
                        unsigned int srcStride,
                        int patchX, 
                        int patchY,
                        unsigned int patchW, 
                        unsigned int patchH,
                        uint32_t* __restrict intgrlImgOut,
                        uint32_t* __restrict intgrlSqrdImgOut )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (patchW*patchH) < 66051 );   // to avoid overflow
#endif
   fcvAssert( srcHeight < 2048 );

   (**ppfcvIntegratePatchu8_v2)( src, srcWidth, srcHeight, srcStride, patchX, patchY,
                                 patchW, patchH, intgrlImgOut, intgrlSqrdImgOut );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvIntegratePatch12x12u8( const uint8_t* __restrict src,
                          unsigned int srcWidth, 
                          unsigned int srcHeight, 
                          int patchX, 
                          int patchY,
                          uint32_t* __restrict intgrlImgOut,
                          uint32_t* __restrict intgrlSqrdImgOut )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
#endif

   (**ppfcvIntegratePatch12x12u8_v2)( src, srcWidth, srcHeight, srcWidth, patchX, patchY,
                                      intgrlImgOut, intgrlSqrdImgOut );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvIntegratePatch12x12u8_v2( const uint8_t* __restrict src,
                             unsigned int srcWidth, 
                             unsigned int srcHeight, 
                             unsigned int srcStride,
                             int patchX, 
                             int patchY,
                             uint32_t* __restrict intgrlImgOut,
                             uint32_t* __restrict intgrlSqrdImgOut )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );       // Stride is at least as much as Width
#endif

   (**ppfcvIntegratePatch12x12u8_v2)( src, srcWidth, srcHeight, srcStride, patchX, patchY,
                                      intgrlImgOut, intgrlSqrdImgOut );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvIntegratePatch18x18u8( const uint8_t* __restrict src,
                           unsigned int width, unsigned int height, int patchX, int patchY,
                           uint32_t* __restrict intgrlImgOut,
                           uint32_t* __restrict intgrlSqrdImgOut )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );           // multiple of 8
#endif

   (**ppfcvIntegratePatch18x18u8_v2)( src, width, height, width, patchX, patchY,
                                      intgrlImgOut, intgrlSqrdImgOut );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvIntegratePatch18x18u8_v2( const uint8_t* __restrict src,
                             unsigned int srcWidth, 
                             unsigned int srcHeight, 
                             unsigned int srcStride,
                             int patchX, 
                             int patchY,
                             uint32_t* __restrict intgrlImgOut,
                             uint32_t* __restrict intgrlSqrdImgOut )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
#endif

   (**ppfcvIntegratePatch18x18u8_v2)( src, srcWidth, srcHeight, srcStride, patchX, patchY,
                                      intgrlImgOut, intgrlSqrdImgOut );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvIntegrateImageLineu8( const uint8_t* __restrict imageIn,
                          unsigned short numPxls,
                          uint32_t* intgrl,
                          uint32_t* intgrlSqrd )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)imageIn & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)intgrl & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( ((int)intgrlSqrd & 0xF) == 0 );  // 128-bit alignment
#endif

   (**ppfcvIntegrateImageLineu8)( imageIn, numPxls, intgrl, intgrlSqrd );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvIntegrateImageLine64u8( const uint8_t* __restrict imageIn,
                            uint16_t* intgrl,
                            uint32_t* intgrlSqrd )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)imageIn & 0xF) == 0 );     // 128-bit alignment
#endif

   (**ppfcvIntegrateImageLine64u8)( imageIn, intgrl, intgrlSqrd );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvNCCPatchOnCircle8x8u8( const uint8_t* __restrict patch_pixels,
                          const uint8_t* __restrict image_pixels,
                          unsigned short            image_w,
                          unsigned short            image_h,
                          unsigned short            search_center_x,
                          unsigned short            search_center_y,
                          unsigned short            search_radius,
                          uint16_t*                 bestX,
                          uint16_t*                 bestY,
                          uint32_t*                 bestNCC,
                          int                       doSubPixel,
                          float*                    subX,
                          float*                    subY )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)image_pixels & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (image_w & 0x7) == 0 );               // multiple of 8
   fcvAssert( patch_pixels != 0 );
   fcvAssert( image_pixels != 0 );
   fcvAssert( search_radius <= 5 );
   fcvAssert( bestX != 0 );
   fcvAssert( bestY != 0 );
   fcvAssert( bestNCC != 0 );
   fcvAssert( (doSubPixel == 0) ||
           ( (subX != 0) && (subY != 0) ) );
   fcvAssert( sizeof(*subX) == 4 );
   fcvAssert( sizeof(*subY) == 4 );
#endif
   const int defaultLowVariance = 0;

   return (**ppfcvNCCPatchOnCircle8x8u8_v2)
              ( patch_pixels, image_pixels,
                image_w, image_h, search_center_x, search_center_y,
                search_radius,  defaultLowVariance, bestX, bestY, bestNCC,
                doSubPixel, subX, subY );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline int
fcvNCCPatchOnSquare8x8u8( const uint8_t* __restrict patch_pixels,
                          const uint8_t* __restrict image_pixels,
                          unsigned short            image_w,
                          unsigned short            image_h,
                          unsigned short            search_center_x,
                          unsigned short            search_center_y,
                          unsigned short            search_w,
                          uint16_t*                 bestX,
                          uint16_t*                 bestY,
                          uint32_t*                 bestNCC,
                          int                       doSubPixel,
                          float*                    subX,
                          float*                    subY )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)image_pixels & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (image_w & 0x7) == 0 );               // multiple of 8
   fcvAssert( patch_pixels != 0 );
   fcvAssert( image_pixels != 0 );
   fcvAssert( search_w <= 11 );
   fcvAssert( bestX != 0 );
   fcvAssert( bestY != 0 );
   fcvAssert( bestNCC != 0 );
   fcvAssert( (doSubPixel == 0) ||
           ( (subX != 0) && (subY != 0) ) );
   fcvAssert( sizeof(*subX) == 4 );
   fcvAssert( sizeof(*subY) == 4 );
#endif
   const int defaultLowVariance = 0;

   return (**ppfcvNCCPatchOnSquare8x8u8_v2)
             ( patch_pixels, image_pixels, image_w, image_h, 
               search_center_x, search_center_y, search_w, defaultLowVariance,
               bestX, bestY, bestNCC, doSubPixel, subX, subY );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvNCCPatchOnCircle8x8u8_v2( const uint8_t* __restrict patch_pixels,
                             const uint8_t* __restrict image_pixels,
                             unsigned short            image_w,
                             unsigned short            image_h,
                             unsigned short            search_center_x,
                             unsigned short            search_center_y,
                             unsigned short            search_radius,
                             int                       filterLowVariance,
                             uint16_t*                 bestX,
                             uint16_t*                 bestY,
                             uint32_t*                 bestNCC,
                             int                       doSubPixel,
                             float*                    subX,
                             float*                    subY )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)image_pixels & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (image_w & 0x7) == 0 );               // multiple of 8
   fcvAssert( patch_pixels != 0 );
   fcvAssert( image_pixels != 0 );
   fcvAssert( search_radius <= 5 );
   fcvAssert( bestX != 0 );
   fcvAssert( bestY != 0 );
   fcvAssert( bestNCC != 0 );
   fcvAssert( (doSubPixel == 0) ||
           ( (subX != 0) && (subY != 0) ) );
   fcvAssert( sizeof(*subX) == 4 );
   fcvAssert( sizeof(*subY) == 4 );
#endif

   return (**ppfcvNCCPatchOnCircle8x8u8_v2)
              ( patch_pixels, image_pixels,
                image_w, image_h, search_center_x, search_center_y,
                search_radius,  filterLowVariance, bestX, bestY, bestNCC,
                doSubPixel, subX, subY );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline int
fcvNCCPatchOnSquare8x8u8_v2( const uint8_t* __restrict patch_pixels,
                             const uint8_t* __restrict image_pixels,
                             unsigned short            image_w,
                             unsigned short            image_h,
                             unsigned short            search_center_x,
                             unsigned short            search_center_y,
                             unsigned short            search_w,
                             int                       filterLowVariance,
                             uint16_t*                 bestX,
                             uint16_t*                 bestY,
                             uint32_t*                 bestNCC,
                             int                       doSubPixel,
                             float*                    subX,
                             float*                    subY )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)image_pixels & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (image_w & 0x7) == 0 );               // multiple of 8
   fcvAssert( patch_pixels != 0 );
   fcvAssert( image_pixels != 0 );
   fcvAssert( search_w <= 11 );
   fcvAssert( bestX != 0 );
   fcvAssert( bestY != 0 );
   fcvAssert( bestNCC != 0 );
   fcvAssert( (doSubPixel == 0) ||
           ( (subX != 0) && (subY != 0) ) );
   fcvAssert( sizeof(*subX) == 4 );
   fcvAssert( sizeof(*subY) == 4 );
#endif

   return (**ppfcvNCCPatchOnSquare8x8u8_v2)
             ( patch_pixels, image_pixels, image_w, image_h, 
               search_center_x, search_center_y, search_w, filterLowVariance,
               bestX, bestY, bestNCC, doSubPixel, subX, subY );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvSumOfAbsoluteDiffs8x8u8( const uint8_t* __restrict patch,
                            const uint8_t* __restrict src,
                            unsigned int width, unsigned int height,
                            unsigned int pitch,
                            uint16_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( (patch != NULL) );
   fcvAssert( (src   != NULL) );
   fcvAssert( (dst   != NULL) );
   fcvAssert( ((int)src & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );       // 128-bit alignment
#endif

   return (**ppfcvSumOfAbsoluteDiffs8x8u8_v2)( patch, 8, src, width, height, pitch, dst, width*sizeof(uint16_t) );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvSumOfAbsoluteDiffs8x8u8_v2( const uint8_t* __restrict patch,
                               unsigned int patchStride,
                               const uint8_t* __restrict src,
                               unsigned int srcWidth, 
                               unsigned int srcHeight,
                               unsigned int srcStride,
                               uint16_t* __restrict dst,
                               unsigned int dstStride )
{
   patchStride = (patchStride==0 ? 8 : patchStride);
   srcStride   = (srcStride==0 ? srcWidth : srcStride);
   dstStride   = (dstStride==0 ? srcWidth*2 : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( (patch != NULL) );
   fcvAssert( (src   != NULL) );
   fcvAssert( (dst   != NULL) );
   fcvAssert( ((int)src & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( (patchStride >= 8) );          // Stride is at least as much as width
   fcvAssert( (srcStride >= srcWidth) );     // Stride is at least as much as width
   fcvAssert( (dstStride >= (srcWidth*2)) ); // Stride is at least as much as 2*width
#endif

   return (**ppfcvSumOfAbsoluteDiffs8x8u8_v2)( patch, patchStride, src, srcWidth, srcHeight, 
                                            srcStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvScaleDownBy2u8( const uint8_t* __restrict src, unsigned int width, unsigned int height,
                   uint8_t* __restrict dst)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );  // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );  // 128-bit alignment
   // fcvAssert( (width & 0x7) == 0 );     // multiple of 8
   // fcvAssert( (height & 0x1) == 0 );    // multiple of 2
#endif

   return (**ppfcvScaleDownBy2u8_v2)( src, width, height, width, dst, width/2 );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvScaleDownBy2u8_v2( const uint8_t* __restrict src, 
                      unsigned int srcWidth, 
                      unsigned int srcHeight,
                      unsigned int srcStride,
                      uint8_t* __restrict dst,
                      unsigned int dstStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth/2 : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );        // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );        // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );       // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );       // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );      // Stride is at least as much as width
   fcvAssert( (dstStride >= (srcWidth/2)) );  // Stride is at least as much as width/2
#endif

   return (**ppfcvScaleDownBy2u8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvScaleDownBy4u8( const uint8_t* __restrict src, unsigned int width, unsigned int height,
                   uint8_t* __restrict dst)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );    // 128-bit alignment
   // fcvAssert( (width & 0x7) == 0 );       // multiple of 8
   // fcvAssert( (height & 0x3) == 0 );      // multiple of 4
#endif

   return (**ppfcvScaleDownBy4u8_v2)( src, width, height, width, dst, width/4 );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvScaleDownBy4u8_v2( const uint8_t* __restrict src, 
                      unsigned int srcWidth, 
                      unsigned int srcHeight,
                      unsigned int srcStride,
                      uint8_t* __restrict dst,
                      unsigned int dstStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth/4 : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );        // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );        // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );       // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );       // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );      // Stride is at least as much as width
   fcvAssert( (dstStride >= (srcWidth/4)) );  // Stride is at least as much as (width/4)
#endif

   return (**ppfcvScaleDownBy4u8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvScaleDown3To2u8( const uint8_t* __restrict src,
                    unsigned                  srcWidth,
                    unsigned                  srcHeight,
                    unsigned int              srcStride,
                    uint8_t* __restrict       dst,
                    unsigned int              dstStride)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );    // 128-bit alignment
#endif

   return (**ppfcvScaleDown3To2u8)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvScaleDownNNu8( const uint8_t* __restrict src,
                  unsigned int              srcWidth,
                  unsigned int              srcHeight,
                  unsigned int              srcStride,
                  uint8_t* __restrict       dst,
                  unsigned int              dstWidth,
                  unsigned int              dstHeight,
                  unsigned int              dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );       // multiple of 8
#endif
   fcvAssert( ( srcWidth % dstWidth ) == 0  && ( srcHeight % dstHeight ) == 0 );

   return (**ppfcvScaleDownNNu8)( src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvScaleDownu8( const uint8_t* __restrict src,
                 unsigned int width, unsigned int height,
                 uint8_t* __restrict dst,
                 unsigned int dstWidth, unsigned int dstHeight)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );       // multiple of 8
#endif
   fcvAssert( ( width % dstWidth ) == 0  && ( height % dstHeight ) == 0 );

   return (**ppfcvScaleDownu8_v2)( src, width, height, width, dst, dstWidth, dstHeight, dstWidth );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvScaleDownu8_v2( const uint8_t* __restrict src,
                   unsigned int srcWidth, 
                   unsigned int srcHeight,
                   unsigned int srcStride,
                   uint8_t* __restrict dst,
                   unsigned int dstWidth, 
                   unsigned int dstHeight,
                   unsigned int dstStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? dstWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );   // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );   // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );  // Stride is at least as much as width
   fcvAssert( (dstStride >= dstWidth) );  // Stride is at least as much as width
#endif
   fcvAssert( ( srcWidth % dstWidth ) == 0  && ( srcHeight % dstHeight ) == 0 );

   return (**ppfcvScaleDownu8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstWidth, dstHeight, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvScaleUpBy2Gaussian5x5u8( const uint8_t* __restrict src,
                             unsigned int width,
                             unsigned int height,
                             uint8_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );       // multiple of 8
#endif

   (**ppfcvScaleUpBy2Gaussian5x5u8_v2)( src, width, height, width, dst, width*2 );

}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvScaleUpBy2Gaussian5x5u8_v2( const uint8_t* __restrict src,
                               unsigned int srcWidth,
                               unsigned int srcHeight,
                               unsigned int srcStride,
                               uint8_t* __restrict dst,
                               unsigned int dstStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth*2 : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );       // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );       // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );      // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );      // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );     // Stride is at least as much as width
   fcvAssert( (dstStride >= (srcWidth*2)) ); // Stride is at least as much as 2*width
#endif

   (**ppfcvScaleUpBy2Gaussian5x5u8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride );

}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvScaleDownBy2Gaussian5x5u8( const uint8_t* __restrict src,
                               unsigned int width,
                               unsigned int height,
                               uint8_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );       // multiple of 8
#endif
   fcvAssert( (width&1)==0 && (height&1)==0 );

   (**ppfcvScaleDownBy2Gaussian5x5u8)( src, width, height, dst );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvScaleDownBy2Gaussian5x5u8_v2( const uint8_t* __restrict src,
                                 unsigned int srcWidth,
                                 unsigned int srcHeight,
                                 unsigned int srcStride,
                                 uint8_t* __restrict dst,
                                 unsigned int dstStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth/2 : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );        // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );        // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );        // multiple of 8
   fcvAssert( (srcHeight & 1) == 0 );         // Height is multiple of 2
   fcvAssert( (srcStride & 0x7) == 0 );       // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );       // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );      // Stride is at least as much as Width
   fcvAssert( (dstStride >= (srcWidth/2)) );  // Stride is at least as much as Width/2
#endif

   (**ppfcvScaleDownBy2Gaussian5x5u8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvSumOfSquaredDiffs36x4s8( const int8_t* __restrict A,
                               float invLenA,
                               const int8_t* __restrict B,
                               const int8_t* __restrict C,
                               const int8_t* __restrict D,
                               const int8_t* __restrict E,
                               const float* __restrict  invLenB,
                               float* __restrict distances )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)invLenB & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)distances & 0xF) == 0 );    // 128-bit alignment
#endif

   return (**ppfcvSumOfSquaredDiffs36x4s8)( A, invLenA, B, C, D, E, invLenB,
                                        distances );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvSumOfSquaredDiffs36xNs8( const int8_t* __restrict A,
                             float invLenA,
                             const int8_t* const * __restrict B,
                             const float* __restrict invLenB,
                             unsigned int numB,
                             float* __restrict distances )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*invLenB) == 4 );
   fcvAssert( sizeof(*distances) == 4 );
#endif

   return (**ppfcvSumOfSquaredDiffs36xNs8)( A, invLenA, B, invLenB, numB,
                                        distances );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvSort8Scoresf32( float* __restrict inScores,
                   float* __restrict outScores )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*inScores) == 4 );
   fcvAssert( sizeof(*outScores) == 4 );
#endif

   return (**ppfcvSort8Scoresf32)( inScores, outScores );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterThresholdu8( const uint8_t* __restrict src,
                       unsigned int width,
                       unsigned int height,
                       uint8_t* __restrict dst,
                       unsigned int threshold )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );       // multiple of 8
#endif
   fcvAssert( threshold >= 0 );

   return (**ppfcvFilterThresholdu8_v2)( src, width, height, width, dst, width, threshold );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterThresholdu8_v2( const uint8_t* __restrict src,
                         unsigned int srcWidth,
                         unsigned int srcHeight,
                         unsigned int srcStride,
                         uint8_t* __restrict dst,
                         unsigned int dstStride,
                         unsigned int threshold )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );   // Stride is at least as much as Width
#endif
   fcvAssert( threshold >= 0 );

   return (**ppfcvFilterThresholdu8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride, threshold );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void
fcvFilterDilate3x3u8( const uint8_t* __restrict src,
                       unsigned int width, unsigned int height,
                       uint8_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );       // multiple of 8
#endif

   return (**ppfcvFilterDilate3x3u8_v2)( src, width, height, width, dst, width );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void
fcvFilterDilate3x3u8_v2( const uint8_t* __restrict src,
                         unsigned int srcWidth, 
                         unsigned int srcHeight,
                         unsigned int srcStride,
                         uint8_t* __restrict dst,
                         unsigned int dstStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );   // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );   // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );   // Stride is at least as much as Width
#endif

   return (**ppfcvFilterDilate3x3u8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void
fcvFilterErode3x3u8( const uint8_t* __restrict src,
                      unsigned int width, unsigned int height,
                      uint8_t* __restrict dst )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );       // multiple of 8
#endif

   return (**ppfcvFilterErode3x3u8_v2)( src, width, height, width, dst, width );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void
fcvFilterErode3x3u8_v2( const uint8_t* __restrict src,
                        unsigned int srcWidth, 
                        unsigned int srcHeight,
                        unsigned int srcStride,
                        uint8_t* __restrict dst,
                        unsigned int dstStride )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );   // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );   // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );  // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );  // Stride is at least as much as Width
#endif

   return (**ppfcvFilterErode3x3u8_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline int
fcvTransformAffine8x8u8( const uint8_t* __restrict src,
                          unsigned int width, unsigned int height,
                          const int32_t* __restrict nPos,
                          const int32_t* __restrict nAffine,
                          uint8_t* __restrict nPatch )

{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)nPos & 0x7) == 0 );      // 64-bit alignment
   fcvAssert( ((int)nAffine & 0xF) == 0 );   // 128-bit alignment
#endif

   return (**ppfcvTransformAffine8x8u8_v2)( src, width, height, width, nPos, nAffine, nPatch, 8 );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline int
fcvTransformAffine8x8u8_v2( const uint8_t* __restrict src,
                            unsigned int srcWidth, 
                            unsigned int srcHeight,
                            unsigned int srcStride,
                            const int32_t* __restrict nPos,
                            const int32_t* __restrict nAffine,
                            uint8_t* __restrict patch,
                            unsigned int patchStride )

{
   srcStride   = (srcStride==0   ? srcWidth : srcStride);
   patchStride = (patchStride==0 ? 8        : patchStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)nPos & 0x7) == 0 );      // 64-bit alignment
   fcvAssert( ((int)nAffine & 0xF) == 0 );   // 128-bit alignment

   fcvAssert( (srcStride >= srcWidth) );     // Stride is at least as much as width
   fcvAssert( (patchStride >= 8) );          // Stride is at least as much as 8 (patchWidth)
#endif

   return (**ppfcvTransformAffine8x8u8_v2)( src, srcWidth, srcHeight, srcStride, 
                                            nPos, nAffine, patch, patchStride );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvWarpPerspectiveu8( const uint8_t* __restrict src,
                      unsigned int srcwidth,
                      unsigned int srcheight,
                      uint8_t* __restrict dst,
                      unsigned int dstwidth,
                      unsigned int dstheight,
                      float* __restrict proj )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( (srcwidth & 0x7) == 0 );   // multiple of 8
   fcvAssert( (srcheight & 0x7) == 0 );  // multiple of 8
   fcvAssert( ((int)dst & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( (dstwidth & 0x7) == 0 );   // multiple of 8
   fcvAssert( (dstheight & 0x7) == 0 );  // multiple of 8
   fcvAssert( ((int)proj & 0xF) == 0 );  // 128-bit alignment
#endif

   return (**ppfcvWarpPerspectiveu8)( src, srcwidth, srcheight, dst,
                                  dstwidth, dstheight, proj );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvWarpPerspectiveu8_v2( const uint8_t* __restrict src,
                         unsigned int srcWidth,
                         unsigned int srcHeight,
                         unsigned int srcStride,
                         uint8_t* __restrict dst,
                         unsigned int dstWidth,
                         unsigned int dstHeight,
                         unsigned int dstStride,
                         float* __restrict proj )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? dstWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );   // multiple of 8
   fcvAssert( (srcHeight & 0x7) == 0 );  // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );  // multiple of 8
   fcvAssert( ((int)dst & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( (dstWidth & 0x7) == 0 );   // multiple of 8
   fcvAssert( (dstHeight & 0x7) == 0 );  // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );  // multiple of 8
   fcvAssert( ((int)proj & 0xF) == 0 );  // 128-bit alignment
   fcvAssert( (srcStride >= srcWidth) ); // at least as much as width
   fcvAssert( (dstStride >= dstWidth) ); // at least as much as width
#endif

   return (**ppfcvWarpPerspectiveu8_v2)( src, srcWidth, srcHeight, srcStride, 
                                         dst, dstWidth, dstHeight, dstStride, 
                                         proj );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcv3ChannelWarpPerspectiveu8( const uint8_t* __restrict src,
                               unsigned int srcwidth,
                               unsigned int srcheight,
                               uint8_t* __restrict dst,
                               unsigned int dstwidth,
                               unsigned int dstheight,
                               float* __restrict proj )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( (srcwidth & 0x7) == 0 );   // multiple of 8
   fcvAssert( (srcheight & 0x7) == 0 );  // multiple of 8
   fcvAssert( ((int)dst & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( (dstwidth & 0x7) == 0 );   // multiple of 8
   fcvAssert( (dstheight & 0x7) == 0 );  // multiple of 8
   fcvAssert( ((int)proj & 0xF) == 0 );  // 128-bit alignment
#endif

   return (**ppfcv3ChannelWarpPerspectiveu8)( src, srcwidth, srcheight, dst,
                                          dstwidth, dstheight, proj );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcv3ChannelWarpPerspectiveu8_v2( const uint8_t* __restrict src,
                                 unsigned int srcWidth,
                                 unsigned int srcHeight,
                                 unsigned int srcStride,
                                 uint8_t* __restrict dst,
                                 unsigned int dstWidth,
                                 unsigned int dstHeight,
                                 unsigned int dstStride,
                                 float* __restrict proj )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? dstWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );   // multiple of 8
   fcvAssert( (srcHeight & 0x7) == 0 );  // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );  // multiple of 8
   fcvAssert( ((int)dst & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( (dstWidth & 0x7) == 0 );   // multiple of 8
   fcvAssert( (dstHeight & 0x7) == 0 );  // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );  // multiple of 8
   fcvAssert( ((int)proj & 0xF) == 0 );  // 128-bit alignment
   fcvAssert( (srcStride >= srcWidth) ); // at least as much as width
   fcvAssert( (dstStride >= dstWidth) ); // at least as much as width
#endif

   return (**ppfcv3ChannelWarpPerspectiveu8_v2)( src, srcWidth, srcHeight, srcStride,
                                                 dst, dstWidth, dstHeight, dstStride,
                                                 proj );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterGaussian5x5s16( const int16_t* __restrict src,
                          unsigned int width,
                          unsigned int height,
                          int16_t* __restrict dst,
                          int border )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );   // 128-bit alignment
#endif

   return (**ppfcvFilterGaussian5x5s16_v2)( src, width, height, width, dst, 
                                         width, border );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterGaussian5x5s16_v2( const int16_t* __restrict src,
                            unsigned int srcWidth,
                            unsigned int srcHeight,
                            unsigned int srcStride,
                            int16_t* __restrict dst,
                            unsigned int dstStride,
                            int border )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );   // Stride is at least as much as Width
#endif

   return (**ppfcvFilterGaussian5x5s16_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride, border );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterGaussian5x5s32( const int32_t* __restrict src,
                          unsigned int width,
                          unsigned int height,
                          int32_t* __restrict dst,
                          int border )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );      // multiple of 8
#endif

   return (**ppfcvFilterGaussian5x5s32_v2)( src, width, height, width, dst, 
                                         width, border );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvFilterGaussian5x5s32_v2( const int32_t* __restrict src,
                            unsigned int srcWidth,
                            unsigned int srcHeight,
                            unsigned int srcStride,
                            int32_t* __restrict dst,
                            unsigned int dstStride,
                            int border )
{
   srcStride = (srcStride==0 ? srcWidth : srcStride);
   dstStride = (dstStride==0 ? srcWidth : dstStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );     // multiple of 8
   fcvAssert( (srcStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (dstStride & 0x7) == 0 );    // multiple of 8
   fcvAssert( (srcStride >= srcWidth) );   // Stride is at least as much as Width
   fcvAssert( (dstStride >= srcWidth) );   // Stride is at least as much as Width
#endif

   return (**ppfcvFilterGaussian5x5s32_v2)( src, srcWidth, srcHeight, srcStride, dst, dstStride, border );
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline int
fcvTransformAffineu8( const uint8_t* __restrict nImage,
                       unsigned int imageWidth,
                       unsigned int imageHeight,
                       const float* __restrict nPos,
                       const float* __restrict nAffine,
                       uint8_t* __restrict nPatch,
                       unsigned int patchWidth,
                       unsigned int patchHeight )

{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)nPos & 0x7) == 0 );      // 64-bit alignment
   fcvAssert( ((int)nAffine & 0xF) == 0 );   // 128-bit alignment
#endif

   return (**ppfcvTransformAffineu8)( nImage,imageWidth,imageHeight,
                            nPos,
                            nAffine,
                            nPatch, patchWidth, patchHeight );
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline int
fcvTransformAffineu8_v2( const uint8_t* __restrict nImage,
                         unsigned int imageWidth,
                         unsigned int imageHeight,
                         unsigned int imageStride,
                         const float* __restrict nPos,
                         const float* __restrict nAffine,
                         uint8_t* __restrict nPatch,
                         unsigned int patchWidth,
                         unsigned int patchHeight,
                         unsigned int patchStride )

{
   imageStride = (imageStride==0 ? imageWidth : imageStride);
   patchStride = (patchStride==0 ? patchWidth : patchStride);

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)nPos & 0x7) == 0 );       // 64-bit alignment
   fcvAssert( ((int)nAffine & 0xF) == 0 );    // 128-bit alignment

   fcvAssert( (imageStride >= imageWidth) );  // Stride is at least as much as width
   fcvAssert( (patchStride >= patchWidth) );  // Stride is at least as much as width
#endif

   return (**ppfcvTransformAffineu8_v2)( nImage, imageWidth, imageHeight, imageStride,
                                         nPos, nAffine,
                                         nPatch, patchWidth, patchHeight, patchStride );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvCopyRotated17x17u8( const uint8_t* __restrict region,
                        uint8_t* __restrict patch,
                        int nOri )
{
   (**ppfcvCopyRotated17x17u8)( region, patch, nOri );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvCornerFast9Scoreu8( const uint8_t* __restrict src,
                        unsigned int width,
                        unsigned int height,
                        unsigned int stride,
                        int barrier,
                        unsigned int border,
                        uint32_t* __restrict xy,
                        uint32_t* __restrict scores,
                        unsigned int maxnumcorners,
                        uint32_t* __restrict numcorners )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );     // 128-bit alignment
   fcvAssert( ((int)xy & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)scores & 0xF) == 0 );  // 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );        // multiple of 8
#endif
   fcvAssert( width <= 2048 );

   (**ppfcvCornerFast9Scoreu8)( src, width, height, barrier,border, xy, scores,
                            maxnumcorners, numcorners, stride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvCornerFast9InMaskScoreu8( const uint8_t* __restrict src,
                              unsigned int width,
                              unsigned int height,
                              unsigned int stride,
                              int barrier,
                              unsigned int border,
                              uint32_t* __restrict xy,
                              uint32_t* __restrict scores,
                              unsigned int maxnumcorners,
                              uint32_t* __restrict numcorners,
                              const uint8_t* __restrict bitMask,
                              unsigned int maskWidth,
                              unsigned int maskHeight )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( ((int)xy & 0xF) == 0 );    // 128-bit alignment
   fcvAssert( ((int)scores & 0xF) == 0 );// 128-bit alignment
   fcvAssert( (width & 0x7) == 0 );      // multiple of 8
#endif
   fcvAssert( width <= 2048 );

   (**ppfcvCornerFast9InMaskScoreu8)( src, width, height, barrier, border, xy, scores,
                                  maxnumcorners, numcorners, stride,
                                  bitMask, maskWidth, maskHeight );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvTrackLKOpticalFlowu8( const uint8_t* __restrict          src1,
                           const uint8_t* __restrict         src2,
                           int                               width,
                           int                               height,
                           const fcvPyramidLevel                 *src1Pyr,
                           const fcvPyramidLevel                 *scr2Pyr,
                           const fcvPyramidLevel                 *dx1Pyr,
                           const fcvPyramidLevel                 *dy1Pyr,
                           const float*                      featureXY,
                           float*                            featureXY_out,
                           int32_t*                              featureStatus,
                           int                               featureLen,
                           int                               windowWidth,
                           int                               windowHeight,
                           int                               maxIterations,
                           int                               nPyramidLevels,
                           float                             maxResidue,
                           float                             minDisplacement,
                           float                             minEigenvalue,
                           int                               lightingNormalized)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*featureXY) == 4 );
   fcvAssert( sizeof(*featureXY_out) == 4 );
   fcvAssert( ((int)src1 & 0xF) == 0 );           // 128-bit alignment
   fcvAssert( ((int)src2 & 0xF) == 0 );           // 128-bit alignment
   fcvAssert( ((int)featureXY & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)featureXY_out & 0xF) == 0 );  // 128-bit alignment
   fcvAssert( ((int)featureStatus & 0xF) == 0 );  // 128-bit alignment
   //fcvAssert( (windowWidth & 0x7) == 0 );         // multiple of 8
   #ifndef NDEBUG
   int div = (1 << (nPyramidLevels - 1)) - 1;
   fcvAssert( ( (width & div) == 0 ) & ( (height & div) == 0 ) ); //width and height multiples of 2^(nPyramidLevels-1)
   #endif
#endif

   (**ppfcvTrackLKOpticalFlowu8)( src1, src2, width, height,
                              src1Pyr, scr2Pyr,
                              dx1Pyr, dy1Pyr,
                              featureXY, featureXY_out,
                              featureStatus, featureLen,
                              windowWidth, windowHeight,
                              maxIterations, nPyramidLevels,
                              maxResidue, minDisplacement,
                              minEigenvalue, lightingNormalized);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline void
fcvTrackLKOpticalFlowf32( const uint8_t* __restrict   src1,
                           const uint8_t* __restrict   src2,
                           unsigned int                               width,
                           unsigned int                               height,
                           const fcvPyramidLevel                 *src1Pyr,
                           const fcvPyramidLevel                 *src2Pyr,
                           const fcvPyramidLevel                 *dx1Pyr,
                           const fcvPyramidLevel                 *dy1Pyr,
                           const float*                      featureXY,
                           float*                            featureXY_out,
                           int32_t*                              featureStatus,
                           int                               featureLen,
                           int                               windowWidth,
                           int                               windowHeight,
                           int                               maxIterations,
                           int                               nPyramidLevels,
                           float                             maxResidue,
                           float                             minDisplacement,
                           float                             minEigenvalue,
                           int                               lightingNormalized )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( sizeof(*featureXY) == 4 );
   fcvAssert( sizeof(*featureXY_out) == 4 );
   fcvAssert( ((int)src1 & 0xF) == 0 );           // 128-bit alignment
   fcvAssert( ((int)src2 & 0xF) == 0 );           // 128-bit alignment
   fcvAssert( ((int)featureXY & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)featureXY_out & 0xF) == 0 );  // 128-bit alignment
   fcvAssert( ((int)featureStatus & 0xF) == 0 );  // 128-bit alignment
   //fcvAssert( (windowWidth & 0x7) == 0 );         // multiple of 8
#endif

   (**ppfcvTrackLKOpticalFlowf32)( src1, src2, width, height,
                                   src1Pyr, src2Pyr,
                                   dx1Pyr,dy1Pyr,
                                   featureXY, featureXY_out,
                                   featureStatus, featureLen, windowWidth, windowHeight,
                                   maxIterations, nPyramidLevels, maxResidue,
                                   minDisplacement, minEigenvalue,
                                   lightingNormalized );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvPyramidCreatef32( const float* __restrict base, unsigned int baseWidth,
                     unsigned int baseHeight,
                     unsigned int numLevels, fcvPyramidLevel* pyramid )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)base & 0xF) == 0 );           // 128-bit alignment
   #ifndef NDEBUG
   int div = (1 << (numLevels - 1)) - 1;
   #endif
   fcvAssert( sizeof(*base) == 4 );
   fcvAssert( ( (baseWidth & div) == 0 ) & ( (baseHeight & div) == 0 ) );
#endif

   return (**ppfcvPyramidCreatef32)( base, baseWidth, baseHeight,numLevels, pyramid );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvPyramidCreateu8( const uint8_t* __restrict base, unsigned int baseWidth, unsigned int baseHeight,
                     unsigned int numLevels, fcvPyramidLevel * pyramid )
{

#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   #ifndef NDEBUG
   int div = (1 << (numLevels - 1)) - 1;
   #endif
   fcvAssert( ( (baseWidth & div) == 0 ) & ( (baseHeight & div) == 0 ) );
#endif
  return (**ppfcvPyramidCreateu8)( base, baseWidth, baseHeight,numLevels, pyramid );

}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvPyramidSobelGradientCreatei16( const fcvPyramidLevel* imgPyr,
                                     fcvPyramidLevel* dxPyr,
                                     fcvPyramidLevel* dyPyr, unsigned int numLevels )
{
   return (**ppfcvPyramidSobelGradientCreatei16)( imgPyr,dxPyr, dyPyr, numLevels );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvPyramidSobelGradientCreatei8 ( const fcvPyramidLevel* imgPyr,
                                     fcvPyramidLevel* dxPyr,
                                     fcvPyramidLevel* dyPyr, unsigned int numLevels )
{
   return (**ppfcvPyramidSobelGradientCreatei8)( imgPyr,dxPyr, dyPyr, numLevels );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvPyramidSobelGradientCreatef32 ( const fcvPyramidLevel* imgPyr,
                                     fcvPyramidLevel* dxPyr,
                                     fcvPyramidLevel* dyPyr, unsigned int numLevels )
{
   return (**ppfcvPyramidSobelGradientCreatef32)( imgPyr,dxPyr, dyPyr, numLevels );
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline uint32_t
fcvBitCountu8( const uint8_t* __restrict src, unsigned int len )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0x3) == 0 );           // 32-bit alignment
#endif

   return (**ppfcvBitCountu8)( src, len );
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline uint32_t
fcvBitCount32x1u8( const uint8_t* __restrict src )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0x3) == 0 );           // 128-bit alignment
#endif

   return (**ppfcvBitCount32x1u8)( src );
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline void
fcvBitCount32x4u8( const uint8_t* __restrict A, const uint8_t* __restrict B,
                    const uint8_t* __restrict C, const uint8_t* __restrict D,
                    uint32_t* __restrict count )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)A & 0x3) == 0 );           // 32-bit alignment
   fcvAssert( ((int)B & 0x3) == 0 );           // 32-bit alignment
   fcvAssert( ((int)C & 0x3) == 0 );           // 32-bit alignment
   fcvAssert( ((int)D & 0x3) == 0 );           // 32-bit alignment
   fcvAssert( ((int)count & 0xF) == 0 );       // 128-bit alignment
#endif

   (**ppfcvBitCount32x4u8)( A, B, C, D, count );
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline uint32_t
fcvBitCount64x1u8( const uint8_t* __restrict src )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0x7) == 0 );           // 64-bit alignment
#endif

   return (**ppfcvBitCount64x1u8)( src );
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline void
fcvBitCount64x4u8( const uint8_t* __restrict A, const uint8_t* __restrict B,
                    const uint8_t* __restrict C, const uint8_t* __restrict D,
                    uint32_t* __restrict count )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)A & 0x7) == 0 );           // 32-bit alignment
   fcvAssert( ((int)B & 0x7) == 0 );           // 32-bit alignment
   fcvAssert( ((int)C & 0x7) == 0 );           // 32-bit alignment
   fcvAssert( ((int)D & 0x7) == 0 );           // 32-bit alignment
   fcvAssert( ((int)count & 0xF) == 0 );       // 128-bit alignment
#endif

   (**ppfcvBitCount64x4u8)( A, B, C, D, count );
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline uint32_t
fcvBitCountu32( const uint32_t* __restrict src, unsigned int len )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0x3) == 0 );           // 32-bit alignment
#endif

   return (**ppfcvBitCountu32)( src, len );
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline uint32_t
fcvHammingDistanceu8( const uint8_t* __restrict a,
                       const uint8_t* __restrict b, unsigned int len )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)a & 0x3) == 0 );           // 32-bit alignment
   fcvAssert( ((int)b & 0x3) == 0 );           // 32-bit alignment
#endif

   return (**ppfcvHammingDistanceu8)( a, b, len );
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline uint32_t
fcvHammingDistance32x1u8a4( const uint8_t* __restrict a,
                             const uint8_t* __restrict b )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)a & 0x3) == 0 );           // 32-bit alignment
   fcvAssert( ((int)b & 0x3) == 0 );           // 32-bit alignment
#endif

   return (**ppfcvHammingDistance32x1u8a4)( a, b );
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline uint32_t
fcvHammingDistance64x1u8a4( const uint8_t* __restrict a,
                             const uint8_t* __restrict b )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)a & 0x7) == 0 );           // 64-bit alignment
   fcvAssert( ((int)b & 0x7) == 0 );           // 64-bit alignment
#endif

   return (**ppfcvHammingDistance64x1u8a4)( a, b );
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline uint32_t
fcvHammingDistance32x1u8( const uint8_t* __restrict a,
                           const uint8_t* __restrict b )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)a & 0x3) == 0 );           // 32-bit alignment
   fcvAssert( ((int)b & 0x3) == 0 );           // 32-bit alignment
#endif

   return (**ppfcvHammingDistance32x1u8)( a, b );
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline uint32_t
fcvHammingDistance64x1u8( const uint8_t* __restrict a,
                           const uint8_t* __restrict b )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)a & 0x7) == 0 );           // 64-bit alignment
   fcvAssert( ((int)b & 0x7) == 0 );           // 64-bit alignment
#endif

   return  (**ppfcvHammingDistance64x1u8)( a, b );
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline void
fcvHammingDistance32x4u8a4( const uint8_t* __restrict A,
                             const uint8_t* __restrict B,
                             const uint8_t* __restrict C,
                             const uint8_t* __restrict D,
                             const uint8_t* __restrict E,
                             uint32_t* __restrict distances )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)A & 0x3) == 0 );               // 32-bit alignment
   fcvAssert( ((int)B & 0x3) == 0 );               // 32-bit alignment
   fcvAssert( ((int)C & 0x3) == 0 );               // 32-bit alignment
   fcvAssert( ((int)D & 0x3) == 0 );               // 32-bit alignment
   fcvAssert( ((int)E & 0x3) == 0 );               // 32-bit alignment
   fcvAssert( ((int)distances & 0xF) == 0 );       // 128-bit alignment
#endif

   (**ppfcvHammingDistance32x4u8a4)( A, B, C, D, E, distances );
}



// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline void
fcvHammingDistance64x4u8a4( const uint8_t* __restrict A,
                             const uint8_t* __restrict B,
                             const uint8_t* __restrict C,
                             const uint8_t* __restrict D,
                             const uint8_t* __restrict E,
                             uint32_t* __restrict distances )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)A & 0x7) == 0 );               // 64-bit alignment
   fcvAssert( ((int)B & 0x7) == 0 );               // 64-bit alignment
   fcvAssert( ((int)C & 0x7) == 0 );               // 64-bit alignment
   fcvAssert( ((int)D & 0x7) == 0 );               // 64-bit alignment
   fcvAssert( ((int)E & 0x7) == 0 );               // 64-bit alignment
   fcvAssert( ((int)distances & 0xF) == 0 );       // 128-bit alignment
#endif

   (**ppfcvHammingDistance64x4u8a4)( A, B, C, D, E, distances );
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

inline void
fcvHammingDistance64x4u8( const uint8_t* __restrict A,
                           const uint8_t* __restrict B,
                           const uint8_t* __restrict C,
                           const uint8_t* __restrict D,
                           const uint8_t* __restrict E,
                           uint32_t* __restrict distances )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)A & 0x7) == 0 );               // 64-bit alignment
   fcvAssert( ((int)B & 0x7) == 0 );               // 64-bit alignment
   fcvAssert( ((int)C & 0x7) == 0 );               // 64-bit alignment
   fcvAssert( ((int)D & 0x7) == 0 );               // 64-bit alignment
   fcvAssert( ((int)E & 0x7) == 0 );               // 64-bit alignment
   fcvAssert( ((int)distances & 0xF) == 0 );       // 128-bit alignment
#endif

   (**ppfcvHammingDistance64x4u8)( A, B, C, D, E, distances );
}

inline void fcvMseru8( void *mserHandle,
                const uint8_t* __restrict srcPtr,unsigned int srcWidth, 
                unsigned int srcHeight, unsigned int srcStride, 
                unsigned int maxContours,
                unsigned int * __restrict numContours, unsigned int * __restrict numPointsInContour   ,
                       unsigned int pointsArraySize,
                       unsigned int* __restrict pointsArray
              )

{
    (**ppfcvMseru8)(mserHandle,srcPtr,srcWidth,srcHeight,srcStride 
                    ,maxContours,numContours,numPointsInContour,pointsArraySize,pointsArray);
}

inline void 
fcvMserExtu8( void *mserHandle,
              const uint8_t* __restrict srcPtr,unsigned int srcWidth, 
              unsigned int srcHeight, unsigned int srcStride, 
              unsigned int maxContours,
              unsigned int * __restrict numContours, unsigned int * __restrict numPointsInContour   ,
              unsigned int* __restrict pointsArray, unsigned int pointsArraySize,
              unsigned int * __restrict contourVariation,
              int * __restrict contourPolarity,
              unsigned int * __restrict contourNodeId,
              unsigned int * __restrict contourNodeCounter
            )
{
    (**ppfcvMserExtu8)(mserHandle,srcPtr,srcWidth,srcHeight,srcStride,maxContours, numContours, numPointsInContour,
                       pointsArray,pointsArraySize,contourVariation,contourPolarity,contourNodeId, contourNodeCounter);
}

inline int fcvMserInit(const unsigned int width,
                 const unsigned int height, 
                 unsigned int delta, 
                 unsigned int minArea ,
                 unsigned int maxArea , 
                 float maxVariation ,
                 float minDiversity , void ** mserHandle )
{
    return (*ppfcvMserInit)(width,height,delta,minArea,maxArea,maxVariation, minDiversity, mserHandle);
}

inline void fcvMserRelease(void *mserHandle)
{
    (**ppfcvMserRelease)(mserHandle);
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline int
fcvTrackBMOpticalFlow16x16u8( const uint8_t* __restrict   src1,
                              const uint8_t* __restrict   src2,
                              uint32_t                    srcWidth,
                              uint32_t                    srcHeight,
                              uint32_t                    srcStride,
                              uint32_t                    roiLeft,
                              uint32_t                    roiTop,
                              uint32_t                    roiRight,
                              uint32_t                    roiBottom,
                              uint32_t                    shiftSize,
                              uint32_t                    searchWidth,
                              uint32_t                    searchHeight,
                              uint32_t                    searchStep,
                              uint32_t                    usePrevious,
                              uint32_t *                  numMv,
                              uint32_t *                  locX,
                              uint32_t *                  locY,
                              uint32_t *                  mvX,
                              uint32_t *                  mvY)
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( (((const uint32_t)src1) & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( (((const uint32_t)src1) & 0xF) == 0 );   // 128-bit alignment
   fcvAssert( numMv != NULL );                         // NULL Check
   fcvAssert( locX  != NULL );                         // NULL Check
   fcvAssert( locY  != NULL );                         // NULL Check
   fcvAssert( mvX   != NULL );                         // NULL Check
   fcvAssert( mvY   != NULL );                         // NULL Check
#endif 

   return (**ppfcvTrackBMOpticalFlow16x16u8)( src1, src2, srcWidth, srcHeight, srcStride, 
                                              roiLeft, roiTop, roiRight, roiBottom, shiftSize,
                                              searchWidth, searchHeight, searchStep, usePrevious,
                                              numMv, locX, locY, mvX, mvY );
}


/**
 *  Function to find the bounding rectangle of a set of points.
 * 
 *
 * 
 * @param [in] xy              Set of points (x,y) for which the bounding rectangle has to be found. 
 *                             The points are expressed in interleaved format:  [x1,y1,x2,y2....] 
 * @param [in] numPoints       Number of points in the array
 * @param [out] rectTopLeftX  Lower left's X value for the rectangle.
 * @param [out] rectTopLeftX  Lower Left's Y value for the rectangle;
 * @param [out] rectWidth       Width of the rectangle.
 * @param [out] rectHeight      Height of the rectangle.
 * 
 * @return FASTCV_API void      
 */
inline void fcvBoundingRectangle (const uint32_t * __restrict xy, uint32_t numPoints, 
                                      uint32_t * rectTopLeftX, uint32_t * rectTopLeftY,
                                      uint32_t * rectWidth, uint32_t *rectHeight)
{
   (**ppfcvBoundingRectangle)(xy,numPoints,rectTopLeftX, rectTopLeftY, rectWidth, rectHeight);
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvUpsampleVerticalu8( const uint8_t* __restrict src,
                       uint32_t                  srcWidth,
                       uint32_t                  srcHeight,
                       uint32_t                  srcStride,
                       uint8_t* __restrict       dst,
                       uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvUpsampleVerticalu8)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvUpsampleHorizontalu8( const uint8_t* __restrict src,
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         uint8_t* __restrict       dst,
                         uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvUpsampleHorizontalu8)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvUpsample2Du8( const uint8_t* __restrict src,
                 uint32_t                  srcWidth,
                 uint32_t                  srcHeight,
                 uint32_t                  srcStride,
                 uint8_t* __restrict       dst,
                 uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvUpsample2Du8)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvUpsampleVerticalInterleavedu8( const uint8_t* __restrict src,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvUpsampleVerticalInterleavedu8)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvUpsampleHorizontalInterleavedu8( const uint8_t* __restrict src,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcStride,
                                    uint8_t* __restrict       dst,
                                    uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvUpsampleHorizontalInterleavedu8)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvUpsample2DInterleavedu8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvUpsample2DInterleavedu8)( src, srcWidth, srcHeight, srcStride, dst, dstStride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr444PlanarToRGB565u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCb & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCr & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst   & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth   & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr444PlanarToRGB565u8)( srcY, srcCb, srcCr, 
           srcWidth, srcHeight, srcYStride, srcCbStride, srcCrStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr444PlanarToRGB888u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCb & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCr & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst   & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth   & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr444PlanarToRGB888u8)( srcY, srcCb, srcCr, 
           srcWidth, srcHeight, srcYStride, srcCbStride, srcCrStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr444PlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                    const uint8_t* __restrict srcCb,
                                    const uint8_t* __restrict srcCr,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcYStride,
                                    uint32_t                  srcCbStride,
                                    uint32_t                  srcCrStride,
                                    uint8_t* __restrict       dst,
                                    uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCb & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCr & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst   & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth   & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr444PlanarToRGBA8888u8)( srcY, srcCb, srcCr, 
           srcWidth, srcHeight, srcYStride, srcCbStride, srcCrStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr422PlanarToRGB565u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCb & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCr & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst   & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth   & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr422PlanarToRGB565u8)( srcY, srcCb, srcCr, 
           srcWidth, srcHeight, srcYStride, srcCbStride, srcCrStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr422PlanarToRGB888u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCb & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCr & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst   & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth   & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr422PlanarToRGB888u8)( srcY, srcCb, srcCr, 
           srcWidth, srcHeight, srcYStride, srcCbStride, srcCrStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr422PlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                    const uint8_t* __restrict srcCb,
                                    const uint8_t* __restrict srcCr,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcYStride,
                                    uint32_t                  srcCbStride,
                                    uint32_t                  srcCrStride,
                                    uint8_t* __restrict       dst,
                                    uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCb & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCr & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst   & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth   & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr422PlanarToRGBA8888u8)( srcY, srcCb, srcCr, 
           srcWidth, srcHeight, srcYStride, srcCbStride, srcCrStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr420PlanarToRGB565u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCb & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCr & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst   & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth   & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr420PlanarToRGB565u8)( srcY, srcCb, srcCr, 
           srcWidth, srcHeight, srcYStride, srcCbStride, srcCrStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr420PlanarToRGB888u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCb & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCr & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst   & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth   & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr420PlanarToRGB888u8)( srcY, srcCb, srcCr, 
           srcWidth, srcHeight, srcYStride, srcCbStride, srcCrStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr420PlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                    const uint8_t* __restrict srcCb,
                                    const uint8_t* __restrict srcCr,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcYStride,
                                    uint32_t                  srcCbStride,
                                    uint32_t                  srcCrStride,
                                    uint8_t* __restrict       dst,
                                    uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCb & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcCr & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst   & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth   & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr420PlanarToRGBA8888u8)( srcY, srcCb, srcCr, 
           srcWidth, srcHeight, srcYStride, srcCbStride, srcCrStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr444PseudoPlanarToRGB565u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcC & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth  & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr444PseudoPlanarToRGB565u8)( srcY, srcC, 
           srcWidth, srcHeight, srcYStride, srcCStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr444PseudoPlanarToRGB888u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcC & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth  & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr444PseudoPlanarToRGB888u8)( srcY, srcC, 
           srcWidth, srcHeight, srcYStride, srcCStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr444PseudoPlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                          const uint8_t* __restrict srcC,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCStride,
                                          uint8_t* __restrict       dst,
                                          uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcC & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth  & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr444PseudoPlanarToRGBA8888u8)( srcY, srcC, 
           srcWidth, srcHeight, srcYStride, srcCStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr422PseudoPlanarToRGB565u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcC & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth  & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr422PseudoPlanarToRGB565u8)( srcY, srcC, 
           srcWidth, srcHeight, srcYStride, srcCStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr422PseudoPlanarToRGB888u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcC & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth  & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr422PseudoPlanarToRGB888u8)( srcY, srcC, 
           srcWidth, srcHeight, srcYStride, srcCStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr422PseudoPlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                          const uint8_t* __restrict srcC,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCStride,
                                          uint8_t* __restrict       dst,
                                          uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcC & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth  & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr422PseudoPlanarToRGBA8888u8)( srcY, srcC, 
           srcWidth, srcHeight, srcYStride, srcCStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr420PseudoPlanarToRGB565u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcC & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth  & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr420PseudoPlanarToRGB565u8)( srcY, srcC, 
           srcWidth, srcHeight, srcYStride, srcCStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr420PseudoPlanarToRGB888u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcC & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth  & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr420PseudoPlanarToRGB888u8)( srcY, srcC, 
           srcWidth, srcHeight, srcYStride, srcCStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvColorYCbCr420PseudoPlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                          const uint8_t* __restrict srcC,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCStride,
                                          uint8_t* __restrict       dst,
                                          uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)srcY & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)srcC & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth  & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvColorYCbCr420PseudoPlanarToRGBA8888u8)( srcY, srcC, 
           srcWidth, srcHeight, srcYStride, srcCStride, dst, dstStride );
}


inline void
fcvEdgeWeightings16( int16_t* __restrict edgeMap,        
                     const uint32_t      edgeMapWidth,          
                     const uint32_t      edgeMapHeight,         
                     const uint32_t      edgeMapStride,        
                     const uint32_t      weight,         
                     const uint32_t      edge_limit,     
                     const uint32_t      hl_threshold,   
                     const uint32_t      hh_threshold,   
                     const uint32_t      edge_denoise_factor )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)edgeMap & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (edgeMapWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvEdgeWeightings16)( edgeMap, edgeMapWidth, edgeMapHeight, edgeMapStride, 
                              weight, edge_limit, hl_threshold, hh_threshold, edge_denoise_factor );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDeinterleaveu8( const uint8_t* __restrict src,
                   uint32_t                  srcWidth,
                   uint32_t                  srcHeight,
                   uint32_t                  srcStride,
                   uint8_t* __restrict       dst0,
                   uint32_t                  dst0Stride,
                   uint8_t* __restrict       dst1,
                   uint32_t                  dst1Stride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst0 & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst1 & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth  & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvDeinterleaveu8)( src, srcWidth, srcHeight, srcStride, 
                            dst0, dst0Stride, dst1, dst1Stride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvInterleaveu8( const uint8_t* __restrict src0,
                 const uint8_t* __restrict src1,
                 uint32_t                  imageWidth,
                 uint32_t                  imageHeight,
                 uint32_t                  src0Stride,
                 uint32_t                  src1Stride,
                 uint8_t* __restrict       dst,
                 uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src0  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)src1  & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst   & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (imageWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvInterleaveu8)( src0, src1, imageWidth, imageHeight, 
                          src0Stride, src1Stride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDWTHarrTransposeu8( const uint8_t* __restrict src,                       
                       uint32_t                  srcWidth,
                       uint32_t                  srcHeight,
                       uint32_t                  srcStride,
                       int16_t* __restrict       dst,
                       uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvDWTHaarTransposeu8)( src, srcWidth, srcHeight, srcStride, 
                                dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDWTHaarTransposeu8( const uint8_t* __restrict src,                       
                       uint32_t                  srcWidth,
                       uint32_t                  srcHeight,
                       uint32_t                  srcStride,
                       int16_t* __restrict       dst,
                       uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvDWTHaarTransposeu8)( src, srcWidth, srcHeight, srcStride, 
                                dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvDWT53TabTransposes16( const int16_t* __restrict src,                       
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         int16_t* __restrict       dst,
                         uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvDWT53TabTransposes16)( src, srcWidth, srcHeight, srcStride, 
                                  dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvIDWT53TabTransposes16( const int16_t*   __restrict src,                       
                          uint32_t                    srcWidth,
                          uint32_t                    srcHeight,
                          uint32_t                    srcStride,
                          int16_t* __restrict         dst,
                          uint32_t                    dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvIDWT53TabTransposes16)( src, srcWidth, srcHeight, srcStride, 
                                   dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvIDWTHarrTransposes16( const int16_t* __restrict src,                       
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         uint8_t* __restrict       dst,
                         uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvIDWTHaarTransposes16)( src, srcWidth, srcHeight, srcStride, 
                                  dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvIDWTHaarTransposes16( const int16_t* __restrict src,                       
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         uint8_t* __restrict       dst,
                         uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvIDWTHaarTransposes16)( src, srcWidth, srcHeight, srcStride, 
                                  dst, dstStride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvScaleUpPolyu8( const uint8_t* __restrict src,
                  uint32_t                  srcWidth,
                  uint32_t                  srcHeight,
                  uint32_t                  srcStride,
                  uint8_t* __restrict       dst,
                  uint32_t                  dstWidth,
                  uint32_t                  dstHeight,
                  uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (dstWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvScaleUpPolyu8)( src, srcWidth, srcHeight, srcStride,
                           dst, dstWidth, dstHeight, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvScaleUpPolyInterleaveu8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstWidth,
                            uint32_t                  dstHeight,
                            uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (dstWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvScaleUpPolyInterleaveu8)( src, srcWidth, srcHeight, srcStride,
                                     dst, dstWidth, dstHeight, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvScaleDownMNu8( const uint8_t* __restrict src,
                  uint32_t                  srcWidth,
                  uint32_t                  srcHeight,
                  uint32_t                  srcStride,
                  uint8_t* __restrict       dst,
                  uint32_t                  dstWidth,
                  uint32_t                  dstHeight,
                  uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (dstWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvScaleDownMNu8)( src, srcWidth, srcHeight, srcStride,
                           dst, dstWidth, dstHeight, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline void
fcvScaleDownMNInterleaveu8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstWidth,
                            uint32_t                  dstHeight,
                            uint32_t                  dstStride )
{
#ifndef FASTCV_DISABLE_API_ENFORCEMENT
   fcvAssert( ((int)src & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( ((int)dst & 0xF) == 0 );      // 128-bit alignment
   fcvAssert( (srcWidth & 0x7) == 0 );      // multiple of 8
   fcvAssert( (dstWidth & 0x7) == 0 );      // multiple of 8
#endif

   (**ppfcvScaleDownMNInterleaveu8)( src, srcWidth, srcHeight, srcStride,
                                     dst, dstWidth, dstHeight, dstStride );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline uint32_t
fcvKMeansTreeSearch36x10s8( const   int8_t* __restrict nodeChildrenCenter,
                            const uint32_t* __restrict nodeChildrenInvLenQ32,
                            const uint32_t* __restrict nodeChildrenIndex,
                            const  uint8_t* __restrict nodeNumChildren,
                                  uint32_t             numNodes,
                            const   int8_t* __restrict key )
{
  return (**ppfcvKMeansTreeSearch36x10s8)( nodeChildrenCenter,
                                           nodeChildrenInvLenQ32,
                                           nodeChildrenIndex,
                                           nodeNumChildren,
                                           numNodes,
                                           key );

}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline void
fcvLinearSearch8x36s8(
   const uint32_t * __restrict dbLUT,
   uint32_t                    numDBLUT,
   const int8_t   * __restrict descDB,
   const uint32_t * __restrict descDBInvLenQ38,
   const uint16_t * __restrict descDBTargetId,
   uint32_t                    numDescDB,
   const int8_t   * __restrict srcDesc,
   const uint32_t * __restrict srcDescInvLenQ38,
   const uint32_t * __restrict srcDescIdx,
   uint32_t                    numSrcDesc,
   const uint16_t * __restrict targetsToIgnore,
   uint32_t                    numTargetsToIgnore,
   uint32_t                    maxDistanceQ31,
   uint32_t       * __restrict correspondenceDBIdx,
   uint32_t       * __restrict correspondencSrcDescIdx,
   uint32_t       * __restrict correspondenceDistanceQ31,
   uint32_t                    maxNumCorrespondences,
   uint32_t       * __restrict numCorrespondences )
{
  return (**ppfcvLinearSearch8x36s8)( dbLUT,
                                      numDBLUT,
                                      descDB,
                                      descDBInvLenQ38,
                                      descDBTargetId,
                                      numDescDB,
                                      srcDesc,
                                      srcDescInvLenQ38,
                                      srcDescIdx,
                                      numSrcDesc,
                                      targetsToIgnore,
                                      numTargetsToIgnore,
                                      maxDistanceQ31,
                                      correspondenceDBIdx,
                                      correspondencSrcDescIdx,
                                      correspondenceDistanceQ31,
                                      maxNumCorrespondences,
                                      numCorrespondences );
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

inline int
fcvLinearSearchPrepare8x36s8(
   uint32_t * __restrict dbLUT,
   uint32_t              numDBLUT,
   int8_t   * __restrict descDB,
   uint32_t * __restrict descDBInvLenQ38,
   uint16_t * __restrict descDBTargetId,
   uint32_t * __restrict idxLUT,
   uint32_t              numDescDB )
{
  return (**ppfcvLinearSearchPrepare8x36s8_v2)( dbLUT,
                                             numDBLUT,
                                             descDB,
                                             descDBInvLenQ38,
                                             descDBTargetId,
                                             idxLUT,
                                             numDescDB  );
}


inline void
fcvFindContoursExternalu8( uint8_t* __restrict   src,
                           uint32_t              srcWidth,
                           uint32_t              srcHeight,
                           uint32_t              srcStride,
                           uint32_t              maxNumContours,
                           uint32_t* __restrict  numContours,
                           uint32_t* __restrict  numContourPoints,
                           uint32_t** __restrict contourStartPoints,
                           uint32_t* __restrict  pointBuffer,
                           uint32_t              pointBufferSize,
                           int32_t               hierarchy[][4],
                           void*                 contourHandle )
{
    return(**ppfcvFindContoursExternalu8)(src,srcWidth,srcHeight,srcStride,maxNumContours,numContours,numContourPoints,
                                          contourStartPoints,pointBuffer,pointBufferSize,hierarchy,contourHandle);
}

inline void
fcvFindContoursListu8( uint8_t* __restrict   src,
                       uint32_t              srcWidth,
                       uint32_t              srcHeight,
                       uint32_t              srcStride,
                       uint32_t              maxNumContours,
                       uint32_t* __restrict  numContours,
                       uint32_t* __restrict  numContourPoints,
                       uint32_t** __restrict contourStartPoints,
                       uint32_t* __restrict  pointBuffer,
                       uint32_t              pointBufferSize,
                       void*                 contourHandle )
{
    return(**ppfcvFindContoursListu8)(src,srcWidth,srcHeight,srcStride,maxNumContours,numContours,
                                      numContourPoints,contourStartPoints,pointBuffer,pointBufferSize,contourHandle);
}

inline void
fcvFindContoursCcompu8( uint8_t* __restrict   src,
                        uint32_t              srcWidth,
                        uint32_t              srcHeight,
                        uint32_t              srcStride,
                        uint32_t              maxNumContours,
                        uint32_t*__restrict   numContours,
                        uint32_t* __restrict  holeFlag,
                        uint32_t* __restrict  numContourPoints,
                        uint32_t** __restrict contourStartPoints,
                        uint32_t* __restrict  pointBuffer,
                        uint32_t              pointBufferSize,
                        int32_t               hierarchy[][4],
                        void*                 contourHandle )
{
    return(**ppfcvFindContoursCcompu8) (src,srcWidth,srcHeight,srcStride,maxNumContours,numContours,
                                        holeFlag,numContourPoints,
                                        contourStartPoints,pointBuffer,pointBufferSize,hierarchy,contourHandle);
}

inline  void
fcvFindContoursTreeu8( uint8_t* __restrict   src,
                       uint32_t              srcWidth,
                       uint32_t              srcHeight,
                       uint32_t              srcStride,
                       uint32_t              maxNumContours,
                       uint32_t* __restrict  numContours,
                       uint32_t* __restrict  holeFlag,
                       uint32_t* __restrict  numContourPoints,
                       uint32_t** __restrict contourStartPoints,
                       uint32_t* __restrict  pointBuffer,
                       uint32_t              pointBufferSize,
                       int32_t               hierarchy[][4],
                       void*                 contourHandle )
{
    return(**ppfcvFindContoursTreeu8)(src,srcWidth,srcHeight,srcStride,maxNumContours,numContours,holeFlag,numContourPoints,
                                      contourStartPoints,pointBuffer,pointBufferSize,hierarchy,contourHandle);
}

inline  void*
fcvFindContoursAllocate( uint32_t srcStride )
{
    return(**ppfcvFindContoursAllocate) (srcStride);
}

inline void 
fcvFindContoursDelete( void* contourHandle )
{
    (**ppfcvFindContoursDelete) (  contourHandle );
}


inline void
fcvSolvef32(const float32_t * __restrict A, 
            int32_t numCols, 
            int32_t numRows, 
            const float32_t * __restrict b, 
            float32_t * __restrict x)
{
    return(**ppfcvSolvef32)(A,numCols,numRows,b,x);

}

inline  void
fcvGetPerspectiveTransformf32( const float32_t src1[8],
                               const float32_t src2[8],
                               float32_t  transformCoefficient[9] )

{
    (**ppfcvGetPerspectiveTransformf32)(src1,src2,transformCoefficient);

}
inline void
fcvSetElementsu8(   uint8_t * __restrict src,
                           uint32_t             srcWidth, 
                           uint32_t             srcHeight,
                           uint32_t             srcStride, 
                           uint8_t              value,
                     const uint8_t * __restrict mask,
                           uint32_t             maskStride  
                    )
{
   (**ppfcvSetElementsu8)( src, srcWidth, srcHeight, srcStride,  value, mask, maskStride );
}

inline void
fcvSetElementss32(   int32_t * __restrict src, 
                            uint32_t             srcWidth, 
                            uint32_t             srcHeight,
                            uint32_t             srcStride, 
                            int32_t              value,
                      const uint8_t * __restrict mask ,
                            uint32_t             maskStride
                     )
{
    (**ppfcvSetElementss32)( src, srcWidth, srcHeight, srcStride, value, mask , maskStride);
  
}

inline void
fcvSetElementsf32(   float32_t * __restrict src, 
                            uint32_t               srcWidth, 
                            uint32_t               srcHeight,
                            uint32_t               srcStride, 
                            float32_t              value,
                      const uint8_t   * __restrict mask,
                            uint32_t               maskStride
                     )
{
    (**ppfcvSetElementsf32)( src, srcWidth, srcHeight, srcStride, value, mask, maskStride);
                 
}

inline  void
fcvSetElementsc4u8(  uint8_t * __restrict src,
                           uint32_t             srcWidth, 
                           uint32_t             srcHeight,
                           uint32_t             srcStride, 
                           uint8_t              value1,
                           uint8_t              value2,
                           uint8_t              value3,
                           uint8_t              value4,
                     const uint8_t * __restrict mask,
                           uint32_t             maskStride
                    )
{
    (**ppfcvSetElementsc4u8)( src, srcWidth, srcHeight, srcStride, value1, value2, value3, value4, mask, maskStride );

}

inline void
fcvSetElementsc4s32(  int32_t * __restrict src, 
                            uint32_t             srcWidth, 
                            uint32_t             srcHeight,
                            uint32_t             srcStride, 
                            int32_t              value1,
                            int32_t              value2,
                            int32_t              value3,
                            int32_t              value4,
                      const uint8_t * __restrict mask,
                            uint32_t             maskStride 
                     )
{
    (**ppfcvSetElementsc4s32)( src, srcWidth, srcHeight, srcStride, value1, value2, value3, value4, mask, maskStride );
}

inline void
fcvSetElementsc4f32(  float32_t * __restrict src, 
                            uint32_t               srcWidth, 
                            uint32_t               srcHeight,
                            uint32_t               srcStride, 
                            float32_t              value1,
                            float32_t              value2,
                            float32_t              value3,
                            float32_t              value4,
                      const uint8_t   * __restrict mask,
                            uint32_t               maskStride
                     )
{
    (**ppfcvSetElementsc4f32)( src, srcWidth, srcHeight, srcStride, value1, value2, value3, value4, mask, maskStride );
}

inline  void
fcvSetElementsc3u8(  uint8_t * __restrict src,
                           uint32_t             srcWidth, 
                           uint32_t             srcHeight,
                           uint32_t             srcStride, 
                           uint8_t              value1,
                           uint8_t              value2,
                           uint8_t              value3,
                     const uint8_t * __restrict mask,
                           uint32_t             maskStride
                    )
{
    return (**ppfcvSetElementsc3u8)( src, srcWidth, srcHeight, srcStride, value1, value2, value3,  mask, maskStride );

}

inline void
fcvSetElementsc3s32(  int32_t * __restrict src, 
                            uint32_t             srcWidth, 
                            uint32_t             srcHeight,
                            uint32_t             srcStride, 
                            int32_t              value1,
                            int32_t              value2,
                            int32_t              value3,
                      const uint8_t * __restrict mask,
                            uint32_t             maskStride 
                     )
{
    (**ppfcvSetElementsc3s32)( src, srcWidth, srcHeight, srcStride, value1, value2, value3,  mask, maskStride );
}

inline void
fcvSetElementsc3f32(  float32_t * __restrict src, 
                            uint32_t               srcWidth, 
                            uint32_t               srcHeight,
                            uint32_t               srcStride, 
                            float32_t              value1,
                            float32_t              value2,
                            float32_t              value3,
                      const uint8_t   * __restrict mask,
                            uint32_t               maskStride
                     )
{
    (**ppfcvSetElementsc3f32)( src, srcWidth, srcHeight, srcStride, value1, value2, value3,  mask, maskStride );
}




inline void
fcvAdaptiveThresholdGaussian3x3u8( const uint8_t* __restrict src,
                                   uint32_t             srcWidth,
                                   uint32_t             srcHeight,
                                   uint32_t             srcStride,
                                   uint8_t              maxValue,
                                   fcvThreshType        thresholdType,
                                   int32_t              value,
                                   uint8_t* __restrict  dst,
                                   uint32_t             dstStride )
{
    (**ppfcvAdaptiveThresholdGaussian3x3u8)(  src,srcWidth, srcHeight, srcStride,
                                             maxValue,thresholdType, value, dst,dstStride );
}
inline void
fcvAdaptiveThresholdGaussian5x5u8( const uint8_t* __restrict src,
                                   uint32_t             srcWidth,
                                   uint32_t             srcHeight,
                                   uint32_t             srcStride,
                                   uint8_t              maxValue,
                                   fcvThreshType        thresholdType,
                                   int32_t              value,
                                   uint8_t* __restrict  dst,
                                   uint32_t             dstStride )
{
    (**ppfcvAdaptiveThresholdGaussian5x5u8)(  src,srcWidth, srcHeight, srcStride,
                                             maxValue,thresholdType, value, dst,dstStride );
}

inline void
fcvAdaptiveThresholdGaussian11x11u8( const uint8_t* __restrict src,
                                     uint32_t             srcWidth,
                                     uint32_t             srcHeight,
                                     uint32_t             srcStride,
                                     uint8_t              maxValue,
                                     fcvThreshType        thresholdType,
                                     int32_t              value,
                                     uint8_t* __restrict  dst,
                                     uint32_t             dstStride )
{
    (**ppfcvAdaptiveThresholdGaussian11x11u8)(  src,srcWidth, srcHeight, srcStride,
                                               maxValue,thresholdType, value, dst,dstStride );
}

inline void
fcvAdaptiveThresholdMean3x3u8( const uint8_t* __restrict src,
                               uint32_t             srcWidth,
                               uint32_t             srcHeight,
                               uint32_t             srcStride,
                               uint8_t              maxValue,
                               fcvThreshType        thresholdType,
                               int32_t              value,
                               uint8_t* __restrict  dst,
                               uint32_t             dstStride )
{
    (**ppfcvAdaptiveThresholdMean3x3u8)(  src,srcWidth, srcHeight, srcStride,
                                         maxValue,thresholdType, value, dst,dstStride );

}

inline void
fcvAdaptiveThresholdMean5x5u8( const uint8_t* __restrict src,
                               uint32_t             srcWidth,
                               uint32_t             srcHeight,
                               uint32_t             srcStride,
                               uint8_t              maxValue,
                               fcvThreshType        thresholdType,
                               int32_t              value,
                               uint8_t* __restrict  dst,
                               uint32_t             dstStride )
{
    (**ppfcvAdaptiveThresholdMean5x5u8)(  src,srcWidth, srcHeight, srcStride,
                                         maxValue,thresholdType, value, dst,dstStride );
}

inline void
fcvAdaptiveThresholdMean11x11u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride )
{
    (**ppfcvAdaptiveThresholdMean11x11u8)(  src,srcWidth, srcHeight, srcStride,
                                         maxValue,thresholdType, value, dst,dstStride );
}

inline void 
fcvBoxFilter3x3u8( const uint8_t* __restrict src, 
                         uint32_t            srcWidth,
                         uint32_t            srcHeight,
                         uint32_t            srcStride, 
                         uint8_t* __restrict dst, 
                         uint32_t            dstStride 
                   )
{
    (*ppfcvBoxFilter3x3u8)(src,srcWidth,srcHeight,srcStride,dst,dstStride);
    
}


inline void 
fcvBoxFilter5x5u8( const uint8_t* __restrict src, 
                         uint32_t            srcWidth,
                         uint32_t            srcHeight,
                         uint32_t            srcStride, 
                         uint8_t* __restrict dst, 
                         uint32_t            dstStride 
                   )
{
    (*ppfcvBoxFilter5x5u8)(src,srcWidth,srcHeight,srcStride,dst,dstStride);
    
}


inline void 
fcvBoxFilter11x11u8(const uint8_t* __restrict src, 
                          uint32_t            srcWidth,
                          uint32_t            srcHeight,
                          uint32_t            srcStride, 
                          uint8_t* __restrict dst, 
                          uint32_t            dstStride 
                   )
{
     (**ppfcvBoxFilter11x11u8)(src,srcWidth,srcHeight,srcStride,dst,dstStride);   
}

inline void 
fcvBilateralFilter5x5u8(const uint8_t* __restrict src, 
                               uint32_t            srcWidth,
                               uint32_t            srcHeight,
                               uint32_t            srcStride, 
                               uint8_t* __restrict dst, 
                               uint32_t            dstStride 
                        )
{
   (**ppfcvBilateralFilter5x5u8)(src,srcWidth,srcHeight,srcStride,dst,dstStride);    
}



inline void 
fcvBilateralFilter7x7u8(const uint8_t* __restrict src, 
                        uint32_t            srcWidth,
                        uint32_t            srcHeight,
                        uint32_t            srcStride, 
                        uint8_t* __restrict dst, 
                        uint32_t            dstStride 
                       )
{

      (**ppfcvBilateralFilter7x7u8)(src,srcWidth,srcHeight,srcStride,dst,dstStride);    
}

inline void 
fcvBilateralFilter9x9u8(const uint8_t* __restrict src, 
                        uint32_t            srcWidth,
                        uint32_t            srcHeight,
                        uint32_t            srcStride, 
                        uint8_t* __restrict dst, 
                        uint32_t            dstStride 
                       )
{
    (**ppfcvBilateralFilter9x9u8)(src,srcWidth,srcHeight,srcStride,dst,dstStride);   
}

inline void
fcvSegmentFGMasku8(uint8_t* __restrict    src,
                 uint32_t               srcWidth,
                 uint32_t               srcHeight,
                 uint32_t               srcStride,
                 uint8_t                Polygonal,
                 uint32_t              perimScale)
{
    (**ppfcvSegmentFGMasku8)(src,srcWidth,srcHeight,srcStride,Polygonal,perimScale);
}


inline void
fcvAbsDiffu8(const uint8_t * __restrict src1, 
                   const uint8_t * __restrict src2,   
                   uint32_t             srcWidth,
                   uint32_t             srcHeight,
                   uint32_t             srcStride,
                   uint8_t * __restrict dst,
                   uint32_t             dstStride )
{
     srcStride = (srcStride==0 ? srcWidth*sizeof (uint8_t) : srcStride);
     dstStride = (dstStride==0 ? srcWidth*sizeof (uint8_t) : dstStride);


     (*ppfcvAbsDiffu8)(src1,src2,srcWidth,srcHeight,srcStride,dst,dstStride);   
}



inline void
fcvAbsDiffs32(const int32_t * __restrict  src1, 
              const int32_t * __restrict  src2,   
                    uint32_t              srcWidth,
                    uint32_t              srcHeight,
                    uint32_t              srcStride,
                    int32_t * __restrict  dst,
                    uint32_t              dstStride )
{
     srcStride = (srcStride==0 ? srcWidth*sizeof (int32_t) : srcStride);
     dstStride = (dstStride==0 ? srcWidth*sizeof (int32_t) : dstStride);

     (*ppfcvAbsDiffs32)(src1,src2,srcWidth,srcHeight,srcStride,dst,dstStride);   
}



inline void
fcvAbsDifff32(const float32_t * __restrict  src1, 
              const float32_t * __restrict  src2,    
                    uint32_t                srcWidth,
                    uint32_t                srcHeight,
                    uint32_t                srcStride,
                    float32_t * __restrict  dst,
                    uint32_t                dstStride )
{
     srcStride = (srcStride==0 ? srcWidth*sizeof (float32_t) : srcStride);
     dstStride = (dstStride==0 ? srcWidth*sizeof (float32_t) : dstStride);

     (*ppfcvAbsDifff32)(src1,src2,srcWidth,srcHeight,srcStride,dst,dstStride);   
}


inline void
fcvAbsDiffVu8(const uint8_t * __restrict src, 
                    uint8_t              value,   
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
                    uint8_t * __restrict dst,
                    uint32_t             dstStride )
{
     srcStride = (srcStride==0 ? srcWidth*sizeof (uint8_t) : srcStride);
     dstStride = (dstStride==0 ? srcWidth*sizeof (uint8_t) : dstStride);

     (**ppfcvAbsDiffVu8)(src,value,srcWidth,srcHeight,srcStride,dst,dstStride);
}

inline void
fcvAbsDiffVs32(const int32_t * __restrict src, 
                     int32_t              value, 
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
                     int32_t * __restrict dst,
                     uint32_t             dstStride )
{
     srcStride = (srcStride==0 ? srcWidth*sizeof (int32_t) : srcStride);
     dstStride = (dstStride==0 ? srcWidth*sizeof (int32_t) : dstStride);

     (**ppfcvAbsDiffVs32)(src,value,srcWidth,srcHeight,srcStride,dst,dstStride); 
}

inline void
fcvAbsDiffVf32(const float32_t * __restrict src, 
                     float32_t              value,   
                     uint32_t               srcWidth,
                     uint32_t               srcHeight,
                     uint32_t               srcStride,
                     float32_t * __restrict dst,
                     uint32_t               dstStride )
{
     srcStride = (srcStride==0 ? srcWidth*sizeof (float32_t) : srcStride);
     dstStride = (dstStride==0 ? srcWidth*sizeof (float32_t) : dstStride);

     (**ppfcvAbsDiffVf32)(src,value,srcWidth,srcHeight,srcStride,dst,dstStride);   
}


inline void
fcvAbsDiffVc4u8(const uint8_t * __restrict src, 
                    uint8_t              value1,
                    uint8_t              value2,
                    uint8_t              value3,
                    uint8_t              value4,
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
                    uint8_t * __restrict dst,
                    uint32_t             dstStride )
{
     srcStride = (srcStride==0 ? srcWidth*4*sizeof (uint8_t) : srcStride);
     dstStride = (dstStride==0 ? srcWidth*4*sizeof (uint8_t) : dstStride);

     (*ppfcvAbsDiffVc4u8)(src,value1,value2,value3,value4,
                          srcWidth,srcHeight,srcStride,dst,dstStride);
}


inline void
fcvAbsDiffVc4s32(const int32_t * __restrict src, 
                     int32_t              value1,
                     int32_t              value2,
                     int32_t              value3,
                     int32_t              value4,
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
                     int32_t * __restrict dst,
                     uint32_t             dstStride )
{
     
     srcStride = (srcStride==0 ? srcWidth*4*sizeof (int32_t) : srcStride);
     dstStride = (dstStride==0 ? srcWidth*4*sizeof (int32_t) : dstStride);

     (*ppfcvAbsDiffVc4s32)(src,value1,value2,value3,value4,
                          srcWidth,srcHeight,srcStride,dst,dstStride);  
}

inline void
fcvAbsDiffVc4f32(const float32_t * __restrict src, 
                     float32_t              value1,   
                     float32_t              value2,
                     float32_t              value3,
                     float32_t              value4,
                     uint32_t               srcWidth,
                     uint32_t               srcHeight,
                     uint32_t               srcStride,
                     float32_t * __restrict dst,
                     uint32_t               dstStride)
{
     
     srcStride = (srcStride==0 ? srcWidth*4*sizeof (float32_t) : srcStride);
     dstStride = (dstStride==0 ? srcWidth*4*sizeof (float32_t) : dstStride);

     (*ppfcvAbsDiffVc4f32)(src,value1,value2,value3,value4,
                          srcWidth,srcHeight,srcStride,dst,dstStride);
}

inline void
fcvAbsDiffVc3u8(const uint8_t * __restrict src, 
                    uint8_t              value1,
                    uint8_t              value2,
                    uint8_t              value3,
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
                    uint8_t * __restrict dst,
                    uint32_t             dstStride )
{
     srcStride = (srcStride==0 ? srcWidth*3*sizeof (uint8_t) : srcStride);
     dstStride = (dstStride==0 ? srcWidth*3*sizeof (uint8_t) : dstStride);

     (**ppfcvAbsDiffVc3u8)(src,value1,value2,value3,srcWidth,srcHeight,srcStride,dst,dstStride);
}

inline void
fcvAbsDiffVc3s32(const int32_t * __restrict src, 
                     int32_t              value1,
                     int32_t              value2,
                     int32_t              value3,
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
                     int32_t * __restrict dst,
                     uint32_t             dstStride )
{
     srcStride = (srcStride==0 ? srcWidth*3*sizeof (int32_t) : srcStride);
     dstStride = (dstStride==0 ? srcWidth*3*sizeof (int32_t) : dstStride);

     (**ppfcvAbsDiffVc3s32)(src,value1,value2,value3,srcWidth,srcHeight,srcStride,dst,dstStride);   
}

inline void
fcvAbsDiffVc3f32(const float32_t * __restrict src, 
                     float32_t              value1,   
                     float32_t              value2,
                     float32_t              value3,
                     uint32_t               srcWidth,
                     uint32_t               srcHeight,
                     uint32_t               srcStride,
                     float32_t * __restrict dst,
                     uint32_t               dstStride)
{
     srcStride = (srcStride==0 ? srcWidth*3*sizeof (float32_t) : srcStride);
     dstStride = (dstStride==0 ? srcWidth*3*sizeof (float32_t) : dstStride);

     (**ppfcvAbsDiffVc3f32)(src,value1,value2,value3,srcWidth,srcHeight,srcStride,dst,dstStride); 
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline
int fcvKDTreeCreate36s8f32( const        int8_t*  __restrict vectors,
                            const     float32_t*  __restrict invLengths,
                                            int              numVectors,
                             fcvKDTreeDatas8f32**            kdtrees )
{
  return (**ppfcvKDTreeCreate36s8f32)(vectors, invLengths, numVectors, kdtrees );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline
int fcvKDTreeDestroy36s8f32( fcvKDTreeDatas8f32* kdtrees )
{
  return (**ppfcvKDTreeDestroy36s8f32) ( kdtrees );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline
int fcvKDTreeQuery36s8f32( fcvKDTreeDatas8f32*       kdtrees,
                           const  int8_t* __restrict query,
                               float32_t             queryInvLen,
                                     int             maxNNs,
                               float32_t             maxDist,
                                     int             maxChecks,
                           const uint8_t* __restrict mask,
                                 int32_t*             numNNsFound,
                                 int32_t* __restrict NNInds,
                               float32_t* __restrict NNDists )
{
  return (**ppfcvKDTreeQuery36s8f32) ( kdtrees, query, queryInvLen, maxNNs,
                                     maxDist, maxChecks, mask, numNNsFound,
                                     NNInds, NNDists );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline
void fcvBitwiseOru8
(
 	const uint8_t* __restrict src1, 
	const uint8_t* __restrict src2,  
	uint32_t                  srcWidth,
	uint32_t                  srcHeight,
	uint32_t                  srcStride,
	uint8_t * __restrict      dst,
	uint32_t                  dstStride,
	uint8_t * __restrict      mask,
	uint32_t                  maskStride 
)
{
  return (**ppfcvBitwiseOru8) ( src1, src2, srcWidth, srcHeight,
                                     srcStride, dst, dstStride, mask,
                                     maskStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline
void fcvBitwiseOrs32
(
 	const int32_t* __restrict src1, 
	const int32_t* __restrict src2,  
	uint32_t                  srcWidth,
	uint32_t                  srcHeight,
	uint32_t                  srcStride,
	int32_t * __restrict      dst,
	uint32_t                  dstStride,
	uint8_t * __restrict      mask,
	uint32_t                  maskStride
)
{
  return (**ppfcvBitwiseOrs32) ( src1, src2, srcWidth, srcHeight,
                                     srcStride, dst, dstStride, mask,
                                     maskStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline
void fcvColorRGB888ToGrayu8
( 
 	const uint8_t* __restrict src,
	uint32_t 			 srcWidth,
	uint32_t 			srcHeight,
	uint32_t 			srcStride,
	uint8_t* 	   __restrict dst,
	uint32_t  			dstStride
)
{
  return (**ppfcvColorRGB888ToGrayu8) ( src, srcWidth, srcHeight,
                                     srcStride, dst, dstStride );
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline
void fcvTiltedIntegralu8s32
( 
 	const uint8_t* __restrict src,
	uint32_t 			 srcWidth,
	uint32_t 			srcHeight,
	uint32_t 			srcStride,
	int32_t* __restrict 	  dst,
	uint32_t 			dstStride
)
{
  return (**ppfcvTiltedIntegralu8s32) ( src, srcWidth, srcHeight,
                                     srcStride, dst, dstStride );
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline
void fcvConvValids16
( 
	const int16_t* __restrict src1,
	uint32_t 			  src1Width,
	uint32_t 			 src1Height,
	uint32_t            src1Stride,
	const int16_t* __restrict src2,
	uint32_t 			  src2Width,
	uint32_t 			 src2Height,
	uint32_t 			 src2Stride,
	int32_t* __restrict 		dst,
	uint32_t 			  dstStride
)
{
  return (**ppfcvConvValids16) ( src1, src1Width, src1Height, src1Stride, 
		  						 src2, src2Width, src2Height, src2Stride, 
								 dst, dstStride );
}








