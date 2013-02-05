#ifndef FASTCV_H
#define FASTCV_H

/**=============================================================================

@file
   fastcv.h

@brief
   Public API


Copyright (c) 2011 QUALCOMM Incorporated.
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

=============================================================================**/

/**=============================================================================
@mainpage FastCV Public API Documentation

@version 1.1.1

@section Overview Overview

FastCV provides two main features to computer vision application developers:
   - First, it provides a library of  frequently used computer vision (CV)
   functions, optimized to run efficiently on mobile devices.
   - Second, it provides a clean processor-agnostic hardware acceleration API,
   under which  chipset vendors can hardware accelerate FastCV functions on
   their hardware.

This initial release (FastCV 1.0) only supports Android mobile developers;
however, we intend to support iOS and Windows devices as soon as possible.
FastCV 1.0 is available for download for free from developer.qualcomm.com.

FastCV 1.0 is released as a unified binary, a single binary containing two
implementations of the library.
   - The first implementation runs on any ARM processor, and is referred to as
   the "FastCV for ARM."
   - The second implementation runs only on Qualcomm Snapdragon S2 and higher
   chipsets, and is called "FastCV for Snapdragon."

Releases are generally motivated for the following reasons:
   - Changes to previously released APIs
   - Addition of new functions
   - Performance improvements and/or bug fixes - also known as implementation 
     modifications

    Each motivation has a varying degree of impact on the user of the library.
    The general release numbering scheme captures this variety of motivations.

    Given release ID: A.B.C

    An increase in "A" indicates that a previously released API has changed, 
    so a developer may encounter compilation issues which require modification 
    of their code in order to adhear to the modified API.  Qualcomm will make 
    every effort to minimize these changes.  Additionally, new functions and 
    implementation modifications may be present.

    An increase in "B" indicates that new functions have been added to the 
    library, so additional functionality is available, however existing APIs 
    have not changed.  Additionally, implementation modifications may be 
    present.

    An increase in "C" indicates that implementation modifications only have 
    been made.

@defgroup math_vector Math / Vector Operations
@description Commonly used vector & math functions

@defgroup image_processing Image processing
@description Image filtering, convolution and scaling operations

@defgroup image_transform Image transformation
@description Warp perspective, affine transformations

@defgroup feature_detection Feature detection
@description Fast corner detection, harris corner detection, canny edge detection

@defgroup object_detection Object detection
@description NCC based template matching object detection functions.

@defgroup 3D_reconstruction 3D reconstruction
@description Homography, pose evaluation functions

@defgroup color_conversion Color conversion
@description Commonly used formats supported: e.g., YUV, RGB, YCrCb, etc.

@defgroup clustering_and_search Clustering and search
@description K clusters best fitting of a set of input points

@defgroup mem_management Memory Management
@description Functions to allocate and deallocate memory for use with fastCV.

@defgroup misc Miscellaneous
@description Support functions

=============================================================================**/

//==============================================================================
// Defines
//==============================================================================

#ifdef __GNUC__
   /// Macro to align memory at 4-bytes (32-bits) for GNU-based compilers.
   #define FASTCV_ALIGN32( VAR ) (VAR)  __attribute__ ((aligned(4)))
   /// Macro to align memory at 8-bytes (64-bits) for GNU-based compilers.
   #define FASTCV_ALIGN64( VAR )  (VAR) __attribute__ ((aligned(8)))
   /// Macro to align memory at 16-bytes (128-bits) for GNU-based compilers.
   #define FASTCV_ALIGN128( VAR ) (VAR) __attribute__ ((aligned(16)))
   #ifdef BUILDING_SO
   /// MACRO enables function to be visible in shared-library case.
   #define FASTCV_API __attribute__ ((visibility ("default")))
   #else
   /// MACRO empty for non-shared-library case.
   #define FASTCV_API
   #endif
#else
   /// Macro to align memory at 4-bytes (32-bits) for MSVC compiler.
   #define FASTCV_ALIGN32( VAR ) __declspec(align(4)) (VAR)
   /// Macro to align memory at 8-bytes (64-bits) for MSVC compiler.
   #define FASTCV_ALIGN64( VAR ) __declspec(align(8)) (VAR)
   /// Macro to align memory at 16-bytes (128-bits) for MSVC compiler.
   #define FASTCV_ALIGN128( VAR ) __declspec(align(16)) (VAR)
   #ifdef BUILDING_DLL
   /// MACRO enables function to be visible in shared-library case.
   #define FASTCV_API __declspec(dllexport)
   #else
   /// MACRO empty for non-shared-library case.
   #define FASTCV_API
   #endif
#endif

//==============================================================================
// Included modules
//==============================================================================

#include <stddef.h>

#ifndef FASTCV_STDINT
#define FASTCV_STDINT
   #ifdef _MSC_VER

      #if _MSC_VER <= 1500
         // stdint.h support for VS2008 and older
         #include "stdint_.h"
      #else
         #include <stdint.h>
      #endif

      typedef float  float32_t;
      typedef double float64_t;

   #else

      #ifdef __ARM_NEON__
         #include <arm_neon.h>
      #else
         #include <stdint.h>
         typedef float  float32_t;
         typedef double float64_t;
      #endif

   #endif
#endif

//==============================================================================
// Declarations
//==============================================================================


//------------------------------------------------------------------------------
/// @brief
///    Defines operational mode of interface to allow the end developer to
///    dictate how the target optimized implementation should behave.
//------------------------------------------------------------------------------
typedef enum
{
   /// Target-optimized implementation uses lowest power consuming
   /// implementation.
   FASTCV_OP_LOW_POWER       = 0,

   /// Target-optimized implementation uses higheset performance implementation.
   FASTCV_OP_PERFORMANCE     = 1,

   /// Target-optimized implementation offloads as much of the CPU as possible.
   FASTCV_OP_CPU_OFFLOAD     = 2,

   /// Values >= 0x80000000 are reserved
   FASTCV_OP_RESERVED        = 0x80000000

} fcvOperationMode;


//------------------------------------------------------------------------------
/// @brief
///   Defines a structure to contain points correspondence data.
//------------------------------------------------------------------------------
typedef struct
{
   ///   Tuples of 3 values: xFrom,yFrom,zFrom. Float array which this points to
   ///   must be greater than or equal to 3 * numCorrespondences.
   const float32_t*               from;
   /*~ FIELD fcvCorrespondences.from
       VARRAY LENGTH ( fcvCorrespondences.numCorrespondences * \
       (fcvCorrespondences.fromStride ? fcvCorrespondences.fromStride : 3) ) */

   ///   Tuples of 2 values: xTo,yTo. Float array which this points to
   ///   must be greater than or equal to 2 * numCorrespondences.
   const float32_t*               to;
   /*~ FIELD fcvCorrespondences.to
       VARRAY LENGTH ( fcvCorrespondences.numCorrespondences * \
       (fcvCorrespondences.toStride ? fcvCorrespondences.toStride : 2) ) */

   ///   Distance in bytes between two coordinates in the from array.
   ///   If this parameter is set to 2 or 3, a dense array is assume (stride will
   ///   be sizeof(float) times 2 or 3). The minimum value of fromStride
   ///   should be 2.
   uint32_t                       fromStride;

   ///   Distance in bytes between two coordinates in the to array.
   ///   If this parameter is set to 2, a dense array is assume (stride will
   ///   be 2 * sizeof(float)). The minimum value of toStride
   ///   should be 2.
   uint32_t                       toStride;

   ///   Number of points in points correspondences.
   uint32_t                       numCorrespondences;

   ///   Array of inlier indices for corrs array. Processing will only occur on
   ///   the indices supplied in this array. Array which this points to must be
   ///   at least numIndices long.
   const uint16_t*                indices;
   /*~ FIELD fcvCorrespondences.indices VARRAY LENGTH (fcvCorrespondences.numIndices) */

   ///   Length of indices array.
   uint32_t                       numIndices;
} fcvCorrespondences;


// -----------------------------------------------------------------------------
/// @brief
///   Structure representing an image pyramid level
//------------------------------------------------------------------------------

typedef struct
{
   const void* ptr;
   unsigned int width;
   unsigned int height;
} fcvPyramidLevel ;

// -----------------------------------------------------------------------------
/// @brief
///   Structure describing node of a tree used for leaf and non-leaf nodes;
///   Assumption is that nodes of all trees are stored in in a single array
///   and all indices refer to this array
/// @remark
///   if indices of both children are negative the node is a leaf
// ----------------------------------------------------------------------------
typedef struct fcvKDTreeNodef32
{
   /// the split value at the node
   float32_t divVal;

   /// dimension at which the split is made;
   /// if this is a leaf (both children equal to -1) then this is
   /// the index of the dataset vector
   int32_t divFeat;

   /// index of the child node with dataset items to the left
   /// of the split value
   int32_t childLeft;

   /// index of the child node with dataset items to the right
   /// of the split value
   int32_t childRight;

} fcvKDTreeNodef32;

// -----------------------------------------------------------------------------
/// @brief
///   structure describing a branch (subtree)
/// @remark
///   branches are stored on the priority queue (heap) for backtracking
// -----------------------------------------------------------------------------
typedef struct fcvKDTreeBranchf32
{
   /// square of minimal distance from query for all nodes below
   float32_t minDistSq;

   /// index of the top node of the branch
   int32_t topNode;

} fcvKDTreeBranchf32;

// -----------------------------------------------------------------------------
/// @brief
///   Structure with KDTrees data
// -----------------------------------------------------------------------------
typedef struct fcvKDTreeDatas8f32
{
   // info about the dataset for which KDTrees are constructed
   /// the dataset of vectors
   const int8_t *dataset;

   /// array with inverse lengths of dataset vectors
   const float32_t* invLen;

   /// number of vectors in the dataset
   int32_t numVectors;

   // info about trees
   /// indice of root nodes of trees
   int32_t* trees;

   /// array of nodes of all trees
   fcvKDTreeNodef32* nodes;

   /// number of all nodes
   int32_t numNodes;

   /// capacity of node array
   int32_t maxNumNodes;

   // info used during lookups
   /// priority queue
   fcvKDTreeBranchf32* heap;

   /// number of branches on the priority queue
   int32_t numBranches;

   /// capactiy of the priority queue
   int32_t maxNumBranches;

   /// array of indices to vectors in the dataset;
   /// during searches used to mark checkID;
   /// should have numVectors capacity
   int32_t* vind;

   /// unique ID for each lookup
   int32_t checkID;

   /// number of nearest neighbors to find
   int32_t numNNs;

} fcvKDTreeDatas8f32;


// -----------------------------------------------------------------------------
/// @brief
///   fixed point kdtrees
///   Structure describing node of tree used for leaf and non-leaf nodes;
///   Assumption is that nodes of all trees are stored in in a single array
///   and all indices refer to this array 
/// @remark
///   if indices of both children are negative the node is a leaf
// ----------------------------------------------------------------------------
typedef struct fcvKDTreeNodes32
{
   /// the split value at the node
   int32_t divVal;

   /// dimension at which the split is made;
   /// if this is a leaf (both children equal to -1) then this is
   /// the index of the dataset vector
   int32_t divFeat;

   /// index of the child node with dataset items to the left
   /// of the split value
   int32_t childLeft;

   /// index of the child node with dataset items to the right
   /// of the split value
   int32_t childRight;

} fcvKDTreeNodes32;

// -----------------------------------------------------------------------------
/// @brief
///   fixed point kdtrees
///   structure describing a branch (subtree)
/// @remark
///   branches are stored on the priority queue (heap) for backtracking
// -----------------------------------------------------------------------------
typedef struct fcvKDTreeBranchs32
{
   /// square of minimal distance from query for all nodes below
   int32_t minDistSq;

   /// index of the top node of the branch
   int32_t topNode;

} fcvKDTreeBranchs32;

// -----------------------------------------------------------------------------
/// @brief
///   fixed point kdtrees
///   Structure with KDTrees data
// -----------------------------------------------------------------------------
typedef struct fcvKDTreeDatas8s32
{
   // info about the dataset for which KDTrees are constructed
   /// the dataset of vectors
   const int8_t *dataset;

   /// array with inverse lengths of dataset vectors
   const int32_t* invLen;

   /// number of vectors in the dataset
   int32_t numVectors;

   // info about trees
   /// indices of root nodes of all trees
   int32_t* trees;

   /// number of trees used
   int32_t numTrees;

   /// array of nodes of all trees
   fcvKDTreeNodes32* nodes;

   /// number of all nodes
   int32_t numNodes;

   /// capacity of node array
   int32_t maxNumNodes;

   // info used during lookups
   /// priority queue
   fcvKDTreeBranchs32* heap;

   /// number of branches on the priority queue
   int32_t numBranches;

   /// capactiy of the priority queue
   int32_t maxNumBranches;

   /// array of indices to vectors in the dataset;
   /// during searches used to mark checkID;
   /// should have numVectors capacity
   int32_t* vind;

   /// unique ID for each lookup
   int32_t checkID;

   /// number of nearest neighbors to find
   int32_t numNNs;

} fcvKDTreeDatas8s32;


//------------------------------------------------------------------------------
/// @brief
///   Blur with 3x3 median filter
///
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterMedian3x3u8_v2(). In the 2.0.0 release, 
///   fcvFilterMedian3x3u8_v2 will be renamed to fcvFilterMedian3x3u8
///   and the signature of fcvFilterMedian3x3u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// 
/// @details
///    TBD.
///
/// @param src
///   Input 8-bit image.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be multiple of 8
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output 8-bit image.
///
/// @test
///   -# Test impulse response. An black image with a single center pixel of
///                     value 255 was created for testing.
///   -# Test step response. An image with values 255 on the left and values
///                     of 0 on the right were used for testing.
///   -# Test corner response. An image containing a single corner (top left
///                     quadrant bright while the other 3/4 black, or the vice versa)
///                     were used for testing.
///   -# Test pepper-salt noise. An image with linear intensity gradient was
///                     corrupted by peper and salt noise for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterMedian3x3u8( const uint8_t* __restrict srcImg,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      uint8_t* __restrict       dstImg );


//------------------------------------------------------------------------------
/// @brief
///   Blur with 3x3 median filter
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterMedian3x3u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterMedian3x3u8,
///   \a fcvFilterMedian3x3u8_v2 will be removed, and the current signature
///   for \a fcvFilterMedian3x3u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterMedian3x3u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///    TBD.
///
/// @param src
///   Input 8-bit image.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be multiple of 8
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Output 8-bit image.
/// 
/// @param dstStride
///   Output stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @test
///   -# Test impulse response. An black image with a single center pixel of
///                     value 255 was created for testing.
///   -# Test step response. An image with values 255 on the left and values
///                     of 0 on the right were used for testing.
///   -# Test corner response. An image containing a single corner (top left
///                     quadrant bright while the other 3/4 black, or the vice versa)
///                     were used for testing.
///   -# Test pepper-salt noise. An image with linear intensity gradient was
///                     corrupted by peper and salt noise for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterMedian3x3u8_v2( const uint8_t* __restrict srcImg,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         unsigned int              srcStride,
                         uint8_t* __restrict       dstImg,
                         unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Blur with 3x3 Gaussian filter
///
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterGaussian3x3u8_v2(). In the 2.0.0 release, 
///   fcvFilterGaussian3x3u8_v2 will be renamed to fcvFilterGaussian3x3u8
///   and the signature of fcvFilterGaussian3x3u8 as it appears now, 
///   will be removed.
///   \n\n
/// 
/// @details
///   Convolution with 3x3 Gaussian kernel:
///   \n 1 2 1
///   \n 2 4 2
///   \n 1 2 1
///
/// @param src
///   Input 8-bit image.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output 8-bit image.
///
/// @param blurBorder
///   If set to 1, border is blurred by 0-padding adjacent values. If set to 0,
///   borders up to half-kernel width are ignored (e.g. 1 pixel in the 3x3
///   case).
///
/// @test
///   -# Test impulse response. Black image with a single pixel of
///                     value 255 at different locations were created for testing kernel
///                     response and anchor point.
///   -# Test using an image with constant intensity values: there should be
///                     no dark corners if proper border interpolations were in place.
///   -# Images with checker board patterns with different block sizes (1 to 11)
///                     were created for testing.
///   -# Images with vertical, horizontal and diagonal stripes of
///                     different width (1 to 11) were created for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian3x3u8( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        uint8_t* __restrict       dst,
                        int                       blurBorder );


//------------------------------------------------------------------------------
/// @brief
///   Blur with 3x3 Gaussian filter
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterGaussian3x3u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterGaussian3x3u8,
///   \a fcvFilterGaussian3x3u8_v2 will be removed, and the current signature
///   for \a fcvFilterGaussian3x3u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterGaussian3x3u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   Convolution with 3x3 Gaussian kernel:
///   \n 1 2 1
///   \n 2 4 2
///   \n 1 2 1
///
/// @param src
///   Input 8-bit image.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Output 8-bit image.
/// 
/// @param dstStride
///   Output stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param blurBorder
///   If set to 1, border is blurred by 0-padding adjacent values. If set to 0,
///   borders up to half-kernel width are ignored (e.g. 1 pixel in the 3x3
///   case).
///
/// @test
///   -# Test impulse response. Black image with a single pixel of
///                     value 255 at different locations were created for testing kernel
///                     response and anchor point.
///   -# Test using an image with constant intensity values: there should be
///                     no dark corners if proper border interpolations were in place.
///   -# Images with checker board patterns with different block sizes (1 to 11)
///                     were created for testing.
///   -# Images with vertical, horizontal and diagonal stripes of
///                     different width (1 to 11) were created for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian3x3u8_v2( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           unsigned int              srcStride,
                           uint8_t* __restrict       dst,
                           unsigned int              dstStride,
                           int                       blurBorder );


//------------------------------------------------------------------------------
/// @brief
///   Blur with 5x5 Gaussian filter
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterGaussian5x5u8_v2(). In the 2.0.0 release, 
///   fcvFilterGaussian5x5u8_v2 will be renamed to fcvFilterGaussian5x5u8
///   and the signature of fcvFilterGaussian5x5u8 as it appears now, 
///   will be removed.
///   \n\n
/// 
/// @details
///   Convolution with 5x5 Gaussian kernel:
///   \n 1  4  6  4 1
///   \n 4 16 24 16 4
///   \n 6 24 36 24 6
///   \n 4 16 24 16 4
///   \n 1  4  6  4 1
///
/// @param src
///   Input int data (can be sq. of gradient, etc).
///   \n\b NOTE: data should be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output 8-bit image.
///   \n\b NOTE: data should be 128-bit aligned.
///
/// @param blurBorder
///   If set to 1, border is blurred by 0-padding adjacent values. If set to 0,
///   borders up to half-kernel width are ignored (e.g. 2 pixel in the 5x5
///   case).
///
/// @test
///   -# Test impulse response. Black image with a single pixel of
///                     value 255 at different locations were created for testing kernel
///                     response and anchor point.
///   -# Test using an image with constant intensity values: there should be
///                     no dark corners if proper border interpolations were in place.
///   -# Images with checker board patterns with different block sizes (1 to 11)
///                     were created for testing.
///   -# Images with vertical, horizontal and diagonal stripes of
///                     different width (1 to 11) were created for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian5x5u8( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        uint8_t* __restrict       dst,
                        int                       blurBorder );

//------------------------------------------------------------------------------
/// @brief
///   Blur with 5x5 Gaussian filter
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterGaussian5x5u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterGaussian5x5u8,
///   \a fcvFilterGaussian5x5u8_v2 will be removed, and the current signature
///   for \a fcvFilterGaussian5x5u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterGaussian5x5u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   Convolution with 5x5 Gaussian kernel:
///   \n 1  4  6  4 1
///   \n 4 16 24 16 4
///   \n 6 24 36 24 6
///   \n 4 16 24 16 4
///   \n 1  4  6  4 1
///
/// @param src
///   Input int data (can be sq. of gradient, etc).
///   \n\b NOTE: data should be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Output 8-bit image.
///   \n\b NOTE: data should be 128-bit aligned.
/// 
/// @param dstStride
///   Output stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param blurBorder
///   If set to 1, border is blurred by 0-padding adjacent values. If set to 0,
///   borders up to half-kernel width are ignored (e.g. 2 pixel in the 5x5
///   case).
///
/// @test
///   -# Test impulse response. Black image with a single pixel of
///                     value 255 at different locations were created for testing kernel
///                     response and anchor point.
///   -# Test using an image with constant intensity values: there should be
///                     no dark corners if proper border interpolations were in place.
///   -# Images with checker board patterns with different block sizes (1 to 11)
///                     were created for testing.
///   -# Images with vertical, horizontal and diagonal stripes of
///                     different width (1 to 11) were created for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian5x5u8_v2( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           unsigned int              srcStride,
                           uint8_t* __restrict       dst,
                           unsigned int              dstStride,
                           int                       blurBorder );


//------------------------------------------------------------------------------
/// @brief
///   Blur with 11x11 Gaussian filter
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterGaussian11x11u8_v2(). In the 2.0.0 release, 
///   fcvFilterGaussian11x11u8_v2 will be renamed to fcvFilterGaussian11x11u8
///   and the signature of fcvFilterGaussian11x11u8 as it appears now, 
///   will be removed.
///   \n\n
///   
/// @details
///   Convolution with 11x11 Gaussian kernel:
///   \n   1,   10,    45,   120,   210,   252,   210,   120,    45,   10,   1
///   \n  10,  100,   450,  1200,  2100,  2520,  2100,  1200,   450,  100,  10
///   \n  45,  450,  2025,  5400,  9450, 11340,  9450,  5400,  2025,  450,  45
///   \n 120, 1200,  5400, 14400, 25200, 30240, 25200, 14400,  5400, 1200, 120
///   \n 210, 2100,  9450, 25200, 44100, 52920, 44100, 25200,  9450, 2100, 210
///   \n 252, 2520, 11340, 30240, 52920, 63504, 52920, 30240, 11340, 2520, 252
///   \n 210, 2100,  9450, 25200, 44100, 52920, 44100, 25200,  9450, 2100, 210
///   \n 120, 1200,  5400, 14400, 25200, 30240, 25200, 14400,  5400, 1200, 120
///   \n  45,  450,  2025,  5400,  9450, 11340,  9450,  5400,  2025,  450,  45
///   \n  10,  100,   450,  1200,  2100,  2520,  2100,  1200,   450,  100,  10
///   \n   1,   10,    45,   120,   210,   252,   210,   120,    45,   10,   1
///
/// @param src
///   Input 8-bit image.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output 8-bit image.
///
/// @param blurBorder
///   If set to 1, border is blurred by 0-padding adjacent values. If set to 0,
///   borders up to half-kernel width are ignored (e.g. 5 pixel in the 11x11
///   case).
///
/// @test
///   -# Test impulse response. Black image with a single pixel of
///                     value 255 at different locations were created for testing kernel
///                     response and anchor point.
///   -# Test using an image with constant intensity values: there should be
///                     no dark corners if proper border interpolations were in place.
///   -# Images with checker board patterns with different block sizes (1 to 11)
///                     were created for testing.
///   -# Images with vertical, horizontal and diagonal stripes of
///                     different width (1 to 11) were created for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian11x11u8( const uint8_t* __restrict src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          uint8_t* __restrict       dst,
                          int                       blurBorder );



//------------------------------------------------------------------------------
/// @brief
///   Blur with 11x11 Gaussian filter
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterGaussian11x11u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterGaussian11x11u8,
///   \a fcvFilterGaussian11x11u8_v2 will be removed, and the current signature
///   for \a fcvFilterGaussian11x11u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterGaussian11x11u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   Convolution with 11x11 Gaussian kernel:
///   \n   1,   10,    45,   120,   210,   252,   210,   120,    45,   10,   1
///   \n  10,  100,   450,  1200,  2100,  2520,  2100,  1200,   450,  100,  10
///   \n  45,  450,  2025,  5400,  9450, 11340,  9450,  5400,  2025,  450,  45
///   \n 120, 1200,  5400, 14400, 25200, 30240, 25200, 14400,  5400, 1200, 120
///   \n 210, 2100,  9450, 25200, 44100, 52920, 44100, 25200,  9450, 2100, 210
///   \n 252, 2520, 11340, 30240, 52920, 63504, 52920, 30240, 11340, 2520, 252
///   \n 210, 2100,  9450, 25200, 44100, 52920, 44100, 25200,  9450, 2100, 210
///   \n 120, 1200,  5400, 14400, 25200, 30240, 25200, 14400,  5400, 1200, 120
///   \n  45,  450,  2025,  5400,  9450, 11340,  9450,  5400,  2025,  450,  45
///   \n  10,  100,   450,  1200,  2100,  2520,  2100,  1200,   450,  100,  10
///   \n   1,   10,    45,   120,   210,   252,   210,   120,    45,   10,   1
///
/// @param src
///   Input 8-bit image.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Output 8-bit image.
/// 
/// @param dstStride
///   Output stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param blurBorder
///   If set to 1, border is blurred by 0-padding adjacent values. If set to 0,
///   borders up to half-kernel width are ignored (e.g. 5 pixel in the 11x11
///   case).
///
/// @test
///   -# Test impulse response. Black image with a single pixel of
///                     value 255 at different locations were created for testing kernel
///                     response and anchor point.
///   -# Test using an image with constant intensity values: there should be
///                     no dark corners if proper border interpolations were in place.
///   -# Images with checker board patterns with different block sizes (1 to 11)
///                     were created for testing.
///   -# Images with vertical, horizontal and diagonal stripes of
///                     different width (1 to 11) were created for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian11x11u8_v2( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             unsigned int              srcStride,
                             uint8_t* __restrict       dst,
                             unsigned int              dstStride,
                             int                       blurBorder );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YUV (YCrCb) 4:2:0 PesudoPlanar (Interleaved) to RGB 8888.
///      
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvColorYUV420toRGB8888u8_v2(). In the 2.0.0 release, 
///   fcvColorYUV420toRGB8888u8_v2 will be renamed to fcvColorYUV420toRGB8888u8
///   and the signature of fcvColorYUV420toRGB8888u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   TBD.
///
/// @param src
///   8-bit image of input YUV 4:2:0 values.
///
/// @param dst
///   32-bit image of output RGB 8888 values.
///   \n\b WARNING: size must match input yuv420.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @test
///   -# Proper interpolation of sharp chromaticity changes. A 1D color
///                     pattern with abrupt color transitions was created for testing.
///   -# Proper interpolation of smooth chromaticity changes. A 2D color
///                     pattern was created by putting primative colors (RGBY) on four
///                     corners of a 16x16 image patch and bilinearly interpolate the
///                     pixels in the middle.
///   -# Proper preservation of achromaticity. A gray image (all channels
///                     having the same intensity of 128) was used for testing.
///   -# Test the range of color variations. An image patch with (u,v)
///                     values varying from 0:16:255 was created for testing.
///   -# Test intensity variations. An image patch with constant (u,v)
///                     and intensities from 0:255 was create for testing. 4 different
///                     extreme colors in the UV space was used to create the patterns.
///   -# Test patches from real image. 50 image patches cropped from an
///                     large color image was used for the tests.
///   -# Test a relatively large (768x512) image.
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYUV420toRGB8888u8( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           uint32_t* __restrict      dst );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YUV (YCrCb) 4:2:0 PesudoPlanar (Interleaved) to RGB 888.
///
/// @details
///   TBD.
///
/// @param src
///   8-bit image of input YUV picture.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcYStride
///   Stride (in bytes) of input image Y component (i.e., number of bytes between 
///   column 0 of row 1 and column 0 of row 2).
///   \n\b WARNING: Must be multiple of 8 (8 * 1-byte values).
///
/// @param srcCStride
///   Stride (in bytes) of input image Chroma component (i.e., number of bytes between 
///   column 0 of row 1 and column 0 of row 2)
///   \n\b WARNING: Must be multiple of 4 (4 * 1-byte values).
/// 
/// @param dst
///   32-bit image of output RGB 8888 values.
///   \n\b WARNING: size must match input yuv420.
/// 
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 1 and column 0 of row 2)
///   \n\b WARNING: Must be multiple of 32 (8 * 4-byte values).
///
/// @test
///   -# Proper interpolation of sharp chromaticity changes. A 1D color
///                     pattern with abrupt color transitions was created for testing.
///   -# Proper interpolation of smooth chromaticity changes. A 2D color
///                     pattern was created by putting primative colors (RGBY) on four
///                     corners of a 16x16 image patch and bilinearly interpolate the
///                     pixels in the middle.
///   -# Proper preservation of achromaticity. A gray image (all channels
///                     having the same intensity of 128) was used for testing.
///   -# Test the range of color variations. An image patch with (u,v)
///                     values varying from 0:16:255 was created for testing.
///   -# Test intensity variations. An image patch with constant (u,v)
///                     and intensities from 0:255 was create for testing. 4 different
///                     extreme colors in the UV space was used to create the patterns.
///   -# Test patches from real image. 50 image patches cropped from an
///                     large color image was used for the tests.
///   -# Test a relatively large (768x512) image.
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCrCb420PseudoPlanarToRGB8888u8( const uint8_t* __restrict src,
                                         unsigned int              srcWidth,
                                         unsigned int              srcHeight,
                                         unsigned int              srcYStride,
                                         unsigned int              srcCStride,
                                         uint32_t* __restrict      dst,
                                         unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YUV (YCbCr) 4:2:0 PesudoPlanar (Interleaved) to RGB 565.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvColorYUV420toRGB565u8_v2(). In the 2.0.0 release, 
///   fcvColorYUV420toRGB565u8_v2 will be renamed to fcvColorYUV420toRGB565u8
///   and the signature of fcvColorYUV420toRGB565u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   TBD.
///
/// @param src
///   8-bit image of input YUV 4:2:0 values.
///
/// @param dst
///   16-bit image of output RGB 565 values.
///   \n\b WARNING: size must match input yuv420.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @test
///   -# Proper interpolation of sharp chromaticity changes. A 1D color
///                     pattern with abrupt color transitions was created for testing.
///   -# Proper interpolation of smooth chromaticity changes. A 2D color
///                     pattern was created by putting primative colors (RGBY) on four
///                     corners of a 16x16 image patch and bilinearly interpolate the
///                     pixels in the middle.
///   -# Proper preservation of achromaticity. A gray image (all channels
///                     having the same intensity of 128) was used for testing.
///   -# Test the range of color variations. An image patch with (u,v)
///                     values varying from 0:16:255 was created for testing.
///   -# Test intensity variations. An image patch with constant (u,v)
///                     and intensities from 0:255 was create for testing. 4 different
///                     extreme colors in the UV space was used to create the patterns.
///   -# Test patches from real image. 50 image patches cropped from an
///                     large color image was used for the tests.
///   -# Test a relatively large (768x512) image.
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYUV420toRGB565u8( const uint8_t* __restrict src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          uint32_t*  __restrict     dst );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCrCb H1V1 to RGB 888.
///
/// @details
///   Color conversion from YCrCb H1V1 to RGB 888.
///   \n R = Y                    + 1.40200*(Cr-128)
///   \n G = Y - 0.34414*(Cb-128) - 0.71414*(Cr-128)
///   \n B = Y + 1.77200*(CB-128)
///
/// @param src
///   8-bit image of input values.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   8-bit image of output RGB 888 values.
///   \n\b WARNING: size must match input crcb.
///
/// @test
///   -# Proper interpolation of sharp chromaticity changes. A 1D color
///                     pattern with abrupt color transitions was created for testing.
///   -# Proper interpolation of smooth chromaticity changes. A 2D color
///                     pattern was created by putting primative colors (RGBY) on four
///                     corners of a 16x16 image patch and bilinearly interpolate the
///                     pixels in the middle.
///   -# Proper preservation of achromaticity. A gray image (all channels
///                     having the same intensity of 128) was used for testing.
///   -# Horizontal and vertical color stripes. A 16x16 image with left half
///                     horizontal stripes with alternating red and green, and
///                     right half horizontal stripes with alternating blue and cyan.
///   -# Test the range of color variations. An image patch with (Cr,Cb)
///                     values varying from 0:16:255 was created for testing.
///   -# Test intensity variations. An image patch with constant (Cr,Cb)
///                     and intensities from 0:255 was create for testing. 4 different
///                     extreme colors in the CrCb space was used to create the patterns.
///   -# Test patches from real image. 50 image patches cropped from an
///                     large color image was used for the tests.
///   -# Test a relatively large (768x512) image.
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCrCbH1V1toRGB888u8( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCrCb to RGB 888.
///
/// @details
///   Color conversion from YCrCb to RGB 888.
///   \n R = Y                    + 1.40200*(Cr-128)
///   \n G = Y - 0.34414*(Cb-128) - 0.71414*(Cr-128)
///   \n B = Y + 1.77200*(CB-128)
///
/// @param src
///   8-bit input values.
///
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   8-bit image of output RGB 888 values.
///   \n\b WARNING: size must match input crcb.
///
/// @test
///   -# Proper interpolation of sharp chromaticity changes. A 1D color
///                     pattern with abrupt color transitions was created for testing.
///   -# Proper interpolation of smooth chromaticity changes. A 2D color
///                     pattern was created by putting primative colors (RGBY) on four
///                     corners of a 16x16 image patch and bilinearly interpolate the
///                     pixels in the middle.
///   -# Proper preservation of achromaticity. A gray image (all channels
///                     having the same intensity of 128) was used for testing.
///   -# Horizontal and vertical color stripes. A 16x16 image with left half
///                     horizontal stripes with alternating red and green, and
///                     right half horizontal stripes with alternating blue and cyan.
///   -# Test the range of color variations. An image patch with (Cr,Cb)
///                     values varying from 0:16:255 was created for testing.
///   -# Test intensity variations. An image patch with constant (Cr,Cb)
///                     and intensities from 0:255 was create for testing. 4 different
///                     extreme colors in the CrCb space was used to create the patterns.
///   -# Test patches from real image. 50 image patches cropped from an
///                     large color image was used for the tests.
///   -# Test a relatively large (768x512) image.
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCrCbH2V2toRGB888u8( const uint8_t* __restrict ysrc,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             uint8_t* __restrict       dst );



//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCrCb to RGB 888.
///
/// @details
///   Color conversion from YCrCb to RGB 888.
///   \n R = Y                    + 1.40200*(Cr-128)
///   \n G = Y - 0.34414*(Cb-128) - 0.71414*(Cr-128)
///   \n B = Y + 1.77200*(CB-128)
///
/// @param src
///   8-bit input values.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   8-bit image of output RGB 888 values.
///   \n\b WARNING: size must match input crcb.
///
/// @test
///   -# Proper interpolation of sharp chromaticity changes. A 1D color
///                     pattern with abrupt color transitions was created for testing.
///   -# Proper interpolation of smooth chromaticity changes. A 2D color
///                     pattern was created by putting primative colors (RGBY) on four
///                     corners of a 16x16 image patch and bilinearly interpolate the
///                     pixels in the middle.
///   -# Proper preservation of achromaticity. A gray image (all channels
///                     having the same intensity of 128) was used for testing.
///   -# Horizontal and vertical color stripes. A 16x16 image with left half
///                     horizontal stripes with alternating red and green, and
///                     right half horizontal stripes with alternating blue and cyan.
///   -# Test the range of color variations. An image patch with (Cr,Cb)
///                     values varying from 0:16:255 was created for testing.
///   -# Test intensity variations. An image patch with constant (Cr,Cb)
///                     and intensities from 0:255 was create for testing. 4 different
///                     extreme colors in the CrCb space was used to create the patterns.
///   -# Test patches from real image. 50 image patches cropped from an
///                     large color image was used for the tests.
///   -# Test a relatively large (768x512) image.
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCrCbH2V1toRGB888u8( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCrCb to RGB 888.
///
/// @details
///   Color conversion from YCrCb to RGB 888.
///   \n R = Y                    + 1.40200*(Cr-128)
///   \n G = Y - 0.34414*(Cb-128) - 0.71414*(Cr-128)
///   \n B = Y + 1.77200*(CB-128)
///
/// @param ysrc
///   8-bit input values.
///
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   8-bit image of output RGB 888 values.
///   \n\b WARNING: size must match input crcb.
///
/// @test
///   -# Proper interpolation of sharp chromaticity changes. A 1D color
///                     pattern with abrupt color transitions was created for testing.
///   -# Proper interpolation of smooth chromaticity changes. A 2D color
///                     pattern was created by putting primative colors (RGBY) on four
///                     corners of a 16x16 image patch and bilinearly interpolate the
///                     pixels in the middle.
///   -# Proper preservation of achromaticity. A gray image (all channels
///                     having the same intensity of 128) was used for testing.
///   -# Horizontal and vertical color stripes. A 16x16 image with left half
///                     horizontal stripes with alternating red and green, and
///                     right half horizontal stripes with alternating blue and cyan.
///   -# Test the range of color variations. An image patch with (u,v)
///                     values varying from 0:16:255 was created for testing.
///   -# Test intensity variations. An image patch with constant (u,v)
///                     and intensities from 0:255 was create for testing. 4 different
///                     extreme colors in the CrCb space was used to create the patterns.
///   -# Test patches from real image. 50 image patches cropped from an
///                     large color image was used for the tests.
///   -# Test a relatively large (768x512) image.
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCrCbH1V2toRGB888u8( const uint8_t* __restrict ysrc,

                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             uint8_t* __restrict       dst );



//------------------------------------------------------------------------------
/// @brief 
///   Color conversion from RGB 888 to YCrCb.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvColorRGB888toYCrCbu8_v2(). In the 2.0.0 release, 
///   fcvColorRGB888toYCrCbu8_v2 will be renamed to fcvColorRGB888toYCrCbu8
///   and the signature of fcvColorRGB888toYCrCbu8 as it appears now, 
///   will be removed.
///   \n\n
/// 
/// @details
///   Color conversion from YCrCb to RGB 888.
///   \n R = Y                    + 1.40200*(Cr-128)
///   \n G = Y - 0.34414*(Cb-128) - 0.71414*(Cr-128)
///   \n B = Y + 1.77200*(CB-128)
///
/// @param src
///   8-bit input values.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   8-bit output values.
///   \n\b WARNING: size must match input crcb.
///
/// @test
///   -# Proper interpolation of sharp chromaticity changes. A 1D color
///                     pattern with abrupt color transitions was created for testing.
///   -# Proper interpolation of smooth chromaticity changes. A 2D color
///                     pattern was created by putting primative colors (RGBY) on four
///                     corners of a 16x16 image patch and bilinearly interpolate the
///                     pixels in the middle.
///   -# Proper preservation of achromaticity. A gray image (all channels
///                     having the same intensity of 128) was used for testing.
///   -# Horizontal and vertical color stripes. A 16x16 image with left half
///                     horizontal stripes with alternating red and green, and
///                     right half horizontal stripes with alternating blue and cyan.
///   -# Test the range of color variations. An image patch with (Cr,Cb)
///                     values varying from 0:16:255 was created for testing.
///   -# Test intensity variations. An image patch with constant (Cr,Cb)
///                     and intensities from 0:255 was create for testing. 4 different
///                     extreme colors in the CrCb space was used to create the patterns.
///   -# Test patches from real image. 50 image patches cropped from an
///                     large color image was used for the tests.
///   -# Test a relatively large (768x512) image.
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888toYCrCbu8( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from RGB 888 to YCrCb 4:4:4 (Full interleaved, similar to
///   3-channel RGB).
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvColorRGB888toYCrCbu8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvColorRGB888toYCrCbu8,
///   \a fcvColorRGB888toYCrCbu8_v2 will be removed, and the current signature
///   for \a fcvColorRGB888toYCrCbu8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvColorRGB888toYCrCbu8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   
/// @param src
///   8-bit input values.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: Must be multiple of 8.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride (in bytes).
///   \n\b WARNING: Must be at least 3*srcWidth.
///
/// @param dst
///   8-bit output values.
///   \n\b WARNING: size must match input crcb.
/// 
/// @param dstStride
///   Output stride (in bytes).
///   \n\b WARNING: Must be at least 3*srcWidth.
///
/// @test
///   -# Proper interpolation of sharp chromaticity changes. A 1D color
///                     pattern with abrupt color transitions was created for testing.
///   -# Proper interpolation of smooth chromaticity changes. A 2D color
///                     pattern was created by putting primative colors (RGBY) on four
///                     corners of a 16x16 image patch and bilinearly interpolate the
///                     pixels in the middle.
///   -# Proper preservation of achromaticity. A gray image (all channels
///                     having the same intensity of 128) was used for testing.
///   -# Horizontal and vertical color stripes. A 16x16 image with left half
///                     horizontal stripes with alternating red and green, and
///                     right half horizontal stripes with alternating blue and cyan.
///   -# Test the range of color variations. An image patch with (Cr,Cb)
///                     values varying from 0:16:255 was created for testing.
///   -# Test intensity variations. An image patch with constant (Cr,Cb)
///                     and intensities from 0:255 was create for testing. 4 different
///                     extreme colors in the CrCb space was used to create the patterns.
///   -# Test patches from real image. 50 image patches cropped from an
///                     large color image was used for the tests.
///   -# Test a relatively large (768x512) image.
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorRGB888toYCrCbu8_v2( const uint8_t* __restrict src,
                            unsigned int              srcWidth,
                            unsigned int              srcHeight,
                            unsigned int              srcStride,
                            uint8_t* __restrict       dst,
                            unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Create a 36-dimension gradient based descriptor on 17x17 patch.
///
/// @details
///   @todo Thorough documentation for all gradient functions including
///   diagrams.
///
/// @param patch
///   Input luminance data for 17x17 patch to describe.
///
/// @param descriptor
///   Output descriptor vector. 36 x 8-bit vector.
///
/// @param descriptorNormSq
///   Output squared norm of the descriptor vector.
///
/// @test
///   -# Test response to a blank patch.
///   -# Test small vertical edges   at different block centers.
///   -# Test small horizontal edges at all block centers.
///   -# Test small 45 degree diagnal edges at all block centers.
///   -# Test pattern with only one non-trival value.
///   -# Test image patch with very high contrast.
///   -# Test image patches cropped around FAST8 corners in a real image.
///
/// @ingroup object_detection
//------------------------------------------------------------------------------

FASTCV_API int
fcvDescriptor17x17u8To36s8( const uint8_t* __restrict patch,
                            int8_t* __restrict        descriptorChar,
                            int32_t* __restrict       descriptorNormSq );


//---------------------------------------------------------------------------
/// @brief
///   Dot product of two 8-bit vectors.
///
/// @param a
///   Vector.
///
/// @param b
///   Vector.
///
/// @param abSize
///   Number of elements in A and B.
///
/// @return
///   Dot product <A|B>.
///
/// @ingroup math_vector
//---------------------------------------------------------------------------

FASTCV_API int32_t
fcvDotProducts8( const int8_t* __restrict a,
                 const int8_t* __restrict b,
                 unsigned int             abSize );


//------------------------------------------------------------------------------
/// @brief
///   Dot product of two 8-bit vectors.
///
/// @param a
///   Vector A.
///
/// @param b
///   Vector B.
///
/// @param abSize
///   Number of elements in A and B.
///
/// @return
///   Dot product <A|B>.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvDotProductu8( const uint8_t* __restrict  a,
                 const uint8_t* __restrict  b,
                 unsigned int               abSize );


//---------------------------------------------------------------------------
/// @brief
///   Dot product of two 36-byte vectors.
///
/// @param a
///   Vector.
///
/// @param b
///   Vector.
///
/// @return
///   Dot product <a|b>.
///
/// @ingroup math_vector
//---------------------------------------------------------------------------

FASTCV_API int32_t
fcvDotProduct36x1s8( const int8_t* __restrict a,
                     const int8_t* __restrict b );


//---------------------------------------------------------------------------
/// @brief
///   Dot product of one 36-byte vector against 4 others.
///
/// @details
///   Dot product of 36-byte vector (a) against 4 others (b,c,d,e):\n
///   <a|b>, <a|c>, <a|d>, <a|e>
///
/// @param a
///   Vector.
///
/// @param b
///   Vector.
///
/// @param c
///   Vector.
///
/// @param d
///   Vector.
///
/// @param e
///   Vector.
///
/// @param dotProducts
///   Output of the 4 results { <a|b>, <a|c>, <a|d>, <a|e> }.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//---------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct36x4s8( const int8_t* __restrict a,
                     const int8_t* __restrict b,
                     const int8_t* __restrict c,
                     const int8_t* __restrict d,
                     const int8_t* __restrict e,
                     int32_t* __restrict      dotProducts );


//------------------------------------------------------------------------------
/// @brief
///   Normalized dot product of one 36-byte vector against 4 others.
///
/// @details
///   Dot product of 36-byte vector (a) against 4 others (b0,b1,b2,b3):\n
///   <a|b0>, <a|b1>, <a|b2>, <a|b3>
///   using their given inverse lengths for normalization.
///
/// @param a
///   Vector.
///
/// @param invLengthA
///   Inverse of vector A.
///
/// @param b0
///   Vector.
///
/// @param b1
///   Vector.
///
/// @param b2
///   Vector.
///
/// @param b3
///   Vector.
///
/// @param invLengthsB
///   Pointer to an array of the inverse values of each B vector.
///   The pointer must point to 4 floating point values.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @param dotProducts
///   Output of the 4 results { <a|b0>, <a|b1>, <a|b2>, <a|b3> }.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProductNorm36x4s8( const int8_t* __restrict a,
                         float                    invLengthA,
                         const int8_t* __restrict b0,
                         const int8_t* __restrict b1,
                         const int8_t* __restrict b2,
                         const int8_t* __restrict b3,
                         float* __restrict        invLengthsB,
                         float* __restrict        dotProducts  );


//------------------------------------------------------------------------------
/// @brief
///   Dot product of two 36-byte vectors.
///
/// @param a
///   Vector.
///
/// @param b
///   Vector.
///
/// @return
///   Dot product <a|b>.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvDotProduct36x1u8( const uint8_t* __restrict a,
                     const uint8_t* __restrict b );


//------------------------------------------------------------------------------
/// @brief
///   Dot product of one 36-byte vector against 4 others.
///
/// @details
///   Dot product of 36-byte vector (a) against 4 others (b,c,d,e):\n
///   <a|b>, <a|c>, <a|d>, <a|e>
///
/// @param a
///   Vector.
///
/// @param b
///   Vector.
///
/// @param c
///   Vector.
///
/// @param d
///   Vector.
///
/// @param e
///   Vector.
///
/// @param dotProducts
///   Output of the 4 results { <a|b>, <a|c>, <a|d>, <a|e> }.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct36x4u8( const uint8_t* __restrict a,
                     const uint8_t* __restrict b,
                     const uint8_t* __restrict c,
                     const uint8_t* __restrict d,
                     const uint8_t* __restrict e,
                     uint32_t* __restrict      dotProducts );


//------------------------------------------------------------------------------
/// @brief
///   Normalized dot product of one 36-byte vector against 4 others.
///
/// @details
///   Dot product of 36-byte vector (a) against 4 others (b0,b1,b2,b3):\n
///   <a|b0>, <a|b1>, <a|b2>, <a|b3>
///   using their given inverse lengths for normalization.
///
/// @param a
///   Vector.
///
/// @param invLengthA
///   Inverse of vector A.
///
/// @param b0
///   Vector.
///
/// @param b1
///   Vector.
///
/// @param b2
///   Vector.
///
/// @param b3
///   Vector.
///
/// @param invLengthsB
///   Pointer to an array of the inverse values of each B vector.
///   The pointer must point to 4 floating point values.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @param dotProducts
///   Output of the 4 results { <a|b0>, <a|b1>, <a|b2>, <a|b3> }.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProductNorm36x4u8( const uint8_t* __restrict  a,
                         float                      invLengthA,
                         const uint8_t* __restrict  b0,
                         const uint8_t* __restrict  b1,
                         const uint8_t* __restrict  b2,
                         const uint8_t* __restrict  b3,
                         float* __restrict          invLengthsB,
                         float* __restrict          dotProducts );


//---------------------------------------------------------------------------
/// @brief
///   Dot product of two 64-byte vectors.
///
/// @param a
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @return
///   Dot product <a|b>.
///
/// @ingroup math_vector
//---------------------------------------------------------------------------

FASTCV_API int32_t
fcvDotProduct64x1s8( const int8_t* __restrict a,
                     const int8_t* __restrict b );


//---------------------------------------------------------------------------
/// @brief
///   Dot product of one 64-byte vector against 4 others.
///
/// @details
///   Dot product of vector (a) against 4 others (b,c,d,e):\n
///   <a|b>, <a|c>, <a|d>, <a|e>
///
/// @param a
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param c
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param d
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param e
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param dotProducts
///   Output of the 4 results { <a|b>, <a|c>, <a|d>, <a|e> }.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//---------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct64x4s8( const int8_t* __restrict a,
                     const int8_t* __restrict b,
                     const int8_t* __restrict c,
                     const int8_t* __restrict d,
                     const int8_t* __restrict e,
                     int32_t* __restrict      dotProducts );


//------------------------------------------------------------------------------
/// @brief
///   Normalized dot product of one 64-byte vector against 4 others.
///
/// @details
///   Dot product of 36-byte vector (a) against 4 others (b0,b1,b2,b3):\n
///   <a|b0>, <a|b1>, <a|b2>, <a|b3>
///   using their given inverse lengths for normalization.
///
/// @param a
///   Vector.
///
/// @param invLengthA
///   Inverse of vector A.
///
/// @param b0
///   Vector.
///
/// @param b1
///   Vector.
///
/// @param b2
///   Vector.
///
/// @param b3
///   Vector.
///
/// @param invLengthsB
///   Pointer to an array of the inverse values of each B vector.
///   The pointer must point to 4 floating point values.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @param dotProducts
///   Output of the 4 results { <a|b0>, <a|b1>, <a|b2>, <a|b3> }.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProductNorm64x4s8( const int8_t* __restrict a,
                         float                    invLengthA,
                         const int8_t* __restrict b0,
                         const int8_t* __restrict b1,
                         const int8_t* __restrict b2,
                         const int8_t* __restrict b3,
                         float* __restrict        invLengthsB,
                         float* __restrict        dotProducts  );


//------------------------------------------------------------------------------
/// @brief
///   Dot product of two 64-byte vectors.
///
/// @param a
///   Vector.
///
/// @param b
///   Vector.
///
/// @return
///   Dot product <a|b>.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvDotProduct64x1u8( const uint8_t* __restrict a,
                     const uint8_t* __restrict b );


//------------------------------------------------------------------------------
/// @brief
///   Dot product of one 64-byte vector against 4 others.
///
/// @details
///   Dot product of 36-byte vector (a) against 4 others (b,c,d,e):\n
///   <a|b>, <a|c>, <a|d>, <a|e>
///
/// @param a
///   Vector.
///
/// @param b
///   Vector.
///
/// @param c
///   Vector.
///
/// @param d
///   Vector.
///
/// @param e
///   Vector.
///
/// @param dotProducts
///   Output of the 4 results { <a|b>, <a|c>, <a|d>, <a|e> }.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct64x4u8( const uint8_t* __restrict a,
                     const uint8_t* __restrict b,
                     const uint8_t* __restrict c,
                     const uint8_t* __restrict d,
                     const uint8_t* __restrict e,
                     uint32_t* __restrict      dotProducts );


//------------------------------------------------------------------------------
/// @brief
///   Dot product of one 36-byte vector against 4 others.
///
/// @details
///   Dot product of vector (a) against 4 others (b,c,d,e):\n
///   <a|b>, <a|c>, <a|d>, <a|e>
///
/// @param a
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param c
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param d
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param e
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param dotProducts
///   Output of the 4 results { <a|b>, <a|c>, <a|d>, <a|e> }.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProductNorm64x4u8( const uint8_t* __restrict  a,
                         float                      invLengthA,
                         const uint8_t* __restrict  b0,
                         const uint8_t* __restrict  b1,
                         const uint8_t* __restrict  b2,
                         const uint8_t* __restrict  b3,
                         float* __restrict          invLengthsB,
                         float* __restrict          dotProducts );


//---------------------------------------------------------------------------
/// @brief
///   Dot product of two 128-byte vectors.
///
/// @param a
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @return
///   Dot product <a|b>.
///
/// @ingroup math_vector
//---------------------------------------------------------------------------

FASTCV_API int32_t
fcvDotProduct128x1s8( const int8_t* __restrict a,
                      const int8_t* __restrict b );


//---------------------------------------------------------------------------
/// @brief
///   Dot product of one 128-byte vector against 4 others.
///
/// @details
///   Dot product of vector (a) against 4 others (b,c,d,e):\n
///   <a|b>, <a|c>, <a|d>, <a|e>
///
/// @param a
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param c
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param d
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param e
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param dotProducts
///   Output of the 4 results { <a|b>, <a|c>, <a|d>, <a|e> }.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//---------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct128x4s8( const int8_t* __restrict a,
                      const int8_t* __restrict b,
                      const int8_t* __restrict c,
                      const int8_t* __restrict d,
                      const int8_t* __restrict e,
                      int32_t* __restrict      dotProducts );


//------------------------------------------------------------------------------
/// @brief
///   Normalized dot product of one 128-byte vector against 4 others.
///
/// @details
///   Dot product of vector (a) against 4 others (b0,b1,b2,b3):\n
///   <a|b0>, <a|b1>, <a|b2>, <a|b3>
///   using their given inverse lengths for normalization.
///
/// @param a
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param invLengthA
///   Inverse of vector A.
///
/// @param b0
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b1
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b2
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b3
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param invLengthsB
///   Pointer to an array of the inverse values of each B vector.
///   The pointer must point to 4 floating point values.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @param dotProducts
///   Output of the 4 results { <a|b0>, <a|b1>, <a|b2>, <a|b3> }.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProductNorm128x4s8( const int8_t* __restrict a,
                          float                    invLengthA,
                          const int8_t* __restrict b0,
                          const int8_t* __restrict b1,
                          const int8_t* __restrict b2,
                          const int8_t* __restrict b3,
                          float* __restrict        invLengthsB,
                          float* __restrict        dotProducts  );


//------------------------------------------------------------------------------
/// @brief
///   Dot product of two 128-byte vectors.
///
/// @param a
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @return
///   Dot product <a|b>.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvDotProduct128x1u8( const uint8_t* __restrict a,
                      const uint8_t* __restrict b );


//------------------------------------------------------------------------------
/// @brief
///   Dot product of one 128-byte vector against 4 others.
///
/// @details
///   Dot product of vector (a) against 4 others (b,c,d,e):\n
///   <a|b>, <a|c>, <a|d>, <a|e>
///
/// @param a
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param c
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param d
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param e
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param dotProducts
///   Output of the 4 results { <a|b>, <a|c>, <a|d>, <a|e> }.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct128x4u8( const uint8_t* __restrict a,
                      const uint8_t* __restrict b,
                      const uint8_t* __restrict c,
                      const uint8_t* __restrict d,
                      const uint8_t* __restrict e,
                      uint32_t* __restrict      dotProducts );


//------------------------------------------------------------------------------
/// @brief
///   Normalized dot product of one 128-byte vector against 4 others.
///
/// @details
///   Dot product of vector (a) against 4 others (b0,b1,b2,b3):\n
///   <a|b0>, <a|b1>, <a|b2>, <a|b3>
///   using their given inverse lengths for normalization.
///
/// @param a
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param invLengthA
///   Inverse of vector A.
///
/// @param b0
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b1
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b2
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b3
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param invLengthsB
///   Pointer to an array of the inverse values of each B vector.
///   The pointer must point to 4 floating point values.
///
/// @param dotProducts
///   Output of the 4 results { <a|b0>, <a|b1>, <a|b2>, <a|b3> }.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProductNorm128x4u8( const uint8_t* __restrict  a,
                          float                      invLengthA,
                          const uint8_t* __restrict  b0,
                          const uint8_t* __restrict  b1,
                          const uint8_t* __restrict  b2,
                          const uint8_t* __restrict  b3,
                          float* __restrict          invLengthsB,
                          float* __restrict          dotProducts );


//------------------------------------------------------------------------------
/// @brief
///   Dot product of 1 patch (8x8 byte square) with several (n) 8x8 squares
///   along a line of pixels in an image.
///
/// @param patchPixels
///   Pointer to 8-bit patch pixel values linearly laid out in memory.
///
/// @param imagePixels
///   Pointer to 8-bit image pixel values linearly laid out in memory.
///
/// @param srcWidth
///   Width in pixels of the image.
///
/// @param srcHeight
///   Height in pixels of the image.
///
/// @param nX
///   X location on image of starting search pixel.
///
/// @param nY
///   Y location on image of starting search pixel.
///
/// @param nNum
///   Number of pixels (in X direction) on image to sweep.
///
/// @param dotProducts
///   Output dot product values of nNum pixels.
///   \n\b WARNING: array size must be a multiple of 4 (e.g., 4, 8, 12, ...)
///   \n\b NOTE: array should be 128-bit aligned
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct8x8u8( const uint8_t* __restrict patch_pixels,
                    const uint8_t* __restrict image_pixels,
                    unsigned short            imgW,
                    unsigned short            imgH,
                    int                       nX,
                    int                       nY,
                    unsigned int              nNum,
                    int32_t* __restrict       dotProducts );


//------------------------------------------------------------------------------
/// @brief
///   Dot product of 1 patch (8x8 byte square) with 8x8 squares in 11x12
///   rectangle around the center search pixel (iX,iY).
///
/// @param patchPixels
///   Pointer to 8-bit patch pixel values linearly laid out in memory.
///
/// @param imagePixels
///   Pointer to 8-bit image pixel values linearly laid out in memory.
///
/// @param srcWidth
///   Width in pixels of the image.
///
/// @param srcHeight
///   Height in pixels of the image.
///
/// @param iX
///   X location on image of the center of the search window.
///
/// @param iY
///   Y location on image of the center of the search window.
///
/// @param dotProducts
///   Output 11x12 dot product values.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @ingroup math_vector
//---------------------------------------------------------------------------

FASTCV_API void
fcvDotProduct11x12u8( const uint8_t* __restrict patch_pixels,
                      const uint8_t* __restrict image_pixels,
                      unsigned short            imgW,
                      unsigned short            imgH,
                      int                       iX,
                      int                       iY,
                      int32_t* __restrict       dotProducts );


//------------------------------------------------------------------------------
/// @brief
///   3x3 Sobel edge filter
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterSobel3x3u8_v2(). In the 2.0.0 release, 
///   fcvFilterSobel3x3u8_v2 will be renamed to fcvFilterSobel3x3u8
///   and the signature of fcvFilterSobel3x3u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///    TBD.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: data must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be multiple of 8
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output 8-bit image.
///   \n\b WARNING: data must be 128-bit aligned.
///
/// @test
///   -# DC inputs. An image with constant intensity of 255 was created for
///                     testing. All outputs should be 0.
///   -# Images with checker board patterns with different block sizes (1 to 11)
///                     were created for testing.
///   -# Images with vertical, horizontal and diagonal stripes of
///                     different width (1 to 11) were created for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterSobel3x3u8( const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
/// @brief
///   3x3 Sobel edge filter
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterSobel3x3u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterSobel3x3u8,
///   \a fcvFilterSobel3x3u8_v2 will be removed, and the current signature
///   for \a fcvFilterSobel3x3u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterSobel3x3u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///    TBD.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: data must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be multiple of 8
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Output 8-bit image.
///   \n\b WARNING: data must be 128-bit aligned.
/// 
/// @param dstStride
///   Output stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @test
///   -# DC inputs. An image with constant intensity of 255 was created for
///                     testing. All outputs should be 0.
///   -# Images with checker board patterns with different block sizes (1 to 11)
///                     were created for testing.
///   -# Images with vertical, horizontal and diagonal stripes of
///                     different width (1 to 11) were created for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterSobel3x3u8_v2( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        uint8_t* __restrict       dst,
                        unsigned int              dstStride );

//------------------------------------------------------------------------------
/// @brief
///   Canny edge filter
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterCanny3x3u8_v2(). In the 2.0.0 release, 
///   fcvFilterCanny3x3u8_v2 will be renamed to fcvFilterCanny3x3u8
///   and the signature of fcvFilterCanny3x3u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   Canny edge detector applied to a 8 bit grayscale image. The Canny edge
///   detector uses min/max threshold to classify an edge. The min threshold
///   is set to 0 and the max threshold is set to 15. The aperture size used
///   in the Canny edge detector will be same as the filter footprint in the
///   Sobel edge detector and is set to 3. This function will output the edge
///   map stored as a binarized image (0x0 - not an edge, 0xFF - edge). The
///   algorithm works in 3 steps: i) calculates the gradients of the input
///   image, ii) does non-max suppression of the input image gradients to find
///   potential edges and iii) does edge/contour following for edge thresholding,
///   i.e. discard edge pixels that have gradient magnitudes outside the min-max
///   threshold provided by the user.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: data must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be multiple of 8
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output 8-bit image containing the edge detection results.
///
/// @param lowThresh
///   For all the intermediate pixels along the edge, the magnitude of the
///   gradient at the pixel locations should be greater than 'low'
///   (sqrt(gx^2 + gy^2) > low, where gx and gy are X and Y gradient)
///
/// @param highThresh
///   For an edge starting point, i.e. either the first or last
///   pixel of the edge, the magnitude of the gradient at the pixel should be
///   greater than 'high' (sqrt(gx^2 + gy^2) > high, where gx and gy are X and
///   Y gradient).
///
/// @test
///   -# Test edge detection thresholds. An image pattern with
///                     increasing gradients along the edge tangent direction was
///                     created to test bit-exactness of the implementation in
///                     response to different thresholds. Extreme thresholds were
///                     tested as well.
///   -# Test sensitivity to edge curvatures, polarity (positive or negative
///                     contrast), and orientation. Small image patches were created
///                     containing short edges that subtend different angles were created.
///                     The image patches were also negated and rotated to test
///                     polarity and orientation.
///   -# Test T/Y junctions and edge linking.
///                     Image patches with different angles subtending the top of the Y
///                     arm were created.
///   -# Test edge detection on real images.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCanny3x3u8( const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     uint8_t* __restrict       dst,
                     int                       low,
                     int                       high );

//------------------------------------------------------------------------------
/// @brief
///   Canny edge filter
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterCanny3x3u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterCanny3x3u8,
///   \a fcvFilterCanny3x3u8_v2 will be removed, and the current signature
///   for \a fcvFilterCanny3x3u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterCanny3x3u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   Canny edge detector applied to a 8 bit grayscale image. The Canny edge
///   detector uses min/max threshold to classify an edge. The min threshold
///   is set to 0 and the max threshold is set to 15. The aperture size used
///   in the Canny edge detector will be same as the filter footprint in the
///   Sobel edge detector and is set to 3. This function will output the edge
///   map stored as a binarized image (0x0 - not an edge, 0xFF - edge). The
///   algorithm works in 3 steps: i) calculates the gradients of the input
///   image, ii) does non-max suppression of the input image gradients to find
///   potential edges and iii) does edge/contour following for edge thresholding,
///   i.e. discard edge pixels that have gradient magnitudes outside the min-max
///   threshold provided by the user.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: data must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be multiple of 8
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride.
///
/// @param dst
///   Output 8-bit image containing the edge detection results.
/// 
/// @param dstStride
///   Output stride.
///
/// @param lowThresh
///   For all the intermediate pixels along the edge, the magnitude of the
///   gradient at the pixel locations should be greater than 'low'
///   (sqrt(gx^2 + gy^2) > low, where gx and gy are X and Y gradient)
///
/// @param highThresh
///   For an edge starting point, i.e. either the first or last
///   pixel of the edge, the magnitude of the gradient at the pixel should be
///   greater than 'high' (sqrt(gx^2 + gy^2) > high, where gx and gy are X and
///   Y gradient).
///
/// @test
///   -# Test edge detection thresholds. An image pattern with
///                     increasing gradients along the edge tangent direction was
///                     created to test bit-exactness of the implementation in
///                     response to different thresholds. Extreme thresholds were
///                     tested as well.
///   -# Test sensitivity to edge curvatures, polarity (positive or negative
///                     contrast), and orientation. Small image patches were created
///                     containing short edges that subtend different angles were created.
///                     The image patches were also negated and rotated to test
///                     polarity and orientation.
///   -# Test T/Y junctions and edge linking.
///                     Image patches with different angles subtending the top of the Y
///                     arm were created.
///   -# Test edge detection on real images.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCanny3x3u8_v2( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        uint8_t* __restrict       dst,
                        unsigned int              dstStride,
                        int                       low,
                        int                       high );

//------------------------------------------------------------------------------
/// @brief
///   Performs image difference by subracting src2 from src1. dst=src1-src2.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvImageDiffu8_v2(). In the 2.0.0 release, 
///   fcvImageDiffu8_v2 will be renamed to fcvImageDiffu8
///   and the signature of fcvImageDiffu8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   
/// @param src1
///   First source image
///
/// @param src2
///   Second source image, must be same size as src1.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be a multiple of 8.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Destination.
///   \n\b NOTE: Must be same size as src1 and src2.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# 
///   -# 
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageDiffu8(   const uint8_t* __restrict src1,
                  const uint8_t* __restrict src2,
                   unsigned int             srcWidth,
                   unsigned int             srcHeight,
                        uint8_t* __restrict dst );

//------------------------------------------------------------------------------
/// @brief
///   Performs image difference by subracting src2 from src1. dst=src1-src2.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvImageDiffu8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvImageDiffu8,
///   \a fcvImageDiffu8_v2 will be removed, and the current signature
///   for \a fcvImageDiffu8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvImageDiffu8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   
/// @param src1
///   First source image
///
/// @param src2
///   Second source image, must be same size as src1.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be a multiple of 8.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Stride of input image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Destination.
///   \n\b NOTE: Must be same size as src1 and src2.
///   \n\b WARNING: must be 128-bit aligned.
/// 
/// @param dstStride
///   Stride of output image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   \n\b WARNING: must be multiple of 8.
///
/// @test
///   -# 
///   -# 
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageDiffu8_v2( const uint8_t* __restrict src1,
                   const uint8_t* __restrict src2,
                   unsigned int              srcWidth,
                   unsigned int              srcHeight,
                   unsigned int              srcStride,
                   uint8_t* __restrict       dst,
                   unsigned int              dstStride );


//--------------------------------------------------------------------------
/// @brief
///   Compute image difference src1-src2
///
/// @param src1
///   Input image1 of int16 type
/// 
/// @param src2
///   Input image2, must have same size as src1
/// 
/// @param srcWidth
///   Input image width
/// 
/// @param srcHeight
///   Input image height
/// 
/// @param srcStride
///   Stride of input image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
/// 
/// @param dst
///   Output image, saturated for int16 type
/// 
/// @param dstStride
///   Stride of output image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
/// 
/// @ingroup image_processing
////------------------------------------------------------------------------
FASTCV_API void 
fcvImageDiffs16( const int16_t* __restrict src1,
                 const int16_t* __restrict src2,
                       unsigned int             srcWidth, 
                       unsigned int             srcHeight, 
                       unsigned int             srcStride, 
                            int16_t* __restrict dst,
                       unsigned int             dstStride );

//------------------------------------------------------------------------------
/// @brief
///   Performs image difference by subracting src2 from src1. dst=src1-src2.
///
/// @details
///   
/// @param src1
///   First source image
///
/// @param src2
///   Second source image, must be same size as src1.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param srcStride
///   Stride of input image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Destination.
///   \n\b NOTE: Must be same size as src1 and src2.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dstStride
///   Stride of output image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   \n\b WARNING: must be multiple of 8.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------
FASTCV_API void 
fcvImageDifff32( const float* __restrict src1,
                 const float* __restrict src2,
                unsigned int             srcWidth, 
                unsigned int             srcHeight, 
                unsigned int             srcStride, 
                       float* __restrict dst,
                unsigned int             dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Performs image difference by promoting both src1 and src 2 to 
///   floating point values and then subracting src2 from src1. dst=src1-src2.
///
/// @details
///   
/// @param src1
///   First source image
///
/// @param src2
///   Second source image, must be same size as src1.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param srcStride
///   Stride of input image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Destination image in float type
///   \n\b NOTE: Must be same size as src1 and src2.
///   \n\b WARNING: must be 128-bit aligned.
/// 
/// @param dstStride
///   Stride of output image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   \n\b WARNING: must be multiple of 8.
/// 
/// @ingroup image_processing
//------------------------------------------------------------------------------
FASTCV_API void 
fcvImageDiffu8f32( const uint8_t* __restrict src1,
                   const uint8_t* __restrict src2,
                    unsigned int             srcWidth, 
                    unsigned int             srcHeight, 
                    unsigned int             srcStride, 
                           float* __restrict dst,
                    unsigned int             dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Performs image difference by subracting src2 from src1. 
///   dst = ( src1 >> 1) - ( src2 >> 1).
///
/// @details
///   
/// @param src1
///   First source image
///
/// @param src2
///   Second source image, must be same size as src1.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param srcStride
///   Stride of input image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Destination image in int8 type
///   \n\b NOTE: Must be same size as src1 and src2.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dstStride
///   Stride of output image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   \n\b WARNING: must be multiple of 8.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------
FASTCV_API void 
fcvImageDiffu8s8( const uint8_t* __restrict src1,
                  const uint8_t* __restrict src2,
                   unsigned int             srcWidth, 
                   unsigned int             srcHeight, 
                   unsigned int             srcStride, 
                         int8_t* __restrict dst,
                    unsigned int             dstStride );

//---------------------------------------------------------------------------
/// @brief
///   Creates 2D gradient from source illuminance data.
///   This function considers only the left/right neighbors
///   for x-gradients and top/bottom neighbors for y-gradients.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvImageGradientInterleaveds16_v2(). In the 2.0.0 release, 
///   fcvImageGradientInterleaveds16_v2 will be renamed to fcvImageGradientInterleaveds16
///   and the signature of fcvImageGradientInterleaveds16 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param  src
///    Input image/patch.
///
/// @param srcWidth
///    Width of src data to create gradient.
///
/// @param srcHeight
///    Height of src data to create gradient.
///
/// @param  srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param  gradients
///    Buffer to store gradient. Must be 2*(width-1)*(height-1) in size.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientInterleaveds16( const uint8_t* __restrict src,
                                unsigned int              srcWidth,
                                unsigned int              srcHeight,
                                unsigned int              srcStride,
                                int16_t* __restrict       gradients
                              );

//---------------------------------------------------------------------------
/// @brief
///   Creates 2D gradient from source illuminance data.
///   This function considers only the left/right neighbors
///   for x-gradients and top/bottom neighbors for y-gradients.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvImageGradientInterleaveds16() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvImageGradientInterleaveds16,
///   \a fcvImageGradientInterleaveds16_v2 will be removed, and the current signature
///   for \a fcvImageGradientInterleaveds16 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvImageGradientInterleaveds16 when transitioning to 2.0.0.
///   \n\n
///
/// @param  src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param  srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///   \n\b WARNING: must be multiple of 8.
///
/// @param  gradients
///   Buffer to store gradient. Must be 2*(width-1)*(height-1) in size.
/// 
/// @param gradStride
///   Stride in bytes of the interleaved gradients array.
///   \n\b WARNING: must be multiple of 16 ( 8 * 2-byte values ).
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientInterleaveds16_v2( const uint8_t* __restrict src,
                                   unsigned int              srcWidth,
                                   unsigned int              srcHeight,
                                   unsigned int              srcStride,
                                   int16_t* __restrict       gradients,
                                   unsigned int              gradStride );

//---------------------------------------------------------------------------
///  @brief
///  Function to initialize MSER. To invoke MSER functionality, 3 functions have to be called:
///    fcvMserInit, fcvMseru8, fcvMserRelease.
///  
///   Heris the typical usage:
///  
///    void *mserHandle;
///     if (fcvMserInit (width,........,&mserHandle))
///      {
///          fcvmseru8 (mserHandle,...);
///          fcvRelease(mserHandle);
///      }
/// 
///   
///   @param width          Width of the image  for which MSER has to be done.
///   @param height         Height of the image for which MSER has to be done.
///   @param delta          Delta to be used in MSER algorithm (the difference in grayscale
///                         values within which the region is stable ). 
///                         Typical value range [0.8 8], typical value 2
///   @param minArea        Minimum area (number of pixels) of a mser contour.
///                         Typical value range [10 50], typical value 30
///   @param maxArea        Maximum area (number of pixels) of a  mser contour.
///                         Typical value 14400 or 0.25*width*height
///   @param maxVariation   Maximum variation in grayscale between 2 levels allowed.
///                         Typical value range [0.1 1.0], typical value 0.15
///   @param minDiversity   Minimum diversity in grayscale between 2 levels allowed.
///                         Typical value range [0.1 1.0], typical value 0.2
///   @param mserHandle     Return value: the mserHandle to be used in subsequent calls.
///  
///   @return  int  1 if mserInit is successful, if 0, mserHandle is invalid.
///
///   @ingroup object_detection
//------------------------------------------------------------------------------
FASTCV_API int 
fcvMserInit(const unsigned int width,
                 const unsigned int height, 
                 unsigned int delta, 
                 unsigned int minArea ,
                 unsigned int maxArea , 
                 float maxVariation ,
                 float minDiversity , void ** mserHandle );

//---------------------------------------------------------------------------
/// @brief
///  Function to release  MSER resources.
///  
/// 
///   
///   @param mserHandle   Handle to be used to free up MSER resources.
///  
///   @ingroup object_detection
//------------------------------------------------------------------------------
FASTCV_API void
fcvMserRelease(void *mserHandle);

///---------------------------------------------------------------------------
/// @brief
///  Function to invoke  MSER. 
/// 
///   
///   @param mserHandle     The MSER Handle returned by init.
///   @param srcPtr         Pointer to an image array (unsigned char ) for which MSER has to be done.
///   @param srcWidth       Width of the source image.
///   @param srcHeight      Height of the source image.
///   @param maxContours    Maximum contours that will be returned. Must be set to 2x the maximum contours.
///   @param numContours    Output, Number of MSER contours in the region.
///   @param numPointsInContour    Output, Number of points in each contour. This will have values filled up
///                                for the first (*numContours) values. This memory has to be allocated by 
///                                the caller. 
///   @param pointsArraySize Size of the output points Array.
///                          Typical size: (# of pixels in source image) * 30
///   @param pointsArray     Output. This is the points in all the contours. This is a linear array, whose memory
///                          has to be allocated by the caller.
///                          Typical allocation size:  pointArraySize*2
///                          pointsArray[0...numPointsInContour[0]-1] defines the first MSER region, 
///                          pointsArray[numPointsInContour[0] .. numPointsInContour[1]-1] defines 2nd MSER region
///                          and so on.
/// 
///   @ingroup object_detection
//------------------------------------------------------------------------------  
FASTCV_API void
fcvMseru8( void *mserHandle,
                const uint8_t* __restrict srcPtr,unsigned int srcWidth, 
                unsigned int srcHeight, unsigned int srcStride, 
                unsigned int maxContours,
                unsigned int * __restrict numContours, unsigned int * __restrict numPointsInContour   ,
                unsigned int pointsArraySize,
                unsigned int* __restrict pointsArray
              );

///---------------------------------------------------------------------------
/// @brief
///  Function to invoke  MSER, with additional outputs for each contour.
/// 
///   
///   @param mserHandle     The MSER Handle returned by init.
///   @param srcPtr         Pointer to an image array (unsigned char ) for which MSER has to be done.
///   @param srcWidth       Width of the source image.
///   @param srcHeight      Height of the source image.
///   @param srcStride      Stride of the source image.
///   @param maxContours    Maximum contours that will be returned. Need to be set to 2x the maximum contours.
///                         Application dependent. OCR usually requires 100-1000 contours
///                         Segmentation usually requires 50-100
///   @param numContours    Output, Number of MSER contours in the region.
///   @param numPointsInContour    Output, Number of points in each contour. This will have values filled up
///                                for the first (*numContours) values. This memory has to be allocated by 
///                                the caller. 
///   @param pointsArraySize Size of the output points Array.
///                          Typical size: (# of pixels in source image)*30
///   @param pointsArray     Output. This is the points in all the contours. This is a linear array, whose memory
///                          has to be allocated by the caller. 
///                          Typical allocation size:  pointArraySize*2
///                          pointsArray[0...numPointsInContour[0]-1] defines the first MSER region;
///                          pointsArray[numPointsInContour[0] .. numPointsInContour[1]-1] defines 2nd MSER region
///                          and so on.
///   @param contourVariation    Output, Variation for each contour from previous grey level.
///                                This will have values filled up
///                                for the first (*numContours) values. This memory has to be allocated by 
///                                the caller. 
///   @param contourPolarity     Output, Polarity for each contour. This value is 1 if this is a MSER+ region, 
///                              -1 if this is a MSER- region. . This will have values filled up
///                                for the first (*numContours) values. This memory has to be allocated by 
///                                the caller. 
///   @param contourNodeId       Output, Node id for each contour.  This will have values filled up
///                                for the first (*numContours) values. This memory has to be allocated by 
///                                the caller. 
///   @param contourNodeCounter    Output, Node counter for each contour. This will have values filled up
///                                for the first (*numContours) values. This memory has to be allocated by 
///                                the caller. 
/// 
///   @ingroup object_detection
//------------------------------------------------------------------------------  
FASTCV_API void
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
              );



//---------------------------------------------------------------------------
/// @brief
///   Creates 2D gradient from source illuminance data.
///   This function considers only the left/right neighbors
///   for x-gradients and top/bottom neighbors for y-gradients.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvImageGradientInterleavedf32_v2(). In the 2.0.0 release, 
///   fcvImageGradientInterleavedf32_v2 will be renamed to fcvImageGradientInterleavedf32
///   and the signature of fcvImageGradientInterleavedf32 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///  Input image/patch.
///
/// @param srcWidth
///    Width of src data to create gradient.
///
/// @param srcHeight
///    Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param gradients
///    Buffer to store gradient. Must be 2*(width-1)*(height-1) in size.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientInterleavedf32( const uint8_t* __restrict src,
                                unsigned int              srcWidth,
                                unsigned int              srcHeight,
                                unsigned int              srcStride,
                                float* __restrict         gradients );

//---------------------------------------------------------------------------
/// @brief
///   Creates 2D gradient from source illuminance data.
///   This function considers only the left/right neighbors
///   for x-gradients and top/bottom neighbors for y-gradients.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvImageGradientInterleavedf32() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvImageGradientInterleavedf32,
///   \a fcvImageGradientInterleavedf32_v2 will be removed, and the current signature
///   for \a fcvImageGradientInterleavedf32 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvImageGradientInterleavedf32 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///   \n\b WARNING: must be multiple of 8.
///
/// @param gradients
///   Buffer to store gradient. Must be 2*(width-1)*(height-1) in size.
/// 
/// @param gradStride
///   Stride (in bytes) of the interleaved gradients array.
///   \n\b WARNING: must be multiple of 32 ( 8 * 4-byte values ).
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientInterleavedf32_v2( const uint8_t* __restrict src,
                                   unsigned int              srcWidth,
                                   unsigned int              srcHeight,
                                   unsigned int              srcStride,
                                   float* __restrict         gradients,
                                   unsigned int              gradStride );

//---------------------------------------------------------------------------
/// @brief
///   Creates 2D gradient from source illuminance data.
///   This function considers only the left/right neighbors
///   for x-gradients and top/bottom neighbors for y-gradients.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvImageGradientPlanars16_v2(). In the 2.0.0 release, 
///   fcvImageGradientPlanars16_v2 will be renamed to fcvImageGradientPlanars16
///   and the signature of fcvImageGradientPlanars16 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///    Width of src data to create gradient.
///
/// @param srcHeight
///    Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param dx
///    Buffer to store horizontal gradient. Must be (width)*(height) in size.
///
/// @param dy
///    Buffer to store vertical gradient. Must be (width)*(height) in size.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientPlanars16( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           unsigned int              srcStride,
                           int16_t* __restrict       dx,
                           int16_t* __restrict       dy );

//---------------------------------------------------------------------------
/// @brief
///   Creates 2D gradient from source illuminance data.
///   This function considers only the left/right neighbors
///   for x-gradients and top/bottom neighbors for y-gradients.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvImageGradientPlanars16() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvImageGradientPlanars16,
///   \a fcvImageGradientPlanars16_v2 will be removed, and the current signature
///   for \a fcvImageGradientPlanars16 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvImageGradientPlanars16 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///   \n\bWARNING: must be multiple of 8.
///
/// @param srcHeight
///    Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param dx
///    Buffer to store horizontal gradient. Must be (width)*(height) in size.
///
/// @param dy
///    Buffer to store vertical gradient. Must be (width)*(height) in size.
/// 
/// @param dxyStride
///   Stride (in bytes) of 'dx' and 'dy' arrays.
///   \n\bWARNING: must be multiple of 16 (8 * 2-bytes per gradient value).
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientPlanars16_v2( const uint8_t* __restrict src,
                              unsigned int              srcWidth,
                              unsigned int              srcHeight,
                              unsigned int              srcStride,
                              int16_t* __restrict       dx,
                              int16_t* __restrict       dy,
                              unsigned int              dxyStride );

//---------------------------------------------------------------------------
/// @brief
///   Creates 2D gradient from source illuminance data.
///   This function considers only the left/right neighbors
///   for x-gradients and top/bottom neighbors for y-gradients.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvImageGradientPlanarf32_v2(). In the 2.0.0 release, 
///   fcvImageGradientPlanarf32_v2 will be renamed to fcvImageGradientPlanarf32
///   and the signature of fcvImageGradientPlanarf32 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///  Input image/patch.
///
/// @param srcWidth
///    Width of src data to create gradient.
///
/// @param srcHeight
///    Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param dx
///    Buffer to store horizontal gradient. Must be (width)*(height) in size.
///
/// @param dy
///    Buffer to store vertical gradient. Must be (width)*(height) in size.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientPlanarf32( const uint8_t* __restrict src,
                           unsigned int              srcWidth,
                           unsigned int              srcHeight,
                           unsigned int              srcStride,
                           float* __restrict         dx,
                           float* __restrict         dy );



//---------------------------------------------------------------------------
/// @brief
///   Creates 2D gradient from source illuminance data.
///   This function considers only the left/right neighbors
///   for x-gradients and top/bottom neighbors for y-gradients.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvImageGradientPlanarf32() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvImageGradientPlanarf32,
///   \a fcvImageGradientPlanarf32_v2 will be removed, and the current signature
///   for \a fcvImageGradientPlanarf32 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvImageGradientPlanarf32 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///   \n\bWARNING: must be multiple of 8.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///   \n\bWARNING: must be multiple of 8.
///
/// @param dx
///   Buffer to store horizontal gradient. Must be (width)*(height) in size.
///
/// @param dy
///   Buffer to store vertical gradient. Must be (width)*(height) in size.
/// 
/// @param dxyStride
///   Stride of Gradient values ('dx' and 'dy' arrays) measured in bytes.
///   \n\bWARNING: must be multiple of 32 (8 * 4-bytes per gradient value).
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientPlanarf32_v2( const uint8_t* __restrict src,
                              unsigned int              srcWidth,
                              unsigned int              srcHeight,
                              unsigned int              srcStride,
                              float* __restrict         dx,
                              float* __restrict         dy,
                              unsigned int              dxyStride );


//------------------------------------------------------------------------------
/// @brief
///   Extracts FAST corners from the image. This function tests the whole image
///   for corners (apart from the border). FAST corners are detected by testing
///   pixels in a Brezenham circle of radius 16 around a center pixel. FAST
///   features are detected around pixels which have a number of contiguous
///   pixels around them all brighter or all darker than the center pixel by a
///   certain threshold (barrier). FAST-9 looks for continuous segments on the
///   pixel ring of 9 pixels or more.
///
/// @param src
///   Pointer to grayscale image with one byte per pixel
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Input image width
///   \n\b WARNING: must be a multiple of 8.
///   \n\b WARNING: must be <= 2048.
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2). If 0 is passed, srcStride is set to width.
///
/// @param barrier
///   FAST threshold. Minimum difference in intensity between center and
///   contiguous segment.
///
/// @param border
///   Number for pixels to ignore from top,bottom,right,left of the image
///
/// @param xy
///   pointer to  the output array containing the interleaved x,y position of the
///   detected corners
///   \n e.g. struct { int x, y; } xy;
///   \n\b WARNING: must be 128-bit aligned.
///   \n\b NOTE: Remember to allocate double the size of @param maxnumcorners
///
/// @param nCornersMax
///   Maximum number of corners. The function exists when the maximum number of
///   corners is exceeded
///
/// @param nCorners
///   pointer to an integer storing the number of corners detected
///
/// @return
///   0 if successful.
///
/// @test
///   -# Test rotational invariance and streak length (FAST 9 corners must
///                     have a continuous streak of 9 or more consistently brigher or
///                     darker pixels compared to the center pixel. An 200x240 image was
///                     created with a checker board pattern (block size of 1 pixel) as
///                     background to prevent any accidental "corner". FAST corners with
///                     streak lengths ranging from 7 to 16, with both polarity (center
///                     brighter/darker) were created, with each corner pattern ( a
///                     combination of streak length and polarity) rotated in all 16
///                     possible ways and put on the same row.
///   -# Test "border" and "barrier". An 200x240 image was created with a
///                     checker board pattern (block size of 1 pixel) as background to
///                     prevent any accidental "corner". FAST12 corners with different
///                     contrasts (5 to 80) was used for tests. The image pattern was
///                     used to test different barriers (30, 40, 50, 60, 70) and
///                     boundaries (9, 11, 13, 15, 17) correspondingly.
///   -# The "Oxford Affine Covariant Features" were used for testing.
///                     The image dataset was created with the following conditions
///                     in mind: extreme geometric transformation; blurriness; lighting
///                     conditions, {\em et al}. This is a good test set for real image
///                     conditions.
///   -# All $3^{16}$ possible patterns of FAST corners. A comparision
///                     between a center pixel and a pixel on the radius 3 circle can
///                     have 3 states, darker/brighter/ambiguous; and there are 16
///                     pixels on the ring. All test vectors were packed as tightly as
///                     possible.
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast9u8( const uint8_t* __restrict src,
                  unsigned int              srcWidth,
                  unsigned int              srcHeight,
                  unsigned int              srcStride,
                  int                       barrier,
                  unsigned int              border,
                  uint32_t* __restrict      xy,
                  unsigned int              nCornersMax,
                  uint32_t* __restrict      nCorners );


//------------------------------------------------------------------------------
/// @brief
///   Extracts FAST corners from the image. This function takes a bit mask so
///   that only image areas masked with '1' are tested for corners (if these
///   areas are also not part of the border). FAST corners are detected by
///   testing pixels in a Brezenham circle of radius 16 around a center pixel.
///   FAST features are detected around pixels which have a number of contiguous
///   pixels around them all brighter or all darker than the center pixel by a
///   certain threshold (barrier). FAST-9 looks for continuous segments on the
///   pixel ring of 9 pixels or more.
///
/// @param src
///   pointer to grayscale image with one byte per pixel
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   image width
///   \n\b WARNING: must be <= 2048.
///   \n\b WARNING: must be a multiple of 8.
///
/// @param srcHeight
///   image height
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param barrier
///   FAST threshold. Minimum difference in intensity between center and
///   contiguous segment.
///
/// @param border
///   Number for pixels to ignore from top,bottom,right,left of the image
///
/// @param xy
///   pointer to the output array containing the interleaved x,y position of the
///   detected corners
///   \n\b WARNING: must be 128-bit aligned.
///   \n\b NOTE: Remember to allocate double the size of @param maxnumcorners
///
/// @param nCornersMax
///   Maximum number of corners. The function exists when the maximum number of corners
///   is exceeded
///
/// @param nCorners
///   pointer to an integer storing the number of corners detected
///
/// @param mask
///   Per-pixel mask for each pixel represented in input image. 
///   If a bit set to 0, pixel will be a candidate for corner detection. 
///   If a bit set to 1, pixel will be ignored.
///
/// @param maskWidth
///   Width of mask. width/maskWidth must a power-of-two or maskWidth/width
///   must a power-of-two.
///
/// @param maskHeight
///   Height of mask. height/maskHeight must a power-of-two or maskHeight/height
///   must a power-of-two.
///
/// @return
///   0 if successful.
///
/// @test
///   Test all test vectors but the exhaustive enumerating set using the
///   following mask combinations
///   -# Full image.
///   -#  Two shifted checker board pattern (with block size of 1 pixel).
///   -# A random quadrant of the input images.
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast9InMasku8( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        int                       barrier,
                        unsigned int              border,
                        uint32_t* __restrict      xy,
                        unsigned int              nCornersMax,
                        uint32_t* __restrict      nCorners,
                        const uint8_t* __restrict mask,
                        unsigned int              maskWidth,
                        unsigned int              maskHeight );


//------------------------------------------------------------------------------
/// @brief
///   Extracts Harris corners from the image. This function tests the whole
///   image for corners (apart from the border). The Harris corner detector
///   looks for areas in an image with strong 2D structure, by looking at the
///   eigenvalues of the structure tensor of the patch. If a patch has 2 large
///   positive eigenvalues in it's structure tensor, it is marked as a corner.
///
/// @param src
///   Pointer to grayscale image with one byte per pixel
///
/// @param srcWidth
///   Input image width
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param border
///   Number for pixels to ignore from top,bottom,right,left of the image
///
/// @param xy
///   pointer to  the output array containing the interleaved x,y position of the
///   detected corners
///
/// @param nCornersMax
///   Maximum number of corners. The function exists when the maximum number of
///   corners is exceeded
///
/// @param nCorners
///   pointer to an integer storing the number of corners detected
/// 
/// @param threshold
///   Minimum "Harris Score" or "Harris Corner Respose" of a pixel for it to be 
///   regarded as a corner.
///
/// @return
///   0 if successful.
///
/// @test
///   -# Test Threshold. Corners with different contrasts (2:2:64) was
///                     created in an image and different threshold was used to test
///                     a Harris corner detector implementation.
///   -# Test curvature. A corner with different angles (small angles
///                     correspond to thin wedges) in the range of 0:10:180 degrees
///                     was created in the middle of a small image. A Harris corner
///                     detector should respond to corners with the right range of
///                     curvatures (around 90 degrees).
///   -# Test location of the corners. A right-angled corner was placed
///                     at different locations of a 32x48 image and the corner should
///                     be detected once it is within the predefined boundary.
///   -# Test number of corners: from no corner to a few dozen corners.
///   -# Image with different sine wave patterns. At the right curvatures
///                     a sine wave image can result in Harris corners.
///   -# Test image with perfect checkerboard patterns (block size of 4
///                     pixels). With improper implementation there will be no corners
///                     being detected.
///   -# Test corners from 50 real image patches.
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerHarrisu8( const uint8_t* __restrict src,
                   unsigned int              srcWidth,
                   unsigned int              srcHeight,
                   unsigned int              srcStride,
                   unsigned int              border,
                   uint32_t* __restrict      xy,
                   unsigned int              nCornersMax,
                   uint32_t* __restrict      nCorners,
                   int                       threshold );

//------------------------------------------------------------------------------
/// @brief
///   Local Harris Max applies the Harris Corner algorithm on an 11x11 patch 
///   within an image to determine if a corner is present. The Harris algorithm 
///   looks for areas in an image with strong 2D structure, by looking at the
///   eigenvalues of the structure tensor of the patch. If a patch has 2 large
///   positive eigenvalues in it's structure tensor, it is deemed as a corner.
///
/// @param src
///   Pointer to grayscale image with one byte per pixel
///
/// @param srcWidth
///   Input image width
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).  If srcStride == 0, then will use srcWidth.
///
/// @param posX
///   Center X coordinate of the search window
///
/// @param posY
///   Center Y coordinate of the search window
///
/// @param maxX
///   pointer to the X coordinate identified as a corner
///
/// @param maxY
///   pointer to the Y coordinate identified as a corner
///
/// @param maxScore
///   pointer to the Harris score associated with the corner
///
/// @return
///   0 if no corner is found (maxX, maxY, and maxScore are invalid)
///     or if posX and/or posY position the patch outside of the range of
///     the source image.
///   1 if a corner is found (maxX, maxY, and maxScore are valid)
///
/// @test
///   -# Test image with 60 corners.  Rolling the patch window through all
///      legal locations within the image
///   -# Test image with 1 corner.  Rolling the patch window through all legal
///      locations within the image
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------

FASTCV_API unsigned int
fcvLocalHarrisMaxu8( const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     unsigned int              srcStride,
                     unsigned int              posX,
                     unsigned int              posY,
                     unsigned int             *maxX,
                     unsigned int             *maxY,
                     int                      *maxScore);


//------------------------------------------------------------------------------
/// @brief
///   Extracts Harris corners from the image. This function takes a bit mask so
///   that only image areas masked with '1' are tested for corners (if these
///   areas are also not part of the border). The Harris corner detector looks
///   for areas in an image with strong 2D structure, by looking at the
///   eigenvalues of the structure tensor of the patch. If a patch has 2 large
///   positive eigenvalues in it's structure tensor, it is marked as a corner.
///
/// @param src
///   pointer to grayscale image with one byte per pixel
///
/// @param srcWidth
///   image width
///
/// @param srcHeight
///   image height
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param border
///   Number for pixels to ignore from top,bottom,right,left of the image
///
/// @param xy
///   pointer to  the output array containing the interleaved x,y position of the
///   detected corners
///
/// @param nCornersMax
///   Maximum number of corners. The function exists when the maximum number of corners
///   is exceeded
///
/// @param nCorners
///   pointer to an integer storing the number of corners detected
///
/// @param mask
///   Per-pixel mask for each pixel represented in input image. If set to 1 pixel
///   will be a candidate for corner detection. If set to 0 will be ignored.
///
/// @param maskWidth
///   Width of mask. width/maskWidth must a power-of-two or maskWidth/width
///   must a power-of-two.
///
/// @param maskHeight
///   Height of mask. height/maskHeight must a power-of-two or maskHeight/height
///   must a power-of-two.
///
/// @return
///   0 if successful.
///
/// @test
///   -# Test all cases from tests defined in cvdlCornerHarrisu8 with full
///                     masks.
///   -# Test on a real image patch: a quadrant of the patch.
///   -# Test a small size mask: the function should automatically scale up
///                     the mask and the mask should not create new corners as a result.
///   -# Test a checkerboard (block size of 1 pixel) mask on a checkerboard
///                     image (block size of 4 pixels).
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerHarrisInMasku8( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         unsigned int              srcStride,
                         unsigned int              border,
                         uint32_t* __restrict      xy,
                         unsigned int              nCornersMax,
                         uint32_t* __restrict      nCorners,
                         int                       threshold,
                         const uint8_t* __restrict mask,
                         unsigned int              maskWidth,
                         unsigned int              maskHeight );


//---------------------------------------------------------------------------
/// @brief
///   Computes affine trans. for a given set of corresponding features points
///   using a linear least square colver based on Cholkesky decomposition.
///
/// @param corrs
///  Correspondence data struct containing coords of points in two frames
///
/// @param affine
///  3 x 3 affine matrix (computed best fit affine transformation)
///
/// @ingroup 3D_reconstruction
//---------------------------------------------------------------------------

FASTCV_API void
fcvGeomAffineFitf32( const fcvCorrespondences* __restrict corrs,
                     float* __restrict                    affine );


//------------------------------------------------------------------------------
/// @brief
///   Evaluates specified affine transformation against provided points
///   correspondences. Checks which correspondence members have a projection
///   error that is smaller than the given one (maxSquErr).
///
/// @param corrs
///   Pointer to correspondences structure.
///
/// @param affine
///   Affine matrix representing relationship between ptTo and ptFrom
///   correspondences stored as 3x3 floating point matrix formatted as
///   @todo r0h0, r0h1
///   Pointer storage must be at least a 9-element floating point array.
///
/// @param maxSquErr
///   Maximum error value squared.
///
/// @param inliers
///   Output array for those indices that passed the test - the array MUST
///   be able to store numIndices items.
///
/// @param numInliers
///   Output number of corrs that passed the test.
///
/// @return
///
/// @test
///   -# Test threshold setting "maxSquErr". A translation only affine is
///                     used to transform points. Two cases were studied: when there is
///                     no error added; when a varying amount of errors were introduced.
///   -# Test stride in the cvdlCorrespondences structure using strides
///                     other than the default values (3 for fromStride and 2 for toStride)
///   -# Test evaluation under other transformations: rotation; shearing;
///                     generic affine.
///
/// @ingroup 3D_reconstruction
//------------------------------------------------------------------------------

FASTCV_API int
fcvGeomAffineEvaluatef32( const fcvCorrespondences* __restrict corrs,
                          float* __restrict                    affine,
                          float                                maxsqerr,
                          uint16_t* __restrict                 inliers,
                          int32_t*                             numinliers );


//------------------------------------------------------------------------------
/// @brief
///   Performs cholesky homography fitting on specified points correspondences.
///
/// @details
///   Output precision is within 3e-3
///
/// @param corrs
///   Pointer to correspondences structure.
///
/// @param homography
///   3x3 floating point matrix formatted as @todo r0h0, r0h1
///   Pointer storage must be at least a 9-element floating point array.
///
/// @test
///   -# Test all 9 cases where there is a 0 in the homography matrix.
///   -# Test a simple translation matrix.
///   -# Test fitting using more than 4 points.
///   -# Test sensitivity to noise corruption.
///   -# Test stride in the cvdlCorrespondences structure using strides
///                     other than the default values (3 for fromStride and 2 for toStride)
///   -# Test the case when there are not enough (4 correspondences is the
///                     minimum) inputs.
///
/// @ingroup 3D_reconstruction
//------------------------------------------------------------------------------

FASTCV_API void
fcvGeomHomographyFitf32( const fcvCorrespondences* __restrict corrs,
                         float* __restrict                    homography );


//------------------------------------------------------------------------------
/// @brief
///   Evaluates specified homography against provided points correspondences.
///   Check which correspondence members have a projection error that is
///   smaller than the given one (maxSquErr).
///
/// @param corrs
///   Pointer to correspondences structure.
///
/// @param homography
///   Homography representing relationship between ptTo and ptFrom
///   correspondences stored as 3x3 floating point matrix formatted as
///   @todo r0h0, r0h1
///   Pointer storage must be at least a 9-element floating point array.
///
/// @param maxSquErr
///   Maximum error value squared.
///
/// @param inliers
///   Output array for those indices that passed the test - the array MUST
///   be able to store numIndices items.
///
/// @param numInliers
///   Output number of corrs that passed the test.
///
/// @return
///   0 that error is less than maximum error, -1 greater or equal to maximum
///   error.
///
/// @test
///   -# Test threshold setting "maxSquErr". A translation only homography is
///                     used to transform points. Two cases were studied: when there is
///                     no error added; when a varying amount of errors were introduced.
///   -# Test stride in the cvdlCorrespondences structure using strides
///                     other than the default values (3 for fromStride and 2 for toStride)
///   -# Test evaluation under other transformations: rotation; shearing;
///                     generic homography.
///
/// @ingroup 3D_reconstruction
//------------------------------------------------------------------------------

FASTCV_API int
fcvGeomHomographyEvaluatef32( const fcvCorrespondences* __restrict corrs,
                              float* __restrict                    homography,
                              float                                maxsqerr,
                              uint16_t* __restrict                 inliers,
                              int32_t*                             numinliers );


//------------------------------------------------------------------------------
/// @brief
///   Performs cholesky pose fitting on specified points correspondences.
///   Takes a pose and uses the correspondences to refine it using iterative
///   Gauss-Newton optimization.
///
/// @param corrs
///   Pointer to correspondences structure.
///
/// @param minIterations
///   Minimum number of iterations to refine.
///
/// @param maxIterations
///   Maximum number of iterations to refine.
///
/// @param stopCriteria
///   Improvement threshold, iterations stop if improvement is less than this
///   value.
///
/// @param initpose
///   Pose representing initial pose
///   correspondences stored as a
///   3x4 transformation matrix in the form [R|t], where R is a 3x3 rotation
///   matrix and t is the translation vector. The matrix  stored in pose is row
///   major ordering: \n
///   a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34 where the
///   matrix is: \n
///   | a11, a12, a13 , a14|\n
///   | a21, a22, a23, a24 |\n
///   | a31, a32, a33, a34 |\n
///   Pointer storage must be at least a 12-element floating point array.
///
/// @param refinedpose
///   Pose representing refined pose
///   correspondences stored as a
///   3x4 transformation matrix in the form [R|t], where R is a 3x3 rotation
///   matrix and t is the translation vector. The matrix  stored in pose is row
///   major ordering: \n
///   a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34 where the
///   matrix is: \n
///   | a11, a12, a13 , a14|\n
///   | a21, a22, a23, a24 |\n
///   | a31, a32, a33, a34 |\n
///   Pointer storage must be at least a 12-element floating point array.
///
/// @return
///   Final reprojection error.
///
/// @test
///   -# Test sensitivity to noise. When there is no noise in the data,
///                     there shouldn't be any update at all.
///   -# Test convergence criterion: number of iterations.
///   -# Test convergence criterion: error tolerance.
///   -# Test influence of noise corrupted 2D points.
///   -# Test influence of noise corrupted 3D points.
///   -# Test influence of noise corrupted 2D and 3D points.
///   -# Test performance when inputing a large quantity of points.
///   -# Test proper handling of outlier points.
///   -# Test stride in the cvdlCorrespondences structure using strides
///      other than the default values (3 for fromStride and 2 for toStride)
///   -# Test using a set of 50 generic poses and randomly generated 3D-2D
///      correspondences.
///
/// @ingroup 3D_reconstruction
//------------------------------------------------------------------------------

FASTCV_API float
fcvGeomPoseRefineGNf32( const fcvCorrespondences* __restrict corrs,
                        short                                minIterations,
                        short                                maxIterations,
                        float                                stopCriteria,
                        float*                               initpose,
                        float*                               refinedpose );

//------------------------------------------------------------------------------
/// @brief
///   Update and compute the differential pose based on the specified points correspondences
///   This function and fcvGeomPoseOptimizeGNf32
///   can be used iteratively to perform poseRefine GN.
///
/// @param projected
///   2D position after projection
///
/// @param reprojError
///   2D reprojection error in camera coordinates (not in pixels!)
///
/// @param invz
///   Inverse depth (z)
///
/// @param reprojVariances
///   Reprojection variance in camera coordinates
///
/// @param numpts
///    Number of points
///
/// @param pose
///   Pose representing differential pose
///   correspondences stored as a
///   3x4 transformation matrix in the form [R|t], where R is a 3x3 rotation
///   matrix and t is the translation vector. The matrix  stored in pose is row
///   major ordering: \n
///   a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34 where the
///   matrix is: \n
///   | a11, a12, a13 , a14|\n
///   | a21, a22, a23, a24 |\n
///   | a31, a32, a33, a34 |\n
///   Pointer storage must be at least a 12-element floating point array.
///
/// @return
///   0 if successfully clustered, otherwise error code
///
/// @test
///   -# Test sensitivity to noise. When there is no noise in the data,
///                     there shouldn't be any update at all.
///   -# Test convergence criterion: error tolerance.
///   -# Test influence of noise corrupted 2D points.
///   -# Test influence of noise corrupted 3D points.
///   -# Test influence of noise corrupted 2D and 3D points.
///   -# Test performance when inputing a large quantity of points.
///   -# Test proper handling of outlier points.
///   -# Test stride in the cvdlCorrespondences structure using strides
///      other than the default values (3 for fromStride and 2 for toStride)
///   -# Test using a set of 50 generic poses and randomly generated 3D-2D
///      correspondences.
///
/// @ingroup 3D_reconstruction
//------------------------------------------------------------------------------

FASTCV_API int
fcvGeomPoseUpdatef32(
   const float* __restrict projected,
   const float* __restrict reprojErr,
   const float* __restrict invz,
   const float* __restrict reprojVariance,
   unsigned int                numpts,
   float*       __restrict pose );

//------------------------------------------------------------------------------
/// @brief
///   Update the pose based on the specified points correspondences
///   using Gauss-Newton optimization. This function and fcvGeomPoseEvaluateErrorf32
///   can be used iteratively to perform poseRefine GN.
///
/// @param projected
///   2D position after projection
///
/// @param reprojError
///   2D reprojection error in camera coordinates (not in pixels!)
///
/// @param invz
///   Inverse depth (z)
///
/// @param reprojVariances
///   Reprojection variance in camera coordinates
///
/// @param numpts
///    Number of points
///
/// @param pose
///   Pose representing updated pose
///   correspondences stored as a
///   3x4 transformation matrix in the form [R|t], where R is a 3x3 rotation
///   matrix and t is the translation vector. The matrix  stored in pose is row
///   major ordering: \n
///   a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34 where the
///   matrix is: \n
///   | a11, a12, a13 , a14|\n
///   | a21, a22, a23, a24 |\n
///   | a31, a32, a33, a34 |\n
///   Pointer storage must be at least a 12-element floating point array.
///
/// @return
///   0 if successfully clustered, otherwise error code
///
/// @test
///   -# Test sensitivity to noise. When there is no noise in the data,
///                     there shouldn't be any update at all.
///   -# Test convergence criterion: error tolerance.
///   -# Test influence of noise corrupted 2D points.
///   -# Test influence of noise corrupted 3D points.
///   -# Test influence of noise corrupted 2D and 3D points.
///   -# Test performance when inputing a large quantity of points.
///   -# Test proper handling of outlier points.
///   -# Test stride in the cvdlCorrespondences structure using strides
///      other than the default values (3 for fromStride and 2 for toStride)
///   -# Test using a set of 50 generic poses and randomly generated 3D-2D
///      correspondences.
///
/// @ingroup 3D_reconstruction
//------------------------------------------------------------------------------

FASTCV_API int
fcvGeomPoseOptimizeGNf32( const float* __restrict projected,
                          const float* __restrict reprojErr,
                          const float* __restrict invz,
                          const float* __restrict reprojVariance,
                          unsigned int            numpts,
                          float*       __restrict pose );


//------------------------------------------------------------------------------
/// @brief
///   Calculate the reprojection error based on the input pose.
///   This function and fcvGeomPoseOptimizef32 can be used iteratively
///   to perform poseRefine (GN or LM)..
///
/// @param corrs
///   Pointer to correspondences structure.
///
/// @param pose
///   Pose representing updated pose
///   correspondences stored as a
///   3x4 transformation matrix in the form [R|t], where R is a 3x3 rotation
///   matrix and t is the translation vector. The matrix  stored in pose is row
///   major ordering: \n
///   a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34 where the
///   matrix is: \n
///   | a11, a12, a13 , a14|\n
///   | a21, a22, a23, a24 |\n
///   | a31, a32, a33, a34 |\n
///   Pointer storage must be at least a 12-element floating point array.
///
/// @param projected
///   2D position after projection
///
/// @param reprojError
///   2D reprojection error in camera coordinates (not in pixels!)
///
/// @param invz
///   Inverse depth (z)
///
/// @param reprojVariances
///   Reprojection variance in camera coordinates
///
/// @return
///   Reprojection error.
///
/// @test
///   -# Test sensitivity to noise. When there is no noise in the data,
///                     there shouldn't be any update at all.
///   -# Test convergence criterion: error tolerance.
///   -# Test influence of noise corrupted 2D points.
///   -# Test influence of noise corrupted 3D points.
///   -# Test influence of noise corrupted 2D and 3D points.
///   -# Test performance when inputing a large quantity of points.
///   -# Test proper handling of outlier points.
///   -# Test stride in the cvdlCorrespondences structure using strides
///      other than the default values (3 for fromStride and 2 for toStride)
///   -# Test using a set of 50 generic poses and randomly generated 3D-2D
///      correspondences.
///
/// @ingroup 3D_reconstruction
//------------------------------------------------------------------------------

FASTCV_API float
fcvGeomPoseEvaluateErrorf32( const fcvCorrespondences* __restrict corrs,
                             const float*              __restrict pose,
                             float*                    __restrict projected,
                             float*                    __restrict reprojErr,
                             float*                    __restrict invz,
                             float*                    __restrict reprojVariance );


//------------------------------------------------------------------------------
/// @brief
///   Checks which members have a projection error that is smaller than the
///   given one.
///
/// @param corrs
///   Pointer to correspondences structure.
///
/// @param pose
///   Pose representing relationship between ptTo and ptFrom
///   correspondences stored as a
///   3x4 transformation matrix in the form [R|t], where R is a 3x3 rotation
///   matrix and t is the translation vector. The matrix  stored in pose is row
///   major ordering: \n
///   a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34 where the
///   matrix is: \n
///   | a11, a12, a13 , a14|\n
///   | a21, a22, a23, a24 |\n
///   | a31, a32, a33, a34 |\n
///   Pointer storage must be at least a 12-element floating point array.
///
/// @param maxSquErr
///   Maximum error value squared.
///
/// @param inliers
///   Output array for those indices that passed the test - the array MUST
///   be able to store numIndices items.
///
/// @param numInliers
///   Output number of corrs that passed the test.
///
/// @return
///   0 that error is less than maximum error, -1 greater or equal to maximum
///   error.
///
/// @test
///   -# Test threshold setting "maxSquErr". A translation only projection is
///                     used to transform points. Two cases were studied: when there is
///                     no error added; when a varying amount of errors were introduced.
///   -# Test stride in the cvdlCorrespondences structure using strides
///                     other than the default values (3 for fromStride and 2 for toStride)
///   -# Test evaluation under other transformations: rotation; shearing;
///                     generic projection matrix.
///
/// @ingroup 3D_reconstruction
//------------------------------------------------------------------------------

FASTCV_API int
fcvGeomPoseEvaluatef32( const fcvCorrespondences* __restrict corrs,
                        const float*                         pose,
                        float                                maxSquErr,
                        uint16_t* __restrict                 inliers,
                        uint32_t*                            numInliers );


//------------------------------------------------------------------------------
/// @brief
///   Estimates a 6DOF pose from three 3D-2D correspondences
///  \n\b NOTE: Given the coordinates of three 3D points (in world reference frame),
///             and their corresponding perspective projections in an image,
///             this algorithm determines the position and orientation of the camera in
///             the world reference frame. The function provides up to four solutions
///             that can be disambiguated using a fourth point.
///             When used in conjunction with RANSAC, this function can perform efficient outlier rejection.
///             Two degenerate cases should be avoided when using this function:
///             - Indeterminate configuration:
///                  When the three points are collinear in space, there will be a family of poses mapping the
///                  three points to the same image points.
///             - Unstable configuration:
///                  The camera center is located on a circular cylinder passing through the three points and
///                  the camera optical axis is perpendicular to the plane derived by the three points.
///                  With this configuration, a small change in the position of the three points will
///                  result in a large change of the estimated pose..
///
/// @param corrs
///  2D-3D correspondence points
///
/// @param pose
///  computed pose (numPoses * 12 data)
///
/// @param numPoses (max = 4)
///
/// @ingroup 3D_reconstruction
//------------------------------------------------------------------------------

FASTCV_API void
fcvGeom3PointPoseEstimatef32( const fcvCorrespondences* __restrict corrs,
                                                 float*            pose,
                                               int32_t*            numPoses );


//------------------------------------------------------------------------------
/// @brief
///   3x3 correlation with non-separable kernel.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterCorr3x3s8_v2(). In the 2.0.0 release, 
///   fcvFilterCorr3x3s8_v2 will be renamed to fcvFilterCorr3x3s8
///   and the signature of fcvFilterCorr3x3s8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   TBD.
///
/// @param kernel
///   2-D 3x3 kernel.
///   \n\b NOTE: Normalized to Q4.4
///
/// @param src
///   Input image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be an even number
///
/// @param srcHeight
///   Image height.
///   \n\b NOTE: must be an even number
///
/// @param dst
///   Output convolution.
///   \n\b NOTE: Must be same size as src
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# Test impulse response. A bilinearly changing 2D gradient pattern were
///                     correlated with all 9 impulse signals of a 3x3 kernel.
///   -# Test polarity: a Sobel filter and negate of it were used to correlate
///                     the same image pattern in 1.
///   -# Test overflow: the image pattern was set to max of unsigned char 8
///                     and the kernel was set to max/min of signed char 8.
///   -# Test frequency response. A fixed frequence sine wave kernel is used
///                     to correlate with 2D sine wave patterns with different x and y
///                     directional frequencies.
///   -# Random kernels were used to correlated with different real images.
///   -# A large image were used to test tolerance to moderately large
///                     image dimensions.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorr3x3s8( const int8_t* __restrict  kernel,
                    const uint8_t* __restrict src,
                    unsigned int              srcWidth,
                    unsigned int              srcHeight,
                    uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
/// @brief
///   3x3 correlation with non-separable kernel.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterCorr3x3s8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterCorr3x3s8,
///   \a fcvFilterCorr3x3s8_v2 will be removed, and the current signature
///   for \a fcvFilterCorr3x3s8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterCorr3x3s8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   TBD.
///
/// @param kernel
///   2-D 3x3 kernel.
///   \n\b NOTE: Normalized to Q4.4
///
/// @param src
///   Input image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be an even number
///
/// @param srcHeight
///   Image height.
///   \n\b NOTE: must be an even number
///
/// @param srcStride
///   Image stride.
///   \n\b WARNING: must be multiple of 8.
/// 
/// @param dst
///   Output convolution.
///   \n\b NOTE: Must be same size as src
///   \n\b WARNING: must be 128-bit aligned.
/// 
/// @param dstStride
///   Output stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @test
///   -# Test impulse response. A bilinearly changing 2D gradient pattern were
///                     correlated with all 9 impulse signals of a 3x3 kernel.
///   -# Test polarity: a Sobel filter and negate of it were used to correlate
///                     the same image pattern in 1.
///   -# Test overflow: the image pattern was set to max of unsigned char 8
///                     and the kernel was set to max/min of signed char 8.
///   -# Test frequency response. A fixed frequence sine wave kernel is used
///                     to correlate with 2D sine wave patterns with different x and y
///                     directional frequencies.
///   -# Random kernels were used to correlated with different real images.
///   -# A large image were used to test tolerance to moderately large
///                     image dimensions.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorr3x3s8_v2( const int8_t* __restrict  kernel,
                       const uint8_t* __restrict src,
                       unsigned int              srcWidth,
                       unsigned int              srcHeight,
                       unsigned int              srcStride,
                       uint8_t* __restrict       dst,
                       unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   9x9 correlation with separable kernel.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterCorrSep9x9s16_v2(). In the 2.0.0 release, 
///   fcvFilterCorrSep9x9s16_v2 will be renamed to fcvFilterCorrSep9x9s16
///   and the signature of fcvFilterCorrSep9x9s16 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   TBD.
///
/// @param kernel
///   1-D kernel in Q15.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param src
///   Input image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///   \n\b WARNING: must be > 8.
///
/// @param srcHeight
///   Image height.
///
/// @param tmpImg
///   Temporary image scratch space used internally.
///   \n\b WARNING: Must be same size as src
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dst
///   Output correlation.
///   \n\b WARNING: Must be same size as src
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# Test overflow. Values of the image and the kernel are set to have
///                     the four combinations of max/min values of int16 to test potential
///                     overflow of the intermediate accumulator.
///   -# Test response to impulse input. The kernel was set to all zero except
///                     for a specific dimension. This is used to test kernel anchor point.
///   -# Test polarity (negate of kernels).
///   -# Test frequency response. A fixed frequence sine wave kernel is used
///                     to correlate with 2D sine wave patterns with different x and y
///                     directional frequencies.
///   -# Random kernels were used to correlated with different real images.
///   -# A large image were used to test tolerance to moderately large
///                     image dimensions.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorrSep9x9s16( const int16_t* __restrict kernel,
                        const int16_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        int16_t* __restrict       tmp,
                        int16_t* __restrict       dst );


//---------------------------------------------------------------------------
/// @brief
///   9x9 FIR filter (convolution) with seperable kernel.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterCorrSep9x9s16() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterCorrSep9x9s16,
///   \a fcvFilterCorrSep9x9s16_v2 will be removed, and the current signature
///   for \a fcvFilterCorrSep9x9s16 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterCorrSep9x9s16 when transitioning to 2.0.0.
///   \n\n
///
/// @detailed
///   TBD.
///
/// @param kernel
///   1-D kernel.
///
/// @param srcImg
///   Input image.
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param srcWidth
///   Image tile width.
///
/// @param srcHeight
///   Image tile height.
///
/// @param srcStride
///   source Image width
///
/// @param tmpImg
///   Temporary image scratch space used internally.
///   \n\b NOTE: Size = width * ( height + knlSize - 1 )
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param dstImg
///   Output correlation
///   \n\b NOTE: Size = width * heigth
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param dstStride
///   dst Image width
/// 
/// @ingroup image_processing
//---------------------------------------------------------------------------
void
fcvFilterCorrSep9x9s16_v2( const int16_t* __restrict kernel,
                           const int16_t* __restrict srcImg,
                           unsigned int              srcWidth, 
                           unsigned int              srcHeight, 
                           unsigned int              srcStride,
                           int16_t* __restrict       tmpImg,
                           int16_t* __restrict       dstImg, 
                           unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   11x11 correlation with separable kernel.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterCorrSep11x11s16_v2(). In the 2.0.0 release, 
///   fcvFilterCorrSep11x11s16_v2 will be renamed to fcvFilterCorrSep11x11s16
///   and the signature of fcvFilterCorrSep11x11s16 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   TBD.
///
/// @param kernel
///   1-D kernel.
///   \n\b NOTE: array must be >=12 elements with kernel[11]=0
///   \n\b WARNING: must be 128-bit aligned.
///   \n\b NOTE: Normalized to Q1.15
///
/// @param src
///   Input image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///   \n\b WARNING: must be > 8.
///
/// @param srcHeight
///   Image height.
///
/// @param tmpImg
///   Temporary image scratch space used internally.
///   \n\b NOTE: Must be same size as src
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dst
///   Output correlation.
///   \n\b NOTE: Must be same size as src
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# Test overflow. Values of the image and the kernel are set to have
///                     the four combinations of max/min values of int16 to test potential
///                     overflow of the intermediate accumulator.
///   -# Test response to impulse input. The kernel was set to all zero except
///                     for a specific dimension. This is used to test kernel anchor point.
///   -# Test polarity (negate of kernels).
///   -# Test frequency response. A fixed frequence sine wave kernel is used
///                     to correlate with 2D sine wave patterns with different x and y
///                     directional frequencies.
///   -# Random kernels were used to correlated with different real images.
///   -# A large image were used to test tolerance to moderately large
///                     image dimensions.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorrSep11x11s16( const int16_t* __restrict kernel,
                          const int16_t* __restrict src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          int16_t* __restrict       tmpImg,
                          int16_t* __restrict       dst );


//---------------------------------------------------------------------------
/// @brief
///   11x11 FIR filter (convolution) with seperable kernel.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterCorrSep11x11s16() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterCorrSep11x11s16,
///   \a fcvFilterCorrSep11x11s16_v2 will be removed, and the current signature
///   for \a fcvFilterCorrSep11x11s16 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterCorrSep11x11s16 when transitioning to 2.0.0.
///   \n\n
///
/// @detailed
///   TBD.
///
/// @param kernel
///   1-D kernel.
///
/// @param srcImg
///   Input image.
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param srcWidth
///   Image tile width.
///
/// @param srcHeight
///   Image tile height.
///
/// @param srcStride
///   source Image width
///
/// @param tmpImg
///   Temporary image scratch space used internally.
///   \n\b NOTE: Size = width * ( height + knlSize - 1 )
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param dstImg
///   Output correlation
///   \n\b NOTE: Size = width * heigth
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param dstStride
///   dst Image width
/// 
/// @ingroup image_processing
//---------------------------------------------------------------------------
void
fcvFilterCorrSep11x11s16_v2( const int16_t* __restrict kernel,
                             const int16_t* __restrict srcImg,
                             unsigned int              srcWidth, 
                             unsigned int              srcHeight, 
                             unsigned int              srcStride,
                             int16_t* __restrict       tmpImg,
                             int16_t* __restrict       dstImg, 
                             unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   13x13 correlation with separable kernel.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterCorrSep13x13s16_v2(). In the 2.0.0 release, 
///   fcvFilterCorrSep13x13s16_v2 will be renamed to fcvFilterCorrSep13x13s16
///   and the signature of fcvFilterCorrSep13x13s16 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   TBD.
///
/// @param kernel
///   1-D kernel.
///   \n\b NOTE: Normalized to Q1.15
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param src
///   Input image.
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///   \n\b WARNING: must be > 8.
///
/// @param srcHeight
///   Image height.
///
/// @param tmpImg
///   Temporary image scratch space used internally.
///   \n\b NOTE: Must be same size as src
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dst
///   Output correlation.
///   \n\b NOTE: Must be same size as src
///
/// @test
///   -# Test overflow. Values of the image and the kernel are set to have
///                     the four combinations of max/min values of int16 to test potential
///                     overflow of the intermediate accumulator.
///   -# Test response to impulse input. The kernel was set to all zero except
///                     for a specific dimension. This is used to test kernel anchor point.
///   -# Test polarity (negate of kernels).
///   -# Test frequency response. A fixed frequence sine wave kernel is used
///                     to correlate with 2D sine wave patterns with different x and y
///                     directional frequencies.
///   -# Random kernels were used to correlated with different real images.
///   -# A large image were used to test tolerance to moderately large
///                     image dimensions.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorrSep13x13s16( const int16_t* __restrict kernel,
                          const int16_t* __restrict src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          int16_t* __restrict       tmpImg,
                          int16_t* __restrict       dst );


//---------------------------------------------------------------------------
/// @brief
///   13x13 FIR filter (convolution) with seperable kernel.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterCorrSep13x13s16() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterCorrSep13x13s16,
///   \a fcvFilterCorrSep13x13s16_v2 will be removed, and the current signature
///   for \a fcvFilterCorrSep13x13s16 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterCorrSep13x13s16 when transitioning to 2.0.0.
///   \n\n
///
/// @detailed
///   TBD.
///
/// @param kernel
///   1-D kernel.
///
/// @param srcImg
///   Input image.
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param srcWidth
///   Image tile width.
///
/// @param srcHeight
///   Image tile height.
///
/// @param srcStride
///   source Image width
///
/// @param tmpImg
///   Temporary image scratch space used internally.
///   \n\b NOTE: Size = width * ( height + knlSize - 1 )
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param dstImg
///   Output correlation
///   \n\b NOTE: Size = width * heigth
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param dstStride
///   dst Image width
/// 
/// @ingroup image_processing
//---------------------------------------------------------------------------
FASTCV_API void
fcvFilterCorrSep13x13s16_v2( const int16_t* __restrict kernel,
                             const int16_t* __restrict srcImg,
                             unsigned int              srcWidth, 
                             unsigned int              srcHeight, 
                             unsigned int              srcStride,
                             int16_t* __restrict       tmpImg,
                             int16_t* __restrict       dstImg, 
                             unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   15x15 correlation with separable kernel.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterCorrSep15x15s16_v2(). In the 2.0.0 release, 
///   fcvFilterCorrSep15x15s16_v2 will be renamed to fcvFilterCorrSep15x15s16
///   and the signature of fcvFilterCorrSep15x15s16 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   TBD.
///
/// @param kernel
///   1-D kernel.
///   \n\b NOTE: array must be 16 elements with kernel[15]=0
///   \n\b NOTE: Normalized to Q1.15
///
/// @param src
///   Input image.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param tmpImg
///   Temporary image scratch space used internally.
///   \n\b NOTE: Must be same size as src
///
/// @param dst
///   Output correlation.
///   \n\b NOTE: Must be same size as src
///
/// @test
///   -# Test overflow. Values of the image and the kernel are set to have
///                     the four combinations of max/min values of int16 to test potential
///                     overflow of the intermediate accumulator.
///   -# Test response to impulse input. The kernel was set to all zero except
///                     for a specific dimension. This is used to test kernel anchor point.
///   -# Test polarity (negate of kernels).
///   -# Test frequency response. A fixed frequence sine wave kernel is used
///                     to correlate with 2D sine wave patterns with different x and y
///                     directional frequencies.
///   -# Random kernels were used to correlated with different real images.
///   -# A large image were used to test tolerance to moderately large
///                     image dimensions.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorrSep15x15s16( const int16_t* __restrict kernel,
                          const int16_t* __restrict src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          int16_t* __restrict       tmpImg,
                          int16_t* __restrict       dst );


//---------------------------------------------------------------------------
/// @brief
///   15x15 FIR filter (convolution) with seperable kernel.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterCorrSep15x15s16() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterCorrSep15x15s16,
///   \a fcvFilterCorrSep15x15s16_v2 will be removed, and the current signature
///   for \a fcvFilterCorrSep15x15s16 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterCorrSep15x15s16 when transitioning to 2.0.0.
///   \n\n
///
/// @detailed
///   TBD.
///
/// @param kernel
///   1-D kernel.
///
/// @param srcImg
///   Input image.
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param srcWidth
///   Image tile width.
///
/// @param srcHeight
///   Image tile height.
///
/// @param srcStride
///   source Image width
///
/// @param tmpImg
///   Temporary image scratch space used internally.
///   \n\b NOTE: Size = width * ( height + knlSize - 1 )
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param dstImg
///   Output correlation
///   \n\b NOTE: Size = width * heigth
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param dstStride
///   dst Image width
/// 
/// @ingroup image_processing
//---------------------------------------------------------------------------
FASTCV_API void
fcvFilterCorrSep15x15s16_v2( const int16_t* __restrict kernel,
                             const int16_t* __restrict srcImg,
                             unsigned int              srcWidth, 
                             unsigned int              srcHeight, 
                             unsigned int              srcStride,
                             int16_t* __restrict       tmpImg,
                             int16_t* __restrict       dstImg, 
                             unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   17x17 correlation with separable kernel.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterCorrSep17x17s16_v2(). In the 2.0.0 release, 
///   fcvFilterCorrSep17x17s16_v2 will be renamed to fcvFilterCorrSep17x17s16
///   and the signature of fcvFilterCorrSep17x17s16 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   TBD.
///
/// @param kernel
///   1-D kernel.
///   \n\b NOTE: Normalized to Q1.15
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param src
///   Input image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///   \n\b WARNING: must be > 8.
///
/// @param srcHeight
///   Image height.
///
/// @param tmpImg
///   Temporary image scratch space used internally.
///   \n\b NOTE: Must be same size as src
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dst
///   Output correlation.
///   \n\b NOTE: Must be same size as src
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# Test overflow. Values of the image and the kernel are set to have
///                     the four combinations of max/min values of int16 to test potential
///                     overflow of the intermediate accumulator.
///   -# Test response to impulse input. The kernel was set to all zero except
///                     for a specific dimension. This is used to test kernel anchor point.
///   -# Test polarity (negate of kernels).
///   -# Test frequency response. A fixed frequence sine wave kernel is used
///                     to correlate with 2D sine wave patterns with different x and y
///                     directional frequencies.
///   -# Random kernels were used to correlated with different real images.
///   -# A large image were used to test tolerance to moderately large
///                     image dimensions.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterCorrSep17x17s16( const int16_t* __restrict kernel,
                          const int16_t* __restrict src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          int16_t* __restrict       tmpImg,
                          int16_t* __restrict       dst );



//---------------------------------------------------------------------------
/// @brief
///   17x17 FIR filter (convolution) with seperable kernel.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterCorrSep17x17s16() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterCorrSep17x17s16,
///   \a fcvFilterCorrSep17x17s16_v2 will be removed, and the current signature
///   for \a fcvFilterCorrSep17x17s16 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterCorrSep17x17s16 when transitioning to 2.0.0.
///   \n\n
///
/// @detailed
///   TBD.
///
/// @param kernel
///   1-D kernel.
///
/// @param srcImg
///   Input image.
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param srcWidth
///   Image tile width.
///
/// @param srcHeight
///   Image tile height.
///
/// @param srcStride
///   source Image width
///
/// @param tmpImg
///   Temporary image scratch space used internally.
///   \n\b NOTE: Size = width * ( height + knlSize - 1 )
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param dstImg
///   Output correlation
///   \n\b NOTE: Size = width * heigth
///   \n\b NOTE: data should be 128-bit aligned
///
/// @param dstStride
///   dst Image width
/// 
/// @ingroup image_processing
//---------------------------------------------------------------------------
FASTCV_API void
fcvFilterCorrSep17x17s16_v2( const int16_t* __restrict kernel,
                             const int16_t* __restrict srcImg,
                             unsigned int              srcWidth, 
                             unsigned int              srcHeight, 
                             unsigned int              srcStride,
                             int16_t* __restrict       tmpImg,
                             int16_t* __restrict       dstImg, 
                             unsigned int              dstStride );

//------------------------------------------------------------------------------
/// @brief
///   Calculates the mean and variance of intensities of a rectangle in a
///   grayscale image.
///
/// @details
///
/// @param src
///   pointer to 8-bit grayscale image
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Width of source image
///   \n\b WARNING: must be multiple of 8.
///
/// @param xBegin
///   x coordinate of of top left of rectangle
///
/// @param yBegin
///   y coordinate of of top left of rectangle
///
/// @param recWidth
///   width of rectangular region
///
/// @param recHeight
///   height of rectangular region
///
/// @param mean
///   output of mean of region
///
/// @param variance
///   output of variance of region
///
/// @test
///   -# Window sizes: from 1x1 window to 512x512 window.
///   -# Potential overflow: image window size 512x512, constant intensity
///                     of 255. Sum of intensities: 255x512x512 ~= 2^26
///   -# Extreme variance values: image window sizes of 8x8, 32x32,
///                     128x128, 512x512, half 255 and half 0.
///   -# Sample window locations and sizes.
///   -# 2D sine wave patterns with different x-y directional periods for
///                     spatially varying signals.
///   -# Real images with varying image sizes.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageIntensityStats( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        int                       xBegin,
                        int                       yBegin,
                        unsigned int              recWidth,
                        unsigned int              recHeight,
                        float*                    mean,
                        float*                    variance );

//------------------------------------------------------------------------------
/// @brief
///   Creates a histogram of intensities for a rectangular region of a grayscale
///   image. Bins each pixel into a histogram of size 256, depending on the
///   intensity of the pixel (in the range 0 to 255).
///
/// @details
///
/// @param src
///   pointer to 8-bit grayscale image
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Width of source image
///   \n\b WARNING: must be multiple of 8.
///
/// @param xBegin
///   x coordinate of of top left of rectangle
///
/// @param yBegin
///   y coordinate of of top left of rectangle
///
/// @param recWidth
///   Width of rectangular region
///
/// @param recHeight
///   Height of rectangular region
///
/// @param histogram
///   Array of size 256 for storing the histogram
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# Nontrivial histogram bins only at bin 0 or bin 255.
///   -# Empty window: an implementation should properly handle window size 0x0
///   -# Uniform distribution: each bin should have exactly 1 count.
///   -# Sample window locations and sizes.
///   -# 2D sine wave patterns with different x-y directional periods for
///                     spatially varying signals.
///   -# Real images with varying image sizes.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageIntensityHistogram( const uint8_t* __restrict src,
                            unsigned int              srcWidth,
                            int                       xBegin,
                            int                       yBegin,
                            unsigned int              recWidth,
                            unsigned int              recHeight,
                            int32_t*                  histogram  );


//------------------------------------------------------------------------------
/// @brief
///   Builds an integral image of the incoming 8-bit image and adds an
///   unfilled border on top and to the left.
///   \n NOTE: border usually zero filled elsewhere.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvIntegratePatchu8_v2(). In the 2.0.0 release, 
///   fcvIntegratePatchu8_v2 will be renamed to fcvIntegratePatchu8
///   and the signature of fcvIntegratePatchu8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
///   \n\b NOTE: height must be <= 2048
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param dst
///   Output integral-image. Should point to a memory of size (width+1)*(height+1).
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# Small constant images (8x4, all 0 or 255) for sanity check.
///   -# 2D sine-wave patterns for testing spatial varying input image.
///   -# Test image strides.
///   -# Test extreme window sizes.
///   -# Test using a set of 50 real image patches.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegrateImageu8( const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     uint32_t* __restrict      dst );


//------------------------------------------------------------------------------
/// @brief
///   Builds an integral image of the incoming 8-bit image and adds an
///   unfilled border on top and to the left.
///   \n NOTE: border usually zero filled elsewhere.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvIntegrateImageu8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvIntegrateImageu8,
///   \a fcvIntegrateImageu8_v2 will be removed, and the current signature
///   for \a fcvIntegrateImageu8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvIntegrateImageu8 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
///   \n\b NOTE: height must be <= 2048
///
/// @param srcStride
///   Stride (in bytes) of the image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Output integral-image. Should point to a memory of size at least (width+1)*(height+1).
///   \n\b WARNING: must be 128-bit aligned.
/// 
/// @param dstStride
///   Stride (in bytes) of integral image.
///   \n\b WARNING: must be multiple of 32 (8 * 4-byte values).
///
/// @test
///   -# Small constant images (8x4, all 0 or 255) for sanity check.
///   -# 2D sine-wave patterns for testing spatial varying input image.
///   -# Test image strides.
///   -# Test extreme window sizes.
///   -# Test using a set of 50 real image patches.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegrateImageu8_v2( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        uint32_t* __restrict      dst,
                        unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Builds an integral image of the incoming 8-bit patch values and their
///   squares and adds an unfilled border on top and to the left.
///   \n NOTE: border usually zero filled elsewhere.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvIntegratePatchu8_v2(). In the 2.0.0 release, 
///   fcvIntegratePatchu8_v2 will be renamed to fcvIntegratePatchu8
///   and the signature of fcvIntegratePatchu8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
///   \n\b WARNING: height must be <= 2048
///
/// @param patchX
///   Patch location on image of upper-left patch corner.
///
/// @param patchY
///   Patch location on image of upper-left patch corner.
///
/// @param patchW
///   Image width.
///
/// @param patchH
///   Image height.
///
/// @param dst
///   Integral image.
///   \n\b NOTE: Memory must be > (patchW+1)(patchH+1)
///
/// @param dstSquared
///   Integral image of squared values.
///   \n\b NOTE: Memory must be > (patchW+1)(patchH+1)
///
/// @test
///   -# Small constant images (8x4, all 0 or 255) for sanity check.
///   -# Check response to an empty window (width = height = 0).
///   -# 2D sine-wave patterns for testing spatial varying input image.
///   -# Test integration window locations.
///   -# Test integration window sizes.
///   -# Test integration window not contained in the image.
///   -# Test window straddling the image.
///   -# Test integral image computing using 50 real image patches and
///                     randomly generated integration windows.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegratePatchu8( const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     int                       patchX,
                     int                       patchY,
                     unsigned int              patchW,
                     unsigned int              patchH,
                     uint32_t* __restrict      intgrlImgOut,
                     uint32_t* __restrict      intgrlSqrdImgOut );


//------------------------------------------------------------------------------
/// @brief
///   Builds an integral image of the incoming 8-bit patch values and their
///   squares and adds an unfilled border on top and to the left.
///   \n NOTE: border usually zero filled elsewhere.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvIntegratePatchu8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvIntegratePatchu8,
///   \a fcvIntegratePatchu8_v2 will be removed, and the current signature
///   for \a fcvIntegratePatchu8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvIntegratePatchu8 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
///   \n\b WARNING: height must be <= 2048
/// 
/// @param srcStride
///   Image stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @param patchX
///   Patch location on image of upper-left patch corner.
///
/// @param patchY
///   Patch location on image of upper-left patch corner.
///
/// @param patchW
///   Patch width.
///   \n\b WARNING: (patchW * patchH) should be less than 66051, to avoid overflow.
///
/// @param patchH
///   Patch height.
///   \n\b WARNING: (patchW * patchH) should be less than 66051, to avoid overflow.
///
/// @param dst
///   Integral image.
///   \n\b NOTE: Memory must be > (patchW+1)(patchH+1)
///
/// @param dstSquared
///   Integral image of squared values.
///   \n\b NOTE: Memory must be > (patchW+1)(patchH+1)
///
/// @test
///   -# Small constant images (8x4, all 0 or 255) for sanity check.
///   -# Check response to an empty window (width = height = 0).
///   -# 2D sine-wave patterns for testing spatial varying input image.
///   -# Test integration window locations.
///   -# Test integration window sizes.
///   -# Test integration window not contained in the image.
///   -# Test window straddling the image.
///   -# Test integral image computing using 50 real image patches and
///                     randomly generated integration windows.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegratePatchu8_v2( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        int                       patchX,
                        int                       patchY,
                        unsigned int              patchW,
                        unsigned int              patchH,
                        uint32_t* __restrict      intgrlImgOut,
                        uint32_t* __restrict      intgrlSqrdImgOut );


//---------------------------------------------------------------------------
/// @brief
///   Builds an integral image of the incoming 12x12 8-bit patch values and
///   their squares.  It also adds an unfilled border on top and to the left.
///   \n NOTE: border usually zero filled elsewhere.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvIntegratePatch12x12u8_v2(). In the 2.0.0 release, 
///   fcvIntegratePatch12x12u8_v2 will be renamed to fcvIntegratePatch12x12u8
///   and the signature of fcvIntegratePatch12x12u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   \n\b WARNING: do not use - under construction.
///
/// @param src
///   Input image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param imageWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param imageHeight
///   Image height.
///
/// @param patchX
///   Patch location on image of upper-left patch corner.
///
/// @param patchY
///   Patch location on image of upper-left patch corner.
///
/// @param intgrlImgOut
///   Integral image.
///   \n\b NOTE: Memory must be > (12+1)(12+1)
///
/// @param intgrlSqrdImgOut
///   Integral image of squared values.
///   \n\b NOTE: Memory must be > (12+1)(12+1)
///
/// @ingroup image_processing
//---------------------------------------------------------------------------

FASTCV_API void
fcvIntegratePatch12x12u8( const uint8_t* __restrict src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          int                       patchX,
                          int                       patchY,
                          uint32_t* __restrict      intgrlImgOut,
                          uint32_t* __restrict      intgrlSqrdImgOut );


//---------------------------------------------------------------------------
/// @brief
///   Builds an integral image of the incoming 12x12 8-bit patch values and
///   their squares.  It also adds an unfilled border on top and to the left.
///   \n NOTE: border usually zero filled elsewhere.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvIntegratePatch12x12u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvIntegratePatch12x12u8,
///   \a fcvIntegratePatch12x12u8_v2 will be removed, and the current signature
///   for \a fcvIntegratePatch12x12u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvIntegratePatch12x12u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   \n\b WARNING: do not use - under construction.
///
/// @param src
///   Input image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @param patchX
///   Patch location on image of upper-left patch corner.
///
/// @param patchY
///   Patch location on image of upper-left patch corner.
///
/// @param intgrlImgOut
///   Integral image.
///   \n\b NOTE: Memory must be > (12+1)(12+1)
///
/// @param intgrlSqrdImgOut
///   Integral image of squared values.
///   \n\b NOTE: Memory must be > (12+1)(12+1)
///
/// @ingroup image_processing
//---------------------------------------------------------------------------

FASTCV_API void
fcvIntegratePatch12x12u8_v2( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             unsigned int              srcStride,
                             int                       patchX,
                             int                       patchY,
                             uint32_t* __restrict      intgrlImgOut,
                             uint32_t* __restrict      intgrlSqrdImgOut );


//------------------------------------------------------------------------------
/// @brief
///   Builds an integral image of the incoming 18x18 8-bit patch values and
///   their squares.  It also adds an unfilled border on top and to the left.
///   \n NOTE: border usually zero filled elsewhere.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvIntegratePatch18x18u8_v2(). In the 2.0.0 release, 
///   fcvIntegratePatch18x18u8_v2 will be renamed to fcvIntegratePatch18x18u8
///   and the signature of fcvIntegratePatch18x18u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image.
///
/// @param srcWidth
///   Image srcWidth.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
///
/// @param patchX
///   Patch location on image of upper-left patch corner.
///
/// @param patchY
///   Patch location on image of upper-left patch corner.
///
/// @param dst
///   Integral image.
///   \n\b NOTE: Memory must be > (18+1)(18+1)
///
/// @param dstSquared
///   Integral image of squared values.
///   \n\b NOTE: Memory must be > (18+1)(18+1)
///
/// @test
///   -# Small constant images (24x18, all 0 or 255) for sanity check.
///   -# 2D sine-wave patterns for testing spatial varying input image.
///   -# Test integration window locations.
///   -# Test integration window not contained in the image.
///   -# Test window straddling the image.
///   -# Test integral image computing using 50 real image patches.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegratePatch18x18u8( const uint8_t* __restrict src,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          int                       patchX,
                          int                       patchY,
                          uint32_t* __restrict      intgrlImgOut,
                          uint32_t* __restrict      intgrlSqrdImgOut );


//------------------------------------------------------------------------------
/// @brief
///   Builds an integral image of the incoming 18x18 8-bit patch values and
///   their squares.  It also adds an unfilled border on top and to the left.
///   \n NOTE: border usually zero filled elsewhere.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvIntegratePatch18x18u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvIntegratePatch18x18u8,
///   \a fcvIntegratePatch18x18u8_v2 will be removed, and the current signature
///   for \a fcvIntegratePatch18x18u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvIntegratePatch18x18u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   \n\b WARNING: do not use - under construction.
///
/// @param src
///   Input image.
///
/// @param srcWidth
///   Image srcWidth.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @param patchX
///   Patch location on image of upper-left patch corner.
///
/// @param patchY
///   Patch location on image of upper-left patch corner.
///
/// @param dst
///   Integral image.
///   \n\b NOTE: Memory must be > (18+1)(18+1)
///
/// @param dstSquared
///   Integral image of squared values.
///   \n\b NOTE: Memory must be > (18+1)(18+1)
///
/// @test
///   -# Small constant images (24x18, all 0 or 255) for sanity check.
///   -# 2D sine-wave patterns for testing spatial varying input image.
///   -# Test integration window locations.
///   -# Test integration window not contained in the image.
///   -# Test window straddling the image.
///   -# Test integral image computing using 50 real image patches.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegratePatch18x18u8_v2( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             unsigned int              srcStride,
                             int                       patchX,
                             int                       patchY,
                             uint32_t* __restrict      intgrlImgOut,
                             uint32_t* __restrict      intgrlSqrdImgOut );


//---------------------------------------------------------------------------
/// @brief
///   Integrates one line of an image or any portion of an image that is
///   contiguous in memory.
///
/// @param src
///   Input image.
///
/// @param srcWidth
///   Number of pixels.
///   \n NOTE: bit width enforces numPxls < 2^16
///
/// @param intgrl
///   Sum of values from specified pixels.
///
/// @param intgrlSqrd
///   Sum of values from specified pixels.
///
/// @ingroup image_processing
//---------------------------------------------------------------------------

FASTCV_API void
fcvIntegrateImageLineu8( const uint8_t* __restrict src,
                         uint16_t                  srcWidth,
                         uint32_t*                 intgrl,
                         uint32_t*                 intgrlSqrd );


//------------------------------------------------------------------------------
/// @brief
///   Integrates 64 contiguous pixels of an image.
///
/// @param src
///   Input image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param numPxls
///   Number of pixels.
///   \n NOTE: bit width enforces numPxls < 2^16
///
/// @param intgrl
///   Sum of values from specified pixels.
///
/// @param intgrlSqrd
///   Sum of values from specified pixels.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvIntegrateImageLine64u8( const uint8_t* __restrict src,
                           uint16_t*                 intgrl,
                           uint32_t*                 intgrlSqrd );


//------------------------------------------------------------------------------
/// @brief
///   compute approximate mean and variance for the range of NFT4 float
///   descriptors where descriptor elements along dimension are treated
///   as random vars
///
/// @param src
///   contiguous block of descriptors of dimension 36
///
/// @param first
///   index of the first descriptor in range for computing mean and var
///
/// @param last
///   index of the last descriptor in range for computing mean and range
///
/// @param vind
///   array of randomized indexes of descriptors
///
/// @param means
///   buffer for approximate means, must be 36 long
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param vars
///   buffer for approximate variances, must be 36 long
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param temp
///   bufffer, must be 46 long
///   \n\b WARNING: must be 128-bit aligned.
///
/// @return
///   0        - success
///   EFAULT   - invalid address
///   EINVAL   - invalid argument
///
/// @remark
///   If descriptor range is > 100 then only
///   100 samples are drawn from the range to compute
///   approximate means and variances.
///
///   Variances computed here do not have to be true variances because their
///   values do not matter in kdtrees. The only thing that matters is that
///   the ordering relation of variances is preserved
///
/// @test
///   -# Simple 1 vector mean and variance: zero vector and ones(1,36)
///                     vector. A proper implementation should take care of denominator
///                     for variance and 0 vector length.
///   -# Test vector with extreme variance.
///   -# Test random subset using vind.
///   -# Test the number of samples. When there are more than 100 samples,
///                     the rest is ignored.
///   -# Vectors with varying length and randomized indices.
///
/// @ingroup object_detection
// -----------------------------------------------------------------------------

FASTCV_API int
fcvDescriptorSampledMeanAndVar36f32( const float* __restrict src,
                                     int                     first,
                                     int                     last,
                                     int32_t*                vind,
                                     float* __restrict       means,
                                     float* __restrict       vars,
                                     float* __restrict       temp );


//------------------------------------------------------------------------------
/// @brief
///   Searches a 8x8 patch within radius around a center pixel for the max NCC.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvNCCPatchOnCircle8x8u8_v2(). In the 2.0.0 release, 
///   fcvNCCPatchOnCircle8x8u8_v2 will be renamed to fcvNCCPatchOnCircle8x8u8
///   and the signature of fcvNCCPatchOnCircle8x8u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param patch
///   Pointer to 8-bit patch pixel values linearly laid out in memory.
///
/// @param src
///   Pointer to 8-bit image pixel values linearly laid out in memory.
///
/// @param srcWidth
///   Width in pixels of the image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcHeight
///   Height in pixels of the image.
///   \n\b WARNING: must be multiple of 8.
///
/// @param search_center_x
///   X location of search center in pixels of the image.
///
/// @param search_center_y
///   Y location of search center in pixels of the image.
///
/// @param search_radius
///   Radius of search in pixels.
///
/// @param best_x
///   Center X location on the image of the best NCC match.  The center X has
///   4 pixels to the left and 3 to the right.
///
/// @param best_y
///   Center Y location on the image of the best NCC match.  The center Y has
///   4 pixels above and 3 pixels below.
///
/// @param bestNCC
///   Largest value of the normalized cross-correlation found in the NCC search.
///
/// @param findSubPixel
///   Use parabolic interpolation of NCC values to find sub-pixel estimates.
///
/// @param subX
///   Sub-pixel estimate for optimal NCC relative to best_x.
///   \n e.g., float x = (float)best_x + subX;
///
/// @param subY
///   Sub-pixel estimate for optimal NCC relative to best_y.
///
/// @return
///   0 = OK \n
///   1 = "search_radius" too large\n
///   2 = invalid "search_center_x,y"\n
///   3 = not found\n
///
/// @ingroup object_detection
//------------------------------------------------------------------------------

FASTCV_API int
fcvNCCPatchOnCircle8x8u8( const uint8_t* __restrict patch,
                          const uint8_t* __restrict src,
                          unsigned short            srcWidth,
                          unsigned short            srcHeight,
                          unsigned short            search_center_x,
                          unsigned short            search_center_y,
                          unsigned short            search_radius,
                          uint16_t*                 best_x,
                          uint16_t*                 best_y,
                          uint32_t*                 bestNCC,
                          int                       findSubPixel,
                          float*                    subX,
                          float*                    subY );


//------------------------------------------------------------------------------
/// @brief
///   Searches a 8x8 patch within radius around a center pixel for the max NCC.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvNCCPatchOnCircle8x8u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvNCCPatchOnCircle8x8u8,
///   \a fcvNCCPatchOnCircle8x8u8_v2 will be removed, and the current signature
///   for \a fcvNCCPatchOnCircle8x8u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvNCCPatchOnCircle8x8u8 when transitioning to 2.0.0.
///   \n\n
///
/// @param patch
///   Pointer to 8-bit patch pixel values linearly laid out in memory.
///
/// @param src
///   Pointer to 8-bit image pixel values linearly laid out in memory.
///
/// @param srcWidth
///   Width in pixels of the image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcHeight
///   Height in pixels of the image.
///   \n\b WARNING: must be multiple of 8.
///
/// @param search_center_x
///   X location of search center in pixels of the image.
///
/// @param search_center_y
///   Y location of search center in pixels of the image.
///
/// @param search_radius
///   Radius of search in pixels.
///
/// @param filterLowVariance
///   Minimum variance.
///
/// @param best_x
///   Center X location on the image of the best NCC match.  The center X has
///   4 pixels to the left and 3 to the right.
///
/// @param best_y
///   Center Y location on the image of the best NCC match.  The center Y has
///   4 pixels above and 3 pixels below.
///
/// @param bestNCC
///   Largest value of the normalized cross-correlation found in the NCC search.
///
/// @param findSubPixel
///   Use parabolic interpolation of NCC values to find sub-pixel estimates.
///
/// @param subX
///   Sub-pixel estimate for optimal NCC relative to best_x.
///   \n e.g., float x = (float)best_x + subX;
///
/// @param subY
///   Sub-pixel estimate for optimal NCC relative to best_y.
///
/// @return
///   0 = OK \n
///   1 = "search_radius" too large\n
///   2 = invalid "search_center_x,y"\n
///   3 = not found\n
///   4 = Patch has too low variance\n
///   5 = Image region has too low variance\n
///
/// @ingroup object_detection
//------------------------------------------------------------------------------

FASTCV_API int
fcvNCCPatchOnCircle8x8u8_v2( const uint8_t* __restrict patch,
                             const uint8_t* __restrict src,
                             unsigned short            srcWidth,
                             unsigned short            srcHeight,
                             unsigned short            search_center_x,
                             unsigned short            search_center_y,
                             unsigned short            search_radius,
                             int                       filterLowVariance,
                             uint16_t*                 best_x,
                             uint16_t*                 best_y,
                             uint32_t*                 bestNCC,
                             int                       findSubPixel,
                             float*                    subX,
                             float*                    subY );




//------------------------------------------------------------------------------
/// @brief
///   Searches a 8x8 patch within square region around a center pixel
///   for the max NCC.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvNCCPatchOnSquare8x8u8_v2(). In the 2.0.0 release, 
///   fcvNCCPatchOnSquare8x8u8_v2 will be renamed to fcvNCCPatchOnSquare8x8u8
///   and the signature of fcvNCCPatchOnSquare8x8u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param patch
///   Pointer to 8-bit patch pixel values linearly laid out in memory.
///
/// @param src
///   Pointer to 8-bit image pixel values linearly laid out in memory.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Width in pixels of the image.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Height in pixels of the image.
///
/// @param search_center_x
///   Center X coordinate of the search window
///
/// @param search_center_y
///   Center Y coordinate of the search window
///
/// @param search_w
///   Width of search square in pixels
///   \n\b WARNING: must be 11 or less.
///
/// @param best_x
///   Center X location on the image of the best NCC match.  The center X has
///   4 pixels to the left and 3 to the right.
///
/// @param best_y
///   Center Y location on the image of the best NCC match.  The center Y has
///   4 pixels above and 3 pixels below.
///
/// @param findSubPixel
///   Use parabolic interpolation of NCC values to find sub-pixel estimates.
///
/// @param subX
///   Sub-pixel estimate for optimal NCC relative to best_x.
///   \n e.g., float x = (float)best_x + subX;
///
/// @param subY
///   Sub-pixel estimate for optimal NCC relative to best_y.
///
/// @return
///   0 = OK \n
///   1 = "search_radius" too large\n
///   2 = invalid "search_center_x,y"\n
///   3 = not found\n
///
/// @ingroup object_detection
//------------------------------------------------------------------------------

FASTCV_API int
fcvNCCPatchOnSquare8x8u8( const uint8_t* __restrict patch,
                          const uint8_t* __restrict src,
                          unsigned short            srcWidth,
                          unsigned short            srcHeight,
                          unsigned short            search_center_x,
                          unsigned short            search_center_y,
                          unsigned short            search_w,
                          uint16_t*                 best_x,
                          uint16_t*                 best_y,
                          uint32_t*                 bestNCC,
                          int                       doSubPixel,
                          float*                    subX,
                          float*                    subY );


//------------------------------------------------------------------------------
/// @brief
///   Searches a 8x8 patch within square region around a center pixel
///   for the max NCC.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvNCCPatchOnSquare8x8u8 with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvNCCPatchOnSquare8x8u8,
///   \a fcvNCCPatchOnSquare8x8u8_v2 will be removed, and the current signature
///   for \a fcvNCCPatchOnSquare8x8u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvNCCPatchOnSquare8x8u8 when transitioning to 2.0.0.
///   \n\n
/// 
/// @param patch
///   Pointer to 8-bit patch pixel values linearly laid out in memory.
///
/// @param src
///   Pointer to 8-bit image pixel values linearly laid out in memory.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Width in pixels of the image.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Height in pixels of the image.
///
/// @param search_center_x
///   Center X coordinate of the search window
///
/// @param search_center_y
///   Center Y coordinate of the search window
///
/// @param search_w
///   Width of search square in pixels
///   \n\b WARNING: must be 11 or less.
///
/// @param filterLowVariance
///   Minimum variance.
///
/// @param best_x
///   Center X location on the image of the best NCC match.  The center X has
///   4 pixels to the left and 3 to the right.
///
/// @param best_y
///   Center Y location on the image of the best NCC match.  The center Y has
///   4 pixels above and 3 pixels below.
///
/// @param findSubPixel
///   Use parabolic interpolation of NCC values to find sub-pixel estimates.
///
/// @param subX
///   Sub-pixel estimate for optimal NCC relative to best_x.
///   \n e.g., float x = (float)best_x + subX;
///
/// @param subY
///   Sub-pixel estimate for optimal NCC relative to best_y.
///
/// @return
///   0 = OK \n
///   1 = "search_radius" too large\n
///   2 = invalid "search_center_x,y"\n
///   3 = not found\n
///   4 = Patch has too low variance\n
///   5 = Image region has too low variance\n
///
/// @ingroup object_detection
//------------------------------------------------------------------------------

FASTCV_API int
fcvNCCPatchOnSquare8x8u8_v2( const uint8_t* __restrict patch,
                             const uint8_t* __restrict src,
                             unsigned short            srcWidth,
                             unsigned short            srcHeight,
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



//------------------------------------------------------------------------------
/// @brief
///   Sum of absolute differences of an image against an 8x8 template.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvSumOfAbsoluteDiffs8x8u8_v2(). In the 2.0.0 release, 
///   fcvSumOfAbsoluteDiffs8x8u8_v2 will be renamed to fcvSumOfAbsoluteDiffs8x8u8
///   and the signature of fcvSumOfAbsoluteDiffs8x8u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   8x8 sum of ||A-B||. The template patch is swept over the entire image and
///   the results are put in dst.
///
/// @param patch
///   8x8 template
///
/// @param src
///   Reference Image.
///
/// @param srcWidth
///    Width of the src image.
///
/// @param srcHeight
///    Height of the src image.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param dst
///   The dst buffer shall be width X height bytes in length.
///   Output of SAD(A,B). dst[0] correspondes to the 0,0 pixel of the template
///   aligned with the 0,0 pixel of src. The dst border values not covered by
///   entire 8x8 patch window will remain unmodified by the function. The caller
///   should either initialize these to 0 or ignore.
///
/// @test
///   -# Test ability to handle max output values (255x64).
///   -# Test capability of localizing a random dot pattern in a scrambled
///                     image.
///   -# Real images with varying image sizes.
///
/// @ingroup object_detection
//------------------------------------------------------------------------------

FASTCV_API void
fcvSumOfAbsoluteDiffs8x8u8( const uint8_t* __restrict patch,
                            const uint8_t* __restrict src,
                            unsigned int              srcWidth,
                            unsigned int              srcHeight,
                            unsigned int              srcStride,
                            uint16_t* __restrict      dst );


//------------------------------------------------------------------------------
/// @brief
///   Sum of absolute differences of an image against an 8x8 template.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvSumOfAbsoluteDiffs8x8u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvSumOfAbsoluteDiffs8x8u8,
///   \a fcvSumOfAbsoluteDiffs8x8u8_v2 will be removed, and the current signature
///   for \a fcvSumOfAbsoluteDiffs8x8u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvSumOfAbsoluteDiffs8x8u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   8x8 sum of ||A-B||. The template patch is swept over the entire image and
///   the results are put in dst.
///
/// @param patch
///   8x8 template
/// 
/// @param dstStride
///   Stride of the patch (in bytes) - i.e., how many bytes between column 0 of row N 
///   and column 0 of row N+1.
///
/// @param src
///   Reference Image.
///
/// @param srcWidth
///    Width of the src image.
///
/// @param srcHeight
///    Height of the src image.
///
/// @param srcStride
///   Stride of image (in bytes) - i.e., how many bytes between column 0 of row N 
///   and column 0 of row N+1.
///
/// @param dst
///   The dst buffer shall be at least ( width x height ) values in length.
///   Output of SAD(A,B). dst[0] correspondes to the 0,0 pixel of the template
///   aligned with the 0,0 pixel of src. The dst border values not covered by
///   entire 8x8 patch window will remain unmodified by the function. The caller
///   should either initialize these to 0 or ignore.
/// 
/// @param dstStride
///   Stride of destination (in bytes) - i.e., how many bytes between column 0 of row N 
///   and column 0 of row N+1.
///
/// @test
///   -# Test ability to handle max output values (255x64).
///   -# Test capability of localizing a random dot pattern in a scrambled
///                     image.
///   -# Real images with varying image sizes.
///
/// @ingroup object_detection
//------------------------------------------------------------------------------

FASTCV_API void
fcvSumOfAbsoluteDiffs8x8u8_v2( const uint8_t* __restrict patch,
                               unsigned int              patchStride,
                               const uint8_t* __restrict src,
                               unsigned int              srcWidth,
                               unsigned int              srcHeight,
                               unsigned int              srcStride,
                               uint16_t* __restrict      dst,
                               unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Down-scale the image to half width and height by averaging 2x2 pixels
///   into one.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvScaleDownBy2u8_v2(). In the 2.0.0 release, 
///   fcvScaleDownBy2u8_v2 will be renamed to fcvScaleDownBy2u8
///   and the signature of fcvScaleDownBy2u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   A box filter downsampling the next pixel, the pixel below, and the next
///   pixel to the pixel below into one pixel.\n
///   | px00 px01 px02 px03 |\n
///   | px10 px11 px12 px13 |\n
///   to:\n
///   | (px00+px01+px10+px11)/4 (px02+px03+px12+px13)/4 |\n
///
/// @param src
///   Input 8-bit image.
///   \n\b NOTE: data must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
///   \n\b NOTE:must be a multiple of 2
///
/// @param dst
///   Output 8-bit image.
///   \n\b NOTE: data must be 128-bit aligned.
///
/// @test
///   -# Constant intensity image inputs: the output images should contain
///                     the same intensities.
///   -# Very small image sizes (8x2) as input.
///   -# 2D sine wave images (various x/y frequencies) as spatially
///                     varying inputs and to test response at different frequencies.
///   -# Real images with varying image sizes.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API int
fcvScaleDownBy2u8( const uint8_t* __restrict src,
                   unsigned int              srcWidth,
                   unsigned int              srcHeight,
                   uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
/// @brief
///   Down-scale the image to half width and height by averaging 2x2 pixels
///   into one.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvScaleDownBy2u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvScaleDownBy2u8,
///   \a fcvScaleDownBy2u8_v2 will be removed, and the current signature
///   for \a fcvScaleDownBy2u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvScaleDownBy2u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   A box filter downsampling the next pixel, the pixel below, and the next
///   pixel to the pixel below into one pixel.\n
///   | px00 px01 px02 px03 |\n
///   | px10 px11 px12 px13 |\n
///   to:\n
///   | (px00+px01+px10+px11)/4 (px02+px03+px12+px13)/4 |\n
///
/// @param src
///   Input 8-bit image.
///   \n\b NOTE: data must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
///   \n\b NOTE:must be a multiple of 2
/// 
/// @param srcStride
///   Image stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @param dst
///   Output 8-bit image.
///   \n\b NOTE: data must be 128-bit aligned.
/// 
/// @param dstStride
///   Output stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @test
///   -# Constant intensity image inputs: the output images should contain
///                     the same intensities.
///   -# Very small image sizes (8x2) as input.
///   -# 2D sine wave images (various x/y frequencies) as spatially
///                     varying inputs and to test response at different frequencies.
///   -# Real images with varying image sizes.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API int
fcvScaleDownBy2u8_v2( const uint8_t* __restrict src,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      unsigned int              srcStride,
                      uint8_t* __restrict       dst,
                      unsigned int              dstStride );

//------------------------------------------------------------------------------
/// @brief
///   Downscale a grayscale image by a factor of two using a 5x5 Gaussian filter kernel
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvScaleDownBy2Gaussian5x5u8_v2(). In the 2.0.0 release, 
///   fcvScaleDownBy2Gaussian5x5u8_v2 will be renamed to fcvScaleDownBy2Gaussian5x5u8
///   and the signature of fcvScaleDownBy2Gaussian5x5u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   Downsamples the image using a 5x5 Gaussian filter kernel.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be multiple of 8
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output 8-bit downscale image of size (width / 2) x (height / 2).
///
/// @test
///   -# Constant intensity image inputs: the output images should contain
///                     the same intensities.
///   -# Very small image sizes (8x2) as input.
///   -# 2D sine wave images (various x/y frequencies) as spatially
///                     varying inputs and to test response at different frequencies.
///   -# Real images with varying image sizes.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownBy2Gaussian5x5u8( const uint8_t* __restrict src,
                              unsigned int              srcWidth,
                              unsigned int              srcHeight,
                              uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
/// @brief
///   Downscale a grayscale image by a factor of two using a 5x5 Gaussian filter kernel
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvScaleDownBy2Gaussian5x5u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvScaleDownBy2Gaussian5x5u8,
///   \a fcvScaleDownBy2Gaussian5x5u8_v2 will be removed, and the current signature
///   for \a fcvScaleDownBy2Gaussian5x5u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvScaleDownBy2Gaussian5x5u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   Downsamples the image using a 5x5 Gaussian filter kernel.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be multiple of 8
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @param dst
///   Output 8-bit downscale image of size (width / 2) x (height / 2).
/// 
/// @param dstStride
///   Output stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @test
///   -# Constant intensity image inputs: the output images should contain
///                     the same intensities.
///   -# Very small image sizes (8x2) as input.
///   -# 2D sine wave images (various x/y frequencies) as spatially
///                     varying inputs and to test response at different frequencies.
///   -# Real images with varying image sizes.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownBy2Gaussian5x5u8_v2( const uint8_t* __restrict src,
                                 unsigned int              srcWidth,
                                 unsigned int              srcHeight,
                                 unsigned int              srcStride,
                                 uint8_t* __restrict       dst,
                                 unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Downscale the image to quarter width and height by averaging 4x4 pixels
///   into one..
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvScaleDownBy4u8_v2(). In the 2.0.0 release, 
///   fcvScaleDownBy4u8_v2 will be renamed to fcvScaleDownBy4u8
///   and the signature of fcvScaleDownBy4u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   A 4x4 downsampling box filter across adjacent pixels is applied.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be multiple of 8
///
/// @param srcHeight
///   Image height.
///   \n\b NOTE:must be a multiple of 4
///
/// @param dst
///   Output 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# Wrong image dimensions: (8x3 or 3x8).
///   -# Constant intensity image inputs: the output images should contain
///                     the same intensities.
///   -# Very small image sizes (8x4) as input.
///   -# 2D sine wave images (various x/y frequencies) as spatially
///                     varying inputs and to test response at different frequencies.
///   -# Real images with varying image sizes.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API int
fcvScaleDownBy4u8( const uint8_t* __restrict src,
                    unsigned int             srcWidth,
                    unsigned int             srcHeight,
                    uint8_t* __restrict      dst );


//------------------------------------------------------------------------------
/// @brief
///   Downscale the image to quarter width and height by averaging 4x4 pixels
///   into one..
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvScaleDownBy4u8_v2() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvScaleDownBy4u8_v2,
///   \a fcvScaleDownBy4u8_v2 will be removed, and the current signature
///   for \a fcvScaleDownBy4u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvScaleDownBy4u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   A 4x4 downsampling box filter across adjacent pixels is applied.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: must be multiple of 8
///
/// @param srcHeight
///   Image height.
///   \n\b NOTE:must be a multiple of 4
/// 
/// @param srcStride
///   Image stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @param dst
///   Output 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
/// 
/// @param dstStride
///   Output stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @test
///   -# Wrong image dimensions: (8x3 or 3x8).
///   -# Constant intensity image inputs: the output images should contain
///                     the same intensities.
///   -# Very small image sizes (8x4) as input.
///   -# 2D sine wave images (various x/y frequencies) as spatially
///                     varying inputs and to test response at different frequencies.
///   -# Real images with varying image sizes.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API int
fcvScaleDownBy4u8_v2( const uint8_t* __restrict src,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      unsigned int              srcStride,
                      uint8_t* __restrict       dst,
                      unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Downscale the image to 2/3 width and height by averaging 3x3 pixels
///   into one..
///
/// @details
///   A 3x3 downsampling box filter across adjacent pixels is applied.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b NOTE: In case of non multiple of 3, it will crop to the closest multiple of 3
///
/// @param srcHeight
///   Image height.
///   \n\b NOTE: In case of non multiple of 3, it will crop to the closest multiple of 3
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2). If 0 is passed, srcStride is set to srcWidth.
/// 
/// @param dst
///   Output 8-bit image.
///   \n\b WARNING: must be 128-bit aligned. 
///   Memory must be pre-allocated at least srcWidth * srcHeight * 2 / 3
///   dstWidth  = srcWidth/3*2
///   dstHeight = srcHeight/3*2
/// 
/// @param dstStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2). If 0 is passed, dstStride is set to dstWidth which is srcWidth *2/3.
/// 
/// @return 0 if successful
/// 
/// @test
///   -# Wrong image dimensions: (9x2 or 2x9).
///   -# Constant intensity image inputs: the output images should contain
///                     the same intensities.
///   -# Very small image sizes (9x2) as input.
///   -# 2D sine wave images (various x/y frequencies) as spatially
///                     varying inputs and to test response at different frequencies.
///   -# Real images with varying image sizes.
///   -# Source and Destination Stride equals to 0, same as width and bigger than the width.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API int
fcvScaleDown3To2u8( const uint8_t* __restrict src,
                    unsigned                  srcWidth,
                    unsigned                  srcHeight,
                    unsigned int              srcStride,
                    uint8_t* __restrict       dst,
                    unsigned int              dstStride);

//---------------------------------------------------------------------------
/// @brief
///   Downsample Horizontaly and/or Vertically by an *integer* scale.
///
/// @details
///    Uses Nearest Neighbor method
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Source Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Source Image height.
/// 
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2). If 0 is passed, srcStride is set to srcWidth.
/// 
/// @param dst
///   Output 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dstWidth
///   Destination Image width.
///
/// @param dstHeight
///   Destination Image height.
/// 
/// @param dstStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2). If 0 is passed, dstStride is set to dstWidth which is srcWidth *2/3.
/// 
/// @return 0 if successful
///
/// @test
///   -# Wrong image dimensions.
///   -# Constant intensity image inputs: the output images should contain
///                     the same intensities.
///   -# Very small image sizes (8x2) as input.
///   -# Stride size other than the input image width.
///   -# Different scale factors (dstWidth and dstHeight can be different)
///   -# Real images with varying image sizes.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API int
fcvScaleDownNNu8( const uint8_t* __restrict src,
                  unsigned int              srcWidth,
                  unsigned int              srcHeight,
                  unsigned int              srcStride,
                  uint8_t* __restrict       dst,
                  unsigned int              dstWidth,
                  unsigned int              dstHeight,
                  unsigned int              dstStride );

//---------------------------------------------------------------------------
/// @brief
///   Downsample Horizontaly and/or Vertically by an *integer* scale.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvScaleDownu8_v2(). In the 2.0.0 release, 
///   fcvScaleDownu8_v2 will be renamed to fcvScaleDownu8
///   and the signature of fcvScaleDownu8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///    Uses an box averaging filter of size MxN where M is the scale factor
///    in horizontal dimension and N is the scale factor in the vertical
///    dimension
///    \n NOTE: On different processors, some output pixel values may be off by 1
///
/// @param srcImg
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Source Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Source Image height.
///
/// @param dstImg
///   Output 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dstWidth
///   Destination Image width.
///
/// @param dstHeight
///   Destination Image height.
///
/// @test
///   -# Wrong image dimensions.
///   -# Constant intensity image inputs: the output images should contain
///                     the same intensities.
///   -# Very small image sizes (8x2) as input.
///   -# 2D sine wave images (various x/y frequencies) as spatially
///                     varying inputs and to test response at different frequencies.
///   -# Stride size other than the input image width.
///   -# Different scale factors (dstWidth and dstHeight can be different)
///   -# Real images with varying image sizes.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownu8( const uint8_t* __restrict src,
                unsigned int              srcWidth,
                unsigned int              srcHeight,
                uint8_t* __restrict       dst,
                unsigned int              dstWidth,
                unsigned int              dstHeight );


//---------------------------------------------------------------------------
/// @brief
///   Downsample Horizontaly and/or Vertically by an *integer* scale.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvScaleDownu8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvScaleDownu8,
///   \a fcvScaleDownu8_v2 will be removed, and the current signature
///   for \a fcvScaleDownu8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvScaleDownu8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///    Uses an box averaging filter of size MxN where M is the scale factor
///    in horizontal dimension and N is the scale factor in the vertical
///    dimension
///    \n NOTE: On different processors, some output pixel values may be off by 1
///
/// @param srcImg
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Source Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Source Image height.
/// 
/// @param srcStride
///   Image stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @param dstImg
///   Output 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dstWidth
///   Destination Image width.
///
/// @param dstHeight
///   Destination Image height.
/// 
/// @param dstStride
///   Output stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @test
///   -# Wrong image dimensions.
///   -# Constant intensity image inputs: the output images should contain
///                     the same intensities.
///   -# Very small image sizes (8x2) as input.
///   -# 2D sine wave images (various x/y frequencies) as spatially
///                     varying inputs and to test response at different frequencies.
///   -# Stride size other than the input image width.
///   -# Different scale factors (dstWidth and dstHeight can be different)
///   -# Real images with varying image sizes.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownu8_v2( const uint8_t* __restrict src,
                   unsigned int              srcWidth,
                   unsigned int              srcHeight,
                   unsigned int              srcStride,
                   uint8_t* __restrict       dst,
                   unsigned int              dstWidth,
                   unsigned int              dstHeight,
                   unsigned int              dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Upscale a grayscale image by a factor of two using a 5x5 Gaussian filter kernel
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvScaleUpBy2Gaussian5x5u8_v2(). In the 2.0.0 release, 
///   fcvScaleUpBy2Gaussian5x5u8_v2 will be renamed to fcvScaleUpBy2Gaussian5x5u8
///   and the signature of fcvScaleUpBy2Gaussian5x5u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   Upsamples the image using a 5x5 Gaussian filter kernel.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output 8-bit upsampled image of size (2*width) x (2*height).
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# 2D sine wave images (various x/y frequencies) as spatially
///                     varying inputs and to test response at different frequencies.
///   -# Real images with varying image sizes.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleUpBy2Gaussian5x5u8( const uint8_t* __restrict src,
                            unsigned int              srcWidth,
                            unsigned int              srcHeight,
                            uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
/// @brief
///   Upscale a grayscale image by a factor of two using a 5x5 Gaussian filter kernel
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvScaleUpBy2Gaussian5x5u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvScaleUpBy2Gaussian5x5u8,
///   \a fcvScaleUpBy2Gaussian5x5u8_v2 will be removed, and the current signature
///   for \a fcvScaleUpBy2Gaussian5x5u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvScaleUpBy2Gaussian5x5u8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   Upsamples the image using a 5x5 Gaussian filter kernel.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
/// 
/// @param srcStride
///   Image stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output 8-bit upsampled image of size (2*width) x (2*height).
///   \n\b WARNING: must be 128-bit aligned.
/// 
/// @param dstStride
///   Output stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @test
///   -# 2D sine wave images (various x/y frequencies) as spatially
///                     varying inputs and to test response at different frequencies.
///   -# Real images with varying image sizes.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleUpBy2Gaussian5x5u8_v2( const uint8_t* __restrict src,
                               unsigned int              srcWidth,
                               unsigned int              srcHeight,
                               unsigned int              srcStride,
                               uint8_t* __restrict       dst,
                               unsigned int              dstStride );


// -----------------------------------------------------------------------------
/// @brief
///   Translate to float and normalize 36 8-bit elements
///
/// @param src
///   Pointer to the first input vector
///
/// @param invLen
///   Pointer to inverse length of the first input vector
///
/// @param numVecs
///   Number of vectors to translate
///
/// @param reqNorm
///   Required norm
///
/// @param srcStride
///   Step in bytes to data of the next vector
///
/// @param dst
///   Pointer to contiguous block for output vectors
///   \n\b WARNING: must be 128-bit aligned.
///
/// @brief stopBuild
///   Allows other threads to break this function in the middle of processing.
///   When set to 1, the function will exit on the next iteration.
///
/// @return
///   0        - success
///   EFAULT   - invalid address
///   EINVAL   - invalid argument
///
/// @ingroup math_vector
// -----------------------------------------------------------------------------

FASTCV_API int
fcvVecNormalize36s8f32( const int8_t* __restrict src,
                        unsigned int             srcStride,
                        const float*  __restrict invLen,
                        unsigned int             numVecs,
                        float                    reqNorm,
                        float*        __restrict dst,
                        int32_t*                 stopBuild );


//---------------------------------------------------------------------------
/// @brief
///   Sum of squared differences of one 36-byte vector against 4 others.
///
/// @details
///   SSD of one vector (a) against 4 others (b0,b1,b2,b3) using their given
///   inverse lengths for normalization.
///   \n\n SSD(a,b0), SSD(a,b1), SSD(a,b2), SSD(a,b3)
///
/// @param a
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param invLenA
///   Inverse of vector A.
///
/// @param b0
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b1
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b2
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param b3
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param invLenB
///   Inverse of vectors b0...b3.
///   \n\b WARNING: array must be 128-bit aligned
///
/// @param distances
///   Output of the 4 results { SSD(a,b0), SSD(a,b1), SSD(a,b2), SSD(a,b3) }.
///   \n ACCURACY: 1.0e-6
///   \n\b WARNING: array must be 128-bit aligned
///
/// @test
///   -# Extreme output values were tested.
///   -# All zero vectors: proper handling of the invLen* values.
///   -# Response to impulse signals. A sine-wave pattern was put in vA
///                     and impulse signals at different location/sign were put in the
///                     rest four.
///   -# Response to sine waves. vA is a fixed sine wave signal while
///                     vB contains sine wave signals of different frequencies and phases.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvSumOfSquaredDiffs36x4s8( const int8_t* __restrict a,
                            float                    invLenA,
                            const int8_t* __restrict b0,
                            const int8_t* __restrict b1,
                            const int8_t* __restrict b2,
                            const int8_t* __restrict b3,
                            const float* __restrict  invLenB,
                            float* __restrict        distances );


//---------------------------------------------------------------------------
/// @brief
///   Sum of squared differences of one 36-byte vector against N others.
///
/// @details
///   SSD of one vector (a) against N other 36-byte vectors
///   ( b[0], b[1], ..., b[n-1] )
///   using their given inverse lengths for normalization.
///   \n\n SSD(a,b[0]), SSD(a,b[1]), ..., SSD(a,b[n-1])
///
/// @param a
///   Vector.
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param invLenA
///   Inverse of vector A.
///
/// @param b
///   Vectors b[0]...b[n-1].
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param invLenB
///   Inverse of vectors B[0]...B[n-1].
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param numB
///   Number of B vectors.
///
/// @param distances
///   Output of the N results { SSD(a,b[0]), SSD(a,b[1]), ..., SSD(a,b[n-1]) }.
///   \n ACCURACY: 1.0e-6
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# Extreme output values were tested.
///   -# All zero vectors: proper handling of the invLen* values.
///   -# Response to impulse signals. A sine-wave pattern was put in vA
///                     and impulse signals at different location/sign were put in the
///                     rest four.
///   -# Response to sine waves. vA is a fixed sine wave signal while
///                     vB contains sine wave signals of different frequencies and phases.
///   -# The number of input vectors vB (numB), from 0 to 100, were tested.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvSumOfSquaredDiffs36xNs8( const int8_t* __restrict         a,
                            float                            invLenA,
                            const int8_t* const * __restrict b,
                            const float* __restrict          invLenB,
                            unsigned int                     numB,
                            float* __restrict                distances );


//---------------------------------------------------------------------------
/// @brief
///   Sorting of 8 float numbers
///
/// @details
///   Perform sorting of 8 scores in ascending order (output of SumOfSquaredDiffs)
///
/// @param inScores
///   Input 8 element float array
///   \n\b NOTE: array should be 128-bit aligned
///
/// @param outScores
///   Output is 8 element sorted float array
///   \n\b WARNING: array must be 128-bit aligned
///
/// @test
///   -# Different combination of float arrays were tested.
///   -# All zero vectors
///
///   @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvSort8Scoresf32( float* __restrict inScores, float* __restrict outScores );

//------------------------------------------------------------------------------
/// @brief
///   Binarizes a grayscale image based on a threshold value.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterThresholdu8_v2(). In the 2.0.0 release, 
///   fcvFilterThresholdu8_v2 will be renamed to fcvFilterThresholdu8
///   and the signature of fcvFilterThresholdu8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   Sets the pixel to max(255) if it's value is greater than the threshold;
///   else, set the pixel to min(0).
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output 8-bit binarized image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param threshold
///   Threshold value for binarization.
///
/// @test
///   -# Test impulse inputs. Positive (white impulse on black) and negative
///                     (black impulse on white) impulse inputs were used for testing
///                     thresholding at value 127.
///   -# Test threshold. An image pattern with linearly varying intensity
///                     values was created for testing using different
///                     threshold settings.
///   -# 2D sine wave patterns with different x and y directional frequencies
///                     were used for testing. The threshold involved is also varying to
///                     test thresholding performances.
///   -# Real image patches with randomly selected thresholds.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterThresholdu8( const uint8_t* __restrict src,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      uint8_t* __restrict       dst,
                      unsigned int              threshold );


//------------------------------------------------------------------------------
/// @brief
///   Binarizes a grayscale image based on a threshold value.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterThresholdu8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterThresholdu8,
///   \a fcvFilterThresholdu8_v2 will be removed, and the current signature
///   for \a fcvFilterThresholdu8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterThresholdu8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   Sets the pixel to max(255) if it's value is greater than the threshold;
///   else, set the pixel to min(0).
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Output 8-bit binarized image.
///   \n\b WARNING: must be 128-bit aligned.
/// 
/// @param dstStride
///   Output stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values).
///
/// @param threshold
///   Threshold value for binarization.
///
/// @test
///   -# Test impulse inputs. Positive (white impulse on black) and negative
///                     (black impulse on white) impulse inputs were used for testing
///                     thresholding at value 127.
///   -# Test threshold. An image pattern with linearly varying intensity
///                     values was created for testing using different
///                     threshold settings.
///   -# 2D sine wave patterns with different x and y directional frequencies
///                     were used for testing. The threshold involved is also varying to
///                     test thresholding performances.
///   -# Real image patches with randomly selected thresholds.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterThresholdu8_v2( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         unsigned int              srcStride,
                         uint8_t* __restrict       dst,
                         unsigned int              dstStride,
                         unsigned int              threshold );


//------------------------------------------------------------------------------
/// @brief
///   Dilate a grayscale image by taking the local maxima of 3x3 nbhd window.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterDilate3x3u8_v2(). In the 2.0.0 release, 
///   fcvFilterDilate3x3u8_v2 will be renamed to fcvFilterDilate3x3u8
///   and the signature of fcvFilterDilate3x3u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output 8-bit dilated image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# Test impulse response. An black image with a single center pixel of
///                     value 255 was created to test dilation.
///   -# Test step response. An image with values 255 on the left and values
///                     of 0 on the right were used to test expansion of bright part.
///   -# Test corner response. An image containing a single corner (top left
///                     quadrant bright while the other 3/4 black, or the vice versa)
///                     were used to test expansion of the bright pixels.
///   -# Test pepper-salt noise. An image with linear intensity gradient was
///                     corrupted by peper and salt noise for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterDilate3x3u8( const uint8_t* __restrict src,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      uint8_t* __restrict       dst );


//------------------------------------------------------------------------------
/// @brief
///   Dilate a grayscale image by taking the local maxima of 3x3 nbhd window.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterDilate3x3u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterDilate3x3u8,
///   \a fcvFilterDilate3x3u8_v2 will be removed, and the current signature
///   for \a fcvFilterDilate3x3u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterDilate3x3u8 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Output 8-bit dilated image.
///   \n\b WARNING: must be 128-bit aligned.
/// 
/// @param dstStride
///   Stride of output image.
///   \n\b WARNING: must be multiple of 8.
///
/// @test
///   -# Test impulse response. An black image with a single center pixel of
///                     value 255 was created to test dilation.
///   -# Test step response. An image with values 255 on the left and values
///                     of 0 on the right were used to test expansion of bright part.
///   -# Test corner response. An image containing a single corner (top left
///                     quadrant bright while the other 3/4 black, or the vice versa)
///                     were used to test expansion of the bright pixels.
///   -# Test pepper-salt noise. An image with linear intensity gradient was
///                     corrupted by peper and salt noise for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterDilate3x3u8_v2( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         unsigned int              srcStride,
                         uint8_t* __restrict       dst,
                         unsigned int              dstStride );

//------------------------------------------------------------------------------
/// @brief
///   Erode a grayscale image by taking the local minima of 3x3 nbhd window.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterErode3x3u8_v2(). In the 2.0.0 release, 
///   fcvFilterErode3x3u8_v2 will be renamed to fcvFilterErode3x3u8
///   and the signature of fcvFilterErode3x3u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output 8-bit eroded image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# Test impulse response. An black image with a single center pixel of
///                     value 255 was created to test erosion.
///   -# Test step response. An image with values 255 on the left and values
///                     of 0 on the right were used to test expansion of dark pixels.
///   -# Test corner response. An image containing a single corner (top left
///                     quadrant bright while the other 3/4 black, or the vice versa)
///                     were used to test erosion of the bright pixels.
///   -# Test pepper-salt noise. An image with linear intensity gradient was
///                     corrupted by peper and salt noise for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterErode3x3u8( const uint8_t* __restrict src,
                     unsigned int              srcWidth,
                     unsigned int              srcHeight,
                     uint8_t* __restrict       dst );

//------------------------------------------------------------------------------
/// @brief
///   Erode a grayscale image by taking the local minima of 3x3 nbhd window.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterErode3x3u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterErode3x3u8,
///   \a fcvFilterErode3x3u8_v2 will be removed, and the current signature
///   for \a fcvFilterErode3x3u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterErode3x3u8 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Output 8-bit eroded image.
///   \n\b WARNING: must be 128-bit aligned.
/// 
/// @param dstStride
///   Stride of output image.
///   \n\b WARNING: must be multiple of 8.
///
/// @test
///   -# Test impulse response. An black image with a single center pixel of
///                     value 255 was created to test erosion.
///   -# Test step response. An image with values 255 on the left and values
///                     of 0 on the right were used to test expansion of dark pixels.
///   -# Test corner response. An image containing a single corner (top left
///                     quadrant bright while the other 3/4 black, or the vice versa)
///                     were used to test erosion of the bright pixels.
///   -# Test pepper-salt noise. An image with linear intensity gradient was
///                     corrupted by peper and salt noise for testing.
///   -# Test frequency response. 2D sine wave patterns with different x
///                     and y directional frequencies were used for testing.
///   -# Real images were used for tests.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvFilterErode3x3u8_v2( const uint8_t* __restrict src,
                        unsigned int              srcWidth,
                        unsigned int              srcHeight,
                        unsigned int              srcStride,
                        uint8_t* __restrict       dst,
                        unsigned int              dstStride );

//---------------------------------------------------------------------------
/// @brief
///   Warps the patch centered at nPos in the input image using the affine
///   transform in nAffine
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvTransformAffine8x8u8_v2(). In the 2.0.0 release, 
///   fcvTransformAffine8x8u8_v2 will be renamed to fcvTransformAffine8x8u8
///   and the signature of fcvTransformAffine8x8u8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param nPos[ 2 ]
///   Position in the image in 32 bit fixed point
///   \n\b WARNING: must be 64-bit aligned.
///
/// @param nAffine[ 2 ][ 2 ]
///   Transformation matrix in 32 bit fixed point
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param nPatch
///   Transformed patch.
///
///
/// @returns 0 if the transformation is valid
///
/// @test
///   -# A small amount of transformations: translation, rotation, affine
///                     shearing.
///   -# Test when the window is close to or straddling the image boundaries.
///   -# Test scale variations (scales from 0.5 to 4 with increments of 0.5).
///   -# Test transforming of a sine wave input image under 20 random
///                     affine transformations, including cases when the integration
///                     straddles image boundaries.
///   -# A set of 50 real image patches with randomly generated
///                     transformations for behavior on real images.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API int
fcvTransformAffine8x8u8( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         const int32_t* __restrict nPos,
                         const int32_t* __restrict nAffine,
                         uint8_t* __restrict       nPatch );


//---------------------------------------------------------------------------
/// @brief
///   Warps the patch centered at nPos in the input image using the affine
///   transform in nAffine
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvTransformAffine8x8u8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvTransformAffine8x8u8,
///   \a fcvTransformAffine8x8u8_v2 will be removed, and the current signature
///   for \a fcvTransformAffine8x8u8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvTransformAffine8x8u8 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Stride of image (in bytes) - i.e., how many bytes between column 0 of row N 
///   and column 0 of row N+1.
///
/// @param nPos[ 2 ]
///   Position in the image in 32 bit fixed point
///   \n\b WARNING: must be 64-bit aligned.
///
/// @param nAffine[ 2 ][ 2 ]
///   Transformation matrix in 32 bit fixed point
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param patch
///   Transformed patch.
/// 
/// @param patchStride
///   Stride of patch (in bytes) - i.e., how many bytes between column 0 of row N 
///   and column 0 of row N+1.
///
///
/// @returns 0 if the transformation is valid
///
/// @test
///   -# A small amount of transformations: translation, rotation, affine
///                     shearing.
///   -# Test when the window is close to or straddling the image boundaries.
///   -# Test scale variations (scales from 0.5 to 4 with increments of 0.5).
///   -# Test transforming of a sine wave input image under 20 random
///                     affine transformations, including cases when the integration
///                     straddles image boundaries.
///   -# A set of 50 real image patches with randomly generated
///                     transformations for behavior on real images.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API int
fcvTransformAffine8x8u8_v2( const uint8_t* __restrict src,
                            unsigned int              srcWidth,
                            unsigned int              srcHeight,
                            unsigned int              srcStride,
                            const int32_t* __restrict nPos,
                            const int32_t* __restrict nAffine,
                            uint8_t* __restrict       patch,
                            unsigned int              patchStride );


//------------------------------------------------------------------------------
/// @brief
///   Warps a grayscale image using the a perspective projection transformation
///   matrix (also known as a homography). This type of transformation is an
///   invertible transformation which maps straight lines to straight lines.
///   Bi-linear interpolation is used where applicable.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvWarpPerspectiveu8_v2(). In the 2.0.0 release, 
///   fcvWarpPerspectiveu8_v2 will be renamed to fcvWarpPerspectiveu8
///   and the signature of fcvWarpPerspectiveu8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   Warps an image taking into consideration the perspective scaling.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Input image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Input image height.
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Warped output image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dstWidth
///   Dst image width.
///   \n\b NOTE: data must be multiple of 8.
///
/// @param dstHeight
///   Dst image height.
///   \n\b NOTE: must be multiple of 8
///
/// @param projectionMatrix
///   3x3 perspective transformation matrix (generally a homography). The
///   matrix stored in homography is row major ordering: \n
///   a11, a12, a13, a21, a22, a23, a31, a32, a33 where the matrix is: \n
///   | a11, a12, a13 |\n
///   | a21, a22, a23 |\n
///   | a31, a32, a33 |\n
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# Test warping of small translation, rotation, affine shearing of a
///                     small checker board pattern (8x8 images with 2x2 blocks).
///   -# Test scale variations. The checker board pattern above was warped
///                     using a scale factor from 0.2 to 2 with an increment of 0.2.
///   -# Test the cases when a sampling window is straddling the source
///                     image.
///   -# Test warping real image patches using randomly selected homography.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API void
fcvWarpPerspectiveu8( const uint8_t* __restrict src,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      uint8_t* __restrict       dst,
                      unsigned int              dstWidth,
                      unsigned int              dstHeight,
                      float* __restrict         projectionMatrix );


//------------------------------------------------------------------------------
/// @brief
///   Warps a grayscale image using the a perspective projection transformation
///   matrix (also known as a homography). This type of transformation is an
///   invertible transformation which maps straight lines to straight lines.
///   Bi-linear interpolation is used where applicable.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvWarpPerspectiveu8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvWarpPerspectiveu8,
///   \a fcvWarpPerspectiveu8_v2 will be removed, and the current signature
///   for \a fcvWarpPerspectiveu8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvWarpPerspectiveu8 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   Warps an image taking into consideration the perspective scaling.
///
/// @param src
///   Input 8-bit image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Input image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Input image height.
///   \n\b WARNING: must be multiple of 8.
/// 
/// @param srcStride
///   Input image stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values)
///
/// @param dst
///   Warped output image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dstWidth
///   Dst image width.
///   \n\b NOTE: data must be multiple of 8.
///
/// @param dstHeight
///   Dst image height.
///   \n\b NOTE: must be multiple of 8
/// 
/// @param dstStride
///   Output image stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values)
///
/// @param projectionMatrix
///   3x3 perspective transformation matrix (generally a homography). The
///   matrix stored in homography is row major ordering: \n
///   a11, a12, a13, a21, a22, a23, a31, a32, a33 where the matrix is: \n
///   | a11, a12, a13 |\n
///   | a21, a22, a23 |\n
///   | a31, a32, a33 |\n
///   \n\b WARNING: must be 128-bit aligned.
///
/// @test
///   -# Test warping of small translation, rotation, affine shearing of a
///                     small checker board pattern (8x8 images with 2x2 blocks).
///   -# Test scale variations. The checker board pattern above was warped
///                     using a scale factor from 0.2 to 2 with an increment of 0.2.
///   -# Test the cases when a sampling window is straddling the source
///                     image.
///   -# Test warping real image patches using randomly selected homography.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API void
fcvWarpPerspectiveu8_v2( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         unsigned int              srcStride,
                         uint8_t* __restrict       dst,
                         unsigned int              dstWidth,
                         unsigned int              dstHeight,
                         unsigned int              dstStride,
                         float* __restrict         projectionMatrix );


//---------------------------------------------------------------------------
/// @brief
///   Warps a 3 color channel image based on a 3x3 perspective projection matrix using
///   bilinear interpolation.
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcv3ChannelWarpPerspectiveu8_v2(). In the 2.0.0 release, 
///   fcv3ChannelWarpPerspectiveu8_v2 will be renamed to fcv3ChannelWarpPerspectiveu8
///   and the signature of fcv3ChannelWarpPerspectiveu8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image.
///   \n\b NOTE: data must be 64-bit aligned.
///
/// @param srcwidth
///   Input image width.
///   \n\b NOTE: must be multiple of 8
///
/// @param srcheight
///   Input image height.
///   \n\b NOTE: must be multiple of 8
///
/// @param dst
///   Warped output image.
///   \n\b NOTE: data must be 64-bit aligned.
///
/// @param dstwidth
///   Output image width.
///   \n\b NOTE: must be multiple of 8.
///
/// @param dstheight
///   Output image height.
///   \n\b NOTE: must be multiple of 8.
///
/// @param projectionMatrix[3][3]
///   3x3 perspective projection matrix.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @ingroup image_transform
//---------------------------------------------------------------------------

FASTCV_API void
fcv3ChannelWarpPerspectiveu8( const uint8_t* __restrict src,
                              unsigned int              srcWidth,
                              unsigned int              srcHeight,
                              uint8_t* __restrict       dst,
                              unsigned int              dstWidth,
                              unsigned int              dstHeight,
                              float* __restrict         projectionMatrix );


//---------------------------------------------------------------------------
/// @brief
///   Warps a 3 color channel image based on a 3x3 perspective projection 
///   matrix using bilinear interpolation.
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcv3ChannelWarpPerspectiveu8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcv3ChannelWarpPerspectiveu8,
///   \a fcv3ChannelWarpPerspectiveu8_v2 will be removed, and the current signature
///   for \a fcv3ChannelWarpPerspectiveu8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcv3ChannelWarpPerspectiveu8 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image.
///   \n\b WARNING: data must be 128-bit aligned.
///
/// @param srcWidth
///   Input image width.
///   \n\b WARNING: must be multiple of 8
///
/// @param srcHeight
///   Input image height.
///   \n\b WARNING: must be multiple of 8
/// 
/// @param srcStride
///   Input image stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values)
///
/// @param dst
///   Warped output image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dstWidth
///   Output image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param dstHeight
///   Output image height.
///   \n\b WARNING: must be multiple of 8.
/// 
/// @param dstStride
///   Output image stride (in bytes).
///   \n\b WARNING: must be multiple of 8 (8 * 1-byte values)
///
/// @param projectionMatrix[3][3]
///   3x3 perspective projection matrix.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @ingroup image_transform
//---------------------------------------------------------------------------

FASTCV_API void
fcv3ChannelWarpPerspectiveu8_v2( const uint8_t* __restrict src,
                                 unsigned int              srcWidth,
                                 unsigned int              srcHeight,
                                 unsigned int              srcStride,
                                 uint8_t* __restrict       dst,
                                 unsigned int              dstWidth,
                                 unsigned int              dstHeight,
                                 unsigned int              dstStride,
                                 float* __restrict         projectionMatrix );


//---------------------------------------------------------------------------
/// @brief
///   General function for computing cluster centers and cluster bindings
///   for a set of points of dimension dim.
///
/// @param points
///   Array of all points. Array size must be greater than
///   numPoints * dim.
///
/// @param numPoints
///   Number of points in points array.
///   \n\b WARNING: must be > numPoints * dim.
///
/// @param dim
///   dimension, e.g. 36
///
/// @param pointStride
///   Byte distance between adjacent points in array
///
/// @param indices
///   Array of point indices in points array. Processing will only
///   occur on points whose indices are in this array. Each index in array
///   must be smaller numPoints.
///
/// @param numIndices
///   Length of indices array. numIndieces must be <= numPoints.
///
/// @param numClusterCenters
///   Number of cluster centers
///
/// @param clusterCenters
///   current cluster centers;
///   elements are distant by clusterCenterStride
///
/// @param clusterCenterStride
///   byte distance between adjacent cluster centers in array
///
/// @param newClusterCenters
///   array for new cluster centers; should be numClusterCenters long
///
/// @param newClusterMemberCounts
///   Element counts for each cluster; should be numClusterCenters long
///
/// @param clusterBindings
///   Output indices of the clusters to which each vector belongs to, array must
///   be numIndices long.
///
/// @return
///   0 if successfully clustered, otherwise error code
///
/// @remark
///   This is general clusterer. There are no assumptions on points other
///   than they belong to a vector space
///
/// @test
///   The test vectors are generated to test the following aspects of the
///         function:
///   -# Ability to cluster two clear clusters separated by a large
///         distance for both normalized and un-normalized vectors
///   -# Ability to use a subset of the vectors specified by "indices"
///   -# Proper handling of sparse input, i.e., more cluster centers than
///         points.
///   -# Proper handling of pointStride and clusterCenterStride
///   -# Proper handling of vector length (dim)
///   -# Proper clustering of 100 sets of points with different point
///         configurations  and initializations.
///
/// @ingroup clustering_and_search
//---------------------------------------------------------------------------

FASTCV_API int
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
                        float*                   sumOfClusterDistances );


//---------------------------------------------------------------------------
/// @brief
///   Function for computing cluster centers and cluster bindings
///   for a set of normalized points of dimension dim. Cluster centers
///   are also normalized (see remark below)
///
/// @param points
///   Array of all points. Array size must be greater than
///   numPoints * dim.
///
/// @param numPoints
///   Number of points in points array.
///
/// @param dim
///   dimension, e.g. 36
///
/// @param pointStride
///   Byte distance between adjacent points in array
///
/// @param indices
///   Array of point indices in points array. Processing will only
///   occur on points whose indices are in this array. Each index in array
///   must be smaller numPoints.
///
/// @param numIndices
///   Length of indices array. numIndieces must be <= numPoints.
///
/// @param numClusterCenters
///   Number of cluster centers
///
/// @param clusterCenters
///   current cluster centers;
///   elements are distant by clusterCenterStride
///
/// @param clusterCenterStride
///   byte distance between adjacent cluster centers in array
///
/// @param newClusterCenters
///   array for new cluster centers; should be numClusterCenters long
///
/// @param newClusterMemberCounts
///   Element counts for each cluster; should be numClusterCenters long
///
/// @param clusterBindings
///   Output indices of the clusters to which each vector belongs to, a
///   rray must be numIndices long.
///
/// @param sumOfClusterDistances
///   Array for sum of distances of cluster elements to cluster centers;
///   Must be numClusters long
///
/// @return
///   0 if successfully clustered, otherwise error code
///
/// @remark
///   this function assumes that points are normalized (e.g. NFT4
///   descriptors). Cluster centers are also normalized. Normalized points
///   are on a surface of unit sphere which is not a vector space but
///   curved manifold of dimension (dim-1) embeded in Euclidean vector space
///   of dimension dim
///
/// @ingroup clustering_and_search
//---------------------------------------------------------------------------

FASTCV_API int
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
                              float*                   sumOfClusterDistances );


//---------------------------------------------------------------------------
/// @brief
///   Function for computing cluster centers and cluster bindings
///   for a set of normalized points of dimension 36. Cluster centers
///   are also normalized (see remark below)
///
/// @param points
///   Array of all points. Array size must be greater than
///   numPoints * 36.
///
/// @param numPoints
///   Number of points in points array.
///
/// @param pointStride
///   Byte distance between adjacent points in array
///
/// @param indices
///   Array of point indices in points array. Processing will only
///   occur on points whose indices are in this array. Each index in array
///   must be smaller numPoints.
///
/// @param numIndices
///   Length of indices array. numIndieces must be <= numPoints.
///
/// @param numClusterCenters
///   Number of cluster centers
///
/// @param clusterCenters
///   current cluster centers;
///   elements are distant by clusterCenterStride
///
/// @param clusterCenterStride
///   byte distance between adjacent cluster centers in array
///
/// @param newClusterCenters
///   array for new cluster centers; should be numClusterCenters long
///
/// @param newClusterMemberCounts
///   Element counts for each cluster; should be numClusterCenters long
///
/// @param clusterBindings
///   Output indices of the clusters to which each vector belongs to, a
///   rray must be numIndices long.
///
/// @return
///   0 if successfully clustered, otherwise error code
///
/// @remark
///   this function assumes that points are normalized (e.g. NFT4
///   descriptors). Cluster centers are also normalized. Normalized points
///   are on a surphace of unit sphere which is not a vector space but
///   curved manifold of dimension (dim-1) embeded in Euclidean vector space
///   of dimension dim
///
/// @ingroup clustering_and_search
//---------------------------------------------------------------------------

FASTCV_API int
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
                                float*                   sumOfClusterDistances );


//---------------------------------------------------------------------------
/// @brief
///   Blur with 5x5 Gaussian filter
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterGaussian5x5s16_v2(). In the 2.0.0 release, 
///   fcvFilterGaussian5x5s16_v2 will be renamed to fcvFilterGaussian5x5s16
///   and the signature of fcvFilterGaussian5x5s16 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   Convolution with 5x5 Gaussian kernel:
///   \n 1  4  6  4 1
///   \n 4 16 24 16 4
///   \n 6 24 36 24 6
///   \n 4 16 24 16 4
///   \n 1  4  6  4 1
///
/// @param src
///   Input int data (can be sq. of gradient, etc).
///   \n\b NOTE: data should be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output int data.
///   \n\b NOTE: data should be 128-bit aligned.
///
/// @param blurBorder
///   If set to 0, border is ignored.
///   If set to 1, border is blurred by 0-padding adjacent values.
///
/// @ingroup image_processing
//---------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian5x5s16( const int16_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         int16_t* __restrict       dst,
                         int                       blurBorder );


//---------------------------------------------------------------------------
/// @brief
///   Blur with 5x5 Gaussian filter
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterGaussian5x5s16() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterGaussian5x5s16,
///   \a fcvFilterGaussian5x5s16_v2 will be removed, and the current signature
///   for \a fcvFilterGaussian5x5s16 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterGaussian5x5s16 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   Convolution with 5x5 Gaussian kernel:
///   \n 1  4  6  4 1
///   \n 4 16 24 16 4
///   \n 6 24 36 24 6
///   \n 4 16 24 16 4
///   \n 1  4  6  4 1
///
/// @param src
///   Input int data (can be sq. of gradient, etc).
///   \n\b NOTE: data should be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Output int data.
///   \n\b NOTE: data should be 128-bit aligned.
/// 
/// @param dstStride
///   Output stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param blurBorder
///   If set to 0, border is ignored.
///   If set to 1, border is blurred by 0-padding adjacent values.
///
/// @ingroup image_processing
//---------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian5x5s16_v2( const int16_t* __restrict src,
                            unsigned int              srcWidth,
                            unsigned int              srcHeight,
                            unsigned int              srcStride,
                            int16_t* __restrict       dst,
                            unsigned int              dstStride,
                            int                       blurBorder );


//---------------------------------------------------------------------------
/// @brief
///   Blur with 5x5 Gaussian filter
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterGaussian5x5s32_v2(). In the 2.0.0 release, 
///   fcvFilterGaussian5x5s32_v2 will be renamed to fcvFilterGaussian5x5s32
///   and the signature of fcvFilterGaussian5x5s32 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   Convolution with 5x5 Gaussian kernel:
///   \n 1  4  6  4 1
///   \n 4 16 24 16 4
///   \n 6 24 36 24 6
///   \n 4 16 24 16 4
///   \n 1  4  6  4 1
///
/// @param src
///   Input int data (can be sq. of gradient, etc).
///   \n\b NOTE: data should be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param dst
///   Output int data.
///   \n\b NOTE: data should be 128-bit aligned.
///
/// @param blurBorder
///   If set to 0, border is ignored.
///   If set to 1, border is blurred by 0-padding adjacent values.
///
/// @ingroup image_processing
//---------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian5x5s32( const int32_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         int32_t* __restrict       dst,
                         int                       blurBorder );


//---------------------------------------------------------------------------
/// @brief
///   Blur with 5x5 Gaussian filter
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvFilterGaussian5x5s32() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvFilterGaussian5x5s32,
///   \a fcvFilterGaussian5x5s32_v2 will be removed, and the current signature
///   for \a fcvFilterGaussian5x5s32 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvFilterGaussian5x5s32 when transitioning to 2.0.0.
///   \n\n
///
/// @details
///   Convolution with 5x5 Gaussian kernel:
///   \n 1  4  6  4 1
///   \n 4 16 24 16 4
///   \n 6 24 36 24 6
///   \n 4 16 24 16 4
///   \n 1  4  6  4 1
///
/// @param src
///   Input int data (can be sq. of gradient, etc).
///   \n\b NOTE: data should be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Image stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param dst
///   Output int data.
///   \n\b NOTE: data should be 128-bit aligned.
/// 
/// @param dstStride
///   Output stride.
///   \n\b WARNING: must be multiple of 8.
///
/// @param blurBorder
///   If set to 0, border is ignored.
///   If set to 1, border is blurred by 0-padding adjacent values.
///
/// @ingroup image_processing
//---------------------------------------------------------------------------

FASTCV_API void
fcvFilterGaussian5x5s32_v2( const int32_t* __restrict src,
                            unsigned int              srcWidth,
                            unsigned int              srcHeight,
                            unsigned int              srcStride,
                            int32_t* __restrict       dst,
                            unsigned int              dstStride,
                            int                       blurBorder );


//---------------------------------------------------------------------------
/// @brief
///   Warps the patch centered at nPos in the input image using the affine
///   transform in nAffine
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvTransformAffineu8_v2(). In the 2.0.0 release, 
///   fcvTransformAffineu8_v2 will be renamed to fcvTransformAffineu8
///   and the signature of fcvTransformAffineu8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param position[ 2 ]
///   Position in the image
///   \n\b WARNING: must be 64-bit aligned.
///
/// @param affine[ 2 ][ 2 ]
///   Transformation matrix
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param patch
///   Transformed patch.
///
/// @param patchWidth
///   Patch width.
///
/// @param patchHeight
///   Patch height.
///
/// @returns true if the transformation is valid
///
/// @ingroup image_transform
//---------------------------------------------------------------------------

FASTCV_API int
fcvTransformAffineu8( const uint8_t* __restrict src,
                      unsigned int              srcWidth,
                      unsigned int              srcHeight,
                      const float* __restrict   position,
                      const float* __restrict   affine,
                      uint8_t* __restrict       patch,
                      unsigned int              patchWidth,
                      unsigned int              patchHeight );


//---------------------------------------------------------------------------
/// @brief
///   Warps the patch centered at nPos in the input image using the affine
///   transform in nAffine
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvTransformAffineu8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvTransformAffineu8,
///   \a fcvTransformAffineu8_v2 will be removed, and the current signature
///   for \a fcvTransformAffineu8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvTransformAffineu8 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
/// 
/// @param srcStride
///   Stride of image (in bytes) - i.e., how many bytes between column 0 of row N 
///   and column 0 of row N+1.
///
/// @param position[ 2 ]
///   Position in the image
///   \n\b WARNING: must be 64-bit aligned.
///
/// @param affine[ 2 ][ 2 ]
///   Transformation matrix
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param patch
///   Transformed patch.
///
/// @param patchWidth
///   Patch width.
///
/// @param patchHeight
///   Patch height.
/// 
/// @param patchStride
///   Stride of patch (in bytes) - i.e., how many bytes between column 0 of row N 
///   and column 0 of row N+1.
///
/// @returns true if the transformation is valid
///
/// @ingroup image_transform
//---------------------------------------------------------------------------

FASTCV_API int
fcvTransformAffineu8_v2( const uint8_t* __restrict src,
                         unsigned int              srcWidth,
                         unsigned int              srcHeight,
                         unsigned int              srcStride,
                         const float* __restrict   position,
                         const float* __restrict   affine,
                         uint8_t* __restrict       patch,
                         unsigned int              patchWidth,
                         unsigned int              patchHeight,
                         unsigned int              patchStride );


//---------------------------------------------------------------------------
/// @brief
///   Extracts a 17x17 rotation corrected patch from a 25x25 image.
///
/// @param src
///   25x25 input image in continuous memory.
///
/// @param dst
///   17x17 output patch.
///
/// @param orientation
///   Rotation angle of patch relative to src.
///   \n10-bit fixed-point angle around unit circle.
///   \nNOTE: 0 = 0 degrees and 1024 = 360 degrees.
///
/// @ingroup image_transform
//---------------------------------------------------------------------------

FASTCV_API void
fcvCopyRotated17x17u8( const uint8_t* __restrict src,
                       uint8_t* __restrict       dst,
                       int                       orientation );


//------------------------------------------------------------------------------
/// @brief
///   Counts bits in supplied vector.
///
/// @param src
///   Pointer to vector to count bits.
///
/// @param srcLength
///   Length in bits to count. Assumed that the remainder of bits modulo 8
///   will be set to 0 a priori.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvBitCountu8( const uint8_t* __restrict src,
               unsigned int              srcLength );


//------------------------------------------------------------------------------
/// @brief
///   Counts bits in supplied 32-byte vector.
///
/// @param src
///   Pointer to 32-byte vector(s) to count bits.
///
/// @return
///   Bit count.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvBitCount32x1u8( const uint8_t* __restrict src );


//------------------------------------------------------------------------------
/// @brief
///   Counts bits in supplied 4, 32-byte vectors.
///
/// @param a
///   Pointer to 32-byte vector to count bits.
///
/// @param b
///   Pointer to 32-byte vector to count bits.
///
/// @param c
///   Pointer to 32-byte vector to count bits.
///
/// @param d
///   Pointer to 32-byte vector to count bits.
///
/// @param bitCount
///   Array to store the four resultant bit counts.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvBitCount32x4u8( const uint8_t* __restrict a,
                   const uint8_t* __restrict b,
                   const uint8_t* __restrict c,
                   const uint8_t* __restrict d,
                   uint32_t* __restrict      count );


//------------------------------------------------------------------------------
/// @brief
///   Counts bits in supplied 64-byte vector.
///
/// @param src
///   Pointer to 64-byte vector(s) to count bits.
///
/// @return
///   Bit count.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvBitCount64x1u8( const uint8_t* __restrict src );


//------------------------------------------------------------------------------
/// @brief
///   Counts bits in supplied 4, 64-byte vectors.
///
/// @param a
///   Pointer to 64-byte vector to count bits.
///
/// @param b
///   Pointer to 64-byte vector to count bits.
///
/// @param c
///   Pointer to 64-byte vector to count bits.
///
/// @param d
///   Pointer to 64-byte vector to count bits.
///
/// @param bitCount
///   Array to store the four resultant bit counts.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvBitCount64x4u8( const uint8_t* __restrict a,
                   const uint8_t* __restrict b,
                   const uint8_t* __restrict c,
                   const uint8_t* __restrict d,
                   uint32_t* __restrict      count );


//------------------------------------------------------------------------------
/// @brief
///   Counts bits in supplied  vector of unsigned intergers.
///
/// @param src
///   Pointer to vector(s) to count bits.
///
/// @param srcLength
///   Number of elements in vector
///
/// @return
///   Bit count.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvBitCountu32( const uint32_t* __restrict src,
                unsigned int               srcLength );


//------------------------------------------------------------------------------
/// @brief
///   Computes the Hamming distance between the two supplied arbitrary length
///   vectors.
///
/// @param a
///   Pointer to vector to compute distance.
///
/// @param b
///   Pointer to vector to compute distance.
///
/// @param abLength
///   Length in bits of each of the vectors. Assumed that the remainder of
///   bits modulo 8 will be set to 0 a priori.
///
/// @return
///   Hamming distance between the two vectors.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvHammingDistanceu8( const uint8_t* __restrict a,
                      const uint8_t* __restrict b,
                      unsigned int              abLength );


//------------------------------------------------------------------------------
/// @brief
///   Computes the Hamming distance between the two supplied 32-byte vectors.
///
/// @param a
///   Pointer to 32-byte vector to compute distance.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param b
///   Pointer to 32-byte vector to compute distance.
///   \n\b WARNING: must be 32-bit aligned
///
/// @return
///   Hamming distance between the two vectors.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvHammingDistance32x1u8a4( const uint8_t* __restrict a,
                            const uint8_t* __restrict b );


//------------------------------------------------------------------------------
/// @brief
///   Computes the Hamming distance between the two supplied 64-byte vectors.
///
/// @param a
///   Pointer to 64-byte vector to compute distance.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param b
///   Pointer to 64-byte vector to compute distance.
///   \n\b WARNING: must be 32-bit aligned
///
/// @return
///   Hamming distance between the two vectors.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvHammingDistance64x1u8a4( const uint8_t* __restrict a,
                            const uint8_t* __restrict b );


//------------------------------------------------------------------------------
/// @brief
///   Computes the Hamming distance between the two supplied 32-byte vectors.
///
/// @param a
///   Pointer to 32-byte vector to compute distance.
///
/// @param b
///   Pointer to 32-byte vector to compute distance.
///
/// @return
///   Hamming distance between the two vectors.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvHammingDistance32x1u8( const uint8_t* __restrict a,
                          const uint8_t* __restrict b );


//------------------------------------------------------------------------------
/// @brief
///   Computes the Hamming distance between the two supplied 64-byte vectors.
///
/// @param b
///   Pointer to 64-byte vector to compute distance.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param b
///   Pointer to 64-byte vector to compute distance.
///   \n\b WARNING: must be 32-bit aligned
///
/// @return
///   Hamming distance between the two vectors.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API uint32_t
fcvHammingDistance64x1u8( const uint8_t* __restrict a,
                          const uint8_t* __restrict b );


//------------------------------------------------------------------------------
/// @brief
///   Computes the Hamming distance between A and each of B,C,D,E 32-byte vectors.
///
/// @param a
///   Pointer to 32-byte vector to compute distance.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param b
///   Pointer to 32-byte vector to compute distance from A.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param c
///   Pointer to 32-byte vector to compute distance from A.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param d
///   Pointer to 32-byte vector to compute distance from A.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param e
///   Pointer to 32-byte vector to compute distance from A.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param hammingDistances
///   Array to store each Hamming distance between the vectors.
///   \n\b WARNING: must be 128-bit aligned
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvHammingDistance32x4u8a4( const uint8_t* __restrict a,
                            const uint8_t* __restrict b,
                            const uint8_t* __restrict c,
                            const uint8_t* __restrict d,
                            const uint8_t* __restrict e,
                            uint32_t* __restrict      hammingDistances );


//------------------------------------------------------------------------------
/// @brief
///   Computes the Hamming distance between A and each of B,C,D,E 64-byte
///   vectors.
///
/// @param a
///   Pointer to 32-byte vector to compute distance.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param B
///   Pointer to 32-byte vector to compute distance from A.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param c
///   Pointer to 32-byte vector to compute distance from A.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param d
///   Pointer to 32-byte vector to compute distance from A.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param e
///   Pointer to 32-byte vector to compute distance from A.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param hammingDistances
///   Array to store each Hamming distance between the vectors.
///   \n\b WARNING: must be 128-bit aligned
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvHammingDistance64x4u8a4( const uint8_t* __restrict a,
                            const uint8_t* __restrict b,
                            const uint8_t* __restrict c,
                            const uint8_t* __restrict d,
                            const uint8_t* __restrict e,
                            uint32_t* __restrict      hammingDistances );


//------------------------------------------------------------------------------
/// @brief
///   Computes the Hamming distance between A and each of B,C,D,E 64-byte vectors.
///
/// @param a
///   Pointer to 64-byte vector to compute distance.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param b
///   Pointer to 64-byte vector to compute distance from A.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param c
///   Pointer to 64-byte vector to compute distance from A.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param d
///   Pointer to 64-byte vector to compute distance from A.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param e
///   Pointer to 64-byte vector to compute distance from A.
///   \n\b WARNING: must be 32-bit aligned
///
/// @param hammingDistances
///   Array to store each Hamming distance between the vectors.
///
/// @ingroup math_vector
//------------------------------------------------------------------------------

FASTCV_API void
fcvHammingDistance64x4u8( const uint8_t* __restrict a,
                          const uint8_t* __restrict b,
                          const uint8_t* __restrict c,
                          const uint8_t* __restrict d,
                          const uint8_t* __restrict e,
                          uint32_t* __restrict      hammingDistances );


//---------------------------------------------------------------------------
/// @brief
///   Extracts FAST corners and scores from the image
///
/// @param src
///   8-bit image
///
/// @param srcWidth
///   Image width
///
/// @param srcHeight
///   image height
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param barrier
///   FAST threshold
///
/// @param border
///   Number for pixels to ignore from top,bottom,right,left of the image
///
/// @param xy
///   pointer to the output array cointaining the interleaved x,y position of the
///   detected corners.
///   \n\b NOTE: Remember to allocate double the size of @param maxnumcorners
///
/// @param scores
///   pointer to the output array cointaining the scores of the detected corners.
///
/// @param nCornersMax
///   Maximum number of corners. The function exists when the maximum number of
///   corners is exceeded.
///
/// @param nCorners
///   pointer to an integer storing the number of corners detected
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast9Scoreu8( const uint8_t* __restrict src,
                       unsigned int              srcWidth,
                       unsigned int              srcHeight,
                       unsigned int              srcStride,
                       int                       barrier,
                       unsigned int              border,
                       uint32_t* __restrict      xy,
                       uint32_t* __restrict      scores,
                       unsigned int              nCornersMax,
                       uint32_t* __restrict      nCorners );


//---------------------------------------------------------------------------
/// @brief
///   Extracts FAST corners and scores from the image
///
/// @param src
///   Grayscale image with one byte per pixel
///
/// @param srcWidth
///   image width
///
/// @param srcHeight
///   image height
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param barrier
///   FAST threshold
///
/// @param border
///   Number for pixels to ignore from top,bottom,right,left of the image
///
/// @param xy
///   Pointer to the output array cointaining the interleaved x,y position of the
///   detected corners.
///   \n\b NOTE: Remember to allocate double the size of @param maxnumcorners
///
/// @param scores
///   pointer to the output array cointaining the scores of the detected corners.
///
/// @param nCornersMax
///   Maximum number of corners. The function exists when the maximum number of
///   corners is exceeded.
///
/// @param nCorners
///   pointer to an integer storing the number of corners detected
///
/// @param mask
///   Per-pixel mask for each pixel represented in input image. 
///   If a bit set to 0, pixel will be a candidate for corner detection. 
///   If a bit set to 1, pixel will be ignored.
///
/// @param maskWidth
///   Width of mask. width/maskWidth must a power-of-two or maskWidth/width
///   must a power-of-two.
///
/// @param maskHeight
///   Height of mask. height/maskHeight must a power-of-two or maskHeight/height
///   must a power-of-two.
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------

FASTCV_API void
fcvCornerFast9InMaskScoreu8( const uint8_t* __restrict src,
                             unsigned int              srcWidth,
                             unsigned int              srcHeight,
                             unsigned int              srcStride,
                             int                       barrier,
                             unsigned int              border,
                             uint32_t* __restrict      xy,
                             uint32_t* __restrict      scores,
                             unsigned int              nCornersMax,
                             uint32_t* __restrict      nCorners,
                             const uint8_t* __restrict mask,
                             unsigned int              maskWidth,
                             unsigned int              maskHeight );


// -----------------------------------------------------------------------------
/// @brief
///   Optical flow. Bitwidth optimized implementation
///
/// @param src1
///   Input image from frame #1.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param src2
///   Input image from frame #2.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Input image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Input image height.
///
/// @param src1Pyr
///   Image Pyradmid of src1
///   \n\b WARNING: obtained by calling fcvPyramidCreateu8
///
/// @param src2Pyr
///   Image Pyradmid of src2
///   \n\b WARNING: obtained by calling fcvPyramidCreateu8
///
/// @param dx1Pyr 
///   Horizontal Sobel gradient pyramid for src1
///  \n\b NOTE: obtained by calling fcvSobelPyramidCreatei8 
///  \n\b NOTE: Can be left NULL. In this case the function will 
///   build the pyramid internally. 
///
/// @param dy1Pyr
///  Vertical Sobel grading pyraid for src1 
///  \n\b NOTE: obtained by calling fcvSobelPyramidCreatei8
///  \n\b NOTE: Can be left NULL. In this case the function will 
///   build the pyramid internally. 
///
/// @param featureXY
///   Pointer to X,Y floating point, sub-pixel coordinates for features to
///   track. Stored as X,Y tuples. featureXY array storage must
///   be >= featureLen*2.
///
/// @param featureXY_out
///   Pointer to X,Y floating point, sub-pixel coordinates for tracked features
///   Stored as X,Y tuples. featureXY array storage must
///   be >= featureLen*2.
///
/// @param featureStatus
///   Pointer to integer array for status of each feature defined in
///   featureXY. featureStatus array storage must
///   be >= featureLen.
///
/// @param featureLen
///   Number of features in featuresXY and featureStatus array.
///
/// @param windowWidth
///   Width of window for optical flow searching.
///    \n\b NOTE: suggested value 6 or 7
///
/// @param windowHeight
///   Height of window for optical flow searching.
///   \n\b NOTE:: suggested value 6 or 7
///
/// @param maxIterations
///   Maximum number of LK iterations to perform per pyramid level.
///   \n\b NOTE: suggested value 7
///
/// @param nPyramidLevels
///   Number of pyramid levels.
///
/// @param maxResidue
///   Maximum feature residue above which feature is declared lost.
///
/// @param minDisplacement
///   Minimum displacement solved below which iterations are stopped.
///   \n\b NOTE : Suggest that be set  to between 0.1 and 0.2, say 0.15 
///
/// @param minEigenvalue 
///  Threshold for feature goodness. If it is  set it to 0, the check is disabled. 
///  \n\b NOTE: If good features are passed  to the function, then it is  suggested 
///  that you set it to 0 to have faster function time
///
/// @param lightingNormalized
///   if 1 Enable  lightning normalization
///   \nif 0 Disable lightning normalization
///
/// @ingroup object_detection
// -----------------------------------------------------------------------------

FASTCV_API void
fcvTrackLKOpticalFlowu8( const uint8_t* __restrict src1,
                         const uint8_t* __restrict src2,
                         int                       srcWidth,
                         int                       srcHeight,
                         const fcvPyramidLevel*    src1Pyr,
                         const fcvPyramidLevel*    src2Pyr,
                         const fcvPyramidLevel*    dx1Pyr,
                         const fcvPyramidLevel*    dy1Pyr,
                         const float*              featureXY,
                         float*                    featureXY_out,
                         int32_t*                  featureStatus,
                         int                       featureLen,
                         int                       windowWidth,                          
                         int                       windowHeight,
                         int                       maxIterations,
                         int                       nPyramidLevels,
                         float                     maxResidue,
                         float                     minDisplacement,
                         float                     minEigenvalue,
                         int                       lightingNormalized );


// -----------------------------------------------------------------------------
/// @brief
///   Optical flow.
///
/// @param src1
///   Input image from frame #1.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param src2
///   Input image from frame #2.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Input image width.
///   \n\b WARNING: must be multiple of 8.
///
/// @param srcHeight
///   Input image height.
///
/// @param src1Pyr
///   Image Pyradmid of src1
///   \n\b WARNING: obtained by calling fcvPyramidCreateu8
///
/// @param src2Pyr
///   Image Pyradmid of src2
///   \n\b WARNING: obtained by calling fcvPyramidCreateu8
///
/// @param dx1Pyr 
///   Horizontal Sobel gradient pyramid for src1
///  \n\b NOTE: obtained by calling fcvSobelPyramidCreatef32
///  \n\b NOTE: Can be left NULL. In this case the function will 
///   build the pyramid internally. 
///
/// @param dy1Pyr
///  Vertical Sobel grading pyraid for src1 
///  \n\b NOTE: obtained by calling fcvSobelPyramidCreatef32
///  \n\b NOTE: Can be left NULL. In this case the function will 
///   build the pyramid internally. 
/// 
///
/// @param featureXY
///   Pointer to X,Y floating point, sub-pixel coordinates for features to
///   track. Stored as X,Y tuples. featureXY array storage must
///   be >= featureLen*2.
///
/// @param featureXY_out
///   Pointer to X,Y floating point, sub-pixel coordinates for tracked features
///   Stored as X,Y tuples. featureXY array storage must
///   be >= featureLen*2.
///
/// @param featureStatus
///   Pointer to integer array for status of each feature defined in
///   featureXY. featureStatus array storage must
///   be >= featureLen.
///   \n\b NOTE: Possible status are :
///   \n    TRACKED           1
///   \n    NOT_FOUND        -1
///   \n    SMALL_DET        -2
///   \n    MAX_ITERATIONS   -3
///   \n    OUT_OF_BOUNDS    -4
///   \n    LARGE_RESIDUE    -5
///   \n    SMALL_EIGVAL     -6
///   \n    INVALID          -99
///
/// @param featureLen
///   Number of features in featuresXY and featureStatus array.
///
/// @param windowWidth
///   Width of window for optical flow searching.
///    \n\b NOTE: suggested value 6 or 7
///
/// @param windowHeight
///   Height of window for optical flow searching.
///   \n\b NOTE:: suggested value 6 or 7
///
/// @param maxIterations
///   Maximum number of LK iterations to perform per pyramid level.
///   \n\b NOTE: suggested value 7
///
/// @param nPyramidLevels
///   Number of pyramid levels.
///
/// @param maxResidue
///   Maximum feature residue above which feature is declared lost.
///
/// @param minDisplacement
///   Minimum displacement solved below which iterations are stopped.
///   \n\b NOTE : Suggest that be set  to between 0.1 and 0.2, say 0.15 
///
/// @param minEigenvalue 
///  Threshold for feature goodness. If it is  set it to 0, the check is disabled. 
///  \n\b NOTE: If good features are passed  to the function, then it is  suggested 
///  that you set it to 0 to have faster function time
///
/// @param lightingNormalized
///   if 1 Enable  lightning normalization
///   \nif 0 Disable lightning normalization
///
/// @ingroup object_detection
//------------------------------------------------------------------------------

FASTCV_API void
fcvTrackLKOpticalFlowf32( const uint8_t* __restrict src1,
                          const uint8_t* __restrict src2,
                          unsigned int              srcWidth,
                          unsigned int              srcHeight,
                          const fcvPyramidLevel*    src1Pyr,
                          const fcvPyramidLevel*    scr2Pyr,
                          const fcvPyramidLevel*    dx1Pyr,
                          const fcvPyramidLevel*    dy1Pyr,
                          const float*              featureXY,
                          float*                    featureXY_out,
                          int32_t*                  featureStatus,
                          int                       featureLen,
                          int                       windowWidth,
                          int                       windowHeight,
                          int                       maxIterations,
                          int                       nPyramidLevels,
                          float                     maxResidue,
                          float                     minDisplacement,
                          float                     minEigenvalue,
                          int                       lightingNormalized );


// -----------------------------------------------------------------------------
/// @brief
///    Builds an image pyramid of float32.
///    \n\b NOTE: Memory should be deallocated using fcvPyramidDelete
///
/// @param src
///    Base image
///
/// @param srcWidth
///    Width of base image
///    \n\b WARNING: must be a multiple of 2^numLevels
///
/// @param srcHeight
///    Height of base image
///    \n\b WARNING: must be a multiple of 2^numLevels
///
/// @param numLevels
///    Number of levels of  the pyramid
///
/// @param fcvPyramidLevel
///    Output pyramid
///
/// @ingroup image_processing
//-------------------------------------------------------------------------------

FASTCV_API int
fcvPyramidCreatef32( const float* __restrict src,
                     unsigned int            srcWidth,
                     unsigned int            srcHeight,
                     unsigned int            numLevels,
                     fcvPyramidLevel*        pyramid );


// -----------------------------------------------------------------------------
/// @brief
///    Builds an image pyramid of uint8_t.
///    \n\b NOTE: Memory should be deallocated using fcvPyramidDelete
///
/// @param src
///    Base image
///
/// @param srcWidth
///    Width of base image
///    \n\b WARNING: must be a multiple of 2^numLevels
///
/// @param srcHeight
///    height of base image
///    \n\b NOTE: must be a multiple of 2^numLevels
///
/// @param numLevels
///    Number of levels of  the pyramid
///
/// @param fcvPyramidLevel
///    Output pyramid
///
/// @ingroup image_processing
//-------------------------------------------------------------------------------

FASTCV_API int
fcvPyramidCreateu8( const uint8_t* __restrict src,
                    unsigned int              srcWidth,
                    unsigned int              srcHeight,
                    unsigned int              numLevels,
                    fcvPyramidLevel*          pyramid );


// -----------------------------------------------------------------------------
/// @brief
///    Creates a gradient pyramid of int16_t from an image pyramid of uint8_t
///
/// @param imgPyr
///    Input Image Pyramid
///
/// @param dxPyr
///    Horizontal Sobel gradient pyramid
///
/// @param dyPyr
///    Verical Sobel gradient pyramid
///
/// @param numLevels
///    Number of levels in the pyramids
///
/// @ingroup image_processing
//-------------------------------------------------------------------------------

FASTCV_API int
fcvPyramidSobelGradientCreatei16( const fcvPyramidLevel* imgPyr,
                                  fcvPyramidLevel*       dxPyr,
                                  fcvPyramidLevel*       dyPyr,
                                  unsigned int           numLevels );


// -----------------------------------------------------------------------------
/// @brief
///    Creates a gradient pyramid of float32 from an image pyramid of uint8_t
///
/// @param imgPyr
///    input Image Pyramid
///
/// @param dxPyr
///    Horizontal Sobel gradient pyramid
///
/// @param dyPyr
///    Verical Sobel gradient pyramid
///
/// @param numLevels
///    Number of levels in the pyramids
///
/// @ingroup image_processing
//-------------------------------------------------------------------------------

FASTCV_API int
fcvPyramidSobelGradientCreatef32( const fcvPyramidLevel* imgPyr,
                                  fcvPyramidLevel*       dxPyr,
                                  fcvPyramidLevel*       dyPyr,
                                  unsigned int           numLevels  );


// -----------------------------------------------------------------------------
/// @brief
///    Creates a gradient pyramid of integer8 from an image pyramid of uint8_t
///
/// @ingroup image_processing
// -----------------------------------------------------------------------------

FASTCV_API int
fcvPyramidSobelGradientCreatei8( const fcvPyramidLevel* imgPyr,
                                 fcvPyramidLevel*       dxPyr,
                                 fcvPyramidLevel*       dyPyr,
                                 unsigned int           numLevels );


//------------------------------------------------------------------------------
/// @brief
///   Creates a 2D gradient image from source luminance data. This function computes
///   central differences on 3x3 neighborhood and then convolves the result with Sobel
///   kernel
///   \n
///   \n      [ -1 0 +1 ]              [ -1 -2 -1 ]
///   \n dx = [ -2 0 +2 ] * src   dy = [  0  0  0 ] * src
///   \n      [ -1 0 +1 ]              [ +1 +2 +1 ]
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvImageGradientSobelPlanars16_v2(). In the 2.0.0 release, 
///   fcvImageGradientSobelPlanars16_v2 will be renamed to fcvImageGradientSobelPlanars16
///   and the signature of fcvImageGradientSobelPlanars16 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
///  @param dx
///   Buffer to store horizontal gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
///
/// @param dy
///   Buffer to store vertical gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelPlanars16( const uint8_t* __restrict  src,
                                unsigned int               srcWidth,
                                unsigned int               srcHeight,
                                unsigned int               srcStride,
                                int16_t* __restrict        dx,
                                int16_t* __restrict        dy);

//------------------------------------------------------------------------------
/// @brief
///   Creates a 2D gradient image from source luminance data. This function computes
///   central differences on 3x3 neighborhood and then convolves the result with Sobel
///   kernel
///   \n
///   \n      [ -1 0 +1 ]              [ -1 -2 -1 ]
///   \n dx = [ -2 0 +2 ] * src   dy = [  0  0  0 ] * src
///   \n      [ -1 0 +1 ]              [ +1 +2 +1 ]
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvImageGradientSobelPlanars16() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvImageGradientSobelPlanars16,
///   \a fcvImageGradientSobelPlanars16_v2 will be removed, and the current signature
///   for \a fcvImageGradientSobelPlanars16 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvImageGradientSobelPlanars16 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
///  @param dx
///   Buffer to store horizontal gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
///
/// @param dy
///   Buffer to store vertical gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
/// 
/// @param dxyStride
///   Stride (in bytes) of 'dx' and 'dy' gradient arrays.
///   \n\bWARNING: must be multiple of 16 (8 * 2-bytes per gradient value).
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelPlanars16_v2( const uint8_t* __restrict  src,
                                   unsigned int               srcWidth,
                                   unsigned int               srcHeight,
                                   unsigned int               srcStride,
                                   int16_t* __restrict        dx,
                                   int16_t* __restrict        dy,
                                   unsigned int               dxyStride );


//------------------------------------------------------------------------------
/// @brief
///   Creates a 2D gradient image from source luminance data. This function computes
///   central differences on 3x3 neighborhood and then convolves the result with Sobel
///   kernel. The output is in interleaved format (i.e.) [dx][dy][dx][dy]....
///   \n
///   \n      [ -1 0 +1 ]              [ -1 -2 -1 ]
///   \n dx = [ -2 0 +2 ] * src   dy = [  0  0  0 ] * src
///   \n      [ -1 0 +1 ]              [ +1 +2 +1 ]
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvImageGradientSobelInterleaveds16_v2(). In the 2.0.0 release, 
///   fcvImageGradientSobelInterleaveds16_v2 will be renamed to fcvImageGradientSobelInterleaveds16
///   and the signature of fcvImageGradientSobelInterleaveds16 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param gradients
///   Buffer to store horizontal and vertical gradient. Must be
///   (width-2)*(height-2) *2 in size.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelInterleaveds16( const uint8_t* __restrict  src,
                                     unsigned int               srcWidth,
                                     unsigned int               srcHeight,
                                     unsigned int               srcStride,
                                     int16_t* __restrict        gradients );

//------------------------------------------------------------------------------
/// @brief
///   Creates a 2D gradient image from source luminance data. This function computes
///   central differences on 3x3 neighborhood and then convolves the result with Sobel
///   kernel. The output is in interleaved format (i.e.) [dx][dy][dx][dy]....
///   \n
///   \n      [ -1 0 +1 ]              [ -1 -2 -1 ]
///   \n dx = [ -2 0 +2 ] * src   dy = [  0  0  0 ] * src
///   \n      [ -1 0 +1 ]              [ +1 +2 +1 ]
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvImageGradientSobelInterleaveds16() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvImageGradientSobelInterleaveds16,
///   \a fcvImageGradientSobelInterleaveds16_v2 will be removed, and the current signature
///   for \a fcvImageGradientSobelInterleaveds16 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvImageGradientSobelInterleaveds16 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param gradients
///   Buffer to store horizontal and vertical gradient. Must be
///   (width-2)*(height-2) *2 in size.
/// 
/// @param gradStride
///   Stride (in bytes) of the interleaved gradients array.
///   \n\b WARNING: must be multiple of 16 ( 8 * 2-byte values ).
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelInterleaveds16_v2( const uint8_t* __restrict  src,
                                        unsigned int               srcWidth,
                                        unsigned int               srcHeight,
                                        unsigned int               srcStride,
                                        int16_t* __restrict        gradients,
                                        unsigned int               gradStride );


//------------------------------------------------------------------------------
/// @brief
///   Creates a 2D gradient image from source luminance data. This function computes
///   central differences on 3x3 neighborhood and then convolves the result with Sobel
///   kernel. The output is in interleaved format (i.e.) [dx][dy][dx][dy]....
///   \n
///   \n      [ -1 0 +1 ]              [ -1 -2 -1 ]
///   \n dx = [ -2 0 +2 ] * src   dy = [  0  0  0 ] * src
///   \n      [ -1 0 +1 ]              [ +1 +2 +1 ]
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvImageGradientSobelInterleavedf32_v2(). In the 2.0.0 release, 
///   fcvImageGradientSobelInterleavedf32_v2 will be renamed to fcvImageGradientSobelInterleavedf32
///   and the signature of fcvImageGradientSobelInterleavedf32 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param gradients
///   Buffer to store horizontal and vertical gradient. Must be
///   (width-2)*(height-2) *2 in size.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelInterleavedf32( const uint8_t* __restrict src,
                                     unsigned int              srcWidth,
                                     unsigned int              srcHeight,
                                     unsigned int              srcStride,
                                     float* __restrict         gradients);

//------------------------------------------------------------------------------
/// @brief
///   Creates a 2D gradient image from source luminance data. This function computes
///   central differences on 3x3 neighborhood and then convolves the result with Sobel
///   kernel. The output is in interleaved format (i.e.) [dx][dy][dx][dy]....
///   \n
///   \n      [ -1 0 +1 ]              [ -1 -2 -1 ]
///   \n dx = [ -2 0 +2 ] * src   dy = [  0  0  0 ] * src
///   \n      [ -1 0 +1 ]              [ +1 +2 +1 ]
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvImageGradientSobelInterleavedf32() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvImageGradientSobelInterleavedf32,
///   \a fcvImageGradientSobelInterleavedf32_v2 will be removed, and the current signature
///   for \a fcvImageGradientSobelInterleavedf32 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvImageGradientSobelInterleavedf32 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param gradients
///   Buffer to store horizontal and vertical gradient. Must be
///   (width-2)*(height-2) *2 in size.
/// 
/// @param gradStride
///   Stride (in bytes) of the interleaved gradients array.
///   \n\b WARNING: must be multiple of 32 ( 8 * 4-byte values ).
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelInterleavedf32_v2( const uint8_t* __restrict src,
                                        unsigned int              srcWidth,
                                        unsigned int              srcHeight,
                                        unsigned int              srcStride,
                                        float* __restrict         gradients,
                                        unsigned int              gradStride);


//------------------------------------------------------------------------------
/// @brief
///   Creates a 2D gradient image from source luminance data. This function
///   computes central differences on 3x3 neighborhood and then convolves the
///   result with Sobel kernel
///   \n
///   \n      [ -1 0 +1 ]              [ -1 -2 -1 ]
///   \n dx = [ -2 0 +2 ] * src   dy = [  0  0  0 ] * src
///   \n      [ -1 0 +1 ]              [ +1 +2 +1 ]
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvFilterGaussian3x3u8_v2(). In the 2.0.0 release, 
///   fcvImageGradientSobelPlanars8_v2 will be renamed to fcvImageGradientSobelPlanars8
///   and the signature of fcvImageGradientSobelPlanars8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
///  @param dx
///   Buffer to store horizontal gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
///
/// @param dy
///   Buffer to store vertical gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelPlanars8( const uint8_t* __restrict src,
                               unsigned int              srcWidth,
                               unsigned int              srcHeight,
                               unsigned int              srcStride,
                               int8_t* __restrict        dx,
                               int8_t* __restrict        dy);

//------------------------------------------------------------------------------
/// @brief
///   Creates a 2D gradient image from source luminance data. This function
///   computes central differences on 3x3 neighborhood and then convolves the
///   result with Sobel kernel
///   \n
///   \n      [ -1 0 +1 ]              [ -1 -2 -1 ]
///   \n dx = [ -2 0 +2 ] * src   dy = [  0  0  0 ] * src
///   \n      [ -1 0 +1 ]              [ +1 +2 +1 ]
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvImageGradientSobelPlanars8() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvImageGradientSobelPlanars8,
///   \a fcvImageGradientSobelPlanars8_v2 will be removed, and the current signature
///   for \a fcvImageGradientSobelPlanars8 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvImageGradientSobelPlanars8 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
///  @param dx
///   Buffer to store horizontal gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
///
/// @param dy
///   Buffer to store vertical gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
/// 
/// @param dxyStride
///   Stride (in bytes) of 'dx' and 'dy' gradient arrays.
///   \n\bWARNING: must be multiple of 8 (8 * 1-bytes per gradient value).
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvImageGradientSobelPlanars8_v2( const uint8_t* __restrict src,
                                  unsigned int              srcWidth,
                                  unsigned int              srcHeight,
                                  unsigned int              srcStride,
                                  int8_t* __restrict        dx,
                                  int8_t* __restrict        dy,
                                  unsigned int              dxyStride );


//------------------------------------------------------------------------------
/// @brief
///   Creates a 2D gradient image from source luminance data. This function computes
///   central differences on 3x3 neighborhood and then convolves the result with Sobel
///   kernel
///   \n
///   \n      [ -1 0 +1 ]              [ -1 -2 -1 ]
///   \n dx = [ -2 0 +2 ] * src   dy = [  0  0  0 ] * src
///   \n      [ -1 0 +1 ]              [ +1 +2 +1 ]
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvImageGradientSobelPlanarf32_v2(). In the 2.0.0 release, 
///   fcvImageGradientSobelPlanarf32_v2 will be renamed to fcvImageGradientSobelPlanarf32
///   and the signature of fcvImageGradientSobelPlanarf32 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
///  @param dx
///   Buffer to store horizontal gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
///
/// @param dy
///   Buffer to store vertical gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
///
/// @ingroup image_processing
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageGradientSobelPlanarf32( const uint8_t* __restrict  src,
                                unsigned int               srcWidth,
                                unsigned int               srcHeight,
                                unsigned int               srcStride,
                                float*                     dx,
                                float*                     dy);

//------------------------------------------------------------------------------
/// @brief
///   Creates a 2D gradient image from source luminance data. This function computes
///   central differences on 3x3 neighborhood and then convolves the result with Sobel
///   kernel
///   \n
///   \n      [ -1 0 +1 ]              [ -1 -2 -1 ]
///   \n dx = [ -2 0 +2 ] * src   dy = [  0  0  0 ] * src
///   \n      [ -1 0 +1 ]              [ +1 +2 +1 ]
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvImageGradientSobelPlanarf32() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvImageGradientSobelPlanarf32,
///   \a fcvImageGradientSobelPlanarf32_v2 will be removed, and the current signature
///   for \a fcvImageGradientSobelPlanarf32 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvImageGradientSobelPlanarf32 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
///  @param dx
///   Buffer to store horizontal gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
///
/// @param dy
///   Buffer to store vertical gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
/// 
/// @param dxyStride
///   Stride (in bytes) of 'dx' and 'dy' gradient arrays.
///   \n\bWARNING: must be multiple of 32 (8 * 4-bytes per gradient value).
///
/// @ingroup image_processing
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageGradientSobelPlanarf32_v2( const uint8_t* __restrict  src,
                                   unsigned int               srcWidth,
                                   unsigned int               srcHeight,
                                   unsigned int               srcStride,
                                   float*                     dx,
                                   float*                     dy,
                                   unsigned int               dxyStride );


//------------------------------------------------------------------------------
/// @brief
///   Creates a 2D gradient image from source luminance data. This function computes
///   central differences on 3x3 neighborhood and then convolves the result with Sobel
///   kernel
///   \n
///   \n      [ -1 0 +1 ]              [ -1 -2 -1 ]
///   \n dx = [ -2 0 +2 ] * src   dy = [  0  0  0 ] * src
///   \n      [ -1 0 +1 ]              [ +1 +2 +1 ]
///   
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvImageGradientSobelPlanarf32f32_v2(). In the 2.0.0 release, 
///   fcvImageGradientSobelPlanarf32f32_v2 will be renamed to fcvImageGradientSobelPlanarf32f32
///   and the signature of fcvImageGradientSobelPlanarf32f32 as it appears now, 
///   will be removed.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
///  @param dx
///   Buffer to store horizontal gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
///
/// @param dy
///   Buffer to store vertical gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
///
/// @ingroup image_processing
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageGradientSobelPlanarf32f32( const float * __restrict  src,
                                   unsigned int              srcWidth,
                                   unsigned int              srcHeight,
                                   unsigned int              srcStride,
                                   float*                    dx,
                                   float*                    dy);


//------------------------------------------------------------------------------
/// @brief
///   Creates a 2D gradient image from source luminance data. This function computes
///   central differences on 3x3 neighborhood and then convolves the result with Sobel
///   kernel
///   \n
///   \n      [ -1 0 +1 ]              [ -1 -2 -1 ]
///   \n dx = [ -2 0 +2 ] * src   dy = [  0  0  0 ] * src
///   \n      [ -1 0 +1 ]              [ +1 +2 +1 ]
/// 
///   \n\b ATTENTION: This function is a duplication of of 
///   fcvImageGradientSobelPlanarf32f32()() with the addition of extra parameters.
///   This function has been added to allow for backward compatibility
///   with the original function.  When the 2.0.0 release of this library
///   is made, this function will be renamed to: \a fcvImageGradientSobelPlanarf32f32(),
///   \a fcvImageGradientSobelPlanarf32f32_v2 will be removed, and the current signature
///   for \a fcvImageGradientSobelPlanarf32f32 will be removed.  Until 2.0.0, the 
///   developer should use this implementation with the expectation of
///   renaming it to \a fcvImageGradientSobelPlanarf32f32 when transitioning to 2.0.0.
///   \n\n
///
/// @param src
///   Input image/patch.
///
/// @param srcWidth
///   Width of src data to create gradient.
///
/// @param srcHeight
///   Height of src data to create gradient.
///
/// @param srcStride
///   Stride (in bytes) of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///   \n\bWARNING: must be multiple of 32 (8 * 4-bytes).
///
///  @param dx
///   Buffer to store horizontal gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
///
/// @param dy
///   Buffer to store vertical gradient. Must be (width)*(height) in size.
///   \n\b NOTE: a border of 1 pixel in size on top, bottom, left, and right
///   contains undefined values
/// 
/// @param dxyStride
///   Stride (in bytes) of 'dx' and 'dy' gradient arrays.
///   \n\bWARNING: must be multiple of 32 (8 * 4-bytes per gradient value).
///
/// @ingroup image_processing
//------------------------------------------------------------------------------
FASTCV_API void
fcvImageGradientSobelPlanarf32f32_v2( const float * __restrict  src,
                                      unsigned int              srcWidth,
                                      unsigned int              srcHeight,
                                      unsigned int              srcStride,
                                      float*                    dx,
                                      float*                    dy,
                                      unsigned int              dxyStride );


//------------------------------------------------------------------------------
/// @brief
///   Block Optical Flow 16x16 - Tracks all 16x16 blocks in the Region of Interest
///   (ROI) from Source-1 to Source-2. Generates Motion Vectors for blocks where
///   motion is detected.
/// 
/// @details
///   
/// @param[in] src1
///   Pointer to source image where the original blocks are present. 
///   \n Dimensions should be same as \a src2, and equal to \a srcWidth, 
///   \a srcHeight, \a srcStride.
///   \n\b WARNING: must be 128-bit aligned.
/// 
/// @param[in] src2
///   Pointer to second source image where motion vectors for blocks in \a img1
///   are to be located. 
///   \n Dimensions should be same as \a src1, and equal to \a srcWidth, 
///   \a srcHeight, \a srcStride.
///   \n\b WARNING: must be 128-bit aligned.
/// 
/// @param[in] srcWidth
///   Width of source images pointed by \a src1 and \a src2.
/// 
/// @param[in] srcHeight
///   Height of source images pointed by \a src1 and \a src2.
/// 
/// @param[in] srcStride
///   Stride of source images pointed by \a src1 and \a src2.
/// 
/// @param[in] roiLeft
///   Left co-ordinates of Region-of-Interest (ROI).
/// 
/// @param[in] roiTop
///   Top co-orgdinates of Region-of-Interest (ROI).
/// 
/// @param[in] roiRight
///   Right co-ordinates of Region-of-Interest (ROI).
/// 
/// @param[in] roiBottom
///   Bottom co-ordinates of Region-of-Interest (ROI).
/// 
/// @param[in] shiftSize
///   Distance in number of pixels (both horizontally and vertically) between
///   consecutive blocks for which motion vector is searched.
///   \n\b NOTE: Larger the value, less number of blocks will be tracked, and 
///   hence the function will run faster.
/// 
/// @param[in] searchWidth
///   Numbers of pixels horizontally on left and right of the source block where a
///   match is searched for. For example, if searchWidth is 8 and searchHeight 
///   is 8, then the search area for any given block will be 32x32 around
///   the location of that block.
/// 
/// @param[in] searchHeight
///   Numbers of pixels vertically on top and bottom of the source block where a
///   match is searched for. For example, if searchWidth is 8 and searchHeight 
///   is 8, then the search area for any given block will be 32x32 around
///   the location of that block.
/// 
/// @param[in] searchStep
///   Distance in number of pixels between consecutive search targets within
///   the search window.
///   \n\b NOTE: Larger the value, more coarse the search will be and thus 
///   will make the fucntion run faster. Smaller the value, more dense the 
///   search will be, making the funciton run slower. 
/// 
/// @param[in] usePrevious
///   Indicates if the function should use the existing motion vectors in 
///   locX and locY as the starting point for motion vector search.
/// 
/// @param[out] numMv
///   Pointer to variable that will store the count of Motion Vectors 
///   generated by the function.
///   \n\b WARNING: This pointer should be Non-NULL.
/// 
/// @param[out] locX
///   Pointer to an array which will store the X co-ordinates of the
///   original Block for which a Motion Vector is generated.
///   \n\b NOTE: The array will contain \a numMv valid entries.
///   \n\b WARNING: This pointer should be Non-NULL, and the array size should 
///   be >= number of 16x16 blocks in ROI.
///
/// @param[out] locY
///   Pointer to an array which will store the Y co-ordinates of the
///   original Block for which a Motion Vector is generated.
///   \n\b NOTE: The array will contain \a numMv valid entries.
///   \n\b WARNING: This pointer should be Non-NULL, and the array size should 
///   be >= number of 16x16 blocks in ROI.
///
/// @param[out] mvX
///   Pointer to an array which will store the X co-ordinates of the block in \a src2 
///   corresponding block in \a src1. (\a mvX[i]-\a locX[i]) will give the motion
///   vector for the block in \a src1.
///   \n\b NOTE: The array will contain \a numMv valid entries.
///   \n\b WARNING: This pointer should be Non-NULL, and the array size should 
///   be >= number of 16x16 blocks in ROI.
///
/// @param[out] mvY
///   Pointer to an array which will store the Y co-ordinates of the block in \a src2 
///   corresponding block in \a src1. (\a mvY[i]-\a locY[i]) will give the motion
///   vector for the block in \a src1.
///   \n\b NOTE: The array will contain \a numMv valid entries.
///   \n\b WARNING: This pointer should be Non-NULL, and the array size should 
///   be >= number of 16x16 blocks in ROI.
/// 
/// @return
///    0 - Success, Failure otherwise.
/// 
/// @ingroup object_detection
//------------------------------------------------------------------------------
FASTCV_API int
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
                              uint32_t *                  mvY);

//------------------------------------------------------------------------------
/// @brief
///   Performs per-element bitwise-OR operation on two 8-bit single channel images. 
///   Two images should have the same size. dst(I)=src1(I) V src2(I) if mask(I) is not zero.
///
/// @param src1
///   Pointer to the 8-bit source image 1.
///
/// @param src2
///   Pointer to the 8-bit source image 2.
///
/// @param srcWidth
///   Width of source images pointed by src1 and src2.
///
/// @param srcHeight
///   Height of source images pointed by src1 and src2.
///
/// @param srcStride
///   Stride of source images (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param dst
///    Pointer to the 8-bit destination image.
///
/// @param dstStride
///   Stride of destination image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param mask
///   Pointer to the 8-bit single channel mask. It specifies elements of the destination array to be changed.
///   The mask is optional. If there is no mask, the value is NULL.
///
/// @param maskStride
///   Stride of the mask (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   If there is no mask, the value is 0.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void fcvBitwiseOru8(const uint8_t* __restrict src1, 
								const uint8_t* __restrict src2,  
								uint32_t                  srcWidth,
								uint32_t                  srcHeight,
								uint32_t                  srcStride,
								uint8_t * __restrict      dst,
								uint32_t                  dstStride,
								uint8_t * __restrict      mask,
								uint32_t                  maskStride );

//------------------------------------------------------------------------------
/// @brief
///   Performs per-element bitwise-OR operation on two 32-bit single channel images. 
///   Two images should have the same size. dst(I)=src1(I) V src2(I) if mask(I) is not zero.
///
/// @param src1
///   Pointer to the 32-bit source image 1.
///
/// @param src2
///   Pointer to the 32-bit source image 2.
///
/// @param srcWidth
///   Width of source images pointed by src1 and src2.
///
/// @param srcHeight
///   Height of source images pointed by src1 and src2.
///
/// @param srcStride
///   Stride of source images (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param dst
///    Pointer to the 8-bit destination image.
///
/// @param dstStride
///   Stride of destination image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param mask
///   Pointer to the 8-bit single channel mask. It specifies elements of the destination array to be changed.
///   The mask is optional. If there is no mask, the value is NULL.
///
/// @param maskStride
///   Stride of the mask (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   If there is no mask, the value is 0.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void fcvBitwiseOrs32(const int32_t* __restrict src1, 
								 const int32_t* __restrict src2,  
								 uint32_t                  srcWidth,
								 uint32_t                  srcHeight,
								 uint32_t                  srcStride,
								 int32_t * __restrict      dst,
								 uint32_t                  dstStride,
								 uint8_t * __restrict      mask,
								 uint32_t                  maskStride);


//------------------------------------------------------------------------------
/// @brief
///   Converts an image from RGB space to grayscale
///
/// @details
///   
/// @param src
///   Source 8-bit image, BGR888 format (R is lowest byte for the pixel)
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Source image width.
///   \n\b NOTE: must be a multiple of 8.
///
/// @param srcHeight
///   Source image height.
///
/// @param srcStride
///   Stride of source image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   If set to 0, srcStride=srcWidth as default
///
/// @param dst
///   Destination 8-bit gray-scale image.
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dstStride
///   Stride of destination image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   If set to 0, dstStride=srcStride as default
///
/// @test
///   -# 
///   -# 
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------
FASTCV_API void
fcvColorRGB888ToGrayu8( const uint8_t* __restrict src,
                     uint32_t srcWidth,
                     uint32_t srcHeight,
                     uint32_t srcStride,
                     uint8_t* __restrict dst,
                     uint32_t  dstStride);


//------------------------------------------------------------------------------
/// @brief
///   Integral of the image tilted by 45 degrees
///
/// @details
///   Calculates the tilted integral image of an input image
///   and adds an zero-filled border on top. Left border not zero.
///   dst[i,j]=sum (src[m,n]), where n<j, abs(m-i+1) <= j-n-1
///
/// @param src
///   Source image, single channel, unsigned char type
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param srcWidth
///   Image width.
///
/// @param srcHeight
///   Image height.
///
/// @param srcStride
///   Stride of source image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   If set to 0, srcStride is srcWidth in bytes as default
///
/// @param dst
///   Destination image of size (srcWidth+1)*(srcHeight+1)
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dstStride
///   Stride of destination image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///
/// @test
///   -# 
///   -# 
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void fcvTiltedIntegralsu8s32( const uint8_t* __restrict src,
          						        uint32_t 			 srcWidth,
                             			uint32_t 			srcHeight,
                             			uint32_t 			srcStride,
                             			int32_t* __restrict 	  dst,
                             			uint32_t 			dstStride);

//------------------------------------------------------------------------------
/// @brief
///   Performs a valid convolution of two images
///
/// @details
///   This function does convolution of two images. Similar to 1-D convolution,
///   one image is flipped and slided through the other image to compute the output.
///   Values are computed for the region where one image is completely within the other image.
///   
/// @param src1
///   First source image of int16 type
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param src1Width
///   Image width.
///
/// @param src1Height
///   Image height.
///
/// @param src1Stride
///   Stride of source image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   If set to 0, srcStride is srcWidth in bytes as default
///
/// @param src2
///   Second source image of int16 type
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param src2Width
///   Image width.
///   Must meet this condition: src2Width <= src1Width
///
/// @param src2Height
///   Image height.
///   Must meet this condition: src2Height <= src1Height
///
/// @param src2Stride
///   Stride of source images (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///   If set to 0, srcStride is src2Width in bytes as default
///
/// @param dst
///   Destination image of int32 type.
///   Size of destination is (src1Width-src2Width+1) x (src1Height-src2Height+1)
///   \n\b WARNING: must be 128-bit aligned.
///
/// @param dstStride
///   Stride of destination image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///
/// @test
///   -# 
///   -# 
///
/// @ingroup image_processing
//------------------------------------------------------------------------------
FASTCV_API void
fcvConvValids16( const int16_t* __restrict src1,
                 uint32_t src1Width,
                 uint32_t src1Height,
                 uint32_t src1Stride,
                 const int16_t* __restrict src2,
                 uint32_t src2Width,
                 uint32_t src2Height,
                 uint32_t src2Stride,
                 int32_t* __restrict dst,
                 uint32_t dstStride);



#ifdef __cplusplus
extern "C"
{
#endif

//------------------------------------------------------------------------------
/// @brief
///   Retrieves version of FastCV library.
///
/// @param version
///   Pointer to location to put string.
///
/// @param versionLength
///   Length of storage for version string.
///
/// @ingroup misc
//------------------------------------------------------------------------------

FASTCV_API void
fcvGetVersion( char*        version,
               unsigned int versionLength );


//---------------------------------------------------------------------------
/// @brief
///   Selects HW units for all routines at run-time.  Can be called anytime to
///   update choice.
///
/// @param choice
///   See enum for details.
///
/// @return
///   0 if successful.
///   999 if minmum HW requirement not met.
///   other #'s if unsuccessful.
///
/// @ingroup misc
//---------------------------------------------------------------------------

FASTCV_API int
fcvSetOperationMode( fcvOperationMode mode );


//---------------------------------------------------------------------------
/// @brief
///    Clean up FastCV resources. Must be called before the program exits.
///
/// @ingroup misc
//---------------------------------------------------------------------------

FASTCV_API void
fcvCleanUp( void );

// -----------------------------------------------------------------------------
/// @brief
///   Allocates memory for Pyramid
///
/// @param pyr
///   Pointer to an array of qcvaPyramidLevel
///
/// @param baseWidth
///   Width of the base level: the value assigned to pyr[0].width
///
/// @param baseHeight
///   Height of the base level: the value assigned to pyr[0].height
///
/// @param bytesPerPixel
///   Number of bytes per pixel:
///   \n e.g for uint8_t pyramid,  bytesPerPixel = 1
///   \n for int32_t pyramid, bytesPerPixel = 4
///
/// @param numLevels
///   number of levels in the pyramid
///
/// @param allocateBase
///   \n if set to 1, memory will be allocated for the base level
///   \n if set to 0, memory for the base level is allocated by the callee
///
/// @ingroup mem_management
//----------------------------------------------------------------------------

FASTCV_API int
fcvPyramidAllocate( fcvPyramidLevel* pyr,
                    unsigned int     baseWidth,
                    unsigned int     baseHeight,
                    unsigned int     bytesPerPixel,
                    unsigned int     numLevels,
                    int              allocateBase );


// -----------------------------------------------------------------------------
/// @brief
///   Deallocates an array of fcvPyramidLevel. Can be used for any
///   type(f32/s8/u8).
///
/// @param pyr
///   pyramid to deallocate
///
/// @param numLevels
///   number of levels in the pyramid
///
/// @ingroup mem_management
//----------------------------------------------------------------------------

FASTCV_API void
fcvPyramidDelete( fcvPyramidLevel* pyr,
                  unsigned int     numLevels,
                  unsigned int     startLevel );


//------------------------------------------------------------------------------
/// @brief
///    Allocates aligned memory.
///
/// @param nBytes
///    Number of bytes.
///
/// @param byteAlignment
///    Alignment specified in bytes (e.g., 16 = 128-bit alignment).
///    \n\b WARNING: must be < 255 bytes
///
/// @return
///    SUCCESS: pointer to aligned memory
///    FAILURE: 0
///
/// @ingroup mem_management
//------------------------------------------------------------------------------

FASTCV_API void*
fcvMemAlloc( unsigned int nBytes,
             unsigned int byteAlignment );


//------------------------------------------------------------------------------
/// @brief
///    Frees memory allocated by fcvMemAlloc().
///
/// @param ptr
///    Pointer to memory.
///
/// @ingroup mem_management
//------------------------------------------------------------------------------

FASTCV_API void
fcvMemFree( void* ptr );


//---------------------------------------------------------------------------
/// @brief
///  Function to find the bounding rectangle of a set of points.
///
/// 
/// @param [in] xy              Set of points (x,y) for which the bounding rectangle has to be found. 
///                             The points are expressed in interleaved format:  [x1,y1,x2,y2....] 
/// @param [in] numPoints       Number of points in the array.
/// @param [out] rectTopLeftX  Lower left's X value for the rectangle.
/// @param [out] rectTopLeftX  Lower Left's Y value for the rectangle;
/// @param [out] rectWidth       Width of the rectangle.
/// @param [out] rectHeight      Height of the rectangle.
/// 
/// @return FASTCV_API void      
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------
FASTCV_API void fcvBoundingRectangle (const uint32_t * __restrict xy, uint32_t numPoints, 
                                      uint32_t * rectTopLeftX, uint32_t * rectTopLeftY,
                                      uint32_t * rectWidth, uint32_t *rectHeight);
                                                                                    

//------------------------------------------------------------------------------
/// @brief
///   Performs vertical upsampling on input Chroma data 
///
/// @details
///   This function performs vertical 1:2 upsampling on the input Chroma data.
///   The input shall be non-inteleaved planar Chroma data. The Chroma data
///   can be either Cb component or Cr component.
///   The output height is doubled after upsampling. Caller needs to pass in 
///   the output buffer large enough to hold the upsampled data.
///
/// @param src
///   Input Chroma component, either Cb or Cr
///
/// @param srcWidth
///   Input width in number of Chroma pixels
///
/// @param srcHeight
///   Input height in number of Chroma lines
///
/// @param srcStride
///   Stride of input data (i.e., number of bytes between column 0 of row 0 and
///   column 0 of row 1)
///
/// @param dst
///   Output Chroma data that has been upsampled vertically
///   \n\b WARNING: output height is doubled
///
/// @param dstStride
///   Stride of output data(i.e., number of bytes between column 0 of row 0 and 
///   column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvUpsampleVerticalu8( const uint8_t* __restrict src,
                       uint32_t                  srcWidth,
                       uint32_t                  srcHeight,
                       uint32_t                  srcStride,
                       uint8_t* __restrict       dst,
                       uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Performs horizontal upsampling on input Chroma data
///
/// @details
///   This function performs horizontal 1:2 upsampling on the input Chroma data.
///   The input shall be non-interleaved planar Chroma data. The Chroma data
///   can be either Cb component or Cr component.
///   The output width is doubled after upsampling. Caller needs to pass in 
///   the output buffer large enough to hold the upsampled data.
///
/// @param src
///   Input Chroma component, either Cb or Cr
///
/// @param srcWidth
///   Input width in number of Chroma pixels
///
/// @param srcHeight
///   Input height in number of Chroma lines
///
/// @param srcStride
///   Stride of input data (i.e., number of bytes between column 0 of row 0 and
///   column 0 of row 1)
///
/// @param dst
///   Output Chroma data that has been upsampled horizontally
///   \n\b WARNING: output width is doubled
///
/// @param dstStride
///   Stride of output data(i.e., number of bytes between column 0 of row 0 and 
///   column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvUpsampleHorizontalu8( const uint8_t* __restrict src,
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         uint8_t* __restrict       dst,
                         uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Performs both horizontal and vertical upsampling on input Chroma data
///
/// @details
///   This function performs horizontal 1:2 upsampling and vertical 1:2 
///   upsampling on the input Chroma data.
///   The input shall be non-interleaved planar Chroma data. The Chroma data
///   can be either Cb component or Cr component.
///   The output width and height are doubled after upsampling. Caller needs 
///   to pass in the output buffer large enough to hold the upsampled data.
///
/// @param src
///   Input Chroma component, either Cb or Cr
///
/// @param srcWidth
///   Input width in number of Chroma pixels
///
/// @param srcHeight
///   Input height in number of Chroma lines
///
/// @param srcStride
///   Stride of input data (i.e., number of bytes between column 0 of row 0 and
///   column 0 of row 1)
///
/// @param dst
///   Output Chroma data that has been upsampled both horizontally and vertically
///   \n\b WARNING: both output width and output height are doubled
///
/// @param dstStride
///   Stride of output data(i.e., number of bytes between column 0 of row 0 and 
///   column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvUpsample2Du8( const uint8_t* __restrict src,
                 uint32_t                  srcWidth,
                 uint32_t                  srcHeight,
                 uint32_t                  srcStride,
                 uint8_t* __restrict       dst,
                 uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Performs vertical upsampling on input interleaved Chroma data 
///
/// @details
///   This function performs vertical 1:2 upsampling on the input 
///   interleaved Chroma data.
///   The input shall be interleaved Chroma data in pairs of CbCr or CrCb. 
///   The output height is doubled after upsampling. Caller needs to pass in 
///   the output buffer large enough to hold the upsampled data.
///
/// @param src
///   Input interleaved Chroma data in pairs of CbCr or CrCb
///
/// @param srcWidth
///   Input width in number of Chroma pairs (CbCr pair or CrCb pair)
///
/// @param srcHeight
///   Input height in number of Chroma lines
///
/// @param srcStride
///   Stride of input data (i.e., number of bytes between column 0 of row 0 and
///   column 0 of row 1)
///
/// @param dst
///   Output Chroma data that has been upsampled vertically
///   \n\b WARNING: output height is doubled
///
/// @param dstStride
///   Stride of output data(i.e., number of bytes between column 0 of row 0 and 
///   column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvUpsampleVerticalInterleavedu8( const uint8_t* __restrict src,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Performs horizontal upsampling on input interleaved Chroma data
///
/// @details
///   This function performs horizontal 1:2 upsampling on the input 
///   interleaved Chroma data.
///   The input shall be interleaved Chroma data in pairs of CbCr or CrCb. 
///   The output width is doubled after upsampling. Caller needs to pass in 
///   the output buffer large enough to hold the upsampled data.
///
/// @param src
///   Input interleaved Chroma data in pairs of CbCr or CrCb
///
/// @param srcWidth
///   Input width in number of Chroma pairs (CbCr pair or CrCb pair)
///
/// @param srcHeight
///   Input height in number of Chroma lines
///
/// @param srcStride
///   Stride of input data (i.e., number of bytes between column 0 of row 0 and
///   column 0 of row 1)
///
/// @param dst
///   Output Chroma data that has been upsampled horizontally
///   \n\b WARNING: output width is doubled
///
/// @param dstStride
///   Stride of output data(i.e., number of bytes between column 0 of row 0 and 
///   column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvUpsampleHorizontalInterleavedu8( const uint8_t* __restrict src,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcStride,
                                    uint8_t* __restrict       dst,
                                    uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Performs both horizontal and vertical upsampling on input interleaved 
///   Chroma data
///
/// @details
///   This function performs horizontal 1:2 upsampling and vertical 1:2 
///   upsampling on the input interleaved Chroma data.
///   The input shall be interleaved Chroma data in pairs of CbCr or CrCb. 
///   The output width and height are doubled after upsampling. Caller needs 
///   to pass in the output buffer large enough to hold the upsampled data.
///
/// @param src
///   Input interleaved Chroma data in pairs of CbCr or CrCb
///
/// @param srcWidth
///   Input width in number of Chroma pairs (CbCr pair or CrCb pair)
///
/// @param srcHeight
///   Input height in number of Chroma lines
///
/// @param srcStride
///   Stride of input data (i.e., number of bytes between column 0 of row 0 and
///   column 0 of row 1)
///
/// @param dst
///   Output Chroma data that has been upsampled both horizontally and vertically
///   \n\b WARNING: both output width and output height are doubled
///
/// @param dstStride
///   Stride of output data(i.e., number of bytes between column 0 of row 0 and 
///   column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvUpsample2DInterleavedu8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstStride );
                                            


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCbCr444 to RGB565  
///
/// @details
///   This function performs color space conversion from YCbCr444 to RGB565.
///
///   The input are three separated Y, Cb and Cr planes:
///   Y plane:  Y0  Y1  Y2  Y3 ...
///   Cb plane: Cb0 Cb1 Cb2 Cb3...
///   Cr plane: Cr0 Cr1 Cr2 Cr3...
///
///   The output is one interleaved RGB565 plane:
///   RGB565 plane: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3...
///
///   RGB565 pixel is arranged with 5-bit Red component, 6-bit Green component,
///   and 5-bit Blue component. One RGB565 pixel is made up of 16-bit data.
///
/// @param srcY
///   Input image Y component
///
/// @param srcCb
///   Input image Cb component
///
/// @param srcCr
///   Input image Cr component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCbStride
///   Stride of input image Cb component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCrStride
///   Stride of input image Cr component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGB565 image 
///   \n\b WARNING: size must match input YCbCr444
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PlanarToRGB565u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCbCr444 to RGB888  
///
/// @details
///   This function performs color space conversion from YCbCr444 to RGB888.
///
///   The input are three separated Y, Cb and Cr planes:
///   Y plane:  Y0  Y1  Y2  Y3 ...
///   Cb plane: Cb0 Cb1 Cb2 Cb3...
///   Cr plane: Cr0 Cr1 Cr2 Cr3...
///
///   The output is one interleaved RGB888 plane:
///   RGB888 plane: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3...
///
///   RGB888 pixel is arranged with 8-bit Red component, 8-bit Green component,
///   and 8-bit Blue component. One RGB888 pixel is made up of 24-bit data.  
///
/// @param srcY
///   Input image Y component
///
/// @param srcCb
///   Input image Cb component
///
/// @param srcCr
///   Input image Cr component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCbStride
///   Stride of input image Cb component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCrStride
///   Stride of input image Cr component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGB888 image  
///   \n\b WARNING: size must match input YCbCr444
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PlanarToRGB888u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCbCr444 to RGBA8888  
///
/// @details
///   This function performs color space conversion from YCbCr444 to RGBA8888.
///
///   The input are three separated Y, Cb and Cr planes:
///   Y plane:  Y0  Y1  Y2  Y3 ...
///   Cb plane: Cb0 Cb1 Cb2 Cb3...
///   Cr plane: Cr0 Cr1 Cr2 Cr3...
///
///   The output is one interleaved RGBA8888 plane:
///   RGBA8888 plane: R0 G0 B0 A0 R1 G1 B1 A1 R2 G2 B2 A2 R3 G3 B3 A3...
///
///   RGBA8888 pixel is arranged with 8-bit Red component, 8-bit Green component,
///   8-bit Blue component, and 8-bit A component. One RGBA8888 pixel is made 
///   up of 32-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcCb
///   Input image Cb component
///
/// @param srcCr
///   Input image Cr component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCbStride
///   Stride of input image Cb component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCrStride
///   Stride of input image Cr component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGBA8888 image 
///   \n\b WARNING: size must match input YCbCr 444
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                    const uint8_t* __restrict srcCb,
                                    const uint8_t* __restrict srcCr,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcYStride,
                                    uint32_t                  srcCbStride,
                                    uint32_t                  srcCrStride,
                                    uint8_t* __restrict       dst,
                                    uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCbCr422 to RGB565  
///
/// @details
///   This function performs color space conversion from YCbCr422 to RGB565.
///
///   The input are three separated Y, Cb and Cr planes, with horizontally
///   sub-sampled Cb and Cr planes:
///   Y plane                          : Y0  Y1  Y2  Y3 ...
///   Horizontally sub-sampled Cb plane:   Cb0     Cb1  ...
///   Horizontally sub-sampled Cr plane:   Cr0     Cr1  ...
///
///   The output is one interleaved RGB565 plane:
///   RGB565 plane: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3...
///
///   RGB565 pixel is arranged with 5-bit Red component, 6-bit Green component,
///   and 5-bit Blue component. One RGB565 pixel is made up of 16-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcCb
///   Input image Cb component that has been sub-sampled horizontally
///
/// @param srcCr
///   Input image Cr component that has been sub-sampled horizontally
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCbStride
///   Stride of input image Cb component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCrStride
///   Stride of input image Cr component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGB565 image 
///   \n\b WARNING: size must match input YCbCr422
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PlanarToRGB565u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCbCr422 to RGB888  
///
/// @details
///   This function performs color space conversion from YCbCr422 to RGB888.
///
///   The input are three separated Y, Cb and Cr planes, with horizontally
///   sub-sampled Cb and Cr planes:
///   Y plane                          : Y0  Y1  Y2  Y3 ...
///   Horizontally sub-sampled Cb plane:   Cb0     Cb1  ...
///   Horizontally sub-sampled Cr plane:   Cr0     Cr1  ...
///
///   The output is one interleaved RGB888 plane:
///   RGB888 plane: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3...
///
///   RGB888 pixel is arranged with 8-bit Red component, 8-bit Green component,
///   and 8-bit Blue component. One RGB888 pixel is made up of 24-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcCb
///   Input image Cb component
///
/// @param srcCr
///   Input image Cr component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCbStride
///   Stride of input image Cb component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCrStride
///   Stride of input image Cr component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGB888 image  
///   \n\b WARNING: size must match input YCbCr422
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PlanarToRGB888u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCbCr422 to RGBA8888  
///
/// @details
///   This function performs color space conversion from YCbCr422 to RGBA8888.
///
///   The input are three separated Y, Cb and Cr planes, with horizontally
///   sub-sampled Cb and Cr planes:
///   Y plane                          : Y0  Y1  Y2  Y3 ...
///   Horizontally sub-sampled Cb plane:   Cb0     Cb1  ...
///   Horizontally sub-sampled Cr plane:   Cr0     Cr1  ...
///
///   The output is one interleaved RGBA8888 plane:
///   RGBA8888 plane: R0 G0 B0 A0 R1 G1 B1 A1 R2 G2 B2 A2 R3 G3 B3 A3...
///
///   RGBA8888 pixel is arranged with 8-bit Red component, 8-bit Green component,
///   8-bit Blue component, and 8-bit A component. One RGBA8888 pixel is made 
///   up of 32-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcCb
///   Input image Cb component
///
/// @param srcCr
///   Input image Cr component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCbStride
///   Stride of input image Cb component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCrStride
///   Stride of input image Cr component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGBA8888 image  
///   \n\b WARNING: size must match input YCbCr422
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                    const uint8_t* __restrict srcCb,
                                    const uint8_t* __restrict srcCr,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcYStride,
                                    uint32_t                  srcCbStride,
                                    uint32_t                  srcCrStride,
                                    uint8_t* __restrict       dst,
                                    uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCbCr420 to RGB565  
///
/// @details
///   This function performs color space conversion from YCbCr420 to RGB565.
///
///   The input are three separated Y, Cb and Cr planes, with horizontally
///   and vertically (2D) sub-sampled Cb and Cr planes:
///   Y plane                : Y00  Y01  Y02  Y03 ...
///                            Y10  Y11  Y12  Y13 ... 
///   2D sub-sampled Cb plane:    Cb0     Cb1     ...
///   2D sub-sampled Cr plane:    Cr0     Cr1     ...
///
///   The output is one interleaved RGB565 plane:
///   RGB565 plane: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3...
///
///   RGB565 pixel is arranged with 5-bit Red component, 6-bit Green component,
///   and 5-bit Blue component. One RGB565 pixel is made up of 16-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcCb
///   Input image Cb component
///
/// @param srcCr
///   Input image Cr component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCbStride
///   Stride of input image Cb component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCrStride
///   Stride of input image Cr component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGB565 image 
///   \n\b WARNING: size must match input YCbCr420
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PlanarToRGB565u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCbCr420 to RGB888  
///
/// @details
///   This function performs color space conversion from YCbCr420 to RGB888.
///
///   The input are three separated Y, Cb and Cr planes, with horizontally
///   and vertically (2D) sub-sampled Cb and Cr planes:
///   Y plane                : Y00  Y01  Y02  Y03 ...
///                            Y10  Y11  Y12  Y13 ... 
///   2D sub-sampled Cb plane:    Cb0     Cb1     ...
///   2D sub-sampled Cr plane:    Cr0     Cr1     ...
///
///   The output is one interleaved RGB888 plane:
///   RGB888 plane: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3...
///
///   RGB888 pixel is arranged with 8-bit Red component, 8-bit Green component,
///   and 8-bit Blue component. One RGB888 pixel is made up of 24-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcCb
///   Input image Cb component
///
/// @param srcCr
///   Input image Cr component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCbStride
///   Stride of input image Cb component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCrStride
///   Stride of input image Cr component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGB888 image 
///   \n\b WARNING: size must match input YCbCr420
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PlanarToRGB888u8( const uint8_t* __restrict srcY,
                                  const uint8_t* __restrict srcCb,
                                  const uint8_t* __restrict srcCr,
                                  uint32_t                  srcWidth,
                                  uint32_t                  srcHeight,
                                  uint32_t                  srcYStride,
                                  uint32_t                  srcCbStride,
                                  uint32_t                  srcCrStride,
                                  uint8_t* __restrict       dst,
                                  uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from YCbCr420 to RGBA8888  
///
/// @details
///   This function performs color space conversion from YCbCr420 to RGBA8888.
///
///   The input are three separated Y, Cb and Cr planes, with horizontally
///   and vertically (2D) sub-sampled Cb and Cr planes:
///   Y plane                : Y00  Y01  Y02  Y03 ...
///                            Y10  Y11  Y12  Y13 ... 
///   2D sub-sampled Cb plane:    Cb0     Cb1     ...
///   2D sub-sampled Cr plane:    Cr0     Cr1     ...
///
///   The output is one interleaved RGBA8888 plane:
///   RGBA8888 plane: R0 G0 B0 A0 R1 G1 B1 A1 R2 G2 B2 A2 R3 G3 B3 A3...
///
///   RGBA8888 pixel is arranged with 8-bit Red component, 8-bit Green component,
///   8-bit Blue component, and 8-bit A component. One RGBA8888 pixel is made 
///   up of 32-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcCb
///   Input image Cb component
///
/// @param srcCr
///   Input image Cr component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCbStride
///   Stride of input image Cb component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCrStride
///   Stride of input image Cr component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGBA8888 image  
///   \n\b WARNING: size must match input YCbCr 420
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                    const uint8_t* __restrict srcCb,
                                    const uint8_t* __restrict srcCr,
                                    uint32_t                  srcWidth,
                                    uint32_t                  srcHeight,
                                    uint32_t                  srcYStride,
                                    uint32_t                  srcCbStride,
                                    uint32_t                  srcCrStride,
                                    uint8_t* __restrict       dst,
                                    uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from pseudo-planar YCbCr444 to RGB565  
///
/// @details
///   This function performs color space conversion from YCbCr444 to RGB565.
///
///   The input are one Y plane followed by one interleaved CbCr (or CrCb) plane:
///   Y plane          :    Y0      Y1      Y2      Y3   ...
///   Interleaved plane: Cb0 Cr0 Cb1 Cr1 Cb2 Cr2 Cb3 Cr3 ...
///
///   The output is one interleaved RGB565 plane:
///   RGB565 plane: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3...
///
///   RGB565 pixel is arranged with 5-bit Red component, 6-bit Green component,
///   and 5-bit Blue component. One RGB565 pixel is made up of 16-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcC
///   Input image Chroma component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCStride
///   Stride of input image Chroma component (i.e., number of bytes between 
///    column 0 of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGB565 image  
///   \n\b WARNING: size must match input YCbCr444
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PseudoPlanarToRGB565u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from pseudo-planar YCbCr444 to RGB888  
///
/// @details
///   This function performs color space conversion from YCbCr444 to RGB888.
///
///   The input are one Y plane followed by one interleaved CbCr (or CrCb) plane:
///   Y plane          :    Y0      Y1      Y2      Y3   ...
///   Interleaved plane: Cb0 Cr0 Cb1 Cr1 Cb2 Cr2 Cb3 Cr3 ...
///
///   The output is one interleaved RGB888 plane:
///   RGB888 plane: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3...
///
///   RGB888 pixel is arranged with 8-bit Red component, 8-bit Green component,
///   and 8-bit Blue component. One RGB888 pixel is made up of 24-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcC
///   Input image Chroma component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCStride
///   Stride of input image Chroma component (i.e., number of bytes between 
///    column 0 of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGB888 image 
///   \n\b WARNING: size must match input YCbCr444
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PseudoPlanarToRGB888u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from pseudo-planar YCbCr444 to RGBA8888  
///
/// @details
///   This function performs color space conversion from YCbCr444 to RGBA8888.
///
///   The input are one Y plane followed by one interleaved CbCr (or CrCb) plane:
///   Y plane          :    Y0      Y1      Y2      Y3   ...
///   Interleaved plane: Cb0 Cr0 Cb1 Cr1 Cb2 Cr2 Cb3 Cr3 ...
///
///   The output is one interleaved RGBA8888 plane:
///   RGBA8888 plane: R0 G0 B0 A0 R1 G1 B1 A1 R2 G2 B2 A2 R3 G3 B3 A3 ...
///
///   RGBA8888 pixel is arranged with 8-bit Red component, 8-bit Green component,
///   8-bit Blue component, and 8-bit A component. One RGBA8888 pixel is made 
///   up of 32-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcC
///   Input image Chroma component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCStride
///   Stride of input image Chroma component (i.e., number of bytes between 
///    column 0 of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGBA8888 image  
///   \n\b WARNING: size must match input YCbCr444
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr444PseudoPlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                          const uint8_t* __restrict srcC,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCStride,
                                          uint8_t* __restrict       dst,
                                          uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from pseudo-planar YCbCr422 to RGB565  
///
/// @details
///   This function performs color space conversion from YCbCr422 to RGB565.
///
///   The input are one Y plane followed by one interleaved and horizontally 
///   sub-sampled CbCr (or CrCb) plane:
///   Y plane                          : Y0  Y1  Y2  Y3  ...
///   Interleaved and sub-sampled plane: Cb0 Cr0 Cb1 Cr1 ...
///
///   The output is one interleaved RGB565 plane:
///   RGB565 plane: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3...
///
///   RGB565 pixel is arranged with 5-bit Red component, 6-bit Green component,
///   and 5-bit Blue component. One RGB565 pixel is made up of 16-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcC
///   Input image Chroma component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCStride
///   Stride of input image Chroma component (i.e., number of bytes between 
///    column 0 of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGB565 image  
///   \n\b WARNING: size must match input YCbCr 422
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PseudoPlanarToRGB565u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from pseudo-planar YCbCr422 to RGB888  
///
/// @details
///   This function performs color space conversion from YCbCr422 to RGB888.
///
///   The input are one Y plane followed by one interleaved and horizontally 
///   sub-sampled CbCr (or CrCb) plane:
///   Y plane                          : Y0  Y1  Y2  Y3  ...
///   Interleaved and sub-sampled plane: Cb0 Cr0 Cb1 Cr1 ...
///
///   The output is one interleaved RGB888 plane:
///   RGB888 plane: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3...
///
///   RGB888 pixel is arranged with 8-bit Red component, 8-bit Green component,
///   and 8-bit Blue component. One RGB888 pixel is made up of 24-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcC
///   Input image Chroma component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCStride
///   Stride of input image Chroma component (i.e., number of bytes between 
///    column 0 of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGB888 image  
///   \n\b WARNING: size must match input YCbCr422
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PseudoPlanarToRGB888u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from pseudo-planar YCbCr422 to RGBA8888  
///
/// @details
///   This function performs color space conversion from YCbCr422 to RGBA8888.
///
///   The input are one Y plane followed by one interleaved and horizontally 
///   sub-sampled CbCr (or CrCb) plane:
///   Y plane                          : Y0  Y1  Y2  Y3  ...
///   Interleaved and sub-sampled plane: Cb0 Cr0 Cb1 Cr1 ...
///
///   The output is one interleaved RGBA8888 plane:
///   RGBA8888 plane: R0 G0 B0 A0 R1 G1 B1 A1 R2 G2 B2 A2 R3 G3 B3 A3...
///
///   RGBA8888 pixel is arranged with 8-bit Red component, 8-bit Green component,
///   8-bit Blue component, and 8-bit A component. One RGBA8888 pixel is made 
///   up of 32-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcC
///   Input image Chroma component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCStride
///   Stride of input image Chroma component (i.e., number of bytes between 
///    column 0 of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGBA8888 image  
///   \n\b WARNING: size must match input YCbCr422
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr422PseudoPlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                          const uint8_t* __restrict srcC,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCStride,
                                          uint8_t* __restrict       dst,
                                          uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from pseudo-planar YCbCr420 to RGB565  
///
/// @details
///   This function performs color space conversion from YCbCr420 to RGB565.
///
///   The input are one Y plane followed by one interleaved and 2D (both
///   horizontally and vertically) sub-sampled CbCr (or CrCb) plane:
///   Y plane                             : Y00  Y01  Y02  Y03 ...
///                                         Y10  Y11  Y12  Y13 ...
///   Interleaved and 2D sub-sampled plane: Cb0  Cr0  Cb1  Cr1 ...
///
///   The output is one interleaved RGB565 plane:
///   RGB565 plane: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3...
///
///   RGB565 pixel is arranged with 5-bit Red component, 6-bit Green component,
///   and 5-bit Blue component. One RGB565 pixel is made up of 16-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcC
///   Input image Chroma component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCStride
///   Stride of input image Chroma component (i.e., number of bytes between 
///    column 0 of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGB565 image  
///   \n\b WARNING: size must match input YCbCr420
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PseudoPlanarToRGB565u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from pseudo-planar YCbCr420 to RGB888  
///
/// @details
///   This function performs color space conversion from YCbCr420 to RGB888.
///
///   The input are one Y plane followed by one interleaved and 2D (both
///   horizontally and vertically) sub-sampled CbCr (or CrCb) plane:
///   Y plane                             : Y00  Y01  Y02  Y03 ...
///                                         Y10  Y11  Y12  Y13 ...
///   Interleaved and 2D sub-sampled plane: Cb0  Cr0  Cb1  Cr1 ...
///
///   The output is one interleaved RGB888 plane:
///   RGB888 plane: R0 G0 B0 R1 G1 B1 R2 G2 B2 R3 G3 B3...
///
///   RGB888 pixel is arranged with 8-bit Red component, 8-bit Green component,
///   and 8-bit Blue component. One RGB888 pixel is made up of 24-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcC
///   Input image Chroma component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCStride
///   Stride of input image Chroma component (i.e., number of bytes between 
///    column 0 of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGB888 image  
///   \n\b WARNING: size must match input YCbCr420
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PseudoPlanarToRGB888u8( const uint8_t* __restrict srcY,
                                        const uint8_t* __restrict srcC,
                                        uint32_t                  srcWidth,
                                        uint32_t                  srcHeight,
                                        uint32_t                  srcYStride,
                                        uint32_t                  srcCStride,
                                        uint8_t* __restrict       dst,
                                        uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Color conversion from pseudo-planar YCbCr420 to RGBA8888  
///
/// @details
///   This function performs color space conversion from YCbCr420 to RGBA8888.
///
///   The input are one Y plane followed by one interleaved and 2D (both
///   horizontally and vertically) sub-sampled CbCr (or CrCb) plane:
///   Y plane                             : Y00  Y01  Y02  Y03 ...
///                                         Y10  Y11  Y12  Y13 ...
///   Interleaved and 2D sub-sampled plane: Cb0  Cr0  Cb1  Cr1 ...
///
///   The output is one interleaved RGBA8888 plane:
///   RGBA8888 plane: R0 G0 B0 A0 R1 G1 B1 A1 R2 G2 B2 A2 R3 G3 B3 A3...
///
///   RGBA8888 pixel is arranged with 8-bit Red component, 8-bit Green component,
///   8-bit Blue component, and 8-bit A component. One RGBA8888 pixel is made 
///   up of 32-bit data.
///   
///
/// @param srcY
///   Input image Y component
///
/// @param srcC
///   Input image Chroma component
///
/// @param srcWidth
///   Image width in number of Y pixels
///
/// @param srcHeight
///   Image height in number of Y lines
///
/// @param srcYStride
///   Stride of input image Y component (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///
/// @param srcCStride
///   Stride of input image Chroma component (i.e., number of bytes between 
///    column 0 of row 0 and column 0 of row 1)
///
/// @param dst
///   The output of interleaved RGBA8888 image  
///   \n\b WARNING: size must match input YCbCr420
///
/// @param dstStride
///   Stride of output RGB image (i.e., number of bytes between column 0 of 
///   row 0 and column 0 of row 1)
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvColorYCbCr420PseudoPlanarToRGBA8888u8( const uint8_t* __restrict srcY,
                                          const uint8_t* __restrict srcC,
                                          uint32_t                  srcWidth,
                                          uint32_t                  srcHeight,
                                          uint32_t                  srcYStride,
                                          uint32_t                  srcCStride,
                                          uint8_t* __restrict       dst,
                                          uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Performs edge weighting on input image.  
///
/// @details
///   The following filtes are used for edge weighting.
///
///                           [  0  1 -1  ]
///   Vertical edge filter:   [  0  2 -2  ]
///                           [  0  1 -1  ]
///
///                           [  0  0  0  ]
///   Horizontal edge filter: [  1  2  1  ]
///                           [ -1 -2 -1  ]
///  
/// @param src
///   Input edge map data
///
/// @param edgeMapWidth
///   Input edge map width
///
/// @param edgeMapHeight
///   Input edge map height
///
/// @param edgeMapStride
///   Stride of input edge map (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provided 0, internally set to edgeMapWidth*2.
///
/// @param weight
///   The given edge weighting weight. 
///   It is set to be 6554 (0.2 in Q15 format). 
///
/// @param edge_limit
///   The threshold to distinguish edges from noises. A pixel is from
///   an edge if the filtered value is greater than the edge_limit.
///    
///
/// @param hl_threshold
///   The limit of a pixel value reduction in HL band.  
///    
///
/// @param hh_threshold
///   The limit of a pixel value reduction in HH band.   
///    
///
/// @param edge_denoise_factor
///   Edge denoising factor to make sure a pixel value is reduced only when 
///   the pixel is a noise pixel.
///
/// @return
///   No return value
///
/// @ingroup image_processing
//------------------------------------------------------------------------------

FASTCV_API void
fcvEdgeWeightings16( int16_t* __restrict edgeMap,        
                     const uint32_t      edgeMapWidth,          
                     const uint32_t      edgeMapHeight,         
                     const uint32_t      edgeMapStride,        
                     const uint32_t      weight,         
                     const uint32_t      edge_limit,     
                     const uint32_t      hl_threshold,   
                     const uint32_t      hh_threshold,   
                     const uint32_t      edge_denoise_factor ); 


//------------------------------------------------------------------------------
/// @brief
///   Performe image deinterleave for unsigned byte data.
///
/// @details
///   Deinterleave color compoentonts from src to dst0 and dst1. 
///   Data in src [d0 t0 d1 t1 d2 t2...]
///   Results in dst0 [d0 d1 d2...]
///   Results in dst1 [t0 t1 t2...]
///
/// @param src
///   Input image
///
/// @param srcWidth
///   Input image width, number of data pairs. For example, CrCb or CbCr pairs. 
///
/// @param srcHeight
///   Input image height
///
/// @param srcStride
///   Stride of input image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provided 0, internally set to srcWidth*2.
///
/// @param dst0
///   Pointer to one of the output image. For example,  Cb or Cr components.
///
/// @param dst0Stride
///   Stride of one of the output image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provided 0, internally set to srcWidth.
///
/// @param dst1
///   Pointer to one of the output image. For example,  Cb or Cr components.
///
/// @param dst1Stride
///   Stride of one of the output image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provided 0, internally set to srcWidth.
///
/// @return
///   No return value
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvDeinterleaveu8( const uint8_t* __restrict src,
                   uint32_t                  srcWidth,
                   uint32_t                  srcHeight,
                   uint32_t                  srcStride,
                   uint8_t* __restrict       dst0,
                   uint32_t                  dst0Stride,
                   uint8_t* __restrict       dst1,
                   uint32_t                  dst1Stride );  


//------------------------------------------------------------------------------
/// @brief
///   Performe image interleave
///
/// @details
///   Interleav data from src0 and src1 to dst.
///   Data in src0 [d0 d1 d2 d3...]
///   Data in src1 [t0 t1 t2 t3...]
///   Results in dst [d0 t0 d1 t1 d2 t2 d3 t3...]
///
/// @param src0
///   One of the input images ( For example, Cb or Cr component)
///
/// @param src1
///   One of the input images ( For example, Cb or Cr component)

/// @param imageWidth
///   Input image width
///
/// @param imageHeight
///   Input image height
///
/// @param src0Stride
///   Stride of input image 0 (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provided 0, internally set to imageWidth.
///
/// @param src1Stride
///   Stride of input image 1 (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provided 0, internally set to imageWidth.
///
/// @param dst
///   Pointer to the output image
///
/// @param dstStride
///   Stride of the output image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provided 0, internally set to imageWidth *2.
///
/// @return
///   No return value
///
/// @ingroup color_conversion
//------------------------------------------------------------------------------

FASTCV_API void
fcvInterleaveu8( const uint8_t* __restrict src0,
                 const uint8_t* __restrict src1,
                 uint32_t                  imageWidth,
                 uint32_t                  imageHeight,
                 uint32_t                  src0Stride,
                 uint32_t                  src1Stride,
                 uint8_t* __restrict       dst,
                 uint32_t                  dstStride );

//------------------------------------------------------------------------------
/// @brief
///   Performs forward Haar discrete wavelet transform on input image and  
///   transpose the result.
///
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvDWTHaarTransposeu8(). In the 2.0.0 release, 
///   the signature of fcvDWTHarrTransposeu8 as it appears now, 
///   will be removed.
///   \n\n
///
/// @details
///   
///
/// @param src
///   Input image 
///
/// @param srcWidth
///   Image width
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of input image (i.e., number of bytes between column 0 of row 0
///   and column 0 of row 1)
///
/// @param dst
///   Output image 
///
/// @param dstStride
///   Stride of output image (i.e., number of bytes between column 0 of row 0 
///   and column 0 of row 1)
///
/// @ingroup image_transform    
//------------------------------------------------------------------------------

FASTCV_API void
fcvDWTHarrTransposeu8( const uint8_t* __restrict src,                       
                       uint32_t                  srcWidth,
                       uint32_t                  srcHeight,
                       uint32_t                  srcStride,
                       int16_t* __restrict       dst,
                       uint32_t                  dstStride );

//------------------------------------------------------------------------------
/// @brief
///   Performs forward Haar discrete wavelet transform on input image and  
///   transposes the result.
///
/// @details
///   This function performs forward discrete wavelet transform on input image
///   using the Haar kernel:
///   Low pass:  [ 1   1 ] * 2^(-1/2)
///   High pass: [ 1  -1 ] * 2^(-1/2)
///   This function also transposes the result.
///
/// @param src
///   Input image 
///
/// @param srcWidth
///   Image width
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of input image (i.e., number of bytes between column 0 of row 0
///   and column 0 of row 1)
///
/// @param dst
///   Output image that has been transformed and transposed 
///
/// @param dstStride
///   Stride of output image (i.e., number of bytes between column 0 of row 0 
///   and column 0 of row 1)
///
/// @ingroup image_transform    
//------------------------------------------------------------------------------

FASTCV_API void
fcvDWTHaarTransposeu8( const uint8_t* __restrict src,                       
                       uint32_t                  srcWidth,
                       uint32_t                  srcHeight,
                       uint32_t                  srcStride,
                       int16_t* __restrict       dst,
                       uint32_t                  dstStride );

//------------------------------------------------------------------------------
/// @brief
///   Performs forward 5-3 Tab discrete wavelet transform on input image and  
///   transposes the result.
///
/// @details
///   This function performs forward discrete wavelet transform on input image
///   using the 5-tab low pass filter and the 3-tab high pass filter:
///   5-tab low pass:  [ -1/8  1/4  3/4  1/4  -1/8 ] * 2^(1/2)
///   3-tab high pass: [      -1/2   1  -1/2       ] * 2^(-1/2)
///   This function also transposes the result.  
///
/// @param src
///   Input image 
///
/// @param srcWidth
///   Image width
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of input image (i.e., number of bytes between column 0 of row 0
///   and column 0 of row 1)
///
/// @param dst
///   Output image that has been transformed and transposed  
///
/// @param dstStride
///   Stride of output image (i.e., number of bytes between column 0 of row 0 
///   and column 0 of row 1)
///
/// @ingroup image_transform    
//------------------------------------------------------------------------------

FASTCV_API void
fcvDWT53TabTransposes16( const int16_t* __restrict src,                       
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         int16_t* __restrict       dst,
                         uint32_t                  dstStride );

//------------------------------------------------------------------------------
/// @brief
///   Performs inverse 5-3 Tab discrete wavelet transform on input image and  
///   transposes the result.
///
/// @details
///   This function performs inverse discrete wavelet transform on input image
///   using the 3-tab low pass filter and the 5-tab high pass filter:
///   3-tab low  pass: [      -1/2   1  -1/2       ] * 2^(-1/2)
///   5-tab high pass: [ -1/8  1/4  3/4  1/4  -1/8 ] * 2^(1/2)
///   This function also transposes the result.  
///
/// @param src
///   Input image 
///
/// @param srcWidth
///   Image width
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of input image (i.e., number of bytes between column 0 of row 0
///   and column 0 of row 1)
///
/// @param dst
///   Output image that has been transformed and transposed 
///
/// @param dstStride
///   Stride of output image (i.e., number of bytes between column 0 of row 0 
///   and column 0 of row 1)
///
/// @ingroup image_transform    
//------------------------------------------------------------------------------

FASTCV_API void
fcvIDWT53TabTransposes16( const int16_t*   __restrict src,                       
                          uint32_t                    srcWidth,
                          uint32_t                    srcHeight,
                          uint32_t                    srcStride,
                          int16_t* __restrict         dst,
                          uint32_t                    dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Performs inverse Haar discrete wavelet transform on input image and  
///   transpose the result.
///
///   \n\b ATTENTION: This function's signature will become \b OBSOLETE in a future
///   release of this library (2.0.0).  The new interface is specified in the 
///   function: fcvIDWTHaarTransposes16(). In the 2.0.0 release, 
///   the signature of fcvIDWTHarrTransposes16 as it appears now, 
///   will be removed.
///   \n\n
/// 
/// @details
///   
///
/// @param src
///   Input image 
///
/// @param srcWidth
///   Image width
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of input image (i.e., number of bytes between column 0 of row 0
///   and column 0 of row 1)
///
/// @param dst
///   Output image 
///
/// @param dstStride
///   Stride of output image (i.e., number of bytes between column 0 of row 0 
///   and column 0 of row 1)
///
/// @ingroup image_transform    
//------------------------------------------------------------------------------

FASTCV_API void
fcvIDWTHarrTransposes16( const int16_t* __restrict src,                       
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         uint8_t* __restrict       dst,
                         uint32_t                  dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Performs inverse Haar discrete wavelet transform on input image and  
///   transposes the result.
/// 
/// @details
///   This function performs inverse discrete wavelet transform on input image
///   using the Haar kernel:
///   Low pass:  [ 1   1 ] * 2^(-1/2)
///   High pass: [ 1  -1 ] * 2^(-1/2)
///   This function also transposes the result.
///
/// @param src
///   Input image 
///
/// @param srcWidth
///   Image width
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of input image (i.e., number of bytes between column 0 of row 0
///   and column 0 of row 1)
///
/// @param dst
///   Output image that has been transformed and transposed 
///
/// @param dstStride
///   Stride of output image (i.e., number of bytes between column 0 of row 0 
///   and column 0 of row 1)
///
/// @ingroup image_transform    
//------------------------------------------------------------------------------

FASTCV_API void
fcvIDWTHaarTransposes16( const int16_t* __restrict src,                       
                         uint32_t                  srcWidth,
                         uint32_t                  srcHeight,
                         uint32_t                  srcStride,
                         uint8_t* __restrict       dst,
                         uint32_t                  dstStride );

//------------------------------------------------------------------------------
/// @brief
///   Perform image upscaling using polyphase filters
///
/// @details
///   Perform image upscaling using polyphase filters. The image data type is 
///   unsigned byte.
///
/// @param src
///   Input image
///
/// @param srcWidth
///   Input image width
///
/// @param srcHeight
///   Input image height
///
/// @param srcStride
///   Stride of input image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provided 0, internally set to srcWidth.
///
/// @param dst
///   Output image
///
/// @param dstWidth
///   Output image width
///
/// @param dstHeight
///   Output image height
///
/// @param dstStride
///   Stride of output image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provided 0, internaly set to dstWidth.
///
/// @return
///   No return value.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleUpPolyu8( const uint8_t* __restrict src,
                  uint32_t                  srcWidth,
                  uint32_t                  srcHeight,
                  uint32_t                  srcStride,
                  uint8_t* __restrict       dst,
                  uint32_t                  dstWidth,
                  uint32_t                  dstHeight,
                  uint32_t                  dstStride );   


//------------------------------------------------------------------------------
/// @brief
///   Interleaved image (CbCr or CrCb) upscaling using polyphase filters
///
/// @details
///   Perform interleaved image (CbCr or CrCb) upscaling using polyphase 
///   filters. Data type is unsigned byte.
///
/// @param src
///   Input image
///
/// @param srcWidth
///   Input image width, number of (CrCb/CbCr) pairs
///
/// @param srcHeight
///   Input image height
///
/// @param srcStride
///   Stride of input image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provide 0, internally set to srcWidth*2.
///
/// @param dst
///   Output image
///
/// @param dstWidth
///   Output image width, number of (CrCb/CbCr) pairs
///
/// @param dstHeight
///   Output image height
///
/// @param dstStride
///   Stride of output image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provide 0, internally set to dstWidth*2.
///
/// @return
///   No return value.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleUpPolyInterleaveu8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstWidth,
                            uint32_t                  dstHeight,
                            uint32_t                  dstStride );  


//------------------------------------------------------------------------------
/// @brief
///   Image downscaling using MN method
///
/// @details
///   The M over N downscale algorithm works on an arbitrary length (N) of 
///   input data, and generates another arbitrary length (M) of output data,
///   with the output length M less or equal to the input length N.
///   This algorithm accumulates the input values while keeps adding the output 
///   length until the updated output length is greater or equal to the input length, 
///   at which point an output value is generated. The output value is the average of 
///   the accumulated input values. Notice that the input value accumulator is reset 
///   to 0 each time an output is generated. 
///
/// @param src
///   Input image
///
/// @param srcWidth
///   Input image width
///
/// @param srcHeight
///   Input image height
///
/// @param srcStride
///   Stride of input image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provided 0, interanlly set to srcWidth.
///
/// @param dst
///   Output image
///
/// @param dstWidth
///   Output image width
///
/// @param dstHeight
///   Output image height
///
/// @param dstStride
///   Stride of output image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1)
///   If provided 0, interanlly set to dstWidth.
///
/// @return
///   No return value.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownMNu8( const uint8_t* __restrict src,
                  uint32_t                  srcWidth,
                  uint32_t                  srcHeight,
                  uint32_t                  srcStride,
                  uint8_t* __restrict       dst,
                  uint32_t                  dstWidth,
                  uint32_t                  dstHeight,
                  uint32_t                  dstStride );   


//------------------------------------------------------------------------------
/// @brief
///   Interleaved image downscaling using MN method
///
/// @details
///   The M over N downscale algorithm works on an arbitrary length (N) of 
///   input data, and generates another arbitrary length (M) of output data,
///   with the output length M less or equal to the input length N.
///   This algorithm accumulates the input values while keeps adding the output 
///   length until the updated output length is greater or equal to the input length, 
///   at which point an output value is generated. The output value is the average of 
///   the accumulated input values. Notice that the input value accumulator is reset 
///   to 0 each time an output is generated. 
///
/// @param src
///   Input image
///
/// @param srcWidth
///   Input image width, number of (CrCb/CbCr) pair
///
/// @param srcHeight
///   Input image height
///
/// @param srcStride
///   Stride of input image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provided 0, internally set to srcWidth*2.
///
/// @param dst
///   Output image
///
/// @param dstWidth
///   Output image width , number of (CrCb/CbCr) pair
///
/// @param dstHeight
///   Output image height
///
/// @param dstStride
///   Stride of output image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///   If provided 0, internally set to dstWidth*2.
///
/// @return
///   No return value.
///
/// @ingroup image_transform
//------------------------------------------------------------------------------

FASTCV_API void
fcvScaleDownMNInterleaveu8( const uint8_t* __restrict src,
                            uint32_t                  srcWidth,
                            uint32_t                  srcHeight,
                            uint32_t                  srcStride,
                            uint8_t* __restrict       dst,
                            uint32_t                  dstWidth,
                            uint32_t                  dstHeight,
                            uint32_t                  dstStride ); 

//---------------------------------------------------------------------------
/// @brief
///   Search K-Means tree, where each node connects to up to 10 children,
///   and the center (mean) is a 36-tuple vector of 8-bit signed value.
///
/// @param nodeChildrenCenter
///   A pointer to uint8_t [numNodes][10][36],
///   which stores the center vectors of node children.
///   The outer-most dimension represents the nodes in the tree.
///   The middle dimension represents the children of each node.
///   The inner-most dimension represents the tuples of the center vector.
///   \n\b WARNING: must be 64-bit aligned.
///
/// @param nodeChildrenInvLenQ32
///   A pointer to uint32_t [numNodes][10],
///   which stores the inverse lengths of the center vectors.
///   The inverse lengths are in Q32 format.
///   The outer-most dimension represents the nodes in the tree.
///   The inner-most dimension represents the children of each node.
///   \n\b WARNING: must be 64-bit aligned.
///
/// @param nodeChildrenIndex
///   A pointer to uint32_t [numNodes][10],
///   which stores the indices of the children nodes.
///   If the MSB is 0, the index points to a node within the tree.
///   If the MSB is 1, the index (with MSB removed) points to a leaf node,
///   which is returned by this function as the search result.
///   See nodeChildrenInvLenQ32 for the definition of each dimension.
///   \n\b WARNING: must be 64-bit aligned.
///
/// @param nodeNumChildren
///   A pointer to uint8_t [numNodes],
///   which stores the number of children in each node.
///
/// @param numNodes
///   Number of nodes in the K-Means tree.
///
/// @param key
///   A pointer to int8_t [36], which stores the key to be searched.
///
/// @return
///   Index of the leaf node.
///
/// @ingroup feature_detection
//---------------------------------------------------------------------------

FASTCV_API uint32_t
fcvKMeansTreeSearch36x10s8( const   int8_t* __restrict  nodeChildrenCenter,
                            const uint32_t* __restrict  nodeChildrenInvLenQ32,
                            const uint32_t* __restrict  nodeChildrenIndex,
                            const  uint8_t* __restrict  nodeNumChildren,
                                  uint32_t              numNodes,
                            const  int8_t * __restrict  key );

//---------------------------------------------------------------------------
/// @brief
///   Sorts in-place the pairs of <descDB, descDBInvLenQ38 > according to
///   descDBTargetId.
///
/// @param dbLUT
///   A pointer to uint32_t [numDBLUT][2],
///   which stores the starting index of descDB and
///   the number of descriptors
///   \n\b WARNING: must be 64-bit aligned.
///
/// @param numDBLUT
///   The size of dbLUT
///
/// @param descDB
///   A pointer to int8_t [numDescDB][36],
///   which stores descriptors
///   \n\b WARNING: must be 64-bit aligned.
///
/// @param descDBInvLenQ38
///   A pointer to uint32_t [numDescDB],
///   which stores the inverse length of descDB.
///   The value is in Q38 format.
///
/// @param descDBTargetId
///   A pointer to uint16_t [numDescDB],
///   which stores the target id.
///
/// @param descDBOldIdx
///   A pointer to uint32_t [numDescDB],
///   which stores the old index of the desc before sorting
///
/// @param numDescDB
///   Number of descriptor in the database.
///
/// @ingroup feature_detection
//---------------------------------------------------------------------------

FASTCV_API int
fcvLinearSearchPrepare8x36s8(  uint32_t * __restrict   dbLUT,
                               uint32_t                numDBLUT,
                               int8_t   * __restrict   descDB,
                               uint32_t * __restrict   descDBInvLenQ38,
                               uint16_t * __restrict   descDBTargetId,
                               uint32_t * __restrict   descDBOldIdx,
                               uint32_t                numDescDB );

//---------------------------------------------------------------------------
/// @brief
///   Perform linear search of descriptor in a database
///
/// @param dbLUT
///   A pointer to uint32_t [numDBLUT][2],
///   which stores the starting index of descDB and
///   the number of descriptors
///   \n\b WARNING: must be 64-bit aligned.
///
/// @param numDBLUT
///   The size of dbLUT
///
/// @param descDB
///   A pointer to int8_t [numDescDB][36],
///   which stores descriptors
///   \n\b WARNING: must be 64-bit aligned.
///
/// @param descDBInvLenQ38
///   A pointer to uint32_t [numDescDB],
///   which stores the inverse length of descDB.
///   The value is in Q38 format.
///
/// @param descDBTargetId
///   A pointer to uint16_t [numDescDB],
///   which stores the target id.
///
/// @param numDescDB
///   Number of descriptor in the database.
///
/// @param srcDesc
///   A pointer to int8_t [numSrcDesc][36],
///   which stores descriptors.
///   \n\b WARNING: must be 64-bit aligned.
///
/// @param srcDescInvLenQ38
///   A pointer to uint32_t [numSrcDec],
///   which stores the inverse length of srcDesc.
///   The value is in Q38 format.
///
/// @param srcDescIdx
///   A pointer to the dbLUT data
///
/// @param numSrcDecc
///   Number of source descriptor
///
/// @param targetsToIgnore
///   A list of target IDs to be ignored
///
/// @param numTargetsToIgnore
///   Number of targets to be ignored
///
/// @param maxDistanceQ31
///   Maximum distance for correspondences.
///   In Q31 format.
///
/// @param correspondenceDBIdx
///   A pointer to uint32_t [maxNumCorrespondences],
///   which will be used by this function to output indices of featuresDB
///   as a part of correspondences.
///
/// @param correspondenceSrcDescIdx
///   A pointer to uint32_t [maxNumCorrespondences],
///   which will be used by this function to output indices of descriptors
///   as a part of correspondences.
///
/// @param correspondenceDistanceQ31
///   A pointer to uint32_t [maxNumCorrespondences],
///   which will be used by this function to output the distances
///   as a part of correspondences.
///   In Q31 format.
///
/// @param maxNumCorrespondences
///   Maximum number of correspondences allowed
///
/// @param numCorrespondences
///   Number of correspondences returned by this function
///
/// @ingroup feature_detection
//---------------------------------------------------------------------------

FASTCV_API void
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
   uint32_t       * __restrict numCorrespondences );


//------------------------------------------------------------------------------
/// @brief
///   Finds only extreme outer contours in a binary image.  There is no nesting
///   relationship between contours.  It sets hierarchy[i][2]=hierarchy[i][3]=-1
///   for all the contours.
///
/// @param src
///   Grayscale image with one byte per pixel.  Non-zero pixels are treated as
///   1’s. Zero pixels remain 0’s, so the image is treated as binary.
///
/// @param srcWidth
///   Image width
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param maxNumContours
///   Maximum number of contours can be found
///
/// @param numContours
///   Number of actually found contours
///
/// @param numContourPoints
///   Number of points in each found contour
///
/// @param contourStartPoints
///   Pointers to the start point of each found contour
///
/// @param pointBuffer
///   Pointer to point buffer for contour points' coordinates.  It should
///   be allocated before calling this function.
///
/// @param pointBufferSize
///   Size of point buffer in terms of uint32_t
///
/// @param hierarchy
///   Information about the image topology.  It has numContours elements.
///   For each contour i, the elements hierarchy[i][0], hiearchy[i][1], 
///   hiearchy[i][2], and hiearchy[i][3] are set to 0-based indices of the
///   next and previous contours at the same hierarchical level, the first
///   child contour and the parent contour, respectively.  If for a contour i
///   there are no next, previous, parent, or nested contours, the corresponding
///   elements of hierarchy[i] will be negative.
///
/// @param contourHandle
///   Pointer to assistant and intermediate data.  It should be allocated by 
///   fcvFindContoursAllocate() and deallocated by fcvFindContoursDelete().
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------
FASTCV_API void
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
                           void*                 contourHandle );


//------------------------------------------------------------------------------
/// @brief
///   Finds contours in a binary image without any hierarchical relationships.
///
/// @param src
///   Grayscale image with one byte per pixel.  Non-zero pixels are treated as
///   1’s. Zero pixels remain 0’s, so the image is treated as binary.
///
/// @param srcWidth
///   Image width
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param maxNumContours
///   Maximum number of contours can be found
///
/// @param numContours
///   Number of actually found contours
///
/// @param numContourPoints
///   Number of points in each found contour
///
/// @param contourStartPoints
///   Pointers to the start point of each found contour
///
/// @param pointBuffer
///   Pointer to point buffer for contour points' coordinates.  It should
///   be allocated before calling this function.
///
/// @param pointBufferSize
///   Size of point buffer in terms of uint32_t
///
/// @param contourHandle
///   Pointer to assistant and intermediate data.  It should be allocated by 
///   fcvFindContoursAllocate() and deallocated by fcvFindContoursDelete().
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------
FASTCV_API void
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
                       void*                 contourHandle );


//------------------------------------------------------------------------------
/// @brief
///   Finds contours in a binary image and organizes them into a two-level 
///   hierarchy.  At the top level, there are external boundaries of the 
///   components. At the second level, there are boundaries of the holes. 
///   If there is another contour inside a hole of a connected component, 
///   it is still put at the top level. 
///
/// @param src
///   Grayscale image with one byte per pixel.  Non-zero pixels are treated as
///   1’s. Zero pixels remain 0’s, so the image is treated as binary.
///
/// @param srcWidth
///   Image width
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param maxNumContours
///   Maximum number of contours can be found (<= 126)
///
/// @param numContours
///   Number of actually found contours
///
/// @param holeFlag
///   Hole flag for each found contour to indicate whether it is a hole or not
///
/// @param numContourPoints
///   Number of points in each found contour
///
/// @param contourStartPoints
///   Pointers to the start point of each found contour
///
/// @param pointBuffer
///   Pointer to point buffer for contour points' coordinates.  It should
///   be allocated before calling this function.
///
/// @param pointBufferSize
///   Size of point buffer in terms of uint32_t
///
/// @param hierarchy
///   Information about the image topology.  It has numContours elements.
///   For each contour i, the elements hierarchy[i][0], hiearchy[i][1], 
///   hiearchy[i][2], and hiearchy[i][3] are set to 0-based indices of the
///   next and previous contours at the same hierarchical level, the first
///   child contour and the parent contour, respectively.  If for a contour i
///   there are no next, previous, parent, or nested contours, the corresponding
///   elements of hierarchy[i] will be negative.
///
/// @param contourHandle
///   Pointer to assistant and intermediate data.  It should be allocated by 
///   fcvFindContoursAllocate() and deallocated by fcvFindContoursDelete().
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------
FASTCV_API void
fcvFindContoursCcompu8( uint8_t* __restrict   src,
                        uint32_t              srcWidth,
                        uint32_t              srcHeight,
                        uint32_t              srcStride,
                        uint32_t              maxNumContours,
                        uint32_t* __restrict   numContours,
                        uint32_t* __restrict  holeFlag,
                        uint32_t* __restrict  numContourPoints,
                        uint32_t** __restrict contourStartPoints,
                        uint32_t* __restrict  pointBuffer,
                        uint32_t              pointBufferSize,
                        int32_t               hierarchy[][4],
                        void*                 contourHandle );


//------------------------------------------------------------------------------
/// @brief
///   Finds contours in a binary image and reconstructs a full hierarchy of 
///   nested contours
///
/// @param src
///   Grayscale image with one byte per pixel.  Non-zero pixels are treated as
///   1’s. Zero pixels remain 0’s, so the image is treated as binary.
///
/// @param srcWidth
///   Image width
///
/// @param srcHeight
///   Image height
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param numContours
///   Number of actually found contours
///
/// @param maxNumContours
///   Maximum number of contours can be found (<= 126)
///
/// @param holeFlag
///   Hole flag for each found contour to indicate whether it is a hole or not
///
/// @param numContourPoints
///   Number of points in each found contour
///
/// @param contourStartPoints
///   Pointers to the start point of each found contour
///
/// @param pointBuffer
///   Pointer to point buffer for contour points' coordinates.  It should
///   be allocated before calling this function.
///
/// @param pointBufferSize
///   Size of point buffer in terms of uint32_t
///
/// @param hierarchy
///   Information about the image topology.  It has numContours elements.
///   For each contour i, the elements hierarchy[i][0], hiearchy[i][1], 
///   hiearchy[i][2], and hiearchy[i][3] are set to 0-based indices of the
///   next and previous contours at the same hierarchical level, the first
///   child contour and the parent contour, respectively.  If for a contour i
///   there are no next, previous, parent, or nested contours, the corresponding
///   elements of hierarchy[i] will be negative.
///
/// @param contourHandle
///   Pointer to assistant and intermediate data.  It should be allocated by 
///   fcvFindContoursAllocate() and deallocated by fcvFindContoursDelete().
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------
FASTCV_API void
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
                       void*                 contourHandle );


//------------------------------------------------------------------------------
/// @brief
///   Allocates assistant and intermediate data for contour
///
/// @param srcStride
///   Stride of image (i.e., how many pixels between column 0 of row 1 and
///   column 0 of row 2).
///
/// @return
///   Pointer to allocated data
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------
FASTCV_API void*
fcvFindContoursAllocate( uint32_t srcStride );


//------------------------------------------------------------------------------
/// @brief
///   Deallocates assistant and intermediate data for contour
///
/// @param contourHandle
///   Pointer to assistant and intermediate data
///
/// @ingroup feature_detection
//------------------------------------------------------------------------------
FASTCV_API void
fcvFindContoursDelete( void* contourHandle );

//------------------------------------------------------------------------------
/// @brief
///   Solve linear equation system
///          Ax = b
///
/// @details
///   
///
/// @param A
///    The matrix contains coefficients of the linear equation system
///
/// @param numRows
///    The number of rows for the matrix A
///
/// @param numCols
///    The number of columns for the matrix A
///
/// @param b
///    The right side value
///
/// @param x
///    The solution vector
///
///
/// @return
///    
/// @ingroup math_vector 
//------------------------------------------------------------------------------
FASTCV_API void 
fcvSolvef32(const float32_t * __restrict A, 
            int32_t numCols, 
            int32_t numRows, 
            const float32_t * __restrict b, 
            float32_t * __restrict x);

//------------------------------------------------------------------------------
/// @brief
///   Calculates a perspective transform from four pairs of the corresponding
///   points
///
/// @param src
///   Coordinates of quadrangle vertices in the source image
///
/// @param dst
///   Coordinates of the corresponding quadrangle vertices in the destination
///   image
///
/// @param transformCoefficient
///   3x3 matrix of a perspective transform 
///
/// @ingroup image_transform
//------------------------------------------------------------------------------
FASTCV_API void
fcvGetPerspectiveTransformf32( const float32_t src1[8],
                               const float32_t src2[8],
                               float32_t  transformCoefficient[9] );

//------------------------------------------------------------------------------
/// @brief
///   Sets every element of a uint8_t single channel array to a given value.
///
/// @details  
///   A non-zero element of the mask array indicates the corresponding element 
///   of the destination array to be changed. The mask itself equals to zero means that
///   all elements of the dst array need to be changed. The mask is assumed to
///    have the same width and height( in terms of pixels) as the destination array.
///
/// @param dst
///    The destination matrix
///
/// @param dstWidth
///    Destination matrix width
///
/// @param dstHeight
///    Destination matrix height
///
/// @param dstStride
///    Stride for the destination matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param value
///    the input uint8_t value 
///
/// @param mask
///    Operation mask, 8-bit single channel array; specifies elements of the src
///       array to be changed. 
///
/// @param maskStride
///    Stride for the mask, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup math_vector 
//------------------------------------------------------------------------------
  
FASTCV_API void
fcvSetElementsu8(        uint8_t * __restrict dst, 
                         uint32_t             dstWidth, 
                         uint32_t             dstHeight,
                         uint32_t             dstStride, 
                         uint8_t              value,
                   const uint8_t * __restrict mask,
                         uint32_t             maskStride
                 );

//------------------------------------------------------------------------------
/// @brief
///   Sets every element of an int32_t  single channel array to a given value.
///
/// @details
///   A non-zero element of the mask array indicates the corresponding element 
///   of the destination array to be changed. The mask itself equals to zero means that
///   all elements of the dst array need to be changed. The mask is assumed to
///    have the same width and height( in terms of pixels) as the destination array.
///
/// @param dst
///    The destination matrix
///
/// @param dstWidth
///    Destination matrix width
///
/// @param dstHeight
///    Destination matrix height
///
/// @param dstStride
///    Stride for the destination matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param value
///    the input int32_t value
///
/// @param mask
///    Operation mask, 8-bit single channel array; specifies elements of the src
///    array to be changed
///
/// @param maskStride
///    Stride for input mask, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row 
///
/// @return
///   No return value
///
/// @ingroup math_vector 
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementss32 (          int32_t * __restrict dst, 
                             uint32_t             dstWidth, 
                             uint32_t             dstHeight,
                             uint32_t             dstStride, 
                             int32_t              value,
                       const uint8_t * __restrict mask ,
                             uint32_t             maskStride
                     );

//------------------------------------------------------------------------------
/// @brief
///   Sets every element of a float32_t single channel array to a given value.
///
/// @details
///   A non-zero element of the mask array indicates the corresponding element 
///   of the destination array to be changed. The mask itself equals to zero means that
///   all elements of the dst array need to be changed. The mask is assumed to
///    have the same width and height( in terms of pixels) as the destination array.
///
/// @param dst
///    The destination matrix
///
/// @param dstWidth
///    Destination matrix width
///
/// @param dstHeight
///    Destination matrix height
///
/// @param dstStride
///    Stride for the destination matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param value
///    the input float32_t value
///
/// @param mask
///    Operation mask, 8-bit single channel array; specifies elements of the src
///    array to be changed
///
/// @param maskStride
///    Stride for input mask, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup math_vector 
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsf32(        float32_t * __restrict dst, 
                          uint32_t               dstWidth, 
                          uint32_t               dstHeight,
                          uint32_t               dstStride, 
                          float32_t              value,
                    const uint8_t   * __restrict mask,
                          uint32_t               maskStride
                   );

//------------------------------------------------------------------------------
/// @brief
///   Sets every element of a uint8_t 4-channel  array to a given 4-element scalar.
///
/// @details
///   A non-zero element of the mask array indicates the corresponding element 
///   of the destination array to be changed. The mask itself equals to zero means that
///   all elements of the dst array need to be changed. The mask is assumed to
///    have the same width and height( in terms of pixels) as the destination array.
///
/// @param dst
///    The destination matrix
///
/// @param dstWidth
///    Destination matrix width
///
/// @param dstHeight
///    Destination matrix height
///
/// @param dstStride
///    Stride for the destination matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param value1
///    First uint8_t value of the Scalar
///
/// @param value2
///    Second uint8_t value of the Scalar
///
/// @param value3
///    Third uint8_t value of the Scalar
///
/// @param value4
///    Fourth uint8_t value of the Scalar
///
/// @param mask
///    Operation mask, 8-bit single channel array; specifies elements of the src
///    array to be changed
///
/// @param maskStride
///    Stride for input mask, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup math_vector 
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsc4u8(         uint8_t * __restrict dst, 
                            uint32_t             dstWidth, 
                            uint32_t             dstHeight,
                            uint32_t             dstStride, 
                            uint8_t              value1,
                            uint8_t              value2,
                            uint8_t              value3,
                            uint8_t              value4,
                      const uint8_t * __restrict mask,
                            uint32_t             maskStride
                    );

//------------------------------------------------------------------------------
/// @brief
///   Sets every element of an int32_t 4-channel  array to a given 4-element scalar.
///
/// @details
///   A non-zero element of the mask array indicates the corresponding element 
///   of the destination array to be changed. The mask itself equals to zero means that
///   all elements of the dst array need to be changed. The mask is assumed to
///    have the same width and height( in terms of pixels) as the destination array.
///
/// @param dst
///    The destination matrix
///
/// @param dstWidth
///    Destination matrix width
///
/// @param dstHeight
///    Destination matrix height
///
/// @param dstStride
///    Stride for the destination matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param value1
///    First int32_t value of the Scalar
///
/// @param value2
///    Second int32_t value of the Scalar
///
/// @param value3
///    Third int32_t value of the Scalar
///
/// @param value4
///    Fourth int32_t value of the Scalar
///
/// @param mask
///    Operation mask, 8-bit single channel array; specifies elements of the src
///    array to be changed. 
///
/// @param maskStride
///    Stride for input mask, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup math_vector 
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsc4s32(         int32_t * __restrict dst, 
                             uint32_t             dstWidth, 
                             uint32_t             dstHeight,
                             uint32_t             dstStride, 
                             int32_t              value1,
                             int32_t              value2,
                             int32_t              value3,
                             int32_t              value4,
                       const uint8_t * __restrict mask,
                             uint32_t             maskStride 
                     );

//------------------------------------------------------------------------------
/// @brief
///   Sets every element of a float32_t 4-channel  array to a given 4-element scalar.
///
/// @details
///   A non-zero element of the mask array indicates the corresponding element 
///   of the destination array to be changed. The mask itself equals to zero means that
///   all elements of the dst array need to be changed. The mask is assumed to
///    have the same width and height( in terms of pixels) as the destination array.
///
/// @param dst
///    The destination matrix
///
/// @param dstWidth
///    Destination matrix width
///
/// @param dstHeight
///    Destination matrix height
///
/// @param dstStride
///    Stride for the destination matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param value1
///    First float32_t value of the Scalar
///
/// @param value2
///    Second float32_t value of the Scalar
///
/// @param value3
///    Third float32_t value of the Scalar
///
/// @param value4
///    Fourth float32_t value of the Scalar
///
/// @param mask
///    Operation mask, 8-bit single channel array; specifies elements of the src
///    array to be changed
///
/// @param maskStride
///    Stride for input mask, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup math_vector 
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsc4f32(         float32_t * __restrict dst, 
                             uint32_t               dstWidth, 
                             uint32_t               dstHeight,
                             uint32_t               dstStride, 
                             float32_t              value1,
                             float32_t              value2,
                             float32_t              value3,
                             float32_t              value4,
                       const uint8_t   * __restrict mask,
                             uint32_t               maskStride
                     );
		
//------------------------------------------------------------------------------
/// @brief
///   Sets every element of a uint8_t 3-channel  array to a given 3-element scalar.
///
/// @details
///   A non-zero element of the mask array indicates the corresponding element 
///   of the destination array to be changed. The mask itself equals to zero means that
///   all elements of the dst array need to be changed. The mask is assumed to
///    have the same width and height( in terms of pixels) as the destination array.
///
/// @param dst
///    The destination matrix
///
/// @param dstWidth
///    Destination matrix width
///
/// @param dstHeight
///    Destination matrix height
///
/// @param dstStride
///    Stride for the destination matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param value1
///    First uint8_t value of the Scalar
///
/// @param value2
///    Second uint8_t value of the Scalar
///
/// @param value3
///    Third uint8_t value of the Scalar
///
/// @param mask
///    Operation mask, 8-bit single channel array; specifies elements of the src
///    array to be changed
///
/// @param maskStride
///    Stride for input mask, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup math_vector 
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsc3u8(         uint8_t * __restrict dst, 
                            uint32_t             dstWidth, 
                            uint32_t             dstHeight,
                            uint32_t             dstStride, 
                            uint8_t              value1,
                            uint8_t              value2,
                            uint8_t              value3,
                      const uint8_t * __restrict mask,
                            uint32_t             maskStride
                    );

//------------------------------------------------------------------------------
/// @brief
///   Sets every element of an int32_t 3-channel  array to a given 3-element scalar.
///
/// @details
///   A non-zero element of the mask array indicates the corresponding element 
///   of the destination array to be changed. The mask itself equals to zero means that
///   all elements of the dst array need to be changed. The mask is assumed to
///    have the same width and height( in terms of pixels) as the destination array.
///
/// @param dst
///    The destination matrix
///
/// @param dstWidth
///    Destination matrix width
///
/// @param dstHeight
///    Destination matrix height
///
/// @param dstStride
///    Stride for the destination matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param value1
///    First int32_t value of the Scalar
///
/// @param value2
///    Second int32_t value of the Scalar
///
/// @param value3
///    Third int32_t value of the Scalar
///
/// @param mask
///    Operation mask, 8-bit single channel array; specifies elements of the src
///    array to be changed. 
///
/// @param maskStride
///    Stride for input mask, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup math_vector 
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsc3s32(         int32_t * __restrict dst, 
                             uint32_t             dstWidth, 
                             uint32_t             dstHeight,
                             uint32_t             dstStride, 
                             int32_t              value1,
                             int32_t              value2,
                             int32_t              value3,
                       const uint8_t * __restrict mask,
                             uint32_t             maskStride 
                     );

//------------------------------------------------------------------------------
/// @brief
///   Sets every element of a float32_t 3-channel  array to a given 3-element scalar.
///
/// @details
///   A non-zero element of the mask array indicates the corresponding element 
///   of the destination array to be changed. The mask itself equals to zero means that
///   all elements of the dst array need to be changed. The mask is assumed to
///    have the same width and height( in terms of pixels) as the destination array.
///
/// @param dst
///    The destination matrix
///
/// @param dstWidth
///    Destination matrix width
///
/// @param dstHeight
///    Destination matrix height
///
/// @param dstStride
///    Stride for the destination matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param value1
///    First float32_t value of the Scalar
///
/// @param value2
///    Second float32_t value of the Scalar
///
/// @param value3
///    Third float32_t value of the Scalar
///
/// @param mask
///    Operation mask, 8-bit single channel array; specifies elements of the src
///    array to be changed
///
/// @param maskStride
///    Stride for input mask, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup math_vector 
//------------------------------------------------------------------------------
FASTCV_API void
fcvSetElementsc3f32(         float32_t * __restrict dst, 
                             uint32_t               dstWidth, 
                             uint32_t               dstHeight,
                             uint32_t               dstStride, 
                             float32_t              value1,
                             float32_t              value2,
                             float32_t              value3,
                       const uint8_t   * __restrict mask,
                             uint32_t               maskStride
                     );


//------------------------------------------------------------------------------
/// @brief
///   Defines an enumeration to list threshold types used in fcvAdaptiveThreshold
//------------------------------------------------------------------------------

typedef enum {
    FCV_THRESH_BINARY      = 0,   // value = value > threshold ? max_value : 0      
    FCV_THRESH_BINARY_INV       // value = value > threshold ? 0 : max_value
} fcvThreshType;


//---------------------------------------------------------------------------
/// @brief
///   Binarizes a grayscale image based on an adaptive threshold value calculated from 3x3 Gaussian kernel.  
///
/// @details
///   For each pixel, the threshold is computed adaptively based on cross-correlation with a
///   3x3 Gaussian kernel minus value (parameter). The standard deviation is used for Gaussian kernel.
///   For FCV_THRESH_BINARY threshold type, the pixel is set as maxValue if it's value is greater than the threshold;
///   else, it is set as zero. For FCV_THRESH_BINARY_INV threshold type, the pixel is set as zero if it's value is greater than the threshold;
///   else, it is set as maxValue.   
///
/// @param src
///   Pointer to the 8-bit input image.
///
/// @param srcWidth
///   Width of source images pointed by src.
///
/// @param srcHeight
///   Height of source images pointed by src.
///
/// @param srcStride
///   Stride of source image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///
/// @param maxValue
///   The maximum integer value to be used. 0<maxValue<256.
///
/// @param thresholdType
///   Threshold type. It could be either FCV_THRESH_BINARY or FCV_THRESH_BINARY_INV.
///
/// @param value 
///   The constant value subtracted after the cross-correlation with Gaussian kernel. 
///   It is usually positive but could be 0 or negative too.
///
/// @param dst
///   Pointer to the 8-bit destination image. Destination iamge has the same size as input image. 
///
/// @param dstStride
///   Stride of destination image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///
/// @ingroup image_processing
//---------------------------------------------------------------------------

FASTCV_API void
fcvAdaptiveThresholdGaussian3x3u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );

//---------------------------------------------------------------------------
/// @brief
///   Binarizes a grayscale image based on an adaptive threshold value calculated from 5x5 Gaussian kernel.  
///
/// @details
///   For each pixel, the threshold is computed adaptively based on cross-correlation with a
///   5x5 Gaussian kernel minus value (parameter). The standard deviation is used for Gaussian kernel.
///   For FCV_THRESH_BINARY threshold type, the pixel is set as maxValue if it's value is greater than the threshold;
///   else, it is set as zero. For FCV_THRESH_BINARY_INV threshold type, the pixel is set as zero if it's value is greater than the threshold;
///   else, it is set as maxValue.    
///
/// @param src
///   Pointer to the 8-bit input image.
///
/// @param srcWidth
///   Width of source images pointed by src.
///
/// @param srcHeight
///   Height of source images pointed by src.
///
/// @param srcStride
///   Stride of source image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///
/// @param maxValue
///   The maximum integer value to be used. 0<maxValue<256.
///
/// @param thresholdType
///   Threshold type. It could be either FCV_THRESH_BINARY or FCV_THRESH_BINARY_INV.
///
/// @param value 
///   The constant value subtracted after the cross-correlation with Gaussian kernel. 
///   It is usually positive but could be 0 or negative too.
///
/// @param dst
///   Pointer to the 8-bit destination image. Destination iamge has the same size as input image. 
///
/// @param dstStride
///   Stride of destination image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///
/// @ingroup image_processing
//---------------------------------------------------------------------------
FASTCV_API void
fcvAdaptiveThresholdGaussian5x5u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );

//---------------------------------------------------------------------------
/// @brief
///   Binarizes a grayscale image based on an adaptive threshold value calculated from 11x11 Gaussian kernel.  
///
/// @details
///   For each pixel, the threshold is computed adaptively based on cross-correlation with a
///   11x11 Gaussian kernel minus value (parameter). The standard deviation is used for Gaussian kernel.
///   For FCV_THRESH_BINARY threshold type, the pixel is set as maxValue if it's value is greater than the threshold;
///   else, it is set as zero. For FCV_THRESH_BINARY_INV threshold type, the pixel is set as zero if it's value is greater than the threshold;
///   else, it is set as maxValue.  
///
/// @param src
///   Pointer to the 8-bit input image.
///
/// @param srcWidth
///   Width of source images pointed by src.
///
/// @param srcHeight
///   Height of source images pointed by src.
///
/// @param srcStride
///   Stride of source image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///
/// @param maxValue
///   The maximum integer value to be used. 0<maxValue<256.
///
/// @param thresholdType
///   Threshold type. It could be either FCV_THRESH_BINARY or FCV_THRESH_BINARY_INV.
///
/// @param value 
///   The constant value subtracted after the cross-correlation with Gaussian kernel. 
///   It is usually positive but could be 0 or negative too.
///
/// @param dst
///   Pointer to the 8-bit destination image. Destination iamge has the same size as input image. 
///
/// @param dstStride
///   Stride of destination image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///
/// @ingroup image_processing
//---------------------------------------------------------------------------
FASTCV_API void
fcvAdaptiveThresholdGaussian11x11u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );

//---------------------------------------------------------------------------
/// @brief
///   Binarizes a grayscale image based on an adaptive threshold value calculated from 3x3 mean.  
///
/// @details
///   For each pixel, the threshold is computed adaptively based on the mean of 3x3 block centered on the pixel 
///   minus value (parameter). For FCV_THRESH_BINARY threshold type, the pixel is set as maxValue if it's value is greater than the threshold;
///   else, it is set as zero. For FCV_THRESH_BINARY_INV threshold type, the pixel is set as zero if it's value is greater than the threshold;
///   else, it is set as maxValue.  
///
/// @param src
///   Pointer to the 8-bit input image.
///
/// @param srcWidth
///   Width of source images pointed by src.
///
/// @param srcHeight
///   Height of source images pointed by src.
///
/// @param srcStride
///   Stride of source image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///
/// @param maxValue
///   The maximum integer value to be used. 0<maxValue<256.
///
/// @param thresholdType
///   Threshold type. It could be either FCV_THRESH_BINARY or FCV_THRESH_BINARY_INV.
///
/// @param value 
///   The constant value subtracted from the mean. 
///   It is usually positive but could be 0 or negative too.
///
/// @param dst
///   Pointer to the 8-bit destination image. Destination iamge has the same size as input image. 
///
/// @param dstStride
///   Stride of destination image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///
/// @ingroup image_processing
//---------------------------------------------------------------------------
FASTCV_API void
fcvAdaptiveThresholdMean3x3u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );


//---------------------------------------------------------------------------
/// @brief
///   Binarizes a grayscale image based on an adaptive threshold value calculated from 5x5 mean.  
///
/// @details
///   For each pixel, the threshold is computed adaptively based on the mean of 5x5 block centered on the pixel 
///   minus value (parameter). For FCV_THRESH_BINARY threshold type, the pixel is set as maxValue if it's value is greater than the threshold;
///   else, it is set as zero. For FCV_THRESH_BINARY_INV threshold type, the pixel is set as zero if it's value is greater than the threshold;
///   else, it is set as maxValue.  
///
/// @param src
///   Pointer to the 8-bit input image.
///
/// @param srcWidth
///   Width of source images pointed by src.
///
/// @param srcHeight
///   Height of source images pointed by src.
///
/// @param srcStride
///   Stride of source image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///
/// @param maxValue
///   The maximum integer value to be used. 0<maxValue<256.
///
/// @param thresholdType
///   Threshold type. It could be either FCV_THRESH_BINARY or FCV_THRESH_BINARY_INV.
///
/// @param value 
///   The constant value subtracted from the mean. 
///   It is usually positive but could be 0 or negative too.
///
/// @param dst
///   Pointer to the 8-bit destination image. Destination iamge has the same size as input image. 
///
/// @param dstStride
///   Stride of destination image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///
/// @ingroup image_processing
//---------------------------------------------------------------------------
FASTCV_API void
fcvAdaptiveThresholdMean5x5u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );
//---------------------------------------------------------------------------
/// @brief
///   Binarizes a grayscale image based on an adaptive threshold value calculated from 11x11 mean.  
///
/// @details
///   For each pixel, the threshold is computed adaptively based on the mean of 11x11 block centered on the pixel 
///   minus value (parameter). For FCV_THRESH_BINARY threshold type, the pixel is set as maxValue if it's value is greater than the threshold;
///   else, it is set as zero. For FCV_THRESH_BINARY_INV threshold type, the pixel is set as zero if it's value is greater than the threshold;
///   else, it is set as maxValue.  
///
/// @param src
///   Pointer to the 8-bit input image.
///
/// @param srcWidth
///   Width of source images pointed by src.
///
/// @param srcHeight
///   Height of source images pointed by src.
///
/// @param srcStride
///   Stride of source image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///
/// @param maxValue
///   The maximum integer value to be used. 0<maxValue<256.
///
/// @param thresholdType
///   Threshold type. It could be either FCV_THRESH_BINARY or FCV_THRESH_BINARY_INV.
///
/// @param value 
///   The constant value subtracted from the mean. 
///   It is usually positive but could be 0 or negative too.
///
/// @param dst
///   Pointer to the 8-bit destination image. Destination iamge has the same size as input image. 
///
/// @param dstStride
///   Stride of destination image (i.e., number of bytes between column 0 
///   of row 0 and column 0 of row 1).
///
/// @ingroup image_processing
//---------------------------------------------------------------------------
FASTCV_API void
fcvAdaptiveThresholdMean11x11u8( const uint8_t* __restrict src,
                        uint32_t             srcWidth,
                        uint32_t             srcHeight,
                        uint32_t             srcStride,
                        uint8_t              maxValue,
                        fcvThreshType        thresholdType,
                        int32_t              value,
                        uint8_t* __restrict  dst,
                        uint32_t             dstStride );

//---------------------------------------------------------------------------
/// @brief
///   Smooth a uint8_t image with a 3x3 box filter
///
/// @detailed
///   smooth with 3x3 box kernel and normalize:
///   \n[ 1 1 1
///   \n  1 1 1
///   \n  1 1 1 ]/9
///
/// @param src
///   Input uint8_t image.
///   
/// @param srcWidth
///   Input image width.
///
/// @param srcHeight
///   Input image height.
///
/// @param srcStride
///   Input image stride, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///   Output image which has the same type, and size as the input image.
///   
/// @param dstStride
///   Output image stride, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup image_processing
//---------------------------------------------------------------------------

FASTCV_API void 
fcvBoxFilter3x3u8( const uint8_t* __restrict src, 
                         uint32_t            srcWidth,
                         uint32_t            srcHeight,
                         uint32_t            srcStride, 
                         uint8_t* __restrict dst, 
                         uint32_t            dstStride 
                   );

//---------------------------------------------------------------------------
/// @brief
///   Smooth a uint8_t image with a 5x5 box filter
///
/// @detailed
///   smooth with 5x5 box kernel and normalize:
///   \n[ 1 1 1 1 1
///   \n  1 1 1 1 1
///   \n  1 1 1 1 1
///   \n  1 1 1 1 1
///   \n  1 1 1 1 1 ]/25
///   
/// @param src
///   Input uint8_t image.
///   
/// @param srcWidth
///   Input image width.
///
/// @param srcHeight
///   Input image height.
///
/// @param srcStride
///   Input image stride, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///   Output image which has the same type, and size as the input image.
///   
/// @param dstStride
///   Output image stride, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup image_processing
//---------------------------------------------------------------------------

FASTCV_API void 
fcvBoxFilter5x5u8( const uint8_t* __restrict src, 
                         uint32_t            srcWidth,
                         uint32_t            srcHeight,
                         uint32_t            srcStride, 
                         uint8_t* __restrict dst, 
                         uint32_t            dstStride 
                   );

//---------------------------------------------------------------------------
/// @brief
///   Smooth a uint8_t image with a 11x11 box filter
///
/// @detailed
///   smooth with 11x11 box kernel and normalize:
///
/// @param src
///   Input uint8_t image.
///   
/// @param srcWidth
///   Input image width.
///
/// @param srcHeight
///   Input image height.
///
/// @param srcStride
///   Input image stride, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///   Output image which has the same type, and size as the input image.
///   
/// @param dstStride
///   Output image stride, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup image_processing
//---------------------------------------------------------------------------

FASTCV_API void 
fcvBoxFilter11x11u8(const uint8_t* __restrict src, 
                          uint32_t            srcWidth,
                          uint32_t            srcHeight,
                          uint32_t            srcStride, 
                          uint8_t* __restrict dst, 
                          uint32_t            dstStride 
                   );


//---------------------------------------------------------------------------
/// @brief
///   bilateral smoothing with a 5x5 bilateral kernel
///
/// @detailed
///   The bilateral filter applied here considered 5-pixel diameter of each pixel's neighborhood 
/// and both the filter sigma in color space and the sigma in coordinate space are set to 50 
///
/// @param src
///   Input uint8_t image.
///   
/// @param srcWidth
///   Input image width.
///
/// @param srcHeight
///   Input image height.
///
/// @param srcStride
///   Input image stride, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///   Output image which has the same type, and size as the input image.
///   
/// @param dstStride
///   Output image stride, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup image_processing
//---------------------------------------------------------------------------
FASTCV_API void 
fcvBilateralFilter5x5u8(const uint8_t* __restrict src, 
                               uint32_t            srcWidth,
                               uint32_t            srcHeight,
                               uint32_t            srcStride, 
                               uint8_t* __restrict dst, 
                               uint32_t            dstStride 
                        );


//---------------------------------------------------------------------------
/// @brief
///   Bilateral smoothing with 7x7 bilateral kernel
///
/// @detailed
///   The bilateral filter applied here considered 7-pixel diameter of each pixel's neighborhood 
/// and both the filter sigma in color space and the sigma in coordinate space are set to 50 
///
/// @param src
///   Input uint8_t image.
///   
/// @param srcWidth
///   Input image width.
///
/// @param srcHeight
///   Input image height.
///
/// @param srcStride
///   Input image stride, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///   Output image which has the same type, and size as the input image.
///   
/// @param dstStride
///   Output image stride, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup image_processing
//---------------------------------------------------------------------------
FASTCV_API void 
fcvBilateralFilter7x7u8(const uint8_t* __restrict src, 
                               uint32_t            srcWidth,
                               uint32_t            srcHeight,
                               uint32_t            srcStride, 
                               uint8_t* __restrict dst, 
                               uint32_t            dstStride 
                    );

//---------------------------------------------------------------------------
/// @brief
///   Bilateral smoothing with 9x9 bilateral kernel
///
/// @detailed
///   The bilateral filter applied here considered 9-pixel diameter of each pixel's neighborhood 
/// and both the filter sigma in color space and the sigma in coordinate space are set to 50 
///
/// @param src
///   Input uint8_t image.
///   
/// @param srcWidth
///   Input image width.
///
/// @param srcHeight
///   Input image height.
///
/// @param srcStride
///   Input image stride, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///   Output image which has the same type, and size as the input image.
///   
/// @param dstStride
///   Output image stride, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///
/// @ingroup image_processing
//---------------------------------------------------------------------------
FASTCV_API void 
fcvBilateralFilter9x9u8(const uint8_t* __restrict src, 
                               uint32_t            srcWidth,
                               uint32_t            srcHeight,
                               uint32_t            srcStride, 
                               uint8_t* __restrict dst, 
                               uint32_t            dstStride );

//---------------------------------------------------------------------------
/// @brief
///   This function will remove small patches in the source image based on the input threshold.
///
/// @detailed
///   The function will remove the small contoured area of the source image. The input is a 8 bit 
/// grayscale image, where zero value denotes the background. The function first extract all the 
/// external contours of the source image. The function then loops through all the countours and check 
/// the perimeter of each contour. If the perimeter is smaller than the input parameter perimscale,
/// the function will remove that contour, otherwise, it will find the convex hull or polygonal 
/// approximation of that countour and draw it on the original image. In the current implementation,
/// since convex hull or polygonal approximation has not been implemented, the function will draw 
/// the corresponding external contour directly and fill it. 
///
/// @param src
///   The input image/patch. Must be 8 bit grayscale and zero value indicates the background.
///
/// @param srcWidth
///   The width of the input source image.
///
/// @param srcHeight
///   The height of the input source image.
///
/// @param srcStride
///   The stride of the input source image (i.e., how many bytes between column 0 of row 1 and
///   column 0 of row 2).
///
/// @param Polygonal
///   If it is 0 then we use convex hull to do approximation on the original contour, otherwise we do
/// polygonal approximation. Currently it simple use the original contour, the parameter will be
/// valid after the convex hull or polygonal approximation function is ready.
///
/// @param perimScale
///   The minimum perimscale of the contours; If a contour's perimeter is smaller than this value,
/// It will be removed from the original source image.
///
/// @return
///   No return value.
///
/// @ingroup image_processing
//------------------------------------------------------------------------------
FASTCV_API void
fcvSegmentFGMasku8(uint8_t* __restrict    src,
                   uint32_t               srcWidth,
                   uint32_t               srcHeight,
                   uint32_t               srcStride,
                   uint8_t                Polygonal,
                   uint32_t               perimScale);

//------------------------------------------------------------------------------
/// @brief
///   Computes the per-element absolute difference between two 
///   uint8_t matrices
///
/// @details
///   
///
/// @param src1
///    The first input matrix
///
/// @param src2
///    Second input matrix which has the same width and length as src1
///
/// @param srcWidth
///    Input matrix width
///
/// @param srcHeight
///    Input matrix height
///
/// @param srcStride
///    Stride for the input matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row 
///
/// @param dst
///    Output matrix which has the same width and length as src1
///
/// @param dstStride
///   Stride for output image, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row 
///
/// @return
///   No return value
///    
/// @ingroup math_vector 
//------------------------------------------------------------------------------
FASTCV_API void
fcvAbsDiffu8(const uint8_t * __restrict src1, 
             const uint8_t * __restrict src2,   
                   uint32_t             srcWidth,
                   uint32_t             srcHeight,
                   uint32_t             srcStride,
                   uint8_t * __restrict dst,
                   uint32_t             dstStride );

//------------------------------------------------------------------------------
/// @brief
///   Computes the per-element absolute difference between two 
///   int32_t matrices
///
/// @details
///   
///
/// @param src1
///    The first input matrix
///
/// @param src2
///    Second input matrix which has the same width and length as src1
///
/// @param srcWidth
///    Input matrix width
///
/// @param srcHeight
///    Input matrix height
///
/// @param srcStride
///    Stride for the input matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///    Output matrix which has the same width and length as src1
///
/// @param dstStride
///   Stride for output image, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row 
///
/// @return
///   No return value
///    
/// @ingroup math_vector 
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffs32(const int32_t * __restrict  src1, 
              const int32_t * __restrict  src2,   
                    uint32_t              srcWidth,
                    uint32_t              srcHeight,
                    uint32_t              srcStride,
                    int32_t * __restrict  dst,
                    uint32_t              dstStride );

//------------------------------------------------------------------------------
/// @brief
///   Computes the per-element absolute difference between two 
///   float32_t matrices
///
/// @details
///   
///
/// @param src1
///    First input matrix
///
/// @param src2
///    Second input matrix which has the same width and length as src1
///
/// @param srcWidth
///    Input matrix width
///
/// @param srcHeight
///    Input matrix height
///
/// @param srcStride
///    Stride for the input matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///    Output matrix which has the same width and length as src1
///
/// @param dstStride
///   Stride for output image, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///    
/// @ingroup math_vector 
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDifff32(const float32_t * __restrict  src1, 
              const float32_t * __restrict  src2,    
                    uint32_t                srcWidth,
                    uint32_t                srcHeight,
                    uint32_t                srcStride,
                    float32_t * __restrict  dst,
                    uint32_t                dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Computes the per-element absolute difference between one matrix and one value 
///
/// @details
///   
///
/// @param src
///    Input matrix
///
/// @param value
///    Input value
///
/// @param srcWidth
///    Input matrix width
///
/// @param srcHeight
///    Input matrix height
///
/// @param srcStride
///    Stride for the input matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst 
///    Output matrix which has the same width and length as src
///
/// @param dstStride
///   Stride for output image, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row 
///
/// @return
///   No return value
///    
/// @ingroup math_vector 
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVu8(const uint8_t * __restrict src, 
                    uint8_t              value,   
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
                    uint8_t * __restrict dst,
                    uint32_t             dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Computes the per-element absolute difference between one matrix and one value 
///
/// @details
///   
///
/// @param src
///    Input matrix
///
/// @param value
///    Input value
///
/// @param srcWidth
///    Input matrix width
///
/// @param srcHeight
///    Input matrix height
///
/// @param srcStride
///    Stride for the input matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///    Output matrix which has the same width and length as src
///
/// @param dstStride
///   Stride for output image , i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///    
/// @ingroup math_vector 
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVs32(const int32_t * __restrict src, 
                     int32_t              value, 
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
                     int32_t * __restrict dst,
                     uint32_t             dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Computes the per-element absolute difference between one matrix and one value 
///
/// @details
///   
///
/// @param src
///    Input matrix
///
/// @param value
///    Input value
///
/// @param srcWidth
///    Input matrix width
///
/// @param srcHeight
///    Input matrix height
///
/// @param srcStride
///    Stride for the input matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///    Output matrix which has the same width and length as src
///
/// @param dstStride
///   Stride for output image, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row 
///
/// @return
///   No return value
///    
/// @ingroup math_vector 
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVf32(const float32_t * __restrict src, 
                     float32_t              value,   
                     uint32_t               srcWidth,
                     uint32_t               srcHeight,
                     uint32_t               srcStride,
                     float32_t * __restrict dst,
                     uint32_t               dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Computes the per-element absolute difference between one 4-channel matrix and a 4-element Scalar  
///
/// @details
///   
///
/// @param src
///    Input matrix
///
/// @param value1
///    First value of the Scalar
///
/// @param value2
///    Second value of the Scalar
///
/// @param value3
///    Third value of the Scalar
///
/// @param value4
///    Fourth value of the Scalar
///
/// @param srcWidth
///    Input matrix width
///
/// @param srcHeight
///    Input matrix height
///
/// @param srcStride
///    Stride for the input matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///    Output matrix which has the same width, length and channel number as src
///
/// @param dstStride
///   Stride for output image, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row 
///
/// @return
///   No return value
///    
/// @ingroup math_vector 
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVc4u8(const uint8_t * __restrict src, 
                    uint8_t              value1,
                    uint8_t              value2,
                    uint8_t              value3,
                    uint8_t              value4,
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
                    uint8_t * __restrict dst,
                    uint32_t             dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Computes the per-element absolute difference between one 4-channel matrix and a 4-element Scalar 
/// 
/// @details
///   
///
/// @param src
///    Input matrix
///
/// @param value1
///    First value of the Scalar
///
/// @param value2
///    Second value of the Scalar
///
/// @param value3
///    Third value of the Scalar
///
/// @param value4
///    Fourth value of the Scalar
///
/// @param srcWidth
///    Input matrix width
///
/// @param srcHeight
///    Input matrix height
///
/// @param srcStride
///    Stride for the input matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///    Output matrix which has the same width, length and channel number as src
///
/// @param dstStride
///   Stride for output image, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row 
///
/// @return
///   No return value
///    
/// @ingroup math_vector 
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVs32c4(const int32_t * __restrict src, 
                     int32_t              value1,
                     int32_t              value2,
                     int32_t              value3,
                     int32_t              value4,
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
                     int32_t * __restrict dst,
                     uint32_t             dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Computes the per-element absolute difference between one 4-channel matrix and a 4-element Scalar 
///
/// @details
///   
///
/// @param src
///    Input matrix
///
/// @param value1
///    First value of the Scalar
///
/// @param value2
///    Second value of the Scalar
///
/// @param value3
///    Third value of the Scalar
///
/// @param value4
///    Fourth value of the Scalar
///
/// @param srcWidth
///    Input matrix width
///
/// @param srcHeight
///    Input matrix height
///
/// @param srcStride
///    Stride for the input matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///    Output matrix which has the same width, length and channel number as src
///
/// @param dstStride
///   Stride for output image , i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @return
///   No return value
///    
/// @ingroup math_vector 
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVc4f32(const float32_t * __restrict src, 
                     float32_t              value1,   
                     float32_t              value2,
                     float32_t              value3,
                     float32_t              value4,
                     uint32_t               srcWidth,
                     uint32_t               srcHeight,
                     uint32_t               srcStride,
                     float32_t * __restrict dst,
                     uint32_t               dstStride);

//------------------------------------------------------------------------------
/// @brief
///   Computes the per-element absolute difference between one 3-channel matrix and a 3-element Scalar  
///
/// @details
///   
///
/// @param src
///    Input matrix
///
/// @param value1
///    First value of the Scalar
///
/// @param value2
///    Second value of the Scalar
///
/// @param value3
///    Third value of the Scalar
///
/// @param srcWidth
///    Input matrix width
///
/// @param srcHeight
///    Input matrix height
///
/// @param srcStride
///    Stride for the input matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///    Output matrix which has the same width, length and channel number as src
///
/// @param dstStride
///   Stride for output image, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row 
///
/// @return
///   No return value
///    
/// @ingroup math_vector 
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVc3u8(const uint8_t * __restrict src, 
                    uint8_t              value1,
                    uint8_t              value2,
                    uint8_t              value3,
                    uint32_t             srcWidth,
                    uint32_t             srcHeight,
                    uint32_t             srcStride,
                    uint8_t * __restrict dst,
                    uint32_t             dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Computes the per-element absolute difference between one 3-channel matrix and a 3-element Scalar 
/// 
/// @details
///   
///
/// @param src
///    Input matrix
///
/// @param value1
///    First value of the Scalar
///
/// @param value2
///    Second value of the Scalar
///
/// @param value3
///    Third value of the Scalar
///
/// @param srcWidth
///    Input matrix width
///
/// @param srcHeight
///    Input matrix height
///
/// @param srcStride
///    Stride for the input matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///    Output matrix which has the same width, length and channel number as src
///
/// @param dstStride
///   Stride for output image, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row 
///
/// @return
///   No return value
///
/// @ingroup math_vector 
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVc3s32(const int32_t * __restrict src, 
                     int32_t              value1,
                     int32_t              value2,
                     int32_t              value3,
                     uint32_t             srcWidth,
                     uint32_t             srcHeight,
                     uint32_t             srcStride,
                     int32_t * __restrict dst,
                     uint32_t             dstStride );


//------------------------------------------------------------------------------
/// @brief
///   Computes the per-element absolute difference between one 3-channel matrix and a 3-element Scalar 
///
/// @details
///   
///
/// @param src
///    Input matrix
///
/// @param value1
///    First value of the Scalar
///
/// @param value2
///    Second value of the Scalar
///
/// @param value3
///    Third value of the Scalar
///
/// @param srcWidth
///    Input matrix width
///
/// @param srcHeight
///    Input matrix height
///
/// @param srcStride
///    Stride for the input matrix, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row
///
/// @param dst
///    Output matrix which has the same width, length and channel number as src
///
/// @param dstStride
///   Stride for output image, i.e. the gap (in terms of bytes) between the first element of a row and that of the successive row 
///
/// @return
///   No return value
///    
/// @ingroup math_vector 
//------------------------------------------------------------------------------

FASTCV_API void
fcvAbsDiffVc3f32(const float32_t * __restrict src, 
                     float32_t              value1,   
                     float32_t              value2,
                     float32_t              value3,
                     uint32_t               srcWidth,
                     uint32_t               srcHeight,
                     uint32_t               srcStride,
                     float32_t * __restrict dst,
                     uint32_t               dstStride);

// -----------------------------------------------------------------------------
/// @brief
///   create KDTrees for dataset of 36D vectors
///
/// @details
///   KDTree is very efficient search structure for multidimensional data.
///   Usage:
///   assume we have 36D of type int8_t data e.g. target image feature 
///   descriptors in array named vectors
///
///      int8_t* descriptors;
///
///   the number of descriptors is numVectors
///
///      int numDescriptors;
///
///   we compute inverse length of each descriptor as floating point number
///   and store them in array invLengths
///
///      float32_t* invLenghts;
///     
///   we declare pointer to KDTree structure
///         
///      fcvKDTreeDatas8f32* kdTree = 0;
///
///   and create it
///
///      int err = fcvKDTreeCreate36s8f32( descriptors, invLengths, 
///                                        numDescriptors, kdTree );
/// @param vectors
///   pointer to dataset being array of 36D vectors
///
/// @param invLengths
///   array of inverse lengths for each vector in the dataset
///
/// @param numVectors
///   number of 36D vectors in the dataset
///
/// @param kdtrees
///   address for pointer to the newly created KDTrees
///
/// @return
///   0      - success
///   EINVAL - invalid parameter
///   ENOMEM - not enough memory
///   -1     - other error
///
/// @ingroup feature_detection
// -----------------------------------------------------------------------------
FASTCV_API 
int fcvKDTreeCreate36s8f32( const        int8_t*  __restrict vectors,
                            const     float32_t*  __restrict invLengths,
                                            int              numVectors,
                             fcvKDTreeDatas8f32**            kdtrees );

// -----------------------------------------------------------------------------
/// @brief
///   release KDTrees data structures
///
/// @details
///   Once we are done with all searches we should release kdTree resources
///
/// @param kdtrees
///   KDTrees to be destroyed
///
/// @return
///   0      - success
///   EINVAL - invalid parameter
///   -1     - other error
///
/// @ingroup feature_detection
// -----------------------------------------------------------------------------
FASTCV_API int fcvKDTreeDestroy36s8f32( fcvKDTreeDatas8f32* kdtrees );

// -----------------------------------------------------------------------------
/// @brief
///   find nearest neighbors (NN) for query
///
/// @details
///   Assuming KD tree creation is successful we may start using our kdTree
///   for nearest neighbors (NN) for descriptors of camera features. Let our
///   camera descriptors be in array camDescriptors and their number
///   in numCamDescriptors
///
///      int8_t* camDescriptors;
///      int numCamDescriptors;
///
///   we compute their inverse lengths as floating point numbers and provide
///   them in
///
///      float* camDescriptorsInvLengths;
///         
///   now for each camera descriptor we may search for its NNs among
///   target image descriptors. Assume we want to find 8 NNs for each camera
///   descriptor. We declare variables for results of NN searches
///
///      #define NUM_NN 8                 // number of NN required
///      #define MAX_CHECKS 32            // max number of checks in kdtree
///
///      int32_t numFound = 0;            // for numer of NNs found
///      int32_t foundInds[ NUM_NN ];     // for indices to target descriptors
///      float32_t foundDists[ NUM_NN ];  // for distances to target descriptors
///      float32_t maxDist = 0.1f;        // max distance to query allowed
///      
///   the search for NNs for i-th query would be like this
///
///      err = fcvKDTreeQuery36s8f32( kdTree, camDescriptors + i * 36, 
///            camDescriptorsInvLengths[ i ], maxDist, MAX_CHECKS, 0,
///            &numFound, foundInds, foundDists );
///
///   where maxDists is an upper bound on distance of NN from the query
///   and MAX_CHECKS is max number of comparisons of query to target
///   descriptors. The higher MAX_CHECKS the better NNs we get at the cost
///   of longer search. Assuming everything went fine will return us
///   search results. numFound will contain the number of target descriptors
///   found whose distance to query is less than maxDist. foundInds will
///   contain indices to target descriptors being NNs and foundDists their
///   distances to query.
///
/// @param kdtrees
///   KDTrees
///
/// @param query
///   query vector
///
/// @param queryInvLen
///   inverse length of query vector
///
/// @param maxNNs
///   max number of NNs to be found
///
/// @param maxDist
///   max distance between NN and query
///
/// @param maxChecks
///   max number of leafs to check
///
/// @param mask
///   array of flags for all vectors in the dataset; may be NULL;
///   if not NULL then its legth must be equal to number of dataset
///   vectors and i-th mask corresponds to i-th vector; values:
///      0x00 - corresponding vector must not be considered NN regardless
///             of its distance to query
///      0xFF - corresponding vector may be candidate for NN
///      other - not supported
/// @param numNNsFound
///   for number of NNs found
///
/// @param NNinds
///   array for indices of found NNs; must have maxNNs length
///
/// @param NNdists
///   array for NN distances to query; must have maxNNs length
///
/// @return
///   0      - success
///   EINVAL - invalid parameter
///   -1     - other error
///
/// @ingroup feature_detection
// -----------------------------------------------------------------------------
FASTCV_API 
int fcvKDTreeQuery36s8f32( fcvKDTreeDatas8f32*       kdtrees,
                           const  int8_t* __restrict query,
                               float32_t             queryInvLen,
                                     int             maxNNs,
                               float32_t             maxDist,
                                     int             maxChecks,
                           const uint8_t* __restrict mask,
                                 int32_t*             numNNsFound,
                                 int32_t* __restrict NNInds,
                               float32_t* __restrict NNDists );

#ifdef __cplusplus
}
#endif


#ifndef FASTCV_NO_INLINE_FUNCTIONS
#include "fastcv.inl"
#endif


#endif
