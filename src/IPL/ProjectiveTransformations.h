/**
 * @file   ProjectiveTransformations.h
 * @brief  Transformation of images using homography.
 * @author Roman Balashevych
 * @version 0.01
 *
 * @section LICENSE
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301  USA
 */

#ifndef IPL_PROJ_TRANS_H
#define IPL_PROJ_TRANS_H

#include "Image.h"

namespace ipl
{
    class Homography;
    
    /// \class ProjectiveTransformations
    ///
    /// \brief Groups everything related to 
    /// perspective transformation
    static class ProjectiveTransformations
    {
        /// \brief Transforms all layers of an image
        ///
        /// \param[in] im Input image.
        /// \param[in] trans transformation.
        static void transform(ImagePtr im, const Homography& trans);

        /// \brief bilinear warp of grayscale data in array
        /// in row-major order.
        ///
        /// \param[in] src Source data.
        /// \param[in] srcSize source image size.
        /// \param[out] dst destination buffer.
        /// \param[in] dstSize destination size.
        /// \param[in] homography projective Transformation.
        static void bilinearWarpGray(const uint8_t * src, 
                                     ImageSize srcSize, 
                                     uint8_t *dst, 
                                     ImageSize dstSize, 
                                     const Homography& homography);

    }; //class ProjectiveTransformations

} //namespace ipl

#endif //IPL_PROJ_TRANS_H