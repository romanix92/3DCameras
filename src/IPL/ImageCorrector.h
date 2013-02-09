/**
 * @file   ImageCorrector.h
 * @brief  Calculation of homography for frame correction.
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

#ifndef IPL_IMAGE_CORRECTOR_H
#define IPL_IMAGE_CORRECTOR_H

#include "Image.h"

namespace ipl
{
    class Homography;

    /// \enum CorrectionAlgorithms
    ///
    /// \brief Available algorithms for image correction
    enum CorrectionAlgorithm
    {
        FAST_Features = (uint8_t)0;
    };

    /// \class ImageCorrector
    ///
    /// \brief Calculates homography to compensate
    /// the distortions made by the second camera using
    /// a reference image.
    /// 
    /// \todo Think if this should be a class.
    static class ImageCorrector
    {
    public:
        /// \brief Find a homography to warp a frame so, that it
        /// would be as if it was taken from a well-positioned camera.
        /// NVI (template method) pattern
        static Homography calculateCorrection(const ImagePtr reference, const ImagePtr distorted, CorrectionAlgorithm);
    protected:
        static virtual Homography FASTCorrection(const ImagePtr reference, const ImagePtr distorted);
    };
} //namespace ipl

#endif //IPL_IMAGE_CORRECTOR_H
