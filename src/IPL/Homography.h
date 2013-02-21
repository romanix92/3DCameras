/**
 * @file   Image.h
 * @brief  Class that represents the image.
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

#ifndef IPL_HOMOGRAPHY_H
#define IPL_HOMOGRAPHY_H

namespace ipl
{
    class Homography
    {
    public:
        
        /// \brief Initialize with identity matrix
        Homography();

        /// \brief Init with float array (row major)
        ///
        /// \param[in] data input data
        Homography( float* data);

        /// \brief Init each element explicitly
        ///
        /// \param[in] e11..e33 elements
        Homography( float e11, float e12, float e13,
                    float e21, float e22, float e23,
                    float e31, float e32, float e33 );
        /// \brief Access to internal data
        float * data();

        /// \brief get access to element by index
        float& operator()(int row, int col);

    private:
        float m_data[9];
    };
}// namespace ipl

#endif //IPL_HOMOGRAPHY_H
