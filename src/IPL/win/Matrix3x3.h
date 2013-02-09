/**
 * @file   Matrix3x3.h
 * @brief  Class that represents 3x3 matrix. Works faster than 
 *         matrix in general.
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

#ifndef IPL_IMAGE_H
#define IPL_IMAGE_H

// \todo Do I need matrices and that kind of staff?
namespace ipl
{
    class Matrix3x3
    {
    public:
        float det() const;
        void inverse();
        Matrix3x3 inversed();
        void transpose();
        Matrix3x3 transposed();

        Matrix3x3(float * data)
        Matrix3x3();
        Matrix3x3(float e11, float e12, float e13,
                  float e21, float e22, float e23,
                  float e31, float e32, float e33 );

        Matrix3x3 operator * (const Matrix3x3& other);
    private:
        float[9] m_data;
    };
}

#endif