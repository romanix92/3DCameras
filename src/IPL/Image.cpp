/**
 * @file   Image.cpp
 * @brief  Implementation of an Image class.
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

#include "Image.h"

namespace ipl
{
    Image::Image()
    {
        m_grayScale = NULL;
        m_RGB24 = NULL;
    }

    Image::Image(const Image& other)
    {
        if (&other != this)
            copyImage(other);
    }

    Image::~Image()
    {
        delete[] m_RGB24;
        delete[] m_grayScale;
    }

    Image& Image::operator=(const Image& other)
    {
        if (this == &other)
            return *this;
        copyImage(other);
        return *this;
    }

    void Image::copyImage(const Image& other)
    {
        const size_t pixNum = other.m_size.w * other.m_size.h;
        bool needRealloc = false;
        if (pixNum > m_size.w * m_size.h)
            needRealloc = true;
        m_size = other.m_size;

        if (other.m_grayScale)
        {
            if (needRealloc)
            {
                if(m_grayScale)
                    realloc(m_grayScale, sizeof(uint8_t) * pixNum);
                else
                    m_grayScale = new uint8_t[pixNum];
            }
            memcpy(m_grayScale, other.m_grayScale, pixNum * sizeof(uint8_t));
        }
        else
            delete[] m_grayScale;

        if (other.m_RGB24)
        {
            if (needRealloc)
            {
                if(m_RGB24)
                    realloc(m_RGB24, 3 * sizeof(uint8_t) * pixNum);
                else
                    m_RGB24 = new uint8_t[3 * pixNum];
            }
            memcpy(m_RGB24, other.m_RGB24, 3 * pixNum * sizeof(uint8_t));
        }
        else
            delete[] m_RGB24;
    }

    uint8_t * Image::grayScale()
    {
        return m_grayScale;
    }

    uint8_t * Image::RGB24()
    {
        return m_RGB24;
    }

    void Image::grayScale(uint8_t * dst) const
    {
        memcpy(dst, m_grayScale, sizeof(uint8_t) * m_size.w * m_size.h);
    }

    void Image::RGB24(uint8_t * dst) const
    {
        memcpy(dst, m_RGB24, 3 * sizeof(uint8_t) * m_size.w * m_size.h);
    }

    ImagePtr Image::fromGrayData(ImageSize size, const uint8_t * data, bool deep /* = true */)
    {
        return (ImagePtr)NULL;
    }

    ImagePtr Image::fromRGB24Data(ImageSize size, const uint8_t * data, bool deep /* = true */)
    {
        return (ImagePtr)NULL;
    }

    ImageSize::ImageSize()
    {
        w = 0; h = 0;
    }

    ImageSize::ImageSize(uint16_t width, uint16_t height)
    {
        w = width; h = height;
    }
}
