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
        m_data = NULL;
    }

    Image::Image(const Image& other)
    {
        m_data = NULL;
        if (&other != this)
            copyImage(other);
    }

    Image::~Image()
    {
        delete[] m_data;
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
        m_format = other.m_format;
        bool needRealloc = false;
        if (pixNum > m_size.w * m_size.h)
            needRealloc = true;
        m_size = other.m_size;

        if (other.m_data)
        {
            if (needRealloc)
            {
                if(m_data)
                    realloc(m_data, sizeof(uint8_t) * pixNum);
                else
                    m_data = new uint8_t[pixNum];
            }
            memcpy(m_data, other.m_data, pixNum * sizeof(uint8_t));
        }
        else if (m_data)
            delete[] m_data;
    }

    uint8_t * Image::data()
    {
        return m_data;
    }


    void Image::data(uint8_t * dst) const
    {
        if (m_data)
            memcpy(dst, m_data, sizeof(uint8_t) * m_size.w * m_size.h);
    }

    ImagePtr Image::fromMemory(ImageSize size,
                               const uint8_t * data, 
                               ImageFormat format,
                               bool deep /* = true */)
    {
        ImagePtr res = (ImagePtr)new Image();
        res->m_size = size;
        const size_t byteNum = (size.w * size.h * formatDepth(format)) >> 3;
        res->m_data = new uint8_t[byteNum];
        if (deep)
            memcpy(res->m_data, data, byteNum);
        else
            res->m_data = const_cast<uint8_t*>(data);
            
        return res;
        return ImagePtr(NULL);
    }

    uint8_t Image::formatDepth(ImageFormat f)
    {
        switch (f)
        {
        case FORMAT_GRAY:
            return 8;
        case FORMAT_YUV420SP:
            return 12;
        case FORMAT_RGB565:
            return 16;
        case FORMAT_RGB24:
            return 24;
        default:
            return 0;
        }
    }

    uint8_t Image::format()
    {
        return m_format;
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
