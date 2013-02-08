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

#ifndef IPL_IMAGE_H
#define IPL_IMAGE_H

#include <string>
#include <fstream>
#include <stdint.h>
#include <memory>

namespace ipl
{
    class Image;
    typedef std::auto_ptr<Image> ImagePtr;
    
    /// \struct ImageSize
    ///
    /// \brief Represents width and height of an image.
    struct ImageSize
    {
        /// \brief Initializer
        ImageSize(uint16_t width, uint16_t height);

        /// \brief Initializes with zero values
        ImageSize();

        bool operator == (ImageSize other);

        uint16_t w; /// width
        uint16_t h; /// height
    };

    /// \class Image
    ///
    /// \brief Encapsulates all data, associated with a particular
    /// image and provides basic image processing operations.
    /// 
    /// \todo Decide what functors should be in this class.
    /// Decide if I need image cache and layers.
    class Image
    {
    public:

        /// \brief Default constructor. Creates an empty image.
        Image();

        /// \brief Copy constructor. Creates a deep copy of an image.
        Image (const Image&);

        /// \brief Assignment operator. Creates a deep copy of an image.
        Image& operator=(const Image& other);

        /// \brief Creates image from a bitmap file.
        ///
        /// \param[in] fileName path to file with an image.
        /// \return A pointer to a new image
        static ImagePtr fromBitmap(const std::string& fileName);

        /// \brief Creates image from a stream with bitmap data.
        ///
        /// \param[in] source Stream with data.
        /// \return A pointer to a new image
        static ImagePtr fromBitmap(const std::istream& source);
        
        /// \brief Creates image from an array with 8-bit grayscale data.
        ///
        /// \return A pointer to a new image
        static ImagePtr fromGrayData(ImageSize size, const uint8_t * data, bool deep = true);

        /// \brief Creates image from an array with RGB24 data.
        ///
        /// \return A pointer to a new image
        static ImagePtr fromRGB24Data(ImageSize size, const uint8_t * data, bool deep = true);

        /// \brief Creates image from camera.
        ///
        /// \return A pointer to a new image
        static ImagePtr fromCamera();
        
        /// \brief Provides access to raw grayscale data
        ///
        /// \return raw 8-bit grayscale data.
        uint8_t * grayScale();

        /// \brief Provides access to raw RGB24 data
        ///
        /// \return raw 8-bit RGB24 data. NULL if there's no data
        uint8_t * RGB24();

        /// \brief Copies grayscale data
        ///
        /// \param[in] dst Destination buffer.
        void grayScale(uint8_t * dst) const;

        /// \brief Copies RGB24 data
        ///
        /// \param[in] dst Destination buffer.
        void RGB24(uint8_t * dst) const;

        ~Image();

    private:
        void copyImage(const Image& other);

        uint8_t * m_grayScale;
        uint8_t * m_RGB24;

        ImageSize m_size;
    
    }; // class Image

} // namespace ipl

#endif //IPL_IMAGE_H
