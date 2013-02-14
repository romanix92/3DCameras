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
    
    /// \enum ImageFormat
    /// \brief Available image formats.
    enum ImageFormat
    {
        FORMAT_GRAY = (uint8_t)0, /// 8-bit grayScale
        FORMAT_RGB24,             /// Bitmap 24
        FORMAT_RGB565,            /// 16 bit bitmap
        FORMAT_YUV420SP           /// Android camera preview format. 12 bpp.
    };

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

        /// \brief Creates image from an array with data.
        ///
        /// \return A pointer to a new image
        static ImagePtr fromMemory(ImageSize size, 
                                   const uint8_t * data,
                                   ImageFormat format,
                                   bool deep = true);

        /// \brief Creates image from camera.
        ///
        /// \return A pointer to a new image
        static ImagePtr fromCamera();
        
        /// \brief Get number of bits per pixel for a
        /// format f.
        ///
        /// \param[in] f A supported format
        /// \return
        static uint8_t formatDepth(ImageFormat f);

        /// \brief Get image format
        ///
        /// \return Format
        uint8_t format();

        /// \brief Provides access to raw data
        ///
        /// \return a pointer to internal data buffer.
        uint8_t * data();

        /// \brief Copies raw data
        ///
        /// \param[out] dst A buffer to which the data will be copied.
        /// dst must be allocated before being passed to this method.
        void data(uint8_t * dst) const;

        ~Image();

    private:
        void copyImage(const Image& other);

        uint8_t * m_data;
        ImageSize m_size;
        ImageFormat m_format;

    
    }; // class Image

} // namespace ipl

#endif //IPL_IMAGE_H
