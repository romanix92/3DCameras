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

namespace ipl
{
	typedef Image * ImagePtr;
	
	/// \class Image
	///
	/// \brief Encapsulates all data, associated with a particular
    /// image and provides basic image processing operations.
	class Image
	{
	public:

		/// \brief Default constructor. Creates an empty image.
		Image();

		/// \brief Copy constructor. Creates a deep copy of an image.
		Image (const Image&);

		/// \brief Assignment operator. Creates a deep copy of an image.
		Image& opertor=(const Image& other);

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
		
		/// \brief Creates image an array with grayscale data.
		///
		static ImagePtr from8BitGrayScale(int w, int h, uint8_t * data, bool deep = true);

		/// \brief .
		uint8_t * grayScaleShallow();

		uint8_t * RGB24Shallow();

		void grayScaleDeep(uint8_t * dst) const;

		void RGB24Deep(uint8_t * dst) const;

		~Image();

	private:
		
		uint8_t * m_grayScale;
		uint8_t * m_RGB24;

		int m_width;
		int m_height;
	
	}; // class Image

} // namespace ipl

#endif