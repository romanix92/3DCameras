#include "ImageTests.h"
#include "../Image.h"
#include "TestUtils.h"

using namespace ipl;

ImageTest::ImageTest()
{

    TEST_ADD(ImageTest::ImageCopyTest);

}

/*void ImageTest::grayDataTest()
{
    uint8_t srcData[] = { 0, 64, 128, 255 };
    ImagePtr im = Image::fromGrayData(ImageSize(2, 2), srcData);
    TEST_ASSERT_MSG(im.get() != NULL , "Image not created. NULL pointer returned");
    TEST_ASSERT_MSG(arraysEqual(srcData, im->grayScale(), 4), "Source arrays differs from image data");
}

void ImageTest::RGBDataTest()
{
    uint8_t srcData[] = { 255, 0, 0,
                          0, 0, 255,
                          0, 255, 0,
                          0, 0, 0 };
    ImagePtr im = Image::fromRGB24Data(ImageSize(2, 2), srcData);
    TEST_ASSERT_MSG(im.get() != NULL , "Image not created. NULL pointer returned");
    TEST_ASSERT_MSG(arraysEqual(srcData, im->RGB24(), 12), "Source arrays differs from image data");
}

void ImageTest::grayCopyTest()
{
    uint8_t data[] = { 0, 64, 128, 255 };
    ImagePtr im = Image::fromGrayData(ImageSize(2, 2), data);
    TEST_ASSERT_MSG(im.get() != NULL , "Image not created. NULL pointer returned");
    memset(data, 0, 4 * sizeof(uint8_t));
    im->grayScale(data);
    TEST_ASSERT_MSG(arraysEqual(data, im->grayScale(), 4), "Grayscale data copied incorrectly");
}

void ImageTest::RGBCopyTest()
{
    uint8_t data[] = { 255, 0, 0,
                       0, 0, 255,
                       0, 255, 0,
                       0, 0, 0 };
    ImagePtr im = Image::fromRGB24Data(ImageSize(2, 2), data);
    TEST_ASSERT_MSG(im.get() != NULL , "Image not created. NULL pointer returned");
    memset(data, 0, 12 * sizeof(uint8_t));
    im->RGB24(data);
    TEST_ASSERT_MSG(arraysEqual(data, im->RGB24(), 12), "RGB data copied incorrectly");
}*/

void ImageTest::ImageCopyTest()
{
    uint8_t data[] = {255, 0, 0,
                      0, 0, 255,
                      0, 255, 0,
                      0, 0, 0 };
    ImagePtr im1 = Image::fromMemory(ImageSize(3, 4), data, FORMAT_GRAY);
    TEST_ASSERT_MSG(im1.get() != NULL , "Image not created. NULL pointer returned");
    Image im1Copy(*im1);
    TEST_ASSERT_MSG(arraysEqual(im1->data(), im1Copy.data(), 12), "Image copied incorrectly");
}