#include "ImageTests.h"
#include "../Image.h"
#include "TestUtils.h"

using namespace ipl;

ImageTest::ImageTest()
{

    TEST_ADD(ImageTest::imageCopyTest);
    TEST_ADD(ImageTest::fromMemoryTest);
    TEST_ADD(ImageTest::dataCopyTest);
}

void ImageTest::fromMemoryTest()
{
    uint8_t grayData[] = { 0, 64, 128, 255 };
    ImagePtr im = Image::fromMemory(ImageSize(2, 2), grayData, FORMAT_GRAY);
    TEST_ASSERT_MSG(im.get() != NULL , "Image not created. NULL pointer returned");
    TEST_ASSERT_MSG(arraysEqual(grayData, im->data(), 4), "Grayscale source array differs from image data");

    uint8_t rgbData[] = { 255, 0, 0,
                            0, 0, 255,
                            0, 255, 0,
                            0, 0, 0 };
    im = Image::fromMemory(ImageSize(2, 2), rgbData, FORMAT_RGB24);
    TEST_ASSERT_MSG(im.get() != NULL , "Image not created. NULL pointer returned");
    TEST_ASSERT_MSG(arraysEqual(rgbData, im->data(), 12), "Source arrays differs from image data");
}

void ImageTest::dataCopyTest()
{
    uint8_t data[] = { 0, 64, 128, 255 };
    ImagePtr im = Image::fromMemory(ImageSize(2, 2), data, FORMAT_GRAY);
    TEST_ASSERT_MSG(im.get() != NULL , "Image not created. NULL pointer returned");
    memset(data, 0, 4 * sizeof(uint8_t));
    im->data(data);
    TEST_ASSERT_MSG(arraysEqual(data, im->data(), 4), "Grayscale data copied incorrectly");
}


void ImageTest::imageCopyTest()
{
    uint8_t data[] = {255, 0, 0,
                      0, 0, 255,
                      0, 255, 0,
                      0, 0, 0 };
    ImagePtr im1 = Image::fromMemory(ImageSize(3, 4), data, FORMAT_GRAY);
    TEST_ASSERT_MSG(im1.get() != NULL , "Image not created. NULL pointer returned");
    Image im1Copy(*im1);
    TEST_ASSERT_MSG(im1->data() != im1Copy.data(),"Copied and original images have the same buffer");
    TEST_ASSERT_MSG(im1->format() == im1Copy.format(), "Copied image has wrong format");
    TEST_ASSERT_MSG(arraysEqual(im1->data(), im1Copy.data(), 12), "Image copied incorrectly");

    Image im1AsgnCp = *im1;
    TEST_ASSERT_MSG(im1->data() != im1AsgnCp.data(),"Copied and original images have the same buffer");
    TEST_ASSERT_MSG(im1->format() == im1AsgnCp.format(), "Copied image has wrong format");
    TEST_ASSERT_MSG(arraysEqual(im1->data(), im1AsgnCp.data(), 12), "Image assigned incorrectly");
    
}