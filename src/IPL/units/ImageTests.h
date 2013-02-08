#include <cpptest.h>

class ImageTest : public Test::Suite
{
public:
    ImageTest();
private:
    void grayDataTest();
    void RGBDataTest();
    void ImageCopyTest();
    void grayCopyTest();
    void RGBCopyTest();	
};