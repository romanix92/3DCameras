#include <cpptest.h>

class ImageTest : public Test::Suite
{
public:
    ImageTest();
private:
    void imageCopyTest();
    void fromMemoryTest();
    void dataCopyTest();
};