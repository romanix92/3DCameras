#include "HomographyTest.h"

#include "../Homography.h"

#include <limits>

using namespace ipl;

static float data[] = 
{
    1.1f, 1.2f, 1.3f,
    2.1f, 2.2f, 2.3f,
    3.1f, 3.2f, 3.3f
};

HomographyTest::HomographyTest()
{
    TEST_ADD(HomographyTest::GetSetTest);
}

void HomographyTest::GetSetTest()
{
    // Check default constructor
    Homography h;
    TEST_ASSERT_DELTA_MSG(1.f, h.data()[0], std::numeric_limits<float>::epsilon(),
        "Default value is not identity matrix");
    TEST_ASSERT_DELTA_MSG(1.f, h.data()[4], std::numeric_limits<float>::epsilon(),
        "Default value is not identity matrix");
    TEST_ASSERT_DELTA_MSG(1.f, h.data()[8], std::numeric_limits<float>::epsilon(),
        "Default value is not identity matrix");
    if (h.data()[1] || h.data()[2] ||
        h.data()[3] || h.data()[5] ||
        h.data()[6] || h.data()[7] )
        TEST_FAIL("Default value is not identity matrix");

    Homography h1(data);
    for (int i = 0; i < 9; ++i)
    {
        TEST_ASSERT_DELTA_MSG(data[i], h1.data()[i], std::numeric_limits<float>::epsilon(),
            "Initialization from array failed");
    }

    Homography h2 ( data[0], data[1], data[2],
                    data[3], data[4], data[5],
                    data[6], data[7], data[8] );
    for (int i = 0; i < 9; ++i)
    {
        TEST_ASSERT_DELTA_MSG(data[i], h2.data()[i], std::numeric_limits<float>::epsilon(),
            "Initialization from 9 values failed");
    }

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            TEST_ASSERT_DELTA_MSG(h2(row, col), h2.data()[3 * row + col], std::numeric_limits<float>::epsilon(),
                "Operator () works incorrectly");
        }
    }
}