#include "Homography.h"

#include <memory>

namespace ipl
{
    Homography::Homography()
    {
        memset(m_data, 0, 9 * sizeof(float));
        m_data[0] = 1.f;
        m_data[4] = 1.f;
        m_data[8] = 1.f;
    }

    Homography::Homography(float * data)
    {
        memcpy(m_data, data, 9 * sizeof(float));
    }

    Homography::Homography( float e11, float e12, float e13,
                            float e21, float e22, float e23,
                            float e31, float e32, float e33 )
    {
        float* elem = m_data;
        *elem++ = e11;
        *elem++ = e12;
        *elem++ = e13;

        *elem++ = e21;
        *elem++ = e22;
        *elem++ = e23;

        *elem++ = e31;
        *elem++ = e32;
        *elem++ = e33;
    }

    float& Homography::operator()(int row, int col)
    {
        return m_data[3 * row + col];
    }
}