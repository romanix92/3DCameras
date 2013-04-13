#include "ImageCorrector.h"
#include <fastcv.h>
#include <cassert>

using namespace ipl;

ImageCorrector* ImageCorrector::instance()
{
    if (m_instance)
        return m_instance;
    else
    {
        m_instance = new ImageCorrector();
    }
    return m_instance;
}

ImageCorrector* ImageCorrector::m_instance = NULL;

Homography ImageCorrector::calculateCorrection(const ImagePtr& reference,
                                               const ImagePtr& distorted,
                                               CorrectionAlgorithm alg)
{
    switch (alg)
    {
        case FAST_Features:
            return ImageCorrector::FASTCorrection(reference, distorted);
        default:
            return Homography();
    }
}

ImageCorrector::~ImageCorrector()
{
    if (m_instance)
        delete m_instance;
}

Homography ImageCorrector::FASTCorrection(const ImagePtr& reference, const ImagePtr& distorted)
{
    /*assert(reference->format() == FORMAT_GRAY && distorted->format() == FORMAT_GRAY);
    assert(reference->width() & 7 == 0 && reference->height() & 7 == 0);
    const int maxNumCorn = 1024;
    uint32_t numCornRef, numCornDist;
    float res[9];
    uint32_t * cornersRef = (uint32_t*)fcvMemAlloc(maxNumCorn * sizeof(uint32_t), 16);
    fcvCornerFast9u8(reference->data(), reference->width(), reference->height(),
                     0, 10, 0, cornersRef, maxNumCorn, &numCornRef);
    uint32_t * cornersDist = (uint32_t*)fcvMemAlloc(maxNumCorn * sizeof(uint32_t), 16);
    fcvCornerFast9u8(distorted->data(), distorted->width(), distorted->height(),
        0, 10, 0, cornersDist, maxNumCorn, &numCornDist);

    //TODO

    fcvMemFree(cornersDist);
    fcvMemFree(cornersRef);*/
    return Homography();
}