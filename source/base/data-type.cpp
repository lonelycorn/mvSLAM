#include <base/data-type.hpp>
namespace mvSLAM
{
constexpr Id::Type Id::INVALID;

KeyPointImage::KeyPointImage():
    id(Id::INVALID),
    keypoints(),
    image()
{
}

KeyPointImagePair::KeyPointImagePair():
    base(),
    pair()
{
}

}
