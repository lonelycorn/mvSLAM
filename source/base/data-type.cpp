#include <base/data-type.hpp>
namespace mvSLAM
{
constexpr Id::Type Id::INVALID;

KeyPointImage::KeyPointImage():
    id(Id::INVALID),
    key_points(),
    image()
{
}

KeyPointImagePair::KeyPointImagePair():
    base(),
    pair()
{
}

}
