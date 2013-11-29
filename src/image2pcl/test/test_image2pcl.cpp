#include <ImageConverter.hpp>
#include <gtest/gtest.h>

using namespace image2pcl;

namespace {

TEST(Image2PclTest, instantiatesCleanly) {
    ImageConverter imageConverter;
}

}

int main(int argc, char ** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
