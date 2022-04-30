#include "gtest/gtest.h"
#include "hello.h"
// #include "SpatialHash.h"

TEST(HelloTests, testHello) {
    ASSERT_STREQ("Hello Jim", generateHelloString("Vis").c_str());
}