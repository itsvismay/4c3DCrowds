#include "gtest/gtest.h"
#include "hello.h"

TEST(HelloTests, testHello) {
    ASSERT_STREQ("Hello Jim", generateHelloString("Vis").c_str());
}