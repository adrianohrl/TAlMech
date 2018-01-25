#include <gtest/gtest.h>

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int) time(NULL));
  return RUN_ALL_TESTS();
}
