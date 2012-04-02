#include "filter.hpp"
#include <gtest/gtest.h>



TEST(FilterTest, AllTrue2) {
   filter f(2);
   f.store(true);
   f.store(true);
   f.store(true);
  EXPECT_EQ(true, f.result());
}

TEST(FilterTest, AllTrue3) {
   filter f(3);
   f.store(true);
   f.store(true);
   f.store(true);
  EXPECT_EQ(true, f.result());
}

TEST(FilterTest, FirstFalse) {
   filter f(2);
   f.store(false);
   f.store(true);
   f.store(true);
  EXPECT_EQ(true, f.result());
}

TEST(FilterTest, 2ndFalse) {
   filter f(2);
   f.store(true);
   f.store(false);
   f.store(true);
  EXPECT_EQ(false, f.result());
  EXPECT_EQ(50, f.percentageNoise());
}

TEST(FilterTest, 3rdFalse) {
   filter f(2);
   f.store(true);
   f.store(true);
   f.store(false);
  EXPECT_EQ(false, f.result());
  EXPECT_EQ(50, f.percentageNoise());
}

TEST(FilterTest, AllFalse) {
   filter f(2);
   f.store(false);
   f.store(false);
   f.store(false);
   f.store(false);
  EXPECT_EQ(false, f.result());
  EXPECT_EQ(0, f.percentageNoise());
}

TEST(FilterTest, All20False) {
   filter f(20);
   for (int i=0 ; i<100 ; i++)
      f.store(false);
  EXPECT_EQ(false, f.result());
  EXPECT_EQ(0, f.percentageNoise());
}

TEST(FilterTest, All20True) {
   filter f(20);
   for (int i=0 ; i<100 ; i++)
      f.store(true);
  EXPECT_EQ(true, f.result());
  EXPECT_EQ(0, f.percentageNoise());
}

TEST(FilterTest, All20TrueExceptFirst) {
   filter f(20);
   f.store(false);
   for (int i=0 ; i<100 ; i++)
      f.store(true);
  EXPECT_EQ(true, f.result());
  EXPECT_EQ(0, f.percentageNoise());
}

TEST(FilterTest, All20TrueExceptLast) {
  filter f(20);
  for (int i=0 ; i<100 ; i++)
    f.store(true);
  f.store(false);
  EXPECT_EQ(false, f.result());
  EXPECT_EQ(100*19/20, f.percentageNoise());
}

TEST(FilterTest, GiantSlowTest) {
   filter f(20);
   f.store(false);
   for (long i=0 ; i<99999999 ; i++)
      f.store(true);
  EXPECT_EQ(true, f.result());
  EXPECT_EQ(0, f.percentageNoise());
}

TEST(FilterTest, EvenBiggerGiantSlowTest) {
   filter f(20);
   for (long i=0 ; i<999999999 ; i++)
      f.store(true);
   f.store(false);
  EXPECT_EQ(false, f.result());
  EXPECT_EQ(100*19/20, f.percentageNoise());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
