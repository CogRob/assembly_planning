#include <gtest/gtest.h>
#include "metareasoning_agent/lego_world.h"

TEST(ObjTypeTest, LengthLess) {
  ObjectType lhs, rhs;
  // lhs values
  lhs.length = 1;
  lhs.width = 1;
  lhs.color = ObjectColor::RED;
  // rhs values
  rhs.length = 2;
  rhs.width = 1;
  rhs.color = ObjectColor::RED;
  EXPECT_TRUE(lhs < rhs);
  EXPECT_FALSE(lhs < lhs);
  EXPECT_FALSE(rhs < lhs);
}
TEST(ObjTypeTest, WidthLess) {
  ObjectType lhs, rhs;
  // lhs values
  lhs.length = 1;
  lhs.width = 1;
  lhs.color = ObjectColor::RED;
  // rhs values
  rhs.length = 1;
  rhs.width = 2;
  rhs.color = ObjectColor::RED;
  EXPECT_TRUE(lhs < rhs);
  EXPECT_FALSE(rhs < lhs);
  rhs.width = 3;
  EXPECT_TRUE(lhs < rhs);
  EXPECT_FALSE(rhs < lhs);
  rhs.width = 4;
  EXPECT_TRUE(lhs < rhs);
  EXPECT_FALSE(rhs < lhs);
}
TEST(ObjTypeTest, ColorLess) {
  ObjectType lhs, rhs;
  // lhs values
  lhs.length = 1;
  lhs.width = 1;
  lhs.color = ObjectColor::RED;
  // rhs values
  rhs.length = 1;
  rhs.width = 1;
  rhs.color = ObjectColor::GREEN;
  EXPECT_TRUE(lhs < rhs);
  EXPECT_FALSE(rhs < lhs);
  rhs.color = ObjectColor::BLUE;
  EXPECT_TRUE(lhs < rhs);
  EXPECT_FALSE(rhs < lhs);
  rhs.color = ObjectColor::YELLOW;
  EXPECT_TRUE(lhs < rhs);
  EXPECT_FALSE(rhs < lhs);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
