#include <gtest/gtest.h>
#include <math.h>

#include <multirotor_manipulation_and_control_framework/utils.h>

TEST(UtilTestSuite, checkMaxEqualMin)
{
  ASSERT_DOUBLE_EQ(-10, common::map(5, 5, 5, -10, 20));
  ASSERT_DOUBLE_EQ(0, common::map(0, -5, 5, -10, 10));
}

TEST(UtilTestSuite, checkAngleMap)
{
  ASSERT_DOUBLE_EQ(-M_PI/2, common::map_angle(3*M_PI/2));
  ASSERT_DOUBLE_EQ(M_PI/2, common::map_angle(-3*M_PI/2));
  ASSERT_DOUBLE_EQ(M_PI, common::map_angle(M_PI));
}

TEST(UtilTestSuite, addTimeString)
{
  std::string input("Hello");
  std::string output1 = common::addtimestring(input);
  std::string output2 = common::addtimestring(input);
  std::cout<<"Output Compare: "<<output1<<"\t"<<output2<<std::endl;
  ASSERT_STREQ(output1.c_str(), output2.c_str());
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
