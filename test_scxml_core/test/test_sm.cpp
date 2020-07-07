#include "gtest/gtest.h"
#include "ros/package.h"
#include "scxml_core/state_machine.h"

using namespace scxml_core;

struct TestSM : public testing::Test
{
  StateMachine* sm;
  void SetUp()
  {
    sm = new StateMachine();
    std::string path_to_scxml = ros::package::getPath("test_scxml_core").append("/test/test_sm.scxml");
    sm->loadFile(path_to_scxml);
  }
};

TEST_F(TestSM, test_list_state)
{
  std::vector<std::string> all_states = sm->getStates();
  EXPECT_EQ(all_states.size(), 4);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}