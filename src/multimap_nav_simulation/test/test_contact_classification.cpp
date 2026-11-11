#include <gtest/gtest.h>
#include "multimap_nav_simulation/contact_classification.hpp"
using multimap_nav_simulation::isSupportContact;

TEST(ContactClassification, Floor1StandingRegression)
{
  EXPECT_TRUE(isSupportContact("floor_1_structure::structure::lower_collision", 1.));
  EXPECT_TRUE(isSupportContact("floor_1_structure::structure::lower_collision", .972650231045));
}

TEST(ContactClassification, DoesNotHideSideOrOverheadContacts)
{
  EXPECT_FALSE(isSupportContact("floor_1_structure::structure::lower_collision", 0.));
  EXPECT_FALSE(isSupportContact("floor_1_structure::structure::upper_collision", -1.));
  EXPECT_FALSE(isSupportContact("scene::structure::unknown_collision", 1.));
}

TEST(ContactClassification, ObstacleNamesOverrideSupportWords)
{
  EXPECT_FALSE(isSupportContact("scene::structure::upper_rail_0_collision", 1.));
  EXPECT_FALSE(isSupportContact("scene::structure::floor_wall_collision", 1.));
  EXPECT_FALSE(isSupportContact("scene::structure::lower_obstacle_collision", 1.));
}

TEST(ContactClassification, RampTransitionRemainsExplicitSupport)
{
  EXPECT_TRUE(isSupportContact("floor_1_structure::structure::upper_collision", .60));
  EXPECT_TRUE(isSupportContact("scene::structure::ramp_collision", .97));
}
TEST(ContactClassification, OtherFloorSlabsAreExplicitUpwardSupport)
{
  EXPECT_TRUE(isSupportContact("floor_3_structure::structure::landing_collision",.819445123123));
  EXPECT_TRUE(isSupportContact("floor_4_structure::structure::west_lower_collision",1.));
  EXPECT_TRUE(isSupportContact("floor_4_structure::structure::south_lower_collision",.978740096521));
  EXPECT_FALSE(isSupportContact("scene::structure::landing_collision",0.));
  EXPECT_FALSE(isSupportContact("scene::structure::west_lower_collision",-1.));
  EXPECT_FALSE(isSupportContact("scene::structure::landing_obstacle_collision",1.));
}
