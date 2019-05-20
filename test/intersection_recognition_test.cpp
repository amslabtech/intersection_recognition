#include <gtest/gtest.h>

#include <ros/ros.h>

TEST(TestSuite, test0)
{
	ros::NodeHandle nh;
	EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);

	ros::init(argc, argv, "intersection_recognition_test");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Duration(3.0).sleep();

	int r_e_t = RUN_ALL_TESTS();

	spinner.stop();

	ros::shutdown();

	return r_e_t;
}
