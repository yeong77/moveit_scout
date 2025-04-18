#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TrajectoryToCmdVel
{
public:
  TrajectoryToCmdVel()
  {
    ros::NodeHandle nh;
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    traj_sub_ = nh.subscribe("/move_group/display_planned_path", 10, &TrajectoryToCmdVel::callback, this);
    ROS_INFO("Trajectory to /cmd_vel bridge started.");
  }

  void callback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
  {
    const auto& traj = msg->trajectory.back().multi_dof_joint_trajectory;
    ROS_INFO("Callback triggered with %lu points", traj.points.size());

    if (traj.points.size() < 2)
    {
      ROS_WARN("Trajectory too short to compute velocity.");
      return;
    }

    for (size_t i = 1; i < traj.points.size(); ++i)
    {
      const auto& p1 = traj.points[i - 1];
      const auto& p2 = traj.points[i];

      double dt = (p2.time_from_start - p1.time_from_start).toSec();
      if (dt == 0.0)
        continue;

      const auto& t1 = p1.transforms[0].translation;
      const auto& t2 = p2.transforms[0].translation;

      double dx = t2.x - t1.x;
      double dy = t2.y - t1.y;
      double dz = t2.z - t1.z;

      tf2::Quaternion q1, q2;
      tf2::fromMsg(p1.transforms[0].rotation, q1);
      tf2::fromMsg(p2.transforms[0].rotation, q2);

      tf2::Quaternion q_diff = q2 * q1.inverse();
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_diff).getRPY(roll, pitch, yaw);

      geometry_msgs::Twist twist;
      twist.linear.x = dx / dt;
      twist.linear.y = dy / dt;
      twist.angular.z = yaw / dt;

      ROS_INFO("Publishing cmd_vel: x=%.4f y=%.4f yaw=%.4f", twist.linear.x, twist.linear.y, twist.angular.z);

      cmd_pub_.publish(twist);
      ros::Duration(dt).sleep();
    }
  }

private:
  ros::Publisher cmd_pub_;
  ros::Subscriber traj_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_to_cmd_vel");
  TrajectoryToCmdVel node;
  ros::spin();
  return 0;
}

