/**
 * @file fake_imu_publisher.cpp
 * @brief This node simulates an IMU by publishing fake IMU messages.
 *
 * The `tf_to_fake_imu` node listens for transformations between a specified world frame and a mobile frame,
 * and publishes fake IMU data. The orientation of the IMU data is derived from the current orientation
 * of the mobile frame relative to the world frame. The linear acceleration data is calculated by transforming
 * a specified gravity vector into the orientation of the mobile frame.
 *
 * Parameters:
 *   - imu_topic_name_out (string): The ROS topic on which the IMU data is published.
 *   - tf_world_frame_id (string): The reference frame ID in which gravity is defined (default: "map").
 *   - tf_mobile_frame_id (string): The frame ID from which the IMU data should appear to originate.
 *   - gravity (vector<double>): A 3-element vector representing the gravity vector in the world frame.
 *                                Typically set to [0.0, 0.0, -9.81] for Earth's gravity.
 *   - rate (int): The frequency in Hz at which the IMU messages are published (default: 100).
 *
 * Subscriptions:
 *   None directly, but listens to TF to get transformations between the specified frames.
 *
 * Publications:
 *   - imu_topic_name_out (sensor_msgs/Imu): Publishes IMU data messages containing orientation,
 *                                          angular velocity (always zero in this implementation),
 *                                          and linear acceleration due to gravity.
 *
 * Usage:
 *   The node is configured via parameters set on the ROS parameter server, typically loaded from
 *   a launch file that may include these parameters statically or from a YAML configuration file.
 *
 * Example launch command:
 *   rosrun tf_to_fake_imu fake_imu_publisher_node _imu_topic_name_out:=/fake_imu _tf_world_frame_id:=map _tf_mobile_frame_id:=base_link
 *
 * @author Burak Aksoy
 * @date 2024-06-13
 */


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf/transform_broadcaster.h> // For tf broadcasting (if needed)
#include <tf/tf.h>                   // For tf helper functions

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_to_fake_imu");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string imu_topic_name_out, tf_world_frame_id, tf_mobile_frame_id;
    std::vector<double> gravity;
    int rate;

    pnh.param("imu_topic_name_out", imu_topic_name_out, std::string("/fake_imu"));
    pnh.param("tf_world_frame_id", tf_world_frame_id, std::string("map"));
    pnh.param("tf_mobile_frame_id", tf_mobile_frame_id, std::string("base_link"));
    pnh.param("gravity", gravity, std::vector<double>{0.0, 0.0, -9.81});
    pnh.param("rate", rate, 100);

    Eigen::Vector3d gravity_vector(gravity[0], gravity[1], gravity[2]);

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic_name_out, 10);
    tf::TransformListener listener;
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        tf::StampedTransform transform;
        try {
            listener.lookupTransform(tf_world_frame_id, tf_mobile_frame_id, ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            // When outputting the error message, include the node name for clarity
            ROS_ERROR("[%s] %s", ros::this_node::getName().c_str(), ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // Eigen Conversion for Transform
        Eigen::Quaterniond orientation(
            transform.getRotation().w(),
            transform.getRotation().x(),
            transform.getRotation().y(),
            transform.getRotation().z()
        );

        // orientation.inverse();

        Eigen::Vector3d transformed_gravity = orientation.conjugate() * gravity_vector;

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = tf_mobile_frame_id;

        tf::quaternionTFToMsg(transform.getRotation(), imu_msg.orientation);

        imu_msg.angular_velocity.x = 0;
        imu_msg.angular_velocity.y = 0;
        imu_msg.angular_velocity.z = 0;

        imu_msg.linear_acceleration.x = -transformed_gravity.x(); // IMU will measure gravity in the opposite direction from F/T sensor, check https://github.com/kth-ros-pkg/force_torque_tools/pull/18
        imu_msg.linear_acceleration.y = -transformed_gravity.y(); 
        imu_msg.linear_acceleration.z = -transformed_gravity.z(); 

        imu_pub.publish(imu_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}