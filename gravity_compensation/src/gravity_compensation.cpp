/*
 *  gravity_compensation.cpp
 *
 *  Created on: Nov 11, 2013
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2013, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <gravity_compensation/gravity_compensation.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>


GravityCompensation::GravityCompensation(GravityCompensationParams *g_comp_params)
{
    m_g_comp_params = g_comp_params;
    m_tf_listener = new tf::TransformListener();
}

GravityCompensation::~GravityCompensation()
{
    delete m_tf_listener;
}


void GravityCompensation::Zero(const geometry_msgs::WrenchStamped &ft_raw,
                          geometry_msgs::WrenchStamped &ft_zeroed)
{
    Eigen::Matrix<double, 6, 1> bias = m_g_comp_params->getBias();

    ft_zeroed = ft_raw;

    // Assign the current ros time to the header stamp of the zeroed wrench
    ft_zeroed.header.stamp = ros::Time::now();

    Eigen::Matrix<double, 6, 1> ft_raw_eigen;
    Eigen::Matrix<double, 6, 1> ft_zeroed_eigen;
    tf::wrenchMsgToEigen(ft_raw.wrench, ft_raw_eigen);

    ft_zeroed_eigen = ft_raw_eigen - bias;
    tf::wrenchEigenToMsg(ft_zeroed_eigen, ft_zeroed.wrench);
}


bool GravityCompensation::Compensate(const geometry_msgs::WrenchStamped &ft_zeroed,
                                     const sensor_msgs::Imu &gravity,
                                     geometry_msgs::WrenchStamped &ft_compensated)
{

    geometry_msgs::Vector3Stamped g;
    g.vector.x = -gravity.linear_acceleration.x; // IMU will measure gravity in the opposite direction from F/T sensor, check https://github.com/kth-ros-pkg/force_torque_tools/pull/18
    g.vector.y = -gravity.linear_acceleration.y;
    g.vector.z = -gravity.linear_acceleration.z;
    g.header = gravity.header;
    g.header.stamp = ros::Time();

    // convert the accelerometer reading to the F/T sensor frame
    geometry_msgs::Vector3Stamped g_ft_frame;
    try
    {
        m_tf_listener->transformVector(ft_zeroed.header.frame_id, g, g_ft_frame);
    }

    catch(tf::TransformException &ex)
    {
        ROS_ERROR("Error transforming gravity vector to ft sensor frame...");
        ROS_ERROR("%s.", ex.what());
        return false;
    }


    double gripper_mass = m_g_comp_params->getGripperMass();
    Eigen::Vector3d g_ft_frame_eigen;
    tf::vectorMsgToEigen(g_ft_frame.vector, g_ft_frame_eigen);


    Eigen::Matrix<double, 6, 1> gripper_wrench_eigen;
    gripper_wrench_eigen.topRows(3) = g_ft_frame_eigen*gripper_mass;


    // grab the gripper_com
    tf::StampedTransform gripper_com_tf = m_g_comp_params->getGripperCOM();

    // convert the gripper_com to geometry_msgs::PoseStamped
    geometry_msgs::PoseStamped gripper_com;
    tf::poseTFToMsg(gripper_com_tf, gripper_com.pose);
    gripper_com.header.stamp = ros::Time();
    gripper_com.header.frame_id = gripper_com_tf.frame_id_;

    // make sure the gripper COM is expressed with respect to the F/T sensor frame
    geometry_msgs::PoseStamped ft_gripper_com;

    try
    {
        m_tf_listener->transformPose(ft_zeroed.header.frame_id,
                                     gripper_com,
                                     ft_gripper_com);
    }

    catch(tf::TransformException &ex)
    {
        ROS_ERROR("Error looking up transform between the gripper COM and the ft sensor frame");
        ROS_ERROR("%s.", ex.what());
        return false;
    }

    // convert to Eigen to do cross product to compute the torque
    Eigen::Vector3d r;
    tf::pointMsgToEigen(ft_gripper_com.pose.position, r);

    // compute torque generated by weight of the gripper
    Eigen::Vector3d gripper_force_eigen = gripper_wrench_eigen.topRows(3);
    Eigen::Vector3d gripper_torque_eigen = r.cross(gripper_force_eigen);
    gripper_wrench_eigen.bottomRows(3) = gripper_torque_eigen;

    // compensate force & torque
    Eigen::Matrix<double, 6, 1> ft_zeroed_eigen;
    tf::wrenchMsgToEigen(ft_zeroed.wrench, ft_zeroed_eigen);
    Eigen::Matrix<double, 6, 1> ft_compensated_eigen;

    ft_compensated_eigen = ft_zeroed_eigen - gripper_wrench_eigen;

    tf::wrenchEigenToMsg(ft_compensated_eigen, ft_compensated.wrench);
    ft_compensated.header = ft_zeroed.header;

    return true;
}
