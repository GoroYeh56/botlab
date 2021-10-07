#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void) : k1_(0.003f),k2_(0.03f),initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if (!initialized_) {
        previousOdometry_ = odometry;
        initialized_ = true;
    }

    float deltaX = odometry.x - previousOdometry_.x;
    float deltaY = odometry.y - previousOdometry_.y;
    float deltaTheta = odometry.theta - previousOdometry_.theta;
    trans_ = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    rot1_ = angle_diff(std::atan2(deltaY, deltaX), previousOdometry_.theta);
    float direction = 1.0;

    if (std::abs(rot1_) > M_PI / 2.0) {
        rot1_ = angle_diff(M_PI, rot1_); // it's going backwards
        direction = -1.0;
    }

    rot2_ = angle_diff(deltaTheta, rot1_);

    moved_ = (deltaX != 0.0) || (deltaY != 0.0) || (deltaTheta != 0.0);
    if (moved_) {
        rot1Std_ = std::sqrt(k1_ * std::abs(rot1_)); // could add translation here maybe?
        transStd_ = std::sqrt(k2_ * std::abs(trans_));
        rot2Std_ = std::sqrt(k1_ * std::abs(rot2_));
    }
    trans_ *= direction;
    previousOdometry_ = odometry;
    utime_ = odometry.utime;

    return moved_;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t newSample = sample;

    float sampledRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
    float sampledTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator_);
    float sampledRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);

    newSample.pose.x += sampledTrans * cos(sample.pose.theta + sampledRot1);
    newSample.pose.y += sampledTrans * sin(sample.pose.theta + sampledRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampledRot1 + sampledRot2);
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;
}



/*
float ActionModel::prob_normal_distribution(float x, float variance) {
    return (1 / std::sqrt(2 * std::_Pi * variance)) * std::exp(-0.5 * x*x / variance);
}

float ActionModel::sample_normal_distribution(float variance) {
    float answer = 0;
    for (int i = 1; i <= 12; i++) {
        answer += sample_uniform_distribution(1.0, -1.0);
    }
    return 
}

float ActionModel::sample_uniform_distribution(float high, float low) {
    low + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (low - high)));
}

*/