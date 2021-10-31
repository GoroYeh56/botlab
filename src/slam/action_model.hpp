 #ifndef SLAM_ACTION_MODEL_HPP
#define SLAM_ACTION_MODEL_HPP

#include <lcmtypes/pose_xyt_t.hpp>
#include <random>

struct particle_t;

/**
* ActionModel implements the sampling-based odometry action model for estimating the motion of the robot between
* time t and t'.
* 
* An action model is used to propagate a sample from the prior distribution, x, into
* the proposal distribution, x', based on the supplied motion estimate of the robot
* in the time interval [t, t'].
*
* To use the ActionModel, a two methods exist:
*
*   - bool updateAction(const pose_xyt_t& odometry);
*   - particle_t applyAction(const particle_t& sample);
*
* updateAction() provides the most recent odometry data so the action model can update the distributions from
* which it will sample.
*
* applyAction() applies the action to the provided sample and returns a new sample that can be part of the proposal 
* distribution for the particle filter.
*/
class ActionModel
{
public:
    
    /**
    * Constructor for ActionModel.
    */
    ActionModel(void);
    
    /**
    * updateAction sets up the motion model for the current update for the localization.
    * After initialization, calls to applyAction() will be made, so all distributions based on sensor data
    * should be created here.
    *
    * \param    odometry            Current odometry data from the robot
    * \return   The pose transform distribution representing the uncertainty of the robot's motion.
    */
    bool updateAction(const pose_xyt_t& odometry);
    bool updateActionOdometry(const pose_xyt_t& odometry);
    /**
    * applyAction applies the motion to the provided sample and returns a new sample that
    * can be part of the proposal distribution for the particle filter.
    *
    * \param    sample          Sample to be moved
    * \return   New sample based on distribution from the motion model at the current update.
    */
    particle_t applyAction(const particle_t& sample);
    
private:
    const float k1_;
    const float k2_;
    
    // previous odometry message
    pose_xyt_t previousOdometry_;
    // Keep track of what rotations are
    double rot1_;
    double trans_;
    double rot2_;
    bool moved_;
    bool initialized_;
    int64_t utime_;

    double rot1Std_;
    double transStd_;
    double rot2Std_;

    // To generate randomness:
    std::mt19937 numberGenerateor_;
    
    ////////// TODO: Add private member variables needed for you implementation ///////////////////
    const float k1_;
    const float k2_;
    //bool initialized_;

    pose_xyt_t previousOdometry_;
    double rot1_;
    double trans_;
    double rot2_;
    bool moved_;
    bool initialized_;
    int64_t utime_;

    double rot1Std_;
    double transStd_;
    double rot2Std_;

    std::mt19937 numberGenerator_;

    /*
    float prob_normal_distribution(float x, float variance);
    float sample_normal_distribution(float variance);
    float sample_uniform_distribution(float high, float low);
    */

};

#endif // SLAM_ACTION_MODEL_HPP
;
