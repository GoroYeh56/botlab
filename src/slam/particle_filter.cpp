#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <common/angle_functions.hpp>
#include <iostream>

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////

    // Initialize particles at some point
    
    double sampleWeight = 1.0 / kNumParticles_; // uniform distribution
    posteriorPose_ = pose; 

    for(auto &p : posterior_){
        p.pose.x = posteriorPose_.x ;// Initial Position: all the same ?
        p.pose.y = posteriorPose_.y ;
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.pose.utime = posteriorPose_.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;  // Initial weight

    }

}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    // Don't update our pose if the robot didn't move.
    if(hasRobotMoved)
    {

        /*
            1. Sample proposal distribution particles
                - need information from last particle set
                - low variance sampling (kNumparticles_)
            2. ComputeNormalizedPosterior (the same)
            3. Generate a posteriorPose_ (the same)
        
        */

        auto prior = LowVarianceResampling();
        // auto prior = resamplePosteriorDistribution(); // from motion model x ~ p(xt|u, x_t-1)
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterActionOnly(const pose_xyt_t&      odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        //auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }
    
    posteriorPose_ = odometry;
    
    return posteriorPose_;
}



pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}

// Jack: Resampling
// sample new particles from posterior_

double ParticleFilter::RandomFloat(double a, double b) {
    double random = ((double) rand()) / (double) RAND_MAX;
    double diff = b - a;
    double r = random * diff;
    return a + r;
}
std::vector<particle_t> ParticleFilter::LowVarianceResampling(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    std::vector<particle_t> prior = posterior_;
    double r = RandomFloat(0.0, 1/kNumParticles_);
    double c = posterior_[0].weight;
    std::cout<<"c[0] "<<c<<std::endl;
    int i = 0;

    for(int m = 0; m < kNumParticles_; m++){
        double U = r + (m) * (1.0/(double)kNumParticles_);
        while( U>c){
            i += 1;
            i = std::min(i, kNumParticles_-1);
            assert(i < kNumParticles_);
            c += posterior_[i].weight;
        }
        prior[m] = posterior_[i];
        prior[m].parent_pose = posteriorPose_;
    }

    return prior;    
}



std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    std::vector<particle_t> prior = posterior_;
    double sampleWeight = 1.0 / kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0, 0.04);

    for(auto &p : prior){
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = posteriorPose_.theta + dist(generator);
        p.pose.utime = posteriorPose_.utime;
        p.parent_pose = posteriorPose_;
        p.weight = sampleWeight;
    }
    
    
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;

    // Sample for prior.size() particles and push_back to proposal
    // Apply actions to these priors and add to Xt_bar (proposal particles)
    for( auto &p: prior){
        proposal.push_back( actionModel_.applyAction(p));
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;

    double sumWeights = 0.0;
    for(auto &p : proposal){
        particle_t weighted = p; // a new particle
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }
    for(auto &p : posterior){
        p.weight /= sumWeights;
    }

    return posterior;
}




pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;

    // // Create priority-queue of posterior:
    // std::priority_queue<particle_t, std::vector<particle_t>, Compare()> better_particles; // take best 40%
    
    // auto compare = [](particle_t a, particle_t b) { return a.weight > b.weight; }  ;
    // int SIZE = kNumParticles_ * 0.4;

    // for(auto &p: posterior){
    //     better_particles.push_back(p);
    //     if(better_particles.size()>= SIZE){
    //         better_particles.pop();
    //     }
    // }

    std::vector<particle_t> posterior_sorted = posterior;
    //Sort the posterior (weighted) distribution
    std::sort(posterior_sorted.begin(), posterior_sorted.end(),
            [](const particle_t& lhs, const particle_t& rhs) -> bool
            {
                return lhs.weight > rhs.weight;
            });


    double sumWeights = 0.0;
    for(int i=0; i<kNumParticles_*0.4; i++){
        sumWeights += posterior_sorted[i].weight;
    }
    for(int i=0; i<kNumParticles_*0.4; i++){
        posterior_sorted[i].weight /= sumWeights;
    }

    // mean x, y, theta of all posterior particles
    double xMean=0.0, yMean = 0.0, cosThetaMean = 0.0, sinThetaMean = 0.0;
    // for(auto &p: posterior){
    int index = 0;
    for(auto &p: posterior_sorted){
        index++;
        if(index > kNumParticles_*0.4) break;
        // xMean += p.weight * p.pose.x;
        // yMean += p.weight * p.pose.y;
        // cosThetaMean += p.weight * std::cos(p.pose.theta);
        // sinThetaMean += p.weight * std::sin(p.pose.theta);
        xMean += p.weight * p.pose.x;
        yMean += p.weight * p.pose.y;
        cosThetaMean += p.weight * std::cos(p.pose.theta);
        sinThetaMean += p.weight * std::sin(p.pose.theta);
    }

    pose.x = xMean;
    pose.y = yMean;
    pose.theta = std::atan2(sinThetaMean, cosThetaMean);

    return pose;
}
