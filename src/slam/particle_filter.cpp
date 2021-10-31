#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <common/angle_functions.hpp>
#include <time.h>

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;
    std::random_device rd;
    std::mt19937 numberGenerator = std::mt19937(rd());
    std::normal_distribution<> dist(0.0, 0.007);
    //std::normal_distribution<> ang(0.0, std::_Pi/4);
    
    for (auto& p : posterior_) {
        p.pose.x = posteriorPose_.x + dist(numberGenerator);
        p.pose.y = posteriorPose_.y + dist(numberGenerator);
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }

}

void ParticleFilter::initializeFilterUniformly(const pose_xyt_t& pose,const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    std::cout << "\ninitializing particles uniformly!\n";
    double sampleWeight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;
    int xcell = 0;
    int ycell = 0;
    
    double xLim = (double)map.widthInMeters();
    double yLim = (double)map.heightInMeters();
    double sideLength = std::sqrt((double)kNumParticles_);
    double xStep = xLim/sideLength; //  m/particle
    double yStep = yLim/sideLength;
   
    double xPos = -xLim / 2.0;
    double yPos = -yLim / 2.0;
    std::cout << "\nnum particles: " << kNumParticles_;
    std::cout << "\nxLim: " << xLim << "  yLim: " << yLim;
    std::cout << "\nxStep: " << xStep << "  yStep: " << yStep;
    for (auto& p : posterior_) {
        std::cout << "\nxPos: " << xPos << "  yPos: " << yPos;
        if(xPos < (xLim / 2.0) - xStep) {
            xPos += xStep;
        }
        else if (xPos >= xLim / 2.0 - xStep){
            xPos = -xLim/2.0;
            yPos += yStep;
        }
        else if (yPos >= yLim / 2.0){
            break;
        }
        p.pose.x = xPos;
        p.pose.y = yPos;
        p.pose.theta = 0.0;
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }

}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{

    //clock_t startTime = clock();
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

    //clock_t endTime = clock();

    //std::cout <<"\n" << (double)(endTime - startTime)/CLOCKS_PER_SEC;

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

double MMA_rand(double high, double low) {
    return low + static_cast <double> (std::rand()) / (static_cast <double> (RAND_MAX / (low - high)));
}

double RandomFloat(float a, float b) {
    double random = ((double) rand()) / (double)RAND_MAX;
    double diff = b - a;
    double r = random * diff;
    return a + r;
}



std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////

    std::vector<particle_t> prior = posterior_;
    // don't just sample random ones in real code?
    const double sampleWeight = 1.0 / kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0, 0.04);

    double r = RandomFloat(0.0, sampleWeight);
    double c = posterior_.at(0).weight;
    int i = 0;
    int m = 1;
    for (m = 1; m < kNumParticles_; m++) {
        double U = r + ((double)m - 1) * (sampleWeight);
        while (U > c) {
            i++;
            if (i < posterior_.size()) {
                c = c + posterior_.at(i).weight;
            }
        }
        if (i < posterior_.size()) {
            
            prior.at(m) = posterior_.at(i);
            prior.at(m).parent_pose = posteriorPose_;
            
        }
        
        
    }


    /*
    for (auto& a : posterior_) {
        particle_t p;
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = posteriorPose_.theta + dist(generator);
        p.pose.utime = posteriorPose_.utime;
        p.parent_pose = posteriorPose_;
        p.weight = sampleWeight;
        prior.push_back(p);
    
    */
        

    return prior;
        
}





std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for (auto& p : prior) {
        proposal.push_back(actionModel_.applyAction(p));
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

    for (auto& p : proposal) {
        particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }

    for (auto& p : posterior) {
        p.weight /= sumWeights;
    }

    return posterior;
}



pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
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
    for (int i = 0; i < kNumParticles_ * 0.4; i++) {
        sumWeights += posterior_sorted[i].weight;
    }
    for (int i = 0; i < kNumParticles_ * 0.4; i++) {
        posterior_sorted[i].weight /= sumWeights;
    }

    // mean x, y, theta of all posterior particles
    double xMean = 0.0, yMean = 0.0, cosThetaMean = 0.0, sinThetaMean = 0.0;
    // for(auto &p: posterior){
    int index = 0;
    for (auto& p : posterior_sorted) {
        index++;
        if (index > kNumParticles_ * 0.4) break;
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

    /*
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;

    double xMean = 0.0;
    double yMean = 0.0;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;
  
    //make a local copy of the particles
    std::vector<particle_t> posterior_sorted = posterior;
    //Sort the posterior (weighted) distribution
    
    std::sort(posterior_sorted.begin(), posterior_sorted.end(),
        [](const particle_t& lhs, const particle_t& rhs) -> bool
        {
            return lhs.weight < rhs.weight;
        });
    double threshold = 0.8;
    std::vector<particle_t> posterior_sorted_cut(posterior_sorted.begin(), posterior_sorted.begin() + threshold * posterior_sorted.size());
    
    double sumWeights = 0.0;
    for (auto& p : posterior_sorted_cut) {
        sumWeights += p.weight;
    }
    for (auto& p : posterior_sorted_cut) {
        p.weight /= sumWeights;
    }

   
    for (auto& p : posterior){
        
        xMean += p.weight * p.pose.x;
        yMean += p.weight * p.pose.y;
        cosThetaMean += p.weight * std::cos(p.pose.theta);
        sinThetaMean += p.weight * std::sin(p.pose.theta);
        //i++;
    }

    pose.x = xMean;
    pose.y = yMean;
    pose.theta = std::atan2(sinThetaMean, cosThetaMean);

    return pose;*/
}


double ParticleFilter::averageParticleDistanceFromMean() {
    pose_xyt_t meanPose = this->estimatePosteriorPose(this->posterior_);
    double xMean = meanPose.x;
    double yMean = meanPose.y;
    double totalDistance = 0.0;

    for (auto& p : this->posterior_){

        double dx = std::abs(p.pose.x - xMean);
        double dy = std::abs(p.pose.y - yMean);

        totalDistance += std::sqrt(dx * dx + dy * dy);

    }

    return totalDistance / this->posterior_.size();

}