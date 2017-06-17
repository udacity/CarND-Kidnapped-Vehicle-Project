/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
//#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    
    // Define the number of particles
    num_particles = 200;

    std::default_random_engine gen;
    
    // Create a normal (Gaussian) distribution for x, y and psi.
    std::normal_distribution<double> dist_x(x, std[0]);
    std::normal_distribution<double> dist_y(y, std[1]);
    std::normal_distribution<double> dist_theta(theta, std[2]);
    
    // Create and set particle values for all
    for (int i = 0; i < num_particles; ++i)
    {
        Particle particle;
        particle.id     = i;
        particle.x      = dist_x(gen);
        particle.y      = dist_y(gen);
        particle.theta  = dist_theta(gen);
        particle.weight = 1;
        
        // Add this particle to the particle filter
        particles.push_back(particle);
        weights.push_back(1);
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    default_random_engine gen;
    // Update all partcile values
    for (int i = 0; i < num_particles; ++i){

        double new_x;
        double new_y;
        double new_theta;
        
        if (yaw_rate == 0)
        {
           new_x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
           new_y = particles[i].y + velocity*delta_t*sin(particles[i].theta);
           new_theta = particles[i].theta;
        }
        else
        {
            new_x = particles[i].x + velocity/yaw_rate*(sin(particles[i].theta+yaw_rate*delta_t)-sin(new_theta));
            new_y = particles[i].y + velocity/yaw_rate*(cos(particles[i].theta)-cos(particles[i].theta+(yaw_rate*delta_t)));
            new_theta = particles[i].theta + yaw_rate*delta_t;
        }
        
        // Adding gaussian noise
        normal_distribution<double> dist_x(new_x, std_pos[0]);
        normal_distribution<double> dist_y(new_y, std_pos[1]);
        normal_distribution<double> dist_theta(new_theta, std_pos[2]);
        
        // Update the current particle in the array
        particles[i].x      = dist_x(gen);
        particles[i].y      = dist_y(gen);
        particles[i].theta  = dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    for (int i=0; i<observations.size(); ++i) {

        double dist_min = 0.0;
        int id_m = 0;

        for (int j=0; j<predicted.size(); ++j) {

            double dist_temp = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

            if (j == 0) {
                dist_min = dist_temp;
                id_m = predicted[j].id;
            }

            if (dist_temp < dist_min) {
                dist_min = dist_temp;
                id_m = predicted[j].id;
            }
        }

        observations[i].id = id_m;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

        double weights_sum = 0.0;

    for (int i = 0; i < particles.size(); i++) {
        Particle &p = particles[i];

        // Make a list of predicted measurements for this particle
        std::vector<LandmarkObs> predicted(map_landmarks.landmark_list.size());
        for (int j = 0; j < predicted.size(); j++) {
            Map::single_landmark_s landmark = map_landmarks.landmark_list[j];

            double sin_theta = sin(p.theta);
            double cos_theta = cos(p.theta);

            double x_pred = (landmark.y_f- p.y)*sin_theta + (landmark.x_f - p.x) * cos_theta;
            double y_pred = (landmark.y_f- p.y)*cos_theta - (landmark.x_f - p.x) * sin_theta;

            predicted[j].x = x_pred;
            predicted[j].y = y_pred;
            predicted[j].id = landmark.id_i;
        }

        dataAssociation(predicted, observations);

        double prob = 1;

        for (LandmarkObs &obs : observations) {

            double obs_x = obs.x * cos(p.theta) - obs.y * sin(p.theta) + p.x;
            double obs_y = obs.x * sin(p.theta) + obs.y * cos(p.theta) + p.y;

            double true_x = map_landmarks.landmark_list[obs.id-1].x_f;
            double true_y = map_landmarks.landmark_list[obs.id-1].y_f;

            double obs_prob = (1/(2*M_PI*std_landmark[0]*std_landmark[1]))
                * exp(-(pow(true_x - obs_x, 2)/(2*std_landmark[0]*std_landmark[0])
                    + (pow(true_y - obs_y, 2)/(2*std_landmark[1]*std_landmark[1]))));

            prob *= obs_prob;
        }

        weights[i] = prob;
    }

    // normalize weights
    for (int i=0; i<num_particles; i++)
        particles[i].weight /= weights_sum;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // Use disctere distribution function to generate random integers in the interbval 0 to n, where the probability of each individual integer i is
    // defined as the weight of the ith integer divided by the sum of all n weights.
    std::default_random_engine gen;
    std::discrete_distribution<int> dist(weights.begin(), weights.end());

    std::vector<Particle> new_particles;
    new_particles.reserve(particles.size());

    for (int i = 0; i < particles.size(); i++) {
        int sampled_index = dist(gen);
        new_particles.push_back(particles[sampled_index]);
    }

    particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
void ParticleFilter::write(std::string filename) {
    std::ofstream dataFile;
    dataFile.open(filename, std::ios::app);
    for (int i = 0; i < num_particles; ++i) {
        Particle &p = particles[i];
        dataFile << p.x << " " << p.y << " " << p.theta << "\n";
    }
    dataFile.close();
}
