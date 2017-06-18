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
    
   num_particles = 200;
    for(int i=0; i< num_particles; i++){
        random_device rd;
        default_random_engine gen(rd());
        normal_distribution<double> gps_error_x(x, std[0]);
        normal_distribution<double> gps_error_y(y, std[1]);
        normal_distribution<double> gps_error_theta(theta, std[2]);

        double particle_x = gps_error_x(gen);
        double particle_y = gps_error_y(gen);
        double particle_theta = gps_error_theta(gen);
        double particle_weight = 1.0;

        Particle new_particle = {i, particle_x, particle_y, particle_theta, particle_weight};
        particles.push_back(new_particle);
        weights.push_back(particle_weight);
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    for(int i=0; i< num_particles; i++){
        if(yaw_rate == 0){
            particles[i].x = particles[i].x + (velocity * delta_t) * cos(particles[i].theta);
            particles[i].y = particles[i].y + (velocity * delta_t) * sin(particles[i].theta);
        }else{
            particles[i].x = particles[i].x + (velocity/yaw_rate)*(sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
            particles[i].y = particles[i].y + (velocity/yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
            particles[i].theta = particles[i].theta + (yaw_rate * delta_t);
        }


        random_device rd;
        default_random_engine gen(rd());
        normal_distribution<double> pos_error_x(particles[i].x, std_pos[0]);
        normal_distribution<double> pos_error_y(particles[i].y, std_pos[1]);
        normal_distribution<double> pos_error_theta(particles[i].theta, std_pos[2]);

        particles[i].x = pos_error_x(gen);
        particles[i].y = pos_error_y(gen);
        particles[i].theta = pos_error_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    for(int obs=0; obs<observations.size(); obs++){
        double obs_x = observations[obs].x;
        double obs_y = observations[obs].y;

        double temp_delta_l = 0.0;
        bool temp_delta_l_initialized = false;

        for(int l=0; l<predicted.size(); l++){
            double delta_x = obs_x - predicted[l].x;
            double delta_y = obs_y - predicted[l].y;

            double delta_l = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

            if((!temp_delta_l_initialized) || (temp_delta_l > delta_l)) {
                temp_delta_l = delta_l;
                temp_delta_l_initialized = true;
                observations[obs].id = l;
            }
        }
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

        for(int i=0; i<num_particles; i++){
        double current_x = particles[i].x;
        double current_y = particles[i].y;
        double current_theta = particles[i].theta;

        vector<LandmarkObs> predicted_landmarks;
        for(int l=0; l<map_landmarks.landmark_list.size(); l++){
            int l_id = map_landmarks.landmark_list[l].id_i;
            double l_x = map_landmarks.landmark_list[l].x_f;
            double l_y = map_landmarks.landmark_list[l].y_f;

            double delta_x = l_x - current_x;
            double delta_y = l_y - current_y;

            double distance = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));
            if(distance<=sensor_range){
                l_x = delta_x * cos(current_theta) + delta_y * sin(current_theta);
                l_y = delta_y * cos(current_theta) - delta_x * sin(current_theta);
                LandmarkObs landmark_in_range = {l_id, l_x, l_y};
                predicted_landmarks.push_back(landmark_in_range);
            }
        }

        dataAssociation(predicted_landmarks, observations);

        double new_weight = 1.0;
        for(int obs=0; obs<observations.size(); obs++) {
            int l_id = observations[obs].id;
            double obs_x = observations[obs].x;
            double obs_y = observations[obs].y;

            double delta_x = obs_x - predicted_landmarks[l_id].x;
            double delta_y = obs_y - predicted_landmarks[l_id].y;

            double numerator = exp(- 0.5 * (pow(delta_x,2.0)*std_landmark[0] + pow(delta_y,2.0)*std_landmark[1] ));
            double denominator = sqrt(2.0 * M_PI * std_landmark[0] * std_landmark[1]);
            new_weight = new_weight * numerator/denominator;
        }
        weights[i] = new_weight;
        particles[i].weight = new_weight;

    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // Use disctere distribution function to generate random integers in the interbval 0 to n, where the probability of each individual integer i is
    // defined as the weight of the ith integer divided by the sum of all n weights.
    std::vector<Particle> resampled_particles;

    random_device rd;
    default_random_engine gen(rd());

    for(int counter=0; counter<particles.size(); counter++){
        discrete_distribution<int> index(weights.begin(), weights.end());
        resampled_particles.push_back(particles[index(gen)]);
    }

    particles = resampled_particles;
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
        dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
    }
    dataFile.close();
}
