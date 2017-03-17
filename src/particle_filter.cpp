/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position and all weights to 1. 
	// Add random Gaussian noise to each particle.

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	// observed measurement to this particular landmark.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian probability function.
	// Pre-compute covariance matrix determinant.

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 

}

double * ParticleFilter::weighted_mean_error(double gt_x, double gt_y, double gt_theta) {
	// TODO: Calculate the weighted mean error of the particle filter.
	static double error[3];

	return error;
}


void ParticleFilter::write(std::string filename) {
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
