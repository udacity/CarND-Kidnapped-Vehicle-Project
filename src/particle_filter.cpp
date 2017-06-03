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
#include <math.h>
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
    std::random_device rd;
    std::default_random_engine gen(rd());

    std::normal_distribution<double> dist_x(x, std[0]);
    std::normal_distribution<double> dist_y(y, std[1]);
    std::normal_distribution<double> dist_theta(theta, std[2]);

    num_particles = 500;
    particles.resize(num_particles);

    for (int i = 0; i < num_particles; ++i) {
        Particle particle = { i, dist_x(gen), dist_y(gen), dist_theta(gen), 1 };
        particles[i] = particle;
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::random_device rd;
	std::default_random_engine gen(rd());

	for (int i = 0; i < num_particles; ++i) {
		Particle particle = particles[i];
		double x = particle.x;
		double y = particle.y;
		double theta = particle.theta;

		// cache intermediate calculations
		double tptddt = theta + yaw_rate * delta_t;
    double vdtddt = velocity / yaw_rate;

    double xf = x + vdtddt * (sin(tptddt) - sin(theta));
    double yf = y + vdtddt * (cos(theta) - cos(tptddt));
    double tf = tptddt;

    std::normal_distribution<double> dist_x(xf, std_pos[0]);
    std::normal_distribution<double> dist_y(yf, std_pos[1]);
    std::normal_distribution<double> dist_t(tf, std_pos[2]);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_t(gen);
	}
}

double ParticleFilter::getDistanceBetweenParticleAndLandmark(const Particle &particle,
                                                             const Map::single_landmark_s &landmark) {
    double dx = landmark.x_f - particle.x;
    double dy = landmark.y_f - particle.y;
    double distance = sqrt(dx * dx + dy * dy);
    return distance;
}

double ParticleFilter::getDistanceBetweenCoords(double ax, double ay, double bx, double by) {
    double dx = bx - ax;
    double dy = by - ay;
    double distance = sqrt(dx * dx + dy * dy);
    return distance;
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
    for (int i = 0; i < particles.size(); ++i) {
        Particle particle = particles[i];

        // find all landmarks within range of particle
        std::vector<LandmarkObs> predicted;
        for (int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
            Map::single_landmark_s landmark = map_landmarks.landmark_list[j];
            double distance_to_landmark = getDistanceBetweenParticleAndLandmark(particle, landmark);
            if (distance_to_landmark <= sensor_range) {
                LandmarkObs in_range_landmark = {
                    landmark.id_i,
                    static_cast<double>(landmark.x_f),
                    static_cast<double>(landmark.y_f)
                };
                predicted.push_back(in_range_landmark);
            }
        }

        // if no nearby landmarks, reject this particle
        if (predicted.size() == 0) {
            particles[i].weight = 0;
            continue;
        }

        // otherwise, associate observations with landmarks
        double w = 1;
        for (int j = 0; j < observations.size(); ++j) {
            LandmarkObs obs = observations[j];
            double obs_x = obs.x * cos(particle.theta) - obs.y * sin(particle.theta) + particle.x;
            double obs_y = obs.x * sin(particle.theta) + obs.y * cos(particle.theta) + particle.y;

            // use nearest neighbor to match predicted
            // landmarks to closest observation.
            // use first prediction as baseline to compare others
            double pred_x = predicted[0].x;
            double pred_y = predicted[0].y;
            double min_dist = getDistanceBetweenCoords(obs_x, obs_y, pred_x, pred_y);
            for (int k = 1; k < predicted.size(); ++k) {
                double curr_dist = getDistanceBetweenCoords(obs_x, obs_y, predicted[k].x, predicted[k].y);
                if (curr_dist < min_dist) {
                    min_dist = curr_dist;
                    pred_x = predicted[k].x;
                    pred_y = predicted[k].y;
                }
            }

            // update weight using Multivariate Gaussian probability
            double dx = obs_x - pred_x;
            double dy = obs_y - pred_y;
            double this_w = exp(-(
                    ( (dx * dx) / (2 * std_landmark[0] * std_landmark[0]) ) +
                    ( (dy * dy) / (2 * std_landmark[1] * std_landmark[1]) )
                    )) * (1. / (2 * M_PI * std_landmark[0] * std_landmark[1]));
            w *= this_w;
        }

        particles[i].weight = w;
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    double total_weight = 0;
    for (int i = 0; i < num_particles; ++i) {
        total_weight += particles[i].weight;
    }
    double w_max = 0;
    for (int i = 0; i < num_particles; ++i) {
        particles[i].weight /= total_weight;
        if (particles[i].weight > w_max) {
            w_max = particles[i].weight;
        }
    }

    std::random_device rd;
    std::default_random_engine gen(rd());

    std::uniform_int_distribution<int> int_dist(0, num_particles - 1);
    int index = int_dist(gen);

    std::uniform_real_distribution<double> weight_dist(0, 2 * w_max);
    double beta = 0;

    // resample particles using wheel method
    std::vector<Particle> resampled_particles;
    for (int i = 0; i < num_particles; ++i) {
        beta += weight_dist(gen);
        while (particles[index].weight < beta) {
            beta -= particles[index].weight;
            index = (index + 1) % num_particles;
        }
        resampled_particles.push_back(particles[index]);
    }

    particles = resampled_particles;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
