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
#include <cfloat>

#include "particle_filter.h"

using namespace std;

default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	// x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  // Normal Distributions for the Gaussian noise
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  double sample_x;
  double sample_y;
  double sample_theta;

  // Number of particles to draw
  num_particles = 500;
  // Resize and assign weights to 1
  weights.resize(num_particles, 1.0f);

  for (int i = 0; i < num_particles; ++i) {
    Particle p;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.id = i;
    p.weight = weights[i];
    particles.push_back(p);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  for (int i = 0; i < particles.size(); ++i) {
    Particle p = particles[i];

    // CTRV (constant turn rate & velocity) Model
      // Normal Distributions for the Gaussian noise
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_yaw(0, std_pos[2]);

    // readability
    double p_x = p.x;
    double p_y = p.y;
    double yaw = p.theta;

    double v = velocity;
    double yawd = yaw_rate;

    // predicted state values
    double px_p;
    double py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.0001) {
      px_p = p_x + v/yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (-cos(yaw + yawd * delta_t) + cos(yaw));
    }
    else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }
    double pyaw_p = yaw + yawd * delta_t;

    // add noise
    px_p += dist_x(gen);
    py_p += dist_y(gen);
    pyaw_p += dist_yaw(gen);

    // reassign the state
    particles[i].x = px_p;
    particles[i].y = py_p;
    particles[i].theta = pyaw_p;
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  double min_distance, curr_dist;
  int min_i;

  for (int i = 0; i < observations.size(); i++) {
    LandmarkObs obs = observations[i];

    min_distance = DBL_MAX;
    min_i = -1;

    // Find the closet predicted measurement to observed measurement
    for (int j = 0; j < predicted.size(); j++) {
      LandmarkObs pred = predicted[j];
      curr_dist = dist(pred.x, pred.y, obs.x, obs.y);

      if (curr_dist < min_distance) {
        min_distance = curr_dist;
        min_i = j;
      }
    }

    // Assign the observed measurement to the closet landmark
    observations[i].id = min_i;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution. You can read
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

    // Get the coordinates of particle
    Particle p = particles[i];
    double p_x = p.x;
    double p_y = p.y;
    double p_theta = p.theta;

    // vector to hold the map landmark predicted to be within sensor range of the particle
    vector<LandmarkObs> predictions;

    for (int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
      // Get the coordinates and i.d. of landmark
      float lm_x = map_landmarks.landmark_list[j].x_f;
      float lm_y = map_landmarks.landmark_list[j].y_f;
      int lm_id = map_landmarks.landmark_list[j].id_i;

      if (fabs(dist(lm_x,lm_y, p_x, p_y)) <= sensor_range) {
        predictions.push_back( LandmarkObs{lm_id, lm_x, lm_y});
      }
    }

    // vector to observations transformed from car coordinates to map coordinates
    vector<LandmarkObs> transformed_ob;
    for (int j = 0; j < observations.size(); ++j) {
      double t_x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
      double t_y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;

      transformed_ob.push_back( LandmarkObs{ observations[j].id, t_x, t_y});
    }

    // Data assocation with landmark predictions and transformed observations
    dataAssociation(predictions, transformed_ob);

    // reinitialize weights
    particles[i].weight = 1.0f;

    for (int j = 0; j < transformed_ob.size(); ++j) {

      // map coordinates of observations
      double ob_x, ob_y, pr_x, pr_y;
      ob_x = transformed_ob[j].x;
      ob_y = transformed_ob[j].y;

      // map coordinates of associated landmark position
      int associated_id = transformed_ob[j].id;
      pr_x = predictions[associated_id].x;
      pr_y = predictions[associated_id].y;

      double s_x = std_landmark[0];
      double s_y = std_landmark[1];

      // weight for the observation with multivariate normal distribution
      double ob_w = (1/(2*M_PI*s_x*s_y)) * exp(-pow(ob_x - pr_x,2)/(2 * s_x*s_x) -pow(ob_y - pr_y,2)/(2 * s_y*s_y));

      // total observation weights
      particles[i].weight *= ob_w;
    }
  }
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  // Resamping Wheel

  // Get weights and max-weights
  vector<double> weights;
  double max_weight = -DBL_MAX;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
    if (particles[i].weight > max_weight) {
      max_weight = particles[i].weight;
    }
  }

  // creating distributions
  uniform_int_distribution<int> dist_index(0, num_particles - 1);
  uniform_real_distribution<double> dist_beta(0.0, 2*max_weight);

  double beta = 0.0;
  int index = dist_index(gen);

  vector<Particle> resampleParticles;
  for (int i = 0; i < num_particles; ++i) {
    beta += dist_beta(gen);
    while(weights[index] < beta) {
      beta -= weights[index];
      index = (index+1)%num_particles;
    }
    resampleParticles.push_back(particles[index]);
  }
  particles = resampleParticles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
