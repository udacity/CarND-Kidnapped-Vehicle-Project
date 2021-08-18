/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <initializer_list>
#include <map>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles

  // (Gaussian) distribution for xm ym theta
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  for (int i=0; i<num_particles; i++)
  {
    Particle newParticle;
    newParticle.id = i;
    newParticle.x = dist_x(gen);
    newParticle.y = dist_y(gen);
    newParticle.theta = dist_theta(gen);
    newParticle.weight = 1;

    particles.push_back(newParticle);
    weights.push_back(newParticle.weight);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

    // add randomness
    std::default_random_engine gen;
  
    std::normal_distribution<double> x_rd{0,std_pos[0]};
    std::normal_distribution<double> y_rd{0,std_pos[1]};
    std::normal_distribution<double> theta_rd{0,std_pos[2]};

  for (auto& currentParticle: particles)
  {
    if (fabs(yaw_rate) < __DBL_EPSILON__)
    {
      double length = velocity * delta_t;
      currentParticle.x += length * std::cos(currentParticle.theta);
      currentParticle.y += length * std::sin(currentParticle.theta);
    }
    else {
      // turn
      double oldTeta = currentParticle.theta;
      currentParticle.theta += yaw_rate * delta_t;
      
      // move
      double division = velocity / yaw_rate;
      currentParticle.x += division * (std::sin(currentParticle.theta) - std::sin(oldTeta));
      currentParticle.y += division * (std::cos(oldTeta) - std::cos(currentParticle.theta));

    }
    currentParticle.x += x_rd(gen);
    currentParticle.y += y_rd(gen);
    currentParticle.theta += theta_rd(gen);

    // TODO: cyclic truncate with world size is missing
  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

}

void ParticleFilter::updateWeights(const double sensor_range, const double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  weights.clear();

  for (auto& currentParticle: particles)
  {
    //reset data
    currentParticle.associations.clear();
    currentParticle.sense_x.clear();
    currentParticle.sense_y.clear();
    currentParticle.weight = 1;


    // Tranform obervations from car to map coordinates
	  double cos_theta = cos(currentParticle.theta);
    double sin_theta = sin(currentParticle.theta);
    for (const auto& currentObservation:observations)
    {
      const double x_map = cos_theta * currentObservation.x 
        - sin_theta * currentObservation.y 
        + currentParticle.x;
      const double y_map = sin_theta * currentObservation.x 
        + cos_theta * currentObservation.y 
        + currentParticle.y;

      // search for neartest neighbour
      double minDistance = __DBL_MAX__;
      int association = -1;
      double mu_x = 0;
      double mu_y = 0;
      for (const auto& currentlandmark:map_landmarks.landmark_list)
      {
        double distanceToOberservation = dist(currentlandmark.x_f, currentlandmark.y_f, x_map, y_map);
        double distanceToParticle = dist(currentlandmark.x_f, currentlandmark.y_f, currentParticle.x, currentParticle.y);
        
		  if (distanceToParticle <= sensor_range && distanceToOberservation < minDistance)
        {
          // landmark is within detection range and nearer than previous landmark
		  minDistance = distanceToOberservation;

          association = currentlandmark.id_i;
          mu_x = currentlandmark.x_f;
          mu_y = currentlandmark.y_f;
        }
      }

      // if nearest neighbour is found
      if (association > -1)
      {
        currentParticle.weight *= multiv_prob(std_landmark[0], std_landmark[1],
          x_map, y_map, mu_x, mu_y);
        //currentParticle.weight = std::max(currentParticle.weight, __DBL_EPSILON__);

      }
      else{
         std::cout << "No LM association found for particel:" << currentParticle.id 
         << " observation:" << currentObservation.id << std::endl;
      }
      
      // debug data
      // write directly instead of using SetAssociations()
      // currentParticle.associations.push_back(association);
      // currentParticle.sense_x.push_back(x_map);
      // currentParticle.sense_y.push_back(y_map);
    }
    weights.push_back(currentParticle.weight);
  }
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::default_random_engine gen;

  std::discrete_distribution<> d(weights.begin(), weights.end());
  
  std::vector<Particle> new_partcle;
  for (int n=0; n<num_particles; ++n) {
    new_partcle.push_back(particles[d(gen)]);
  }

  particles = new_partcle;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}