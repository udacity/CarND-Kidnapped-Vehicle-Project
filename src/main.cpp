/*
 * main.cpp
 * Reads in data and runs 2D particle filter.
 *  Created on: Dec 13, 2016
 *      Author: Tiffany Huang
 */

#include <iostream>
#include <ctime>
#include <iomanip>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;



int main() {
	
	// Start timer.
	int start = clock();
	
	//Set up parameters here
	double delta_t = 0.1; // Time elapsed between measurements [sec]
	double sensor_range = 50; // Sensor range [m]
	double max_runtime = 45; // Max allowable runtime to pass [sec]
	double max_translation_error = 2; // Max allowable translation error to pass [m]
	double max_yaw_error = 0.05; // Max allowable yaw error [rad]
	
	/*
	 * Sigmas - just an estimate, usually comes from uncertainty of sensor, but
	 * if you used fused data from multiple sensors, it's difficult to find
	 * these uncertainties directly.
	 */
	double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
	double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]
	
	// Read map data
	Map map;
	if (!read_map_data("data/map_data.txt", map)) {
		cout << "Error: Could not open map file" << endl;
		return -1;
	}

	// Read position data
	vector<control_s> position_meas;
	if (!read_control_data("data/control_data.txt", position_meas)) {
		cout << "Error: Could not open position/control measurement file" << endl;
		return -1;
	}
	
	// Read ground truth data
	vector<ground_truth> gt;
	if (!read_gt_data("data/gt_data.txt", gt)) {
		cout << "Error: Could not open ground truth data file" << endl;
		return -1;
	}
	
	// Run particle filter!
	int num_time_steps = position_meas.size();
	ParticleFilter pf;
	double total_error[3] = {0,0,0};
	double cum_mean_error[3] = {0,0,0};
	
	for (int i = 0; i < num_time_steps; ++i) {
		cout << "Time step: " << i+1 << endl;
		// Read in landmark observations for current time step.
		ostringstream file;
		file << "data/observation/observations_" << setfill('0') << setw(6) << i+1 << ".txt";
		vector<LandmarkObs> observations;
		if (!read_landmark_data(file.str(), observations)) {
			cout << "Error: Could not open observation file " << i+1 << endl;
			return -1;
		}
		
		// Initialize particle filter if this is the first time step.
		if (!pf.initialized()) {
			pf.init(gt[i].x, gt[i].y, gt[i].theta, sigma_pos);
		}
		else {
			// Predict the vehicle's next state
			pf.prediction(delta_t, sigma_pos, position_meas[i-1].velocity, position_meas[i-1].yawrate);
		}
		
		// Update the weights and resample
		pf.updateWeights(sensor_range, sigma_landmark, observations, map);
		pf.resample();
		
		// Calculate and output the average weighted error of the particle filter over all time steps so far.
		double *avg_error = pf.weighted_mean_error(gt[i].x, gt[i].y, gt[i].theta);

		for (int j = 0; j < 3; ++j) {
			total_error[j] += avg_error[j];
			cum_mean_error[j] = total_error[j] / (double)(i + 1);
		}
		
		// Print the cumulative weighted error
		cout << "Cumulative mean weighted error: x " << cum_mean_error[0] << " y " << cum_mean_error[1] << " yaw " << cum_mean_error[2] << endl;
		
		// If the error is too high, say so and then exit.
		if (cum_mean_error[0] > max_translation_error || cum_mean_error[1] > max_translation_error || cum_mean_error[2] > max_yaw_error) {
			if (cum_mean_error[0] > max_translation_error) {
				cout << "Your x error, " << cum_mean_error[0] << " is larger than the maximum allowable error, " << max_translation_error << endl;
			}
			else if (cum_mean_error[1] > max_translation_error) {
				cout << "Your y error, " << cum_mean_error[1] << " is larger than the maximum allowable error, " << max_translation_error << endl;
			}
			else {
				cout << "Your yaw error, " << cum_mean_error[2] << " is larger than the maximum allowable error, " << max_yaw_error << endl;
			}
			return -1;
		}
	}
	
	// Output the runtime for the filter.
	int stop = clock();
	double runtime = (stop - start) / double(CLOCKS_PER_SEC);
	cout << "Runtime (sec): " << runtime << endl;
	
	// Print success if accuracy and runtime are sufficient (and this isn't just the starter code).
	if (runtime < max_runtime && pf.initialized()) {
		cout << "Success! Your particle filter passed!" << endl;
	}
	else if (!pf.initialized()) {
		cout << "This is the starter code. You haven't initialized your filter." << endl;
	}
	else {
		cout << "Your runtime " << runtime << " is larger than the maximum allowable runtime, " << max_runtime << endl;
		return -1;
	}
	
	return 0;
}


