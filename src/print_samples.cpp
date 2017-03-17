/*
 * print_samples.cpp
 *
 * Print out to the terminal 3 samples from a normal distribution with
 * mean equal to the GPS position and IMU heading measurements and
 * standard deviation of 2 m for the x and y position and 0.05 radians
 * for the heading of the car. 
 *
 * Author: Tiffany Huang
 */

#include <random> // Need this for sampling from distributions
#include <iostream>

using namespace std;

// @param gps_x 	GPS provided x position
// @param gps_y 	GPS provided y position
// @param psi 		GPS provided yaw
void printSamples(double gps_x, double gps_y, double psi) {
	default_random_engine gen;
	double std_x, std_y, std_psi; // Standard deviations for x, y, and psi

	// TODO: Set standard deviations for x, y, and psi.
	 
	
	
	// This line creates a normal (Gaussian) distribution for x.
	normal_distribution<double> dist_x(gps_x, std_x);
	
	// TODO: Create normal distributions for y and psi.
	
	

	for (int i = 0; i < 3; ++i) {
		double sample_x, sample_y, sample_psi;
		
		// TODO: Sample  and from these normal distrubtions like this: 
		// sample_x = dist_x(gen);
		// where "gen" is the random engine initialized earlier (line 18).

		
		
		// Print your samples to the terminal.
		cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_psi << endl;
	}

}

int main() {
	
	// Set GPS provided state of the car.
	double gps_x = 4983;
	double gps_y = 5029;
	double psi = 1.201;
	
	// Sample from the GPS provided position.
	printSamples(gps_x, gps_y, psi);
	
	return 0;
}

