#include <iostream>
#include "gtest/gtest.h"
#include "../src/helper_functions.h"
#include "../src/particle_filter.h"

using std::vector;
using namespace std;

TEST(Helpers, bivariate_gausian) {
    // These test values were calculated using the following example Python snippet
    // from scipy.stats import multivariate_normal
    // import numpy as np
    // x = np.array([[1,2], [3,4]])
    // multivariate_normal.pdf(x, mean=[0, 1], cov=[5, 2])

    float comparison_tolerance = 1e-10;

    double center[] = {1, 1};
    double point[]  = {1, 1};
    double sigmas[] = {1, 1};
    EXPECT_NEAR(0.15915494309189535, bivariate_gausian(center, point, sigmas), comparison_tolerance);

    center[0] = 0;
    center[1] = 0;
    point[0] = 0;
    point[1] = 0;
    EXPECT_NEAR(0.15915494309189535, bivariate_gausian(center, point, sigmas), comparison_tolerance);

    center[0] = 0;
    center[1] = 1;
    point[0] = 1;
    point[1] = 2;
    sigmas[0] = 5;
    sigmas[1] = 2;
    EXPECT_NEAR(0.013767257383326089, bivariate_gausian(center, point, sigmas), comparison_tolerance);

    point[0] = 3;
    point[1] = 4;
    EXPECT_NEAR(0.0043158449383267454, bivariate_gausian(center, point, sigmas), comparison_tolerance);

    point[0] = 2;
    point[1] = -1;
    EXPECT_NEAR(0.0089110592667961709, bivariate_gausian(center, point, sigmas), comparison_tolerance);

    point[0] = -4;
    point[1] = -3;
    EXPECT_NEAR(0.001564072692429866, bivariate_gausian(center, point, sigmas), comparison_tolerance);
}

TEST(Helpers, sort_landmarks) {
    vector<LandmarkObs> sorted_landmarks;
    vector<LandmarkObs> landmarks;

    for (unsigned int i = 0; i < 5; i++) {
        LandmarkObs landmark;
        landmark.id = i;
        landmark.x  = i;
        landmark.y  = i;
        sorted_landmarks.push_back(landmark);
    }

    landmarks.push_back(sorted_landmarks[3]);
    landmarks.push_back(sorted_landmarks[0]);
    landmarks.push_back(sorted_landmarks[4]);
    landmarks.push_back(sorted_landmarks[1]);
    landmarks.push_back(sorted_landmarks[2]);

    sort(landmarks.begin(), landmarks.end());

    for (unsigned int i = 0; i < 5; i++) {
        EXPECT_EQ(i, landmarks[i].id);
    }
}

TEST(ParticleFilter, transformLandmarks) {
    // NB: remove 'test/' from the file paths to test the project data.
    float comparison_tolerance = 1.2e-2;

    ParticleFilter pf;
    Particle ground_truth_particle;

    vector<ground_truth> gt;
    if (!read_gt_data("test/data/gt_data.txt", gt)) {
        cout << "Error: Could not open ground truth data file" << endl;
    }

    Map map;
    if (!read_map_data("test/data/map_data.txt", map)) {
        cout << "Error: Could not open map file" << endl;
    }

    const unsigned int num_time_steps = 7;

    vector<LandmarkObs> transformed_landmarks;
    for (unsigned int i = 0; i < num_time_steps; ++i) {
        ostringstream file;
        vector<LandmarkObs> observations;
		file << "test/data/observation/observations_" << setfill('0') << setw(6) << i+1 << ".txt";
        read_landmark_data(file.str(), observations);

        ground_truth_particle.x = gt[i].x;
        ground_truth_particle.y = gt[i].y;
        ground_truth_particle.theta = gt[i].theta;

        // cout << "Observation: " << i+1 << "   ***************************" << endl;

        transformed_landmarks = pf.transformLandmarks(ground_truth_particle, map);
        sort(transformed_landmarks.begin(), transformed_landmarks.end());
        for (unsigned int j = 0; j < observations.size(); j++) {
            // cout << "landmark id: " << transformed_landmarks[j].id << endl;
            EXPECT_NEAR(observations[j].x, transformed_landmarks[j].x, comparison_tolerance);
            EXPECT_NEAR(observations[j].y, transformed_landmarks[j].y, comparison_tolerance);
        }
    }
}
