#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "particle_filter.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Set up parameters here
  double delta_t = 0.1;  // Time elapsed between measurements [sec]
  double sensor_range = 50;  // Sensor range [m]

  // GPS measurement uncertainty [x [m], y [m], theta [rad]]
  double sigma_pos [3] = {0.3, 0.3, 0.01};
  // Landmark measurement uncertainty [x [m], y [m]]
  double sigma_landmark [2] = {0.3, 0.3};

  // Read map data
  Map map;
  if (!read_map_data("../data/map_data.txt", map)) {
    std::cout << "Error: Could not open map file" << std::endl;
    return -1;
  }

  // Create particle filter
  ParticleFilter pf;

  h.onMessage([&pf,&map,&delta_t,&sensor_range,&sigma_pos,&sigma_landmark]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          if (!pf.initialized()) {
            // Sense noisy position data from the simulator
            double sense_x = std::stod(j[1]["sense_x"].get<string>());
            double sense_y = std::stod(j[1]["sense_y"].get<string>());
            double sense_theta = std::stod(j[1]["sense_theta"].get<string>());

            pf.init(sense_x, sense_y, sense_theta, sigma_pos);
          } else {
            // Predict the vehicle's next state from previous 
            //   (noiseless control) data.
            double previous_velocity = std::stod(j[1]["previous_velocity"].get<string>());
            double previous_yawrate = std::stod(j[1]["previous_yawrate"].get<string>());

            pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);
          }

          // receive noisy observation data from the simulator
          // sense_observations in JSON format 
          //   [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}] 
          vector<LandmarkObs> noisy_observations;
          string sense_observations_x = j[1]["sense_observations_x"];
          string sense_observations_y = j[1]["sense_observations_y"];

          vector<float> x_sense;
          std::istringstream iss_x(sense_observations_x);

          std::copy(std::istream_iterator<float>(iss_x),
          std::istream_iterator<float>(),
          std::back_inserter(x_sense));

          vector<float> y_sense;
          std::istringstream iss_y(sense_observations_y);

          std::copy(std::istream_iterator<float>(iss_y),
          std::istream_iterator<float>(),
          std::back_inserter(y_sense));

          for (int i = 0; i < x_sense.size(); ++i) {
            LandmarkObs obs;
            obs.x = x_sense[i];
            obs.y = y_sense[i];
            noisy_observations.push_back(obs);
          }

          // Update the weights and resample
          pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
          pf.resample();

          // Calculate and output the average weighted error of the particle 
          //   filter over all time steps so far.
          vector<Particle> particles = pf.particles;
          int num_particles = particles.size();
          double highest_weight = -1.0;
          Particle best_particle;
          double weight_sum = 0.0;
          for (int i = 0; i < num_particles; ++i) {
            if (particles[i].weight > highest_weight) {
              highest_weight = particles[i].weight;
              best_particle = particles[i];
            }

            weight_sum += particles[i].weight;
          }

          std::cout << "highest w " << highest_weight << std::endl;
          std::cout << "average w " << weight_sum/num_particles << std::endl;

          json msgJson;
          msgJson["best_particle_x"] = best_particle.x;
          msgJson["best_particle_y"] = best_particle.y;
          msgJson["best_particle_theta"] = best_particle.theta;

          // Optional message data used for debugging particle's sensing 
          //   and associations
          msgJson["best_particle_associations"] = pf.getAssociations(best_particle);
          msgJson["best_particle_sense_x"] = pf.getSenseCoord(best_particle, "X");
          msgJson["best_particle_sense_y"] = pf.getSenseCoord(best_particle, "Y");

          auto msg = "42[\"best_particle\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}