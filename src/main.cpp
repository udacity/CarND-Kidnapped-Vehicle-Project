#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "particle_filter.h"
#include <thread>
#include "helper_functions.h"
#include <ctime>


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s, size_t length) {
  return s.substr(0, length);
}

int main()
{
  uWS::Hub h;
  
  //Set up parameters here
  double delta_t = 0.1; // Time elapsed between measurements [sec]
  double sensor_range = 50; // Sensor range [m]

  double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
  double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

  // Read map data
  Map map;
  if (!read_map_data("../data/map_data.txt", map)) {
           cout << "Error: Could not open map file" << endl;
           return -1;
  }

  // Create particle filter
  ParticleFilter pf;
  

  h.onMessage([&pf,&map,&delta_t,&sensor_range,&sigma_pos,&sigma_landmark](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
    clock_t begin = clock();

    if (length && length > 2)  
    {
      auto s = hasData(std::string(data),length);
      if (s != "") {
        auto j = json::parse(s);
        int process = std::stod(j.at("telemetry")["process"].get<std::string>());      
        
        if(process == 1){
          if (!pf.initialized()) {
            double sense_x = std::stod(j.at("telemetry")["sense_x"].get<std::string>());
            double sense_y = std::stod(j.at("telemetry")["sense_y"].get<std::string>());     
            double sense_theta = std::stod(j.at("telemetry")["sense_theta"].get<std::string>());  
            pf.init(sense_x, sense_y, sense_theta, sigma_pos);
          }
          else {
                double previous_velocity  = std::stod(j.at("telemetry")["previous_velocity"].get<std::string>());
                double previous_yawrate  = std::stod(j.at("telemetry")["previous_yawrate"].get<std::string>());
                pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);
          }

        // receive noisy observation data from the simulator
		// sense_observations in JSON format [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
		  	
        vector<LandmarkObs> noisy_observations;
        string sense_observations_x = j.at("telemetry")["sense_observations_x"];
        string sense_observations_y = j.at("telemetry")["sense_observations_y"];

        std::vector<float> x_sense;
        std::istringstream iss_x(sense_observations_x);

        std::copy(std::istream_iterator<float>(iss_x),
        std::istream_iterator<float>(),
        std::back_inserter(x_sense));

        std::vector<float> y_sense;
        std::istringstream iss_y(sense_observations_y);

        std::copy(std::istream_iterator<float>(iss_y),
        std::istream_iterator<float>(),
        std::back_inserter(y_sense));

          for(int i = 0; i < x_sense.size(); i++)
          {
            LandmarkObs obs;
            obs.x = x_sense[i];
        	obs.y = y_sense[i];
        	noisy_observations.push_back(obs);
          }

        // Update the weights and resample
        pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
        pf.resample();

        // Calculate and output the average weighted error of the particle filter over all time steps so far.
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
        cout << "highest w " << highest_weight << endl;
        cout << "average w " << weight_sum/num_particles << endl;
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            json msgJson;
            msgJson["best_particle_x"] = best_particle.x;
            msgJson["best_particle_y"] = best_particle.y;
            msgJson["best_particle_theta"] = best_particle.theta;
            

            //Optional message data used for debugging particle's sensing and associations
            msgJson["best_particle_associations"] = pf.getAssociations(best_particle);
            msgJson["best_particle_sense_x"] = pf.getSenseX(best_particle);
            msgJson["best_particle_sense_y"] = pf.getSenseY(best_particle);
            
            //time for workspaces
            msgJson["time"] = elapsed_secs;
            

            auto msg = "{\"best_particle\"," + msgJson.dump() + "}";
            ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    
        }
      else {
        std::string msg = "{\"manual\",{}}";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
    }

  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 3001;

  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
