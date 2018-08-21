/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

#define PARTICLE_DEFAULT_WEIGHT 1.0
#define EPS 0.00001

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    num_particles = 20; // TODO: tune this parameter for better performance

    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    for (int i = 0; i < num_particles; i++) {
        Particle current_particle;
        current_particle.x = dist_x(gen);
        current_particle.y = dist_y(gen);
        current_particle.theta = dist_theta(gen);
        current_particle.weight = PARTICLE_DEFAULT_WEIGHT;
        current_particle.id = i;

        particles.push_back(current_particle);
        weights.push_back(current_particle.weight);
    }

    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    double std_x = std_pos[0];
    double std_y = std_pos[1];
    double std_theta = std_pos[2];

    normal_distribution<double> dist_x(0, std_x);
    normal_distribution<double> dist_y(0, std_y);
    normal_distribution<double> dist_theta(0, std_theta);

    for (int i = 0; i < num_particles; i++) {
        double theta = particles[i].theta;

        // check for division by zero when yaw rate is not chainging
        if (fabs(yaw_rate) < EPS) {
            particles[i].x += velocity * delta_t * cos(theta);
            particles[i].y += velocity * delta_t * sin(theta);
        } else {
            particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
            particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }

        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

    for (LandmarkObs &observation : observations) {
        double min_distance = numeric_limits<double>::max();    // init to big number
        int closest_landmark_id = -1;   // landmark not found

        for (LandmarkObs &predicted_element : predicted) {
            double distance = dist(observation.x, observation.y, predicted_element.x, predicted_element.y);

            if (distance < min_distance) {
                min_distance = distance;
                closest_landmark_id = predicted_element.id;
            }
        }

        observation.id = closest_landmark_id;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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

    const double sigma_x = std_landmark[0];
    const double sigma_y = std_landmark[1];
    double weight_normalizer = 0.0;

    for (int i = 0; i < num_particles; i++) {

        double x = particles[i].x;
        double y = particles[i].y;
        double theta = particles[i].theta;

        // SEARCH FOR LANDMARKS IN PARTICLE RANGE
        double squared_sensor_range = sensor_range * sensor_range;
        vector<LandmarkObs> in_landmarks_range;

        for (Map::single_landmark_s map_landmark : map_landmarks.landmark_list) {
            float landmark_x = map_landmark.x_f;
            float landmark_y = map_landmark.y_f;

            double dx = x - landmark_x;
            double dy = y - landmark_y;
            if (dx * dx + dy * dy <= squared_sensor_range)
                in_landmarks_range.push_back(LandmarkObs{map_landmark.id_i, landmark_x, landmark_y});
        }

        // TRANSFORM TO MAP COORDINATES
        vector<LandmarkObs> observations_in_map_coords;
        for (const LandmarkObs &observation : observations) {
            double landmark_x = cos(theta) * observation.x - sin(theta) * observation.y + x;
            double landmark_y = sin(theta) * observation.x + cos(theta) * observation.y + y;
            observations_in_map_coords.push_back(LandmarkObs{observation.id, landmark_x, landmark_y});
        }

        // ASSOCIATE OBSERVATIONS TO LANDMARKS
        dataAssociation(in_landmarks_range, observations_in_map_coords);


        // COMPUTE WEIGHT USING MULTIVARIATE GAUSSIAN DISTRIBUTION
        particles[i].weight = PARTICLE_DEFAULT_WEIGHT;

        double sigma_x_2 = pow(sigma_x, 2);
        double sigma_y_2 = pow(sigma_y, 2);
        double normalizer = (1.0 / (2.0 * M_PI * sigma_x * sigma_y));

        for (const LandmarkObs &observation_in_map_coords : observations_in_map_coords) {
            double trans_obs_x = observation_in_map_coords.x;
            double trans_obs_y = observation_in_map_coords.y;
            double trans_obs_id = observation_in_map_coords.id;
            double multi_prob;

            for(const LandmarkObs &in_landmark_range : in_landmarks_range){
                double pred_landmark_x = in_landmark_range.x;
                double pred_landmark_y = in_landmark_range.y;
                double pred_landmark_id = in_landmark_range.id;

                if (trans_obs_id == pred_landmark_id) {
                    multi_prob = normalizer * exp(-1.0 *
                                                  ((pow((trans_obs_x - pred_landmark_x), 2) / (2.0 * sigma_x_2)) +
                                                   (pow((trans_obs_y - pred_landmark_y), 2) / (2.0 * sigma_y_2))));
                    particles[i].weight *= multi_prob;
                }
            }
        }
        weight_normalizer += particles[i].weight;
    }

    // NORMALIZE WEIGHTS
    for (int i = 0; i < particles.size(); i++) {
        particles[i].weight /= weight_normalizer;
        weights[i] = particles[i].weight;
    }


}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    uniform_int_distribution<int> dist_index(0, num_particles - 1);
    int index = dist_index(gen);

    double beta = 0.0;
    double max_weight_2 = 2.0 * *max_element(weights.begin(), weights.end());
    uniform_real_distribution<double> dist_random_weight(0.0, max_weight_2);

    vector<Particle> resampled_particles;
    for (int i = 0; i < num_particles; i++) {
        beta += dist_random_weight(gen) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        resampled_particles.push_back(particles[index]);
    }

    particles = resampled_particles;

}

Particle ParticleFilter::SetAssociations(Particle &particle, const std::vector<int> &associations,
                                         const std::vector<double> &sense_x, const std::vector<double> &sense_y) {
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best) {
    vector<double> v = best.sense_x;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best) {
    vector<double> v = best.sense_y;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}
