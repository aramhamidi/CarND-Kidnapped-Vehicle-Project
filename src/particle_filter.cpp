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
	num_particles = 10;

	default_random_engine gen;
	// This line creates a normal (Gaussian) distribution for x, y and theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles ; i++){

		Particle particle;
		particle.id = i;
		particle.weight = 1;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);

		weights.push_back(1.0);
		particles.push_back(particle);
	}

   is_initialized = true;
   cout << "initialization is done" <<endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;



	for (int i = 0; i < num_particles; i++){
        
        double x = particles[i].x;
        double y = particles[i].y;
        double theta = particles[i].theta;
        
		if (fabs(yaw_rate) > 0.0001){
            particles[i].x =  x + ((velocity / yaw_rate) * (sin(theta + (yaw_rate * delta_t)) - sin(theta)));
            particles[i].y = y + ((velocity / yaw_rate) * (cos(theta) - cos(theta + (yaw_rate * delta_t))));
            particles[i].theta = theta + (yaw_rate * delta_t);
		}
		else{
            particles[i].x = x + (velocity * delta_t) * cos(theta);
            particles[i].y = y + (velocity * delta_t) * sin(theta);
            particles[i].theta = theta;
		}
		// Add some noise to the predition
		normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
		normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
		normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
	}
	cout<<"prediction step is done" <<endl;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	

	// We want to associate each landmark to a measurment.
	// in each step we have a set of sensor measurements and a set up particles
	// number of landmarks are base on map.
	// predictions are particles
	// observed measurements are where the sensor thinks the landmark is
	// predicted -> valid_landmarks and observations -> transformed_observations
	
	LandmarkObs nearest_Landmark;
	for (int i = 0; i < observations.size(); i++){
		double min_distance = numeric_limits<double>::max();
		// pick the nearest landmark which is also in sensor range
		for (int j = 0; j < predicted.size() ; j++){
			double distance = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
			if (distance < min_distance){
				min_distance = distance;
				observations[i].id = predicted[j].id;
				nearest_Landmark = predicted[j];		
			}

		}

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
	for (int k = 0; k < num_particles ; k++){
		
		// STEP1: Evaluate the accuarcy of landmark observations by comparing the distance of landmark to sensor range
		vector<LandmarkObs> valid_landmarks;
        LandmarkObs valid_land;
		for (int i = 0; i < map_landmarks.landmark_list.size() ; i++){
			//double dist_to_particle = dist(map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f, particles[k].x, particles[k].y);
			//if (dist_to_particle <= sensor_range){
			if((fabs(particles[k].x - map_landmarks.landmark_list[i].x_f) <= sensor_range) && (fabs(particles[k].y - map_landmarks.landmark_list[i].y_f) <= sensor_range) ){
				valid_land.id = map_landmarks.landmark_list[i].id_i;
				valid_land.x = map_landmarks.landmark_list[i].x_f;
				valid_land.y = map_landmarks.landmark_list[i].y_f;
				valid_landmarks.push_back(valid_land);
			}
		}
		//cout<< "number of valid landmarks for particle " << k << " is " << valid_landmarks.size() << endl;
		
        
		// STEP2: Transform the observations coordinates to Map cordinates
		vector <LandmarkObs> transformed_observations;
		LandmarkObs landObj;
		for (int i = 0 ; i < observations.size(); i++){
			LandmarkObs translan;
			landObj = observations[i];
			translan.x = particles[k].x + ((landObj.x * cos(particles[k].theta)) - (landObj.y * sin(particles[k].theta)));
			translan.y = particles[k].y + ((landObj.x * sin(particles[k].theta)) + (landObj.y * cos(particles[k].theta)));
			translan.id = observations[i].id;
			transformed_observations.push_back(translan);
		}
		//cout <<"observations are transformed to global coordinate system" << endl;

		// STEP3: find the nearest landmark for each transformed observation and associate the observation to that landmark
		vector <int> associations;
		vector <double> sense_x;
		vector <double> sense_y;

		dataAssociation(valid_landmarks, transformed_observations);
		//cout <<"for particle " << k << " total of " << valid_landmarks.size() << " landmarks are associated to observations " << endl ;
		// update the particle weights
		particles[k].weight = 1.0;
		weights[k] = 1.0;

		for (int i = 0; i < transformed_observations.size() ; i++){
			for (int j = 0; j < valid_landmarks.size(); j++ ){
				if (transformed_observations[i].id == valid_landmarks[j].id){
					double normalizer = 1.0 / (2.0 * M_PI * std_landmark[0] * std_landmark[1]);
                    //cout << "Normalaizer   " << normalizer << endl;
					double x_part = pow((transformed_observations[i].x - valid_landmarks[j].x) / (std_landmark[0]),2)/2.0;
                    //cout << "x_part     " << x_part << endl;
					double y_part = pow((transformed_observations[i].y - valid_landmarks[j].y) / (std_landmark[1]),2)/2.0;
                    //cout << "y_part         " << y_part <<endl;
					double prob_weight = normalizer * exp(-(x_part + y_part));
                    //cout << "x_part + y_part                " <<(x_part + y_part)<<endl;
                    //cout << "exp(-(x_part + y_part))                " <<exp(-(x_part + y_part))<<endl;
					//cout << "prob_weight is now :                   " << prob_weight <<endl;
					particles[k].weight *= prob_weight;
					weights[k] = particles[k].weight;
				}
			}
			associations.push_back(transformed_observations[i].id);
			sense_x.push_back(transformed_observations[i].x);
			sense_y.push_back(transformed_observations[i].y);

		}
		SetAssociations(particles[k] , associations, sense_x, sense_y);	
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	vector<Particle> resampled;
	default_random_engine gen;
	discrete_distribution <int> weights_dis (weights.begin(), weights.end());

	for (int i = 0; i < num_particles ; i++){
		resampled.push_back(particles[weights_dis(gen)]);
	}

	particles = resampled;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    //Associations - vector of ints - stores id of the landmarks, associated with the observations
	//Sense_x for each association stores observation.x in global (map) coordinate
	//Sense_y for each association stores observation.y in global (map) coordinate

    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
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
