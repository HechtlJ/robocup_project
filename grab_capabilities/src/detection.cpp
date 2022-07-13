#include "detection.h"

SceneObservation::SceneObservation(){

}

void SceneObservation::addObservation(Observation new_observation){
    for(auto obs : observations){
        if(obs.id != new_observation.id){
            continue;
        }
        double diff_x = obs.xcenter - new_observation.xcenter;
        double diff_y = obs.ycenter - new_observation.ycenter;
        double distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));

        if(distance < MAX_DISTANCE_BETWEEN_SAME_OBSERVATIONS){
            return;
        }
    }
    observations.push_back(new_observation);
}

void SceneObservation::print(){
    std::cout << "**********Observations************" << std::endl;
    for(auto obs : observations){
        std::cout << obs.class_name << ": \t" << obs.xcenter << " \t" << obs.ycenter << std::endl;
    }
}

void SceneObservation::clear(){
    observations.clear();
}
