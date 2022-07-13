#include <string>
#include <vector>
#include <iostream>

#include <darknet_ros_msgs/BoundingBoxes.h>


#define MAX_DISTANCE_BETWEEN_SAME_OBSERVATIONS 20.0

struct Observation{
    std::string class_name;
    int xmin;
    int ymin;
    int xmax;
    int ymax;
    int xcenter;
    int ycenter;
    int id;

    Observation(darknet_ros_msgs::BoundingBox box){
        xmin = box.xmin;
        ymin = box.ymin;
        xmax = box.xmax;
        ymax = box.ymax;
        xcenter = (xmin + xmax)/2;
        ycenter = (ymin + ymax)/2;
        id = box.id;
        class_name = box.Class;
    }
};

class SceneObservation{
public:
    SceneObservation();
    void addObservation(Observation new_observation);
    void print();
    void clear();
private:
    
public:

private:
    std::vector<Observation> observations;

};

/*
class Observation{
public:
    Observation();

    std::vector<std::string> observed_classes;

    void add_class(std::string &new_class){
        for(int i=0; i<observed_classes.size(); i++){
            if(new_class.compare(observed_classes[i])==0){
                return;
            }
        }
        observed_classes.push_back(new_class);
    }

    void add_bounding_boxes_msg(darknet_ros_msgs::BoundingBoxesConstPtr &msg){
        for(int i=0; i<msg->bounding_boxes.size(); i++){
            add_class(msg->bounding_boxes[i].Class);
        }
    }
};*/