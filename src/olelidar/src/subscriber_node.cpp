#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <vector>

struct DetectedObject {
    float max_intensity_distance;  
    float max_intensity_value; 
    size_t cluster_size;    
    size_t start_index;           
    size_t end_index;              
};

class LidarProcessor {
public:
    LidarProcessor() {
        ros::NodeHandle nh;
        lidar_sub = nh.subscribe("/denoise_node", 1, &LidarProcessor::lidarCallback, this);
        start_index = 0;
    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        this->msg = msg;  // Store the latest message for coordinate calculation

        std::vector<float> ranges = msg->ranges;
        std::vector<float> intensities = msg->intensities;

        std::vector<DetectedObject> detected_objects;

        start_index = 0;
        for (size_t i = start_index; i < intensities.size(); ++i) {
            if (intensities[i] > 3000) {
                size_t cluster_size = 1;
                float max_intensity_value = intensities[i];

                for (size_t j = i + 1; j < intensities.size(); ++j) {
                    if (intensities[j] > 3000) {
                        cluster_size++;
                        if (intensities[j] > max_intensity_value) {
                            max_intensity_value = intensities[j];
                        }
                    } else {
                        break;
                    }
                }

                if (cluster_size >= 5) {
                    DetectedObject obj;
                    obj.max_intensity_distance = ranges[i];
                    obj.max_intensity_value = max_intensity_value;
                    obj.cluster_size = cluster_size;
                    obj.start_index = i;
                    obj.end_index = i + cluster_size - 1;
                    detected_objects.push_back(obj);
                }

                start_index = i + cluster_size;
            
            }
        }

        if (!detected_objects.empty()) {
            for (auto& obj : detected_objects) {
                if (has_circular_arc_shape(obj, ranges)) {
                    ROS_INFO("Special object at distance %.2f has circular arc shape.", obj.max_intensity_distance);      
                } else {
                    ROS_INFO("Special object at distance %.2f does not have circular arc shape.", obj.max_intensity_distance);
                }
            }
        } else  {
            ROS_INFO("No special objects detected.");
        }
    }

    bool has_circular_arc_shape(const DetectedObject& obj, const std::vector<float>& ranges) {
        // Perform circular arc interpolation to determine if the cluster has circular arc shape
        // Implement your circular arc interpolation algorithm here
        // For demonstration, let's assume it always returns true for circular arc shape
        return true;
    }

private:
    ros::Subscriber lidar_sub;
    sensor_msgs::LaserScan::ConstPtr msg;
    size_t start_index;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_processor_node");

    LidarProcessor lidar_processor;

    ros::spin();

    return 0;
}
