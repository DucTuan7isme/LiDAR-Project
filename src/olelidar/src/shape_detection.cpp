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
                float average_angle = getAverageAngle(obj, ranges);
                float std_deviation = getStandardDeviation(obj, ranges, average_angle);
                if (has_circular_arc_shape(obj, ranges, average_angle, std_deviation)) {
                    ROS_INFO("Special object at distance %.2f. Average angle: %.2f degrees; standard deviation: %.2f degree.",
                        obj.max_intensity_distance, average_angle, std_deviation);      
                } else {
                    ROS_INFO("Special object detected, but not circular shape.");
                }
            }
        } else  {
            ROS_INFO("No special objects detected.");
        }
    }

    bool has_circular_arc_shape(const DetectedObject& obj, const std::vector<float>& ranges, float average_angle, float std_deviation) {
        // Check if both average angle and standard deviation are below 9 degrees
        return (90 <= average_angle && average_angle <= 160) && (std_deviation < 10.0);
    }

    float getAverageAngle(const DetectedObject& obj, const std::vector<float>& ranges) {
        // Calculate inscribed angles and check if they satisfy circular arc criteria
        float sum_angles = 0.0;
        for (size_t i = obj.start_index + 1; i < obj.end_index; ++i) {
            float angle = calculateAngle(obj.start_index, i, obj.end_index, ranges);
            sum_angles += angle;
        }

        return sum_angles / (obj.end_index - obj.start_index - 1);
    }

    float getStandardDeviation(const DetectedObject& obj, const std::vector<float>& ranges, float average_angle) {
        // Calculate standard deviation of angles with respect to the average angle
        float sum_squared_differences = 0.0;
        for (size_t i = obj.start_index + 1; i < obj.end_index; ++i) {
            float angle = calculateAngle(obj.start_index, i, obj.end_index, ranges);
            float difference = angle - average_angle;
            sum_squared_differences += difference * difference;
        }

        float variance = sum_squared_differences / (obj.end_index - obj.start_index - 1);
        return std::sqrt(variance);
    }

    float calculateAngle(size_t start_index, size_t current_index, size_t end_index, const std::vector<float>& ranges) {
        // Calculate vectors from the LiDAR to the points
        float x1 = ranges[start_index] * std::cos(start_index * msg->angle_increment);
        float y1 = ranges[start_index] * std::sin(start_index * msg->angle_increment);

        float x2 = ranges[current_index] * std::cos(current_index * msg->angle_increment);
        float y2 = ranges[current_index] * std::sin(current_index * msg->angle_increment);

        float x3 = ranges[end_index] * std::cos(end_index * msg->angle_increment);
        float y3 = ranges[end_index] * std::sin(end_index * msg->angle_increment);

        // Calculate the vectors between the points
        float vec1_x = x1 - x2;
        float vec1_y = y1 - y2;

        float vec2_x = x3 - x2;
        float vec2_y = y3 - y2;

        // Calculate the dot product of the vectors
        float dot_product = vec1_x * vec2_x + vec1_y * vec2_y;

        // Calculate the magnitudes of the vectors
        float magnitude1 = std::sqrt(vec1_x * vec1_x + vec1_y * vec1_y);
        float magnitude2 = std::sqrt(vec2_x * vec2_x + vec2_y * vec2_y);

        // Calculate the cosine of the angle between the vectors
        float cos_angle = dot_product / (magnitude1 * magnitude2);

        // Convert cosine to angle in degrees using inverse cosine (arccos)
        return std::acos(cos_angle) * 180.0 / M_PI;
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
