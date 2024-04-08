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
        this->msg = msg; 

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
                if (isCircularArc(intensities, ranges, obj.start_index, obj.end_index)) {
                    ROS_INFO("Special object at distance %.2f. Circular arc detected.",
                             obj.max_intensity_distance);
                } else {
                    ROS_INFO("Special object detected, but not circular shape.");
                }
            }
        } else {
            ROS_INFO("No special objects detected.");
        }
    }

    bool isCircularArc(const std::vector<float>& intensities, const std::vector<float>& ranges,
                       size_t start_index, size_t end_index) {
        // Extract points for circle fitting
        float x1 = ranges[start_index] * cos(start_index * msg->angle_increment);
        float y1 = ranges[start_index] * sin(start_index * msg->angle_increment);
        float x2 = ranges[end_index] * cos(end_index * msg->angle_increment);
        float y2 = ranges[end_index] * sin(end_index * msg->angle_increment);
        float x3 = ranges[(start_index + end_index) / 2] * cos((start_index + end_index) / 2 * msg->angle_increment);
        float y3 = ranges[(start_index + end_index) / 2] * sin((start_index + end_index) / 2 * msg->angle_increment);

        // Calculate center of the circle
        float x_center, y_center;
        float denominator = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
        if (denominator == 0)
            return false; // Points are collinear, can't form a circle
        x_center = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) / denominator;
        y_center = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) / denominator;

        // Calculate radius of the circle
        float total_distance = 0.0;
        for (size_t i = start_index; i <= end_index; ++i) {
            float x = ranges[i] * cos(i * msg->angle_increment);
            float y = ranges[i] * sin(i * msg->angle_increment);
            float distance_to_center = sqrt((x - x_center) * (x - x_center) + (y - y_center) * (y - y_center));
            total_distance += distance_to_center;
        }
        float radius = total_distance / (end_index - start_index + 1);

        ROS_INFO("Radius of the detected circular arc: %.2f centimeters", radius * 100);

        // Check if the radius is less than 10 centimeters
        if (radius <= 0.1) {
            // Check if all other points in the cluster are within the error margin of the circle
            for (size_t i = start_index; i <= end_index; ++i) {
                float x = ranges[i] * cos(i * msg->angle_increment);
                float y = ranges[i] * sin(i * msg->angle_increment);
                float distance_to_center = sqrt((x - x_center) * (x - x_center) + (y - y_center) * (y - y_center));
                if (fabs(distance_to_center - radius) > max_error)
                    return false; // Point is not within the error margin
            }
            return true; // All points are within the error margin, cluster forms a circular arc
        }

        return false; // Radius is greater than 10 centimeters
    }

private:
    ros::Subscriber lidar_sub;
    sensor_msgs::LaserScan::ConstPtr msg;
    size_t start_index;
    float max_error = 0.05; // maximum error allowed for circular arc detection
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_processor_node");

    LidarProcessor lidar_processor;

    ros::spin();

    return 0;
}
