#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include <cmath>

struct DetectedObject {
    float max_intensity_distance;  // Distance to the point with the maximum intensity
    float max_intensity_value;     // Maximum intensity value
    size_t cluster_size;           // Number of consecutive points with intensities greater than 4000
    float max_intensity_angle;     // Angle in degrees of the maximum point in the cluster
};

class LidarProcessor {
public:
    LidarProcessor() {
        // Initialize ROS node and subscribe to the Lidar data
        ros::NodeHandle nh;
        lidar_sub = nh.subscribe("/front_scan", 1, &LidarProcessor::lidarCallback, this);
        alert_pub = nh.advertise<std_msgs::Bool>("/lidar_alert", 1);
    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        std::vector<float> ranges = msg->ranges;
        std::vector<float> intensities = msg->intensities;

        // Create a vector to store information about detected objects
        std::vector<DetectedObject> detected_objects;

        // Iterate through the intensities and check for clusters with at least 5 consecutive points greater than 4000
        for (size_t i = 0; i < intensities.size(); ++i) {
            if (intensities[i] > 4000) {
                size_t cluster_size = 1;
                float max_intensity_value = intensities[i];
                float max_intensity_distance = ranges[i];
                float max_intensity_angle = calculateAngle(msg->angle_min + i * msg->angle_increment);

                // Check the next points in the cluster
                for (size_t j = i + 1; j < intensities.size(); ++j) {
                    if (intensities[j] > 4000) {
                        cluster_size++;
                        // Update max intensity, distance, angle if a higher intensity is found
                        if (intensities[j] > max_intensity_value) {
                            max_intensity_value = intensities[j];
                            max_intensity_distance = ranges[j];
                            max_intensity_angle = calculateAngle(msg->angle_min + j * msg->angle_increment);
                        }
                    } else {
                        // Break the loop if consecutive points with intensity greater than 4000 are not found
                        break;
                    }
                }

                // Check if the cluster size is at least 5
                if (cluster_size >= 5) {
                    // Calculate distance and intensity values for the equations
                    float distance_for_equation = max_intensity_distance;  
                    float calculated_intensity = 0.0;

                    // Apply the corresponding equation based on the distance range
                    if (distance_for_equation < 0.75) {
                        calculated_intensity = -61070 * std::pow(distance_for_equation, 2) + 73960 * distance_for_equation - 7600;
                    } else {
                        calculated_intensity = 44 * std::pow(distance_for_equation, 4) - 640 * std::pow(distance_for_equation, 3)
                                              + 3760 * std::pow(distance_for_equation, 2) - 11700 * distance_for_equation + 21950;
                    }

                    // Check if the intensity satisfies the conditions (within 7% error)
                    float error_threshold = 0.07 * calculated_intensity;
                    if (std::abs(max_intensity_value - calculated_intensity) <= error_threshold) {
                        DetectedObject obj;
                        obj.max_intensity_distance = max_intensity_distance;
                        obj.max_intensity_value = max_intensity_value;
                        obj.cluster_size = cluster_size;
                        obj.max_intensity_angle = max_intensity_angle;
                        detected_objects.push_back(obj);

                        // Print out the value of the max intensity
                        ROS_INFO("Max Intensity Value: %.2f", obj.max_intensity_value);
                    }
                }

                // Move the index to the end of the cluster
                i += cluster_size - 1;
            }
        }

        // Display information about detected objects
        displayDetectedObjects(detected_objects);

        // Check if any detected objects are present and if their distance is less than 0.4 meters (40cm)
        if (!detected_objects.empty()) {
            for (const auto& obj : detected_objects) {
                if (obj.max_intensity_distance < 0.4) {
                    ROS_WARN("ALERT: Special Object is too close!");

                    // Publish an alert message
                    std_msgs::Bool alert_msg;
                    alert_msg.data = true;
                    alert_pub.publish(alert_msg);
                    break;  // Break out of the loop if any detected object is too close
                }
            }
        } else {
            ROS_INFO("No special objects detected.");
        }
    }

    void displayDetectedObjects(const std::vector<DetectedObject>& detected_objects) {
        for (const auto& obj : detected_objects) {
            ROS_INFO("Detected special object at distance %.2f meters with max intensity angle %.2f degrees",
                     obj.max_intensity_distance, obj.max_intensity_angle);
        }
    }

private:
    ros::Subscriber lidar_sub;
    ros::Publisher alert_pub;

    float calculateAngle(double radian_angle) {
        // Convert radians to degrees and adjust the angle based on your criteria
        float degree_angle = static_cast<float>(radian_angle * 180.0 / M_PI);

        // Ensure the angle is in the range [0, 90) and [90, 0)
        if (degree_angle < 0.0) {
            degree_angle += 90.0;
        } else if (degree_angle > 0.0) {
            degree_angle = 90.0 - degree_angle;
        }

        return degree_angle;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_processor_node");

    LidarProcessor lidar_processor;

    ros::spin();

    return 0;
}
