#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"

struct DetectedObject {
    float max_intensity_distance;  // Distance to the point with the maximum intensity
    float max_intensity_value;     // Maximum intensity value
    size_t cluster_size;           // Number of consecutive points with intensities greater than 8000
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

                // Check the next points in the cluster
                for (size_t j = i + 1; j < intensities.size(); ++j) {
                    if (intensities[j] > 4000) {
                        cluster_size++;
                        // Update max intensity and distance if a higher intensity is found
                        if (intensities[j] > max_intensity_value) {
                            max_intensity_value = intensities[j];
                            max_intensity_distance = ranges[j];
                        }
                    } else {
                        // Break the loop if consecutive points with intensity greater than 8000 are not found
                        break;
                    }
                }

                // Check if the cluster size is at least 5
                if (cluster_size >= 5) {
                    DetectedObject obj;
                    obj.max_intensity_distance = max_intensity_distance;
                    obj.max_intensity_value = max_intensity_value;
                    obj.cluster_size = cluster_size;
                    detected_objects.push_back(obj);
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
            ROS_INFO("Detected special object at distance %.2f meters", obj.max_intensity_distance);
        }
    }

private:
    ros::Subscriber lidar_sub;
    ros::Publisher alert_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_processor_node");

    LidarProcessor lidar_processor;

    ros::spin();

    return 0;
}