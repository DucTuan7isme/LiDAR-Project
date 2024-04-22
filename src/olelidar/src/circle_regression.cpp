#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include <vector>
#include <Eigen/Dense>

struct DetectedObject {
    float max_intensity_distance;  
    float max_intensity_value; 
    size_t cluster_size;    
    size_t start_index;           
    size_t end_index;   
    float radius;    
    float mean_residual;    
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
        size_t i = start_index;
        while (i < intensities.size()) {
            if (intensities[i] > 3000) {
                size_t cluster_size = 1;
                float max_intensity_value = intensities[i];
                size_t j = i + 1;

                while (j < intensities.size() && intensities[j] > 3000) {
                    cluster_size++;
                    if (intensities[j] > max_intensity_value) {
                        max_intensity_value = intensities[j];
                    }
                    j++;
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

                start_index = j; // Update start_index to the next cluster
                i = start_index;
            } else {
                i++;
            }
        }

        if (!detected_objects.empty()) {
            for (auto& obj : detected_objects) {
                if (has_circular_arc_shape(obj, ranges, msg) && obj.radius < 0.1) {
                    ROS_INFO("Special object at distance %.2f has circular arc shape with radius %.2f centimeters.", 
                             obj.max_intensity_distance, obj.radius * 100);      
                } else {
                    ROS_INFO("Special object does not have circular arc shape.");
                }
            }
        } else  {
            ROS_INFO("No special objects detected.");
        }
    }

    bool has_circular_arc_shape(DetectedObject& obj, const std::vector<float>& ranges, const sensor_msgs::LaserScan::ConstPtr& msg) {
        // Extract points from the cluster
        std::vector<std::pair<float, float>> points;
        for (size_t i = obj.start_index; i <= obj.end_index; ++i) {
            // Calculate x and y coordinates in the new coordinate system
            float angle = msg->angle_min + i * msg->angle_increment - M_PI; 
            float x = ranges[i] * sin(angle); 
            float y = ranges[i] *  (-cos(angle)); 
            // Store the coordinates in the points vector
            points.emplace_back(x, y);
        }

        // Print the number of points in DetectedObject
        ROS_INFO("Number of points in DetectedObject: %zu", obj.cluster_size);
        
        // Print points
        ROS_INFO("Points:");
        for (const auto& point : points) {
            ROS_INFO("x: %.3f, y: %.3f", point.first, point.second);
        }
        
        // Fit circle to points
        Eigen::MatrixXd A(points.size(), 3);
        Eigen::VectorXd b(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            A(i, 0) = points[i].first;
            A(i, 1) = points[i].second;
            A(i, 2) = 1;
            b(i) = points[i].first * points[i].first + points[i].second * points[i].second;
        }
        Eigen::VectorXd center_radius = ( (A.transpose() * A).inverse() ) * A.transpose() * b;
        float x_center = center_radius(0) / 2;
        float y_center = center_radius(1) / 2;
        obj.radius = sqrt(x_center * x_center + y_center * y_center + center_radius(2)); 

        // Calculate residuals
        std::vector<float> residuals;
        for (const auto& point : points) {
            float distance_to_circle = fabs(sqrt(pow(point.first - x_center, 2) + pow(point.second - y_center, 2)) - obj.radius);
            residuals.push_back(distance_to_circle);
        }

        // Calculate the mean residual
        float sum_residuals = 0.0;
        for (float residual : residuals) {
            sum_residuals += residual;
        }
        float mean_residual = sum_residuals / static_cast<float>(residuals.size());

        // Check if the mean residual is below a threshold
        if (mean_residual < MAX_MEAN_RESIDUAL)  {
            return true; // Cluster has circular arc shape
        }

        return false; // Cluster does not have circular arc shape
    }

private:
    ros::Subscriber lidar_sub;
    sensor_msgs::LaserScan::ConstPtr msg;
    size_t start_index;

    static constexpr float MAX_MEAN_RESIDUAL = 0.02; 
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_processor_node");

    LidarProcessor lidar_processor;

    ros::spin();

    return 0;
}
