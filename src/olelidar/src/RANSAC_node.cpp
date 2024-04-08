#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <cmath>

struct DetectedObject {
    float max_intensity_distance;
    float max_intensity_value;
    size_t cluster_size;
    size_t cluster_start_index;
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
                    obj.cluster_start_index = i; 
                    detected_objects.push_back(obj);
                }

                start_index = i + cluster_size;
                i += cluster_size - 1;
            }
        }

        if (!detected_objects.empty()) {
            bool has_cylinder_shape = false;
            for (auto& obj : detected_objects) {
                if (isCylinderShape(obj)) {
                    has_cylinder_shape = true;
                    ROS_INFO("Special object at distance %.2f meters with intensity %.2f",
                             obj.max_intensity_distance, obj.max_intensity_value);
                    ROS_INFO("Number of points in the cluster: %zu", obj.cluster_size);
                }
            }
            if (!has_cylinder_shape) {
                ROS_INFO("Special object detected, but not circular shape.");
            }
        } else  {
            ROS_INFO("No special objects detected.");
        }
    }

    bool isCylinderShape(const DetectedObject& obj) {
        // Extract the points corresponding to the detected cluster
        std::vector<double> cluster_angles;
        std::vector<double> cluster_distances;

        for (size_t k = 0; k < obj.cluster_size; ++k) {
            double angle = msg->angle_min + (obj.cluster_start_index + k) * msg->angle_increment;
            double distance = obj.max_intensity_distance;

            cluster_angles.push_back(angle);
            cluster_distances.push_back(distance);
        }

        const size_t num_points = cluster_angles.size();

        // Least squares circle fitting
        Eigen::MatrixXd A(num_points, 3);
        Eigen::VectorXd b(num_points);

        for (size_t i = 0; i < num_points; ++i) {
            double x = cluster_distances[i] * std::cos(cluster_angles[i]);
            double y = cluster_distances[i] * std::sin(cluster_angles[i]);

            A(i, 0) = 2 * x;
            A(i, 1) = 2 * y;
            A(i, 2) = -1;
            b(i) = x * x + y * y;
        }

        Eigen::Vector3d x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

        // Calculate circle parameters
        double cx = x(0);
        double cy = x(1);
        double radius = std::sqrt(cx * cx + cy * cy - x(2));

        // RANSAC parameters
        const int max_iterations = 200;
        const double distance_threshold = 0.01;
        const size_t min_inliers = 10;

        // RANSAC for circular fitting
        for (int iteration = 0; iteration < max_iterations; ++iteration) {
            // Randomly select three points
            size_t idx1 = rand() % num_points;
            size_t idx2 = rand() % num_points;
            size_t idx3 = rand() % num_points;

            // Ensure distinct indices
            if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3) {
                continue;
            }

            // Fit a circle to the selected points
            double x1 = cluster_distances[idx1] * std::cos(cluster_angles[idx1]);
            double y1 = cluster_distances[idx1] * std::sin(cluster_angles[idx1]);

            double x2 = cluster_distances[idx2] * std::cos(cluster_angles[idx2]);
            double y2 = cluster_distances[idx2] * std::sin(cluster_angles[idx2]);

            double x3 = cluster_distances[idx3] * std::cos(cluster_angles[idx3]);
            double y3 = cluster_distances[idx3] * std::sin(cluster_angles[idx3]);

            double ma = (y2 - y1) / (x2 - x1);
            double mb = (y3 - y2) / (x3 - x2);

            // Check for colinearity
            if (std::abs(ma - mb) < 1e-5) {
                continue;
            }

            // Calculate circle parameters
            cx = (ma * mb * (y1 - y3) + mb * (x1 + x2) - ma * (x2 + x3)) / (2 * (mb - ma));
            cy = -1 / ma * (cx - (x1 + x2) / 2) + (y1 + y2) / 2;
            radius = std::sqrt((x1 - cx) * (x1 - cx) + (y1 - cy) * (y1 - cy));

            // Count inliers
            size_t inliers = 0;
            for (size_t i = 0; i < num_points; ++i) {
                double distance_to_circle = std::abs(radius - std::sqrt((x1 - cx) * (x1 - cx) + (y1 - cy) * (y1 - cy)));
                if (distance_to_circle < distance_threshold) {
                    ++inliers;
                }
            }

            // Check if it's a valid circle
            if (inliers >= min_inliers) {
                return true;
            }
        }

        // If no valid circle is found after max_iterations, return false
        return false;
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
