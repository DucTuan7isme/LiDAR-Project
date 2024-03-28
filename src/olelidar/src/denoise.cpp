#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <algorithm>

class LidarDenoising {
public:
    LidarDenoising() : nh_("~") {
        sub_ = nh_.subscribe("/front_scan", 1, &LidarDenoising::LidarCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/denoise_node", 1);
    }

    // Callback function for the input LaserScan data
    void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        sensor_msgs::LaserScan denoised_scan = denoiseScan(*msg);
        pub_.publish(denoised_scan);
    }

    // Denoising algorithm using k-Nearest Neighbor
    sensor_msgs::LaserScan denoiseScan(const sensor_msgs::LaserScan& scan) {
        sensor_msgs::LaserScan denoised_scan = scan;

        // Parameters for k-Nearest Neighbor
        int k = 10; // Number of nearest neighbors to consider

        // Precalculate angle increments
        float angle_increment = scan.angle_increment;
        std::vector<float> cos_values(scan.ranges.size());
        std::vector<float> sin_values(scan.ranges.size());
        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            float angle = scan.angle_min + i * angle_increment;
            cos_values[i] = cos(angle);
            sin_values[i] = sin(angle);
        }

        // Iterate over the scan points
        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            // Get the current point coordinates
            float x1 = scan.ranges[i] * cos_values[i];
            float y1 = scan.ranges[i] * sin_values[i];

            // Calculate distances to other points
            std::vector<float> distances;
            for (size_t j = 0; j < scan.ranges.size(); ++j) {
                if (i != j) {
                    float x2 = scan.ranges[j] * cos_values[j];
                    float y2 = scan.ranges[j] * sin_values[j];
                    float distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
                    distances.push_back(distance);
                }
            }

            // Sort distances and select k nearest neighbors
            std::sort(distances.begin(), distances.end());
            float kth_distance = distances[k - 1];

            // If the distance to the kth nearest neighbor is greater than epsilon, mark the point as noise
            if (kth_distance > eps) {
                denoised_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN(); // Set as NaN to represent noise
            }
        }

        return denoised_scan;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    float eps = 0.06; // Epsilon value for denoising
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_denoising_node");
    LidarDenoising lidar_denoising;
    ros::spin();
    return 0;
}
