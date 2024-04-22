#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <deque>

class LidarDenoising {
public:
    LidarDenoising() : nh_("~"), num_scans_to_average_(4), k_(5), eps_(0.08) {
        sub_ = nh_.subscribe("/front_scan", 1, &LidarDenoising::lidarCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/denoise_node", 1);
    }

    // Callback function for the input LaserScan data
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        sensor_msgs::LaserScan smoothed_scan = temporalSmoothing(*msg);
        sensor_msgs::LaserScan denoised_scan = denoiseScan(smoothed_scan);
        sensor_msgs::LaserScan processed_scan = preprocessScan(denoised_scan);
        pub_.publish(processed_scan);
    }

    // Preprocess raw scan data to remove outliers
    sensor_msgs::LaserScan preprocessScan(const sensor_msgs::LaserScan& scan) {
        sensor_msgs::LaserScan processed_scan = scan;

        // Remove any points with intensity equal to 0
        for (size_t i = 0; i < processed_scan.ranges.size(); ++i) {
            if (processed_scan.intensities[i] == 0) {
                processed_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
            }
        }

        return processed_scan;
    }
    
    // Temporal smoothing algorithm
    sensor_msgs::LaserScan temporalSmoothing(const sensor_msgs::LaserScan& scan) {
        sensor_msgs::LaserScan smoothed_scan = scan;

        // Apply temporal smoothing using moving average
        std::vector<float> sum(scan.ranges.size(), 0.0);
        for (size_t i = 0; i < std::min(scan_buffer_.size(), num_scans_to_average_); ++i) {
            const sensor_msgs::LaserScan& current_scan = scan_buffer_[i];
            for (size_t j = 0; j < current_scan.ranges.size(); ++j) {
                sum[j] += current_scan.ranges[j];
            }
        }

        // Compute average range values for each point
        for (size_t i = 0; i < smoothed_scan.ranges.size(); ++i) {
            smoothed_scan.ranges[i] = sum[i] / std::min(scan_buffer_.size(), num_scans_to_average_);
        }

        // Update scan buffer with latest scan
        scan_buffer_.push_back(scan);
        if (scan_buffer_.size() > num_scans_to_average_) {
            scan_buffer_.pop_front(); // Remove oldest scan
        }

        return smoothed_scan;
    }

    // Denoising algorithm using k-Nearest Neighbor
    sensor_msgs::LaserScan denoiseScan(const sensor_msgs::LaserScan& scan) {
        sensor_msgs::LaserScan denoised_scan = scan;

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
            float kth_distance = distances[k_ - 1];

            // If the distance to the kth nearest neighbor is greater than epsilon, mark the point as noise
            if (kth_distance > eps_) {
                denoised_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN(); // Set as NaN to represent noise
            }
        }

        return denoised_scan;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    size_t num_scans_to_average_;
    int k_; 
    float eps_; 
    std::deque<sensor_msgs::LaserScan> scan_buffer_; 
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_denoising_node");
    LidarDenoising lidar_denoising;
    ros::spin();
    return 0;
}
