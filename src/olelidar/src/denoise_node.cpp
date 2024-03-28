#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>

class LidarDenoising {
public:
    LidarDenoising() : nh_("~") {
        sub_ = nh_.subscribe("/front_scan", 1, &LidarDenoising::LidarCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/denoising_node", 1);
    }

    // Callback function for the input LaserScan data
    void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        sensor_msgs::LaserScan denoised_scan = denoiseScan(*msg);
        pub_.publish(denoised_scan);
    }

    // Denoising algorithm using DBSCAN
    sensor_msgs::LaserScan denoiseScan(const sensor_msgs::LaserScan& scan) {
        sensor_msgs::LaserScan denoised_scan = scan;

        // Parameters for DBSCAN
        float eps = 0.02; 
        int min_pts = 10; 
        // Initialize vectors
        visited_.assign(scan.ranges.size(), false);
        labels_.assign(scan.ranges.size(), -1);
        clusters_.clear();

        // Get the range values
        const std::vector<float>& ranges = scan.ranges;

        // Iterate over the scan points
        for (size_t i = 0; i < ranges.size(); ++i) {
            // Check if the point has already been visited
            if (visited_[i])
                continue;

            visited_[i] = true;

            // Get the current point range
            float range = ranges[i];

            // Find neighbors within eps distance using Euclidean distance
            std::vector<int> neighbors = findNeighbors(scan, i, eps);

            // Check if the point is a core point
            if (neighbors.size() < min_pts){
                // Mark the point as noise
                labels_[i] = -1; 
            }
            
            // Expand the cluster
            int cluster_id = clusters_.size();
            expandCluster(scan, i, neighbors, cluster_id, eps, min_pts);

            // Denoise the points in the cluster
            for (size_t j = 0; j < clusters_[cluster_id].size(); ++j) {
                int idx = clusters_[cluster_id][j];
                denoised_scan.ranges[idx] = range;
            }
        }


        // Filter out the noise points and create a denoised LaserScan message
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (labels_[i] != -1) {
                denoised_scan.ranges[i] = ranges[i]; // Keep non-noise points
            } else {
                denoised_scan.ranges[i] = std::numeric_limits<float>::infinity(); // Set noise points to infinity
            }
        }
        
        return denoised_scan;
    }

    // Find neighbors of a point within eps distance (using Euclidean distance)
    std::vector<int> findNeighbors(const sensor_msgs::LaserScan& scan, int idx, float eps) {
        std::vector<int> neighbors;

        const std::vector<float>& ranges = scan.ranges;
        const float& angle_increment = scan.angle_increment;
        const float& current_range = ranges[idx];

        for (size_t i = 0; i < ranges.size(); ++i) {
            if (i != idx) {
                // Calculate the angular distance between points
                float angle_distance = fabs((int)i - idx) * angle_increment;
                
                // Calculate the Euclidean distance
                float distance = sqrt(pow(current_range, 2) + pow(ranges[i], 2) - 2 * current_range * ranges[i] * cos(angle_distance));
                
                // Check if the distance is within epsilon
                if (distance < eps) {
                    neighbors.push_back(i);
                }
            }
        }

        return neighbors;
    }

    // Expand the cluster starting from a core point
    void expandCluster(const sensor_msgs::LaserScan& scan, int idx, std::vector<int>& neighbors, int cluster_id, float eps, int min_pts) {
        std::vector<int> cluster;
        cluster.push_back(idx);

        const std::vector<float>& ranges = scan.ranges;
        const float& angle_increment = scan.angle_increment;

        for (size_t i = 0; i < neighbors.size(); ++i) {
            int neighbor_idx = neighbors[i];

            if (!visited_[neighbor_idx]) {
                visited_[neighbor_idx] = true;
                std::vector<int> new_neighbors = findNeighbors(scan, neighbor_idx, eps);

                if (new_neighbors.size() >= min_pts) {
                    neighbors.insert(neighbors.end(), new_neighbors.begin(), new_neighbors.end());
                }
            }

            if (labels_[neighbor_idx] == -1) {
                labels_[neighbor_idx] = cluster_id;
                cluster.push_back(neighbor_idx);
            }
        }

        clusters_.push_back(cluster);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::vector<bool> visited_;
    std::vector<int> labels_;
    std::vector<std::vector<int>> clusters_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_denoise_node");

    LidarDenoising lidar_denoising;

    ros::spin();

    return 0;
}
