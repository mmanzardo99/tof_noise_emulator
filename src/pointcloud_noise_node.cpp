#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <vector>
#include <string>

class PCNoiseNode {
public:
    PCNoiseNode(ros::NodeHandle& nh) : nh_(nh) {
        nh_.param("noise_scale", noise_scale_, 0.01);  // sigma = noise_scale * range
        nh_.getParam("input_topics", input_topics_);

        if(input_topics_.empty()) {
            ROS_ERROR("No input_topics provided!");
            return;
        }

        std::random_device rd;
        rng_.seed(rd());

        for(const auto& topic : input_topics_) {
            std::string out_topic = topic + "_noisy";

            ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2>(
                topic, 1,
                boost::bind(&PCNoiseNode::callback, this, _1, topic)
            );

            ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(out_topic, 1);

            subs_.push_back(sub);
            pubs_[topic] = pub;

            ROS_INFO_STREAM("Subscribing to: " << topic
                            << " -> publishing noisy cloud to: " << out_topic);
        }
    }

private:
    ros::NodeHandle nh_;
    std::vector<std::string> input_topics_;
    std::vector<ros::Subscriber> subs_;
    std::map<std::string, ros::Publisher> pubs_;
    double noise_scale_;

    std::mt19937 rng_;

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg, const std::string& in_topic)
    {
        sensor_msgs::PointCloud2 noisy_cloud = *msg;

        sensor_msgs::PointCloud2Modifier modifier(noisy_cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");

        sensor_msgs::PointCloud2Iterator<float> it_x(noisy_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(noisy_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(noisy_cloud, "z");

        for(; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
        {
            float x = *it_x;
            float y = *it_y;
            float z = *it_z;

            float r = std::sqrt(x*x + y*y + z*z);
            if(r < 1e-6) continue;

            float sigma = noise_scale_ * r;

            std::normal_distribution<float> dist(0.0, sigma);
            float noise = dist(rng_);

            float new_r = r + noise;
            if(new_r < 1e-6) new_r = 1e-6;

            float scale = new_r / r;

            *it_x = x * scale;
            *it_y = y * scale;
            *it_z = z * scale;
        }

        noisy_cloud.header.stamp = ros::Time::now();
        pubs_[in_topic].publish(noisy_cloud);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_noise_cpp");
    ros::NodeHandle nh("~");

    PCNoiseNode node(nh);

    ros::spin();
    return 0;
}
