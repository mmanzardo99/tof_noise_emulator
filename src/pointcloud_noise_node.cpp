// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/point_cloud2_iterator.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <random>
// #include <vector>
// #include <string>

// class PCNoiseNode {
// public:
//     PCNoiseNode(ros::NodeHandle& nh) : nh_(nh) {
//         nh_.param("noise_scale", noise_scale_, 0.01);  // sigma = noise_scale * range
//         nh_.getParam("input_topics", input_topics_);

//         if(input_topics_.empty()) {
//             ROS_ERROR("No input_topics provided!");
//             return;
//         }

//         std::random_device rd;
//         rng_.seed(rd());

//         for(const auto& topic : input_topics_) {
//             std::string out_topic = topic + "_noisy";

//             ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2>(
//                 topic, 1,
//                 boost::bind(&PCNoiseNode::callback, this, _1, topic)
//             );

//             ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(out_topic, 1);

//             subs_.push_back(sub);
//             pubs_[topic] = pub;

//             ROS_INFO_STREAM("Subscribing to: " << topic
//                             << " -> publishing noisy cloud to: " << out_topic);
//         }
//     }

// private:
//     ros::NodeHandle nh_;
//     std::vector<std::string> input_topics_;
//     std::vector<ros::Subscriber> subs_;
//     std::map<std::string, ros::Publisher> pubs_;
//     double noise_scale_;

//     std::mt19937 rng_;

//     void callback(const sensor_msgs::PointCloud2ConstPtr& msg, const std::string& in_topic)
//     {
//         sensor_msgs::PointCloud2 noisy_cloud = *msg;

//         sensor_msgs::PointCloud2Modifier modifier(noisy_cloud);

//         sensor_msgs::PointCloud2Iterator<float> it_x(noisy_cloud, "x");
//         sensor_msgs::PointCloud2Iterator<float> it_y(noisy_cloud, "y");
//         sensor_msgs::PointCloud2Iterator<float> it_z(noisy_cloud, "z");

//         for(; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
//         {
//             float x = *it_x;
//             float y = *it_y;
//             float z = *it_z;

//             float r = std::sqrt(x*x + y*y + z*z);
//             if(r < 1e-6) continue;

//             float sigma = noise_scale_ * r;

//             std::normal_distribution<float> dist(0.0, sigma);
//             float noise = dist(rng_);

//             float new_r = r + noise;
//             if(new_r < 1e-6) new_r = 1e-6;

//             float scale = new_r / r;

//             *it_x = x * scale;
//             *it_y = y * scale;
//             *it_z = z * scale;
//         }

//         noisy_cloud.header.stamp = ros::Time::now();
//         pubs_[in_topic].publish(noisy_cloud);
//     }
// };

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "pointcloud_noise_cpp");
//     ros::NodeHandle nh("~");

//     PCNoiseNode node(nh);

//     ros::spin();
//     return 0;
// }



#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <random>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>

class PCNoiseNode {
public:
    PCNoiseNode(ros::NodeHandle& nh) : nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_) {
        nh_.param("noise_scale", noise_scale_, 0.01);
        nh_.param("box_size_x", box_size_x_, 0.2);
        nh_.param("box_size_y", box_size_y_, 6.6);
        nh_.param("box_size_z", box_size_z_, 1.0);
        nh_.param<std::string>("tf_target_frame", tf_target_frame_, "face_shell_front");
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
    double box_size_x_, box_size_y_, box_size_z_;
    std::string tf_target_frame_;

    std::mt19937 rng_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Helper per trasformazione manuale
    Eigen::Matrix4f transformMsgToEigen(const geometry_msgs::TransformStamped& t)
    {
        Eigen::Quaternionf q(
            t.transform.rotation.w,
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z
        );
        Eigen::Vector3f trans(
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        );

        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
        mat.block<3,3>(0,0) = q.toRotationMatrix();
        mat.block<3,1>(0,3) = trans;

        return mat;
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg, const std::string& in_topic)
    {
        // Converti in PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Trasforma nel frame target
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        try {
            geometry_msgs::TransformStamped transformStamped = tf_buffer_.lookupTransform(
                tf_target_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));

            Eigen::Matrix4f tf = transformMsgToEigen(transformStamped);
            pcl::transformPointCloud(*cloud, *cloud_transformed, tf);
        } catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM("TF exception: " << ex.what());
            return;
        }

        // Filtra punti dentro la box
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setInputCloud(cloud_transformed);
        Eigen::Vector4f min_pt(-box_size_x_, -box_size_y_, -box_size_z_, 1.0);
        Eigen::Vector4f max_pt(box_size_x_, box_size_y_, box_size_z_, 1.0);
        crop.setMin(min_pt);
        crop.setMax(max_pt);
        crop.setNegative(true); // rimuovi punti dentro la box
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        crop.filter(*cloud_filtered);

        // Aggiungi rumore
        for(auto& pt : cloud_filtered->points)
        {
            float r = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
            if(r < 1e-6) continue;

            float sigma = noise_scale_ * r;
            std::normal_distribution<float> dist(0.0, sigma);
            float noise = dist(rng_);
            float new_r = r + noise;
            if(new_r < 1e-6) new_r = 1e-6;
            float scale = new_r / r;
            pt.x *= scale;
            pt.y *= scale;
            pt.z *= scale;
        }

        // Trasforma indietro nel frame originale
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        try {
            geometry_msgs::TransformStamped transformStamped = tf_buffer_.lookupTransform(
                msg->header.frame_id, tf_target_frame_, msg->header.stamp, ros::Duration(0.1));

            Eigen::Matrix4f tf = transformMsgToEigen(transformStamped);
            pcl::transformPointCloud(*cloud_filtered, *cloud_out, tf);
        } catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM("TF exception: " << ex.what());
            return;
        }

        // Pubblica
        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(*cloud_out, out_msg);
        out_msg.header.frame_id = msg->header.frame_id;
        out_msg.header.stamp = ros::Time::now();
        pubs_[in_topic].publish(out_msg);
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
