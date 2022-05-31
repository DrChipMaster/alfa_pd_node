﻿//ROS includes
#include <ros/ros.h>

//PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

//ALFA Includes
#include "alfa_msg/AlfaConfigure.h"
#include "alfa_msg/AlfaMetrics.h"
#include "alfa_msg/AlfaAlivePing.h"

#define TIMER_SLEEP 50000 // Time between alive messages

#define CLOUD_TOPIC "alfa_pointcloud"  //Name of the subsriver topic where this node gets point clouds

#define DEBUG False

using namespace std;

class AlfaNode
{
public:
    /**
     * @brief Construct a new Alfa Node object
     * 
     * @param node_name Define the name of the node. This is used to define all the associeted topic names 
     * @param node_type Defines the node type. This is used to classify this node depending on its function.
     * @param default_configurations Default configurations to serve as a base line for configurations using the ALFA-Monitor.
     */
    AlfaNode(string node_name, string node_type, vector<alfa_msg::ConfigMessage> *default_configurations);

    /**
     * @brief Function that publishes a point cloud in a ROS Topic
     * 
     * @param input_cloud Point cloud that will be published in a ROS Topic
     */
    void publish_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);

    /**
     * @brief Function that published the collected metrics during execution in a ROS Topic
     * 
     * @param metrics The metrics that will be published in a ROS Topic
     */
    void publish_metrics(alfa_msg::AlfaMetrics &metrics);

    /**
     * @brief Function that is called when a new point cloud is avaliable in the ROS Topic. This needs to be implemented in the child class
     * 
     * @param input_cloud The point cloud that is recieved from the ROS Topic
     */
    virtual void process_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);

    /**
     * @brief Function that is called with a call to the configuration service is made
     * 
     * @param req The received configurations to be aplied
     * @return alfa_msg::AlfaConfigure::Response An int that represents the success of the configurations
     */
    virtual alfa_msg::AlfaConfigure::Response process_config(alfa_msg::AlfaConfigure::Request &req);

    /**
     * @brief Variable responsible for signaling the current state of the node. This status is sent with all the message in the Alive Message that is sent periodically
     * 
     */
    int node_status;
    virtual ~AlfaNode();

private:
    /**
     * @brief Callback of the subcrived point cloud topic. Every point cloud published will trigger a call to this function where the point cloud is converted to the
     *  pcl XYZI format
     */
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud); 

    /**
     * @brief Callback of the configuration service that this node class provides. It receives a set of configurations and sends the configuration result, if it was
     * successfully or not
     * 
     * @param req The configurations recieved
     * @param res The response sent back
     * @return true 
     * @return false 
     */
    bool parameters_cb(alfa_msg::AlfaConfigure::Request &req, alfa_msg::AlfaConfigure::Response &res);

    /**
     * @brief The point cloud subscriver
     * 
     */
    ros::Subscriber sub_cloud;

    /**
     * @brief the service server of the configurations
     * 
     */
    ros::ServiceServer sub_parameters;

    /**
     * @brief A ROS Nodehandler
     * 
     */
    ros::NodeHandle nh;

    /**
     * @brief Intern point cloud that is used in conversions
     * 
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcloud;

    /**
     * @brief Initializes the ROS environment of this node
     * 
     */
    void init();

    /**
     * @brief Function responsible for subscribing to all the topic needed to enable full communication with ALFA-Monitor
     * 
     */
    void subscribe_topics();

    /**
     * @brief Runs on another trhead, and is responsible for publishing the alive messages
     * 
     */
    void ticker_thread();

    /**
     * @brief thread that runs the ros "spin"
     * 
     */
    boost::thread *m_spin_thread;

    /**
     * @brief publisher of the node metrics
     * 
     */
    ros::Publisher node_metrics;

    /**
     * @brief publisher of the alive messages
     * 
     */
    ros::Publisher alive_publisher;

    /**
     * @brief publisher of the point cloud
     * 
     */
    ros::Publisher cloud_publisher;

    /**
     * @brief thread that runs the "ticker_thread"
     * 
     */
    boost::thread *alive_ticker;

    string node_name;
    string node_type;
    vector<alfa_msg::ConfigMessage> *default_configurations;
    void spin();
    uint pcl2_Header_seq;
};
