#include "alfa_node.h"
#include <thread>
#include <unistd.h>
#include <chrono>


AlfaNode::AlfaNode(string node_name,string node_type,vector<alfa_msg::ConfigMessage>* default_configurations )
{
    this->node_name = node_name;
    this->node_type = node_type;
    this->default_configurations = default_configurations;
    pcl2_Header_seq = 0;
    pcloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    subscribe_topics();
    alive_ticker = new boost::thread(&AlfaNode::ticker_thread,this);

}

void AlfaNode::publish_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud)
{
    sensor_msgs::PointCloud2 pcl2_frame;
    pcl::toROSMsg(*output_cloud,pcl2_frame);
    pcl2_frame.header.frame_id = node_name+"_pointcloud";
    pcl2_frame.header.seq = pcl2_Header_seq;
    pcl2_frame.header.stamp = ros::Time::now();
    pcl2_Header_seq++;
    cloud_publisher.publish(pcl2_frame);
}

void AlfaNode::publish_metrics(alfa_msg::AlfaMetrics &metrics)
{
    filter_metrics.publish(metrics);
}

void AlfaNode::process_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud)
{
    cout << "Please implement the process_pointcloud function"<<endl;
}

alfa_msg::AlfaConfigure::Response AlfaNode::process_config(alfa_msg::AlfaConfigure::Request &req)
{
    cout << "Please implement the process_config function"<<endl;

}

AlfaNode::~AlfaNode()
{

}

void AlfaNode::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    //cout<<"Recieved pointcloud"<<endl;
    if ((cloud->width * cloud->height) == 0)
    {
        cout <<"Recieved empty point cloud"<<endl;
        return;
    }
    pcl::fromROSMsg(*cloud,*pcloud);

    process_pointcloud(pcloud);

}

bool AlfaNode::parameters_cb(alfa_msg::AlfaConfigure::Request &req, alfa_msg::AlfaConfigure::Response &res)
{
    cout<<"Recieved FilterSettings with size" <<req.configurations.size()<<"... Updating"<<endl;
    for (int i=0; i< req.configurations.size();i++) {
        cout <<"Configuration: "<<i<< " With name: "<< req.configurations[i].config_name<< " with value: "<< req.configurations[i].config<<endl;
    }

    res = process_config(req);
    return true;
}

void AlfaNode::init()
{
        char arg0[]= "filter_node";
        char *argv[]={arg0,NULL};
        int argc=(int)(sizeof(argv) / sizeof(char*)) - 1;;
        ros::init (argc, argv, node_name);
          if (!ros::master::check()) {
              cout <<"Failed to inicialize ros"<<endl;
            return;
          }

}

void AlfaNode::subscribe_topics()
{
    sub_cloud = nh.subscribe("alfa_pointcloud",1,&AlfaNode::cloud_cb,this);
    sub_parameters = nh.advertiseService(node_name.append("_settings"),&AlfaNode::parameters_cb,this);
    ros::NodeHandle n;
    filter_metrics = n.advertise<alfa_msg::AlfaMetrics>(node_name.append("_metrics"), 1);
    alive_publisher = n.advertise<alfa_msg::AlfaAlivePing>(node_name.append("_alive"),1);
    cloud_publisher = n.advertise<sensor_msgs::PointCloud2>(node_name.append("_cloud"),1);
    m_spin_thread = new boost::thread(&AlfaNode::spin, this);


}

void AlfaNode::ticker_thread()
{
    while(ros::ok())
    {
        alfa_msg::AlfaAlivePing newPing;
        newPing.node_name= node_name;
        newPing.node_type = node_type;
        newPing.config_service_name = node_name+"_settings";
        newPing.config_tag = "Default configuration";
        newPing.default_configurations = *default_configurations;
        newPing.current_status = node_status;
        alive_publisher.publish(newPing);
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMER_SLEEP));
    }
}

void AlfaNode::spin()
{
    cout<<"started spinning with success"<<endl;
    ros::spin();
}
