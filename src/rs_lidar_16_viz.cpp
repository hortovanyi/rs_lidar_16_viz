/**
 * this code is based on the copyright 2020 Robosense demo
 * https://github.com/RoboSense-LiDAR/rs_driver/blob/release/demo/demo.cpp
**/
#include <rs_driver/api/lidar_driver.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <deque>
#include <string>
#include <chrono>
#include <unistd.h>
#include "rs_lidar_16_viz.h"

using namespace robosense::lidar;


class LIDARViz {
    
public:
  LIDARViz(const RSDriverParam &param ) {


    param.print();
  
    auto f_exception_callback = std::bind(&LIDARViz::exceptionCallback, this, std::placeholders::_1);
    lidar_driver_.regExceptionCallback(f_exception_callback);

    auto f_pointcloud_callback = std::bind(&LIDARViz::pointCloudCallback, this, std::placeholders::_1);
    lidar_driver_.regPointRecvCallback(f_pointcloud_callback);

    if (!lidar_driver_.init(param))
    {
        std::cerr << "Lidar Driver Initialise Error ... exiting" << std::endl;
        exit (-1);
    }

    lidar_driver_.start();

    std::cout << "Initialising PCL visualizer ..." << std::endl;
    viewer_ = pcl::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("RS LIDAR 16 Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle);

  }
  ~LIDARViz() {
    lidar_driver_.stop();
  }

  void spinOnce() 
  {
    // if no point clouds queued just return
    if (point_cloud_msg_queue_.size() == 0)
      return;

    while (point_cloud_msg_queue_.size() > 0) {
      auto msg = std::move(point_cloud_msg_queue_[0]);
      point_cloud_msg_queue_.pop_front();
      std::cout << "spin msg seq: " << msg.seq << " point size: " << msg.pointcloud_ptr->size() << endl;
      
      // extract point cloud from rs_lidar to PCL point cloud
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

      point_cloud->header.stamp = msg.timestamp; 
      point_cloud->header.seq = msg.seq;
      point_cloud->header.frame_id = msg.frame_id;
      point_cloud->width = msg.width;
      point_cloud->height = msg.height;
      point_cloud->is_dense = msg.is_dense;

      size_t n_points = msg.pointcloud_ptr->size();
      point_cloud->points.resize(n_points);
      for (size_t i = 0; i < n_points; i++) {
          point_cloud->points.at(i) = msg.pointcloud_ptr->at(i);
      }

      // visualise point cloud
      viewer_->removeAllPointClouds();
      viewer_->removeAllShapes();
      renderPointCloud(point_cloud, msg.frame_id+std::to_string(msg.seq));
      viewer_->spinOnce();
    }
  }

private:
  std::deque<PointcloudMsg<pcl::PointXYZI>> point_cloud_msg_queue_;
  LidarDriver<pcl::PointXYZI> lidar_driver_;
  RSDriverParam lidar_param_;
  pcl::visualization::PCLVisualizer::Ptr viewer_;

  /**
   * @description: The point cloud callback function. This funciton will be registered to lidar driver.
   *              When the point cloud message is ready, driver can send out message through this function.
   * @param msg  The lidar point cloud message.
   */
  void pointCloudCallback(const PointcloudMsg<pcl::PointXYZI>& msg)
  {
    /* Note: Please do not put time-consuming operations in the callback function! */
    /* Make a copy of the message and process it in another thread is recommended*/
    std::cout << "msg: " << msg.seq << " pointcloud size: " << msg.pointcloud_ptr->size() << std::endl;
    point_cloud_msg_queue_.push_back(std::move(msg));
  }

  /**
   * @description: The exception callback function. This function will be registered to lidar driver.
   * @param code The error code struct.
   */
  void exceptionCallback(const Error& code)
  {
    /* Note: Please do not put time-consuming operations in the callback function! */
    /* Make a copy of the error message and process it in another thread is recommended*/
    std::cerr << "Error code : " << code.toString() << std::endl;
  }

  //setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
  void initCamera(CameraAngle setAngle)
  {
      viewer_->setBackgroundColor (0, 0, 0);
      
      // set camera position and angle
      viewer_->initCameraParameters();
      // distance away in meters
      int distance = 16;
      
      switch(setAngle)
      {
          case XY : viewer_->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
          case TopDown : viewer_->setCameraPosition(0, 0, distance, 1, 0, 1); break;
          case Side : viewer_->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
          case FPS : viewer_->setCameraPosition(-10, 0, 0, 0, 0, 1);
      }

      if(setAngle!=FPS)
          viewer_->addCoordinateSystem (1.0);
  } 

  void renderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color = Color(-1,-1,-1)){

    if(color.r==-1)
    {
      // Select color based off of cloud intensity
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud,"intensity");
        viewer_->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
    }
    else
    {
      // Select color based off input value
      viewer_->addPointCloud<pcl::PointXYZI> (cloud, name);
      viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    }

    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
  }

};

int main(int argc, char* argv[])
{
  std::cout << "\033[1m\033[35m"
            << "------------------------------------------------------" << std::endl;
  std::cout << "            RS_Driver Core Version: V " << RSLIDAR_VERSION_MAJOR << "." << RSLIDAR_VERSION_MINOR << "."
            << RSLIDAR_VERSION_PATCH << std::endl;
  std::cout << "\033[1m\033[35m"
            << "------------------------------------------------------"
            << "\033[0m" << std::endl;


  RSDriverParam param;                  ///< Creat a parameter object
  param.input_param.msop_port = 6699;   ///< Set the lidar msop port number the default 6699
  param.input_param.difop_port = 7788;  ///< Set the lidar difop port number the default 7788
  param.lidar_type = LidarType::RS16;   ///< Set the lidar type. Make sure this type is correct!

  auto lidarViz = LIDARViz(param);

  std::cout << "RoboSense Lidar-Driver Linux visualisation start......" << std::endl;
  while (true)
  {
    lidarViz.spinOnce();
    usleep(20);
  }

  return 0;
}