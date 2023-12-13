/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/static_layer.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/inflation_layer.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>
#include <boost/chrono.hpp>
//#include  <boost/config.hpp>
//#include  <boost/shared_ptr.hpp>
//#include  <boost/shared_ptr.hpp>
//using  boost::make_shared;

using namespace std;

namespace costmap_2d
{

/*
void move_parameter(ros::NodeHandle& old_h, ros::NodeHandle& new_h, std::string name, bool should_delete = true)
{
  if (!old_h.hasParam(name))
    return;

  XmlRpc::XmlRpcValue value;
  old_h.getParam(name, value);
  new_h.setParam(name, value);
  if (should_delete) old_h.deleteParam(name);
}*/

Costmap2DROS::Costmap2DROS(std::string name) :
    layered_costmap_(NULL), name_(name),stop_updates_(false), initialized_(true), stopped_(false),
    robot_stopped_(false), map_update_thread_(NULL)// last_publish_(0)
{

  // check if we want a rolling window version of the costmap
  bool rolling_window, track_unknown_space, always_send_full_costmap;
 
  track_unknown_space = false;  //参数描述：用于设置是否将未知空间视为空闲空间。如果为false，则会将未知空间视为空闲空间；否则视为未知空间。
  always_send_full_costmap = false;
  std::cout << "costmap2DROS Init..."<<std::endl;

  if(name == "global_costmap") 
  {
      rolling_window = false;
      layered_costmap_ = new LayeredCostmap(rolling_window, track_unknown_space);
      
      //初始化静态地图层           
      //costmap_2d::StaticLayer  boost::make_shared<costmap_2d::StaticLayer>();
      //boost::shared_ptr<Layer> pluginStatic = boost::make_shared<costmap_2d::StaticLayer>();//     new costmap_2d::StaticLayer();
      std::cout << "costmap2DROS Init2..."<<std::endl;
      boost::shared_ptr<Layer> pluginStatic(new costmap_2d::StaticLayer());//boost::make_shared<costmap_2d::StaticLayer>();
      //boost::shared_ptr<costmap_2d::StaticLayer> pluginStatic = boost::make_shared<costmap_2d::StaticLayer>();
      std::cout << "costmap2DROS Init2.5.."<<std::endl;
      layered_costmap_->addPlugin(pluginStatic);
      //std::cout <<  "costmap2DROS Init2.6.." << std::endl;
      
      // oninitialize方法（Layer类中的虚函数，被定义在各层地图中）
      pluginStatic->initialize(layered_costmap_, name + "/" + "static_layer");
      std::cout << "costmap2DROS Init3.."<<std::endl;

      //初始化膨胀层
     boost::shared_ptr<Layer> pluginInflation(new costmap_2d::InflationLayer());//=  boost::make_shared<costmap_2d::InflationLayer>();// new InflationLayer();
     layered_costmap_->addPlugin(pluginInflation);
     pluginInflation->initialize(layered_costmap_, name + "/" + "inflation_layer");

  } 

  if(name == "local_costmap"){
      rolling_window = true;
      layered_costmap_ = new LayeredCostmap(rolling_window, track_unknown_space);
      //std::cout << pluginStatic->getName()<< std::endl;

      //初始化障碍物层
      boost::shared_ptr<Layer> pluginObstacle(new costmap_2d::ObstacleLayer()); //boost::make_shared<costmap_2d::ObstacleLayer>(); //new ObstacleLayer();
      layered_costmap_->addPlugin(pluginObstacle);
      pluginObstacle->initialize(layered_costmap_, name + "/" + "obstacle_layer");

      //初始化膨胀层
      boost::shared_ptr<Layer> pluginInflation(new costmap_2d::InflationLayer());//=  boost::make_shared<costmap_2d::InflationLayer>();// new InflationLayer();
      layered_costmap_->addPlugin(pluginInflation);
      pluginInflation->initialize(layered_costmap_, name + "/" + "inflation_layer");
  }

  //todo:RObot footprint
  geometry_msgs::Polygon footprint;
  
  geometry_msgs::Point32 pointvalue;
  
  pointvalue.x = -1;
  pointvalue.y = 1;
  pointvalue.z = 0;
  footprint.points.push_back(pointvalue);
   
  pointvalue.x = 1;
  pointvalue.y = 1;
  pointvalue.z = 0;
  footprint.points.push_back(pointvalue);

  pointvalue.x = 1;
  pointvalue.y =-1;
  pointvalue.z = 0;
  footprint.points.push_back(pointvalue);

  pointvalue.x = -1;
  pointvalue.y =-1;
  pointvalue.z = 0;
  footprint.points.push_back(pointvalue);

  Costmap2DROS::setUnpaddedRobotFootprintPolygon(footprint);

  // create a thread to handle updating the map
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;

  robot_stopped_ = false;

  //debug 
  Costmap2DROS::reconfigureCB();
 

}

void Costmap2DROS::setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint)
{
  setUnpaddedRobotFootprint(toPointVector(footprint));
}

Costmap2DROS::~Costmap2DROS()
{
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_->join();
    delete map_update_thread_;
  }

  delete layered_costmap_;
}

void Costmap2DROS::reconfigureCB()
{

  if (map_update_thread_ != NULL)
  { 
    std::cout<<"map_update_thread_ not null"<<std::endl;
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_frequency = 5;//config.update_frequency;

  //toDo:
  //find size parameters  
  double map_width_meters = 0.2, map_height_meters = 0.2, resolution = 0.05, origin_x =
            0,
         origin_y = 0;
  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }
   
  //std::cout<<"test"<<std::endl;
  map_update_thread_ = new boost::thread(boost::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
  //std::cout<<"test3"<<std::endl;

}


void Costmap2DROS::setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  padFootprint(padded_footprint_, footprint_padding_);

  layered_costmap_->setFootprint(padded_footprint_);
}

void Costmap2DROS::mapUpdateLoop(double frequency)
{
  //mapUpdateLoop这个成员函数是用于不停刷新代价地图的，它在每个循环通过updateMap函数实现
  //，而Costmap2DROS::updateMap函数是利用LayeredCostmap的updateMap函数进行更新，在前面已经分析过。
  if (frequency == 0.0)
    return;

  int i = 0;
  //std::cout<<"map_update_thread_shutdown_"<<map_update_thread_shutdown_<<std::endl;
  while (!map_update_thread_shutdown_)
  { 
    i++;
    if(i % 2 == 0)
      continue;
    struct timeval start, end;
    double start_t, end_t, t_diff;
    //gettimeofday(&start, NULL);
    //std::cout<<i<<std::endl;

    updateMap();
  
    //std::cout<<"mapUpdateLoop:Enter..."<<std::endl;
    //std::cout<<"i="<<i<<std::endl;
    gettimeofday(&end, NULL);
    //start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    /// end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    //t_diff = end_t - start_t;
    //std::cout << "Map update time:" << t_diff << std::endl;

    if (layered_costmap_->isInitialized())
    {
      unsigned int x0, y0, xn, yn;
      layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
    }
   
  }
}

void Costmap2DROS::updateMap()
{
  //std::cout<<"stop_updates_:"<<stop_updates_<<std::endl;
  if (!stop_updates_)
  {
     //todo:
     
     double robot_x = 0;
     double robot_y = 0;
     double robot_yaw = 0;
     double x = robot_x, y = robot_y, yaw = robot_yaw;

    // std::cout << "Please input robot pose:robot_x,robot_y" << std::endl;
    // std::cin >> robot_x;
    // std::cin >> robot_y;
     //std::cout << "Costmap2DROS::updateMap()"<<std::endl;
      //调用layered_costmap_的updateMap函数，参数是机器人位姿
     layered_costmap_->updateMap(x, y, yaw);

     geometry_msgs::PolygonStamped footprint;
      //footprint.header.frame_id = global_frame_;
      //footprint.header.stamp = ros::Time::now();
      //transformFootprint(x, y, yaw, padded_footprint_, footprint);//make coordiates to global coordiates
      //footprint_pub_.publish(footprint);
      transformFootprint(x, y, yaw, padded_footprint_, footprint);//make coordiates to global coordiates
      initialized_ = true;
   
  }
}

void Costmap2DROS::start()
{
}

void Costmap2DROS::stop()
{
  stop_updates_ = true;
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // unsubscribe from topics
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void Costmap2DROS::resume()
{
  /*
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  ros::Rate r(100.0);
  while (!initialized_)
    r.sleep();
    */
}


void Costmap2DROS::resetLayers()
{
  Costmap2D* top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->reset();
  }
}

}  // namespace costmap_2d
