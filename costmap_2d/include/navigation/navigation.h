
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include<iostream>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
using namespace std;

namespace navigation_system
{




   class navigation
   {
    public:
        navigation(costmap_2d::Costmap2DROS* planner_costmap_ros);
        bool makePlan();
        void start();

    public:
       costmap_2d::Costmap2DROS* planner_costmap_ros_;
       unsigned char* costs_; 
       costmap_2d::Costmap2D* global_cost_map_;
         boost::recursive_mutex planner_mutex_;

   };
 




}







#endif  // NAVIGATION_H


