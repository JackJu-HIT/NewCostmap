#include<iostream>
//#include <costmap_2d/costmap_2d_ros.h>
//#include <costmap_2d/costmap_2d.h>
#include <navigation/navigation.h>
//using namespace std;

int main(){

    // 下面通过模块costmap_2d构建了全局代价地图对象，并通过其成员函数pause()暂停运行，将在必要的功能模块初始化结束之后通过成员接口start()开启。
   costmap_2d::Costmap2DROS* planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap");
   navigation_system::navigation *navigation_  = new navigation_system::navigation(planner_costmap_ros_);
   navigation_->start();

    //costmap_2d::Costmap2DROS* local_planner_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap");
   // navigation_system::navigation *local_navigation_  = new navigation_system::navigation(local_planner_costmap_ros_);
   // local_navigation_->start();
    //planner_costmap_ros_->start();
    //costmap_2d::Costmap2DROS::reconfigureCB();
    //costmap_2d::Costmap2D* global_cost_map = planner_costmap_ros_->getCostmap();
    // unsigned char* costs =  global_cost_map->getCharMap();
    
    //std::cout<<"main"<<std::endl;
    //getCostmap();
    /*
   for(int i = 0; i < (global_cost_map->getSizeInCellsX()) * (global_cost_map->getSizeInCellsY()) ;i++) //* (global_cost_map->getSizeInCellsY())
   {
      ;
   }*/
   // cout<<  static_cast<int>(costs[i]) << " ";
//cout<<  static_cast<int>(costs[0]);
   // cout<<endl;
}




