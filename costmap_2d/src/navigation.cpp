#include <navigation/navigation.h>
namespace navigation_system
{

   navigation::navigation(costmap_2d::Costmap2DROS* planner_costmap_ros):planner_costmap_ros_(planner_costmap_ros){}
   
   void navigation::start()
   {
            {
           
               boost::thread *navigation_thread = new boost::thread(&navigation::makePlan,this);
               //lock.unlock();
               navigation_thread->join();
            }
            //boost::thread navigation_thread(&navigation::makePlan,this);
   }

   bool navigation::makePlan()
   {
      //while(1)
      {
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
         global_cost_map_ = planner_costmap_ros_->getCostmap();
         costs_           = global_cost_map_->getCharMap(); 
         //boost::unique_lock<Costmap2D::mutex_t> lock(*(global_cost_map_->getMutex()));
         //std::cout << "global_cost_map_->getSizeInCellsX() =" << global_cost_map_->getSizeInCellsX() << std::endl;
         //std::cout << "global_cost_map_->getSizeInCellsY() =" << global_cost_map_->getSizeInCellsY() << std::endl;
         int leng = 0;
         //std::cout << "charmap:"<<std::endl;
        // std::cout << " (global_cost_map_->getSizeInCellsX()) = " <<  (global_cost_map_->getSizeInCellsX()) << std::endl;
       //  std::cout << "global_cost_map_->getSizeInCellsY() = " << global_cost_map_->getSizeInCellsY() << std::endl;
        // std::cout << "global_cost_map_ ->getOriginX() = "  << global_cost_map_ ->getOriginX() << std::endl;
        // std::cout << "global_cost_map_ ->getOriginY() = "  << global_cost_map_ ->getOriginY() << std::endl;
         //for(int i = 0; i < (global_cost_map_->getSizeInCellsX()) * (global_cost_map_->getSizeInCellsY());i++)
        // {    //std::cout<<"printf enter....";
             //std::cout<<static_cast<int>(costs_[i]) << " ";
            // leng++;
        // }

        #if 1
       
        cv::Mat m4 = cv::imread("/home/juchunyu/20231013/globalPlanner/AStar-ROS/map/map.pgm",cv::IMREAD_GRAYSCALE);
        //cv::Mat m4 = cv::imread("/home/juchunyu/20231013/globalPlanner/PM.pgm",cv::IMREAD_GRAYSCALE);
        cv::imshow("originalcharmap",m4);
        cv::Mat map_info(global_cost_map_->getSizeInCellsX(), global_cost_map_->getSizeInCellsY(), CV_8UC1);//initlize
        cv::Mat  map_info_Rotate(global_cost_map_->getSizeInCellsX(), global_cost_map_->getSizeInCellsY(), CV_8UC1);//initlize

        for(int i = (global_cost_map_->getSizeInCellsX())*(global_cost_map_->getSizeInCellsY()) - 1; i > 0; i--)
        {
            int x = i%global_cost_map_->getSizeInCellsX();  //还原为像素坐标
            int y = i/global_cost_map_->getSizeInCellsX();  //还原为像素坐标
            //unsigned char value = 255;
            map_info.at<unsigned char>(x, y) = 254 - costs_[i];
            //cout<<endl;
        }   

         int l = 0;int k = global_cost_map_->getSizeInCellsY();
         for(int j = 0,k =  global_cost_map_->getSizeInCellsY(); j < global_cost_map_->getSizeInCellsX(); j++,k--){
                  for(int i = 0,l = 0; i < global_cost_map_->getSizeInCellsX();i++,l++){
                  map_info_Rotate.at<unsigned char>(l,k) = map_info.at<unsigned char>(i, j);
            }
         }
         cv::imshow("map_info_char_map", map_info);
         cv::rotate(map_info, map_info_Rotate,cv::ROTATE_90_COUNTERCLOCKWISE);// cv::ROTATE_90_CLOCKWISE);
         cv::imshow("map_info_Rotate_char_map", map_info_Rotate);
         //boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        //	cv::imshow("map_info", map_info.t());
         cv::waitKey(0.5);
         #endif
        // std::cout << "length" << leng << std::endl; 
         
         //std::cout << "map length = " << leng << std::endl;
         //std::cout<<std::endl;
      }
   }
   

}