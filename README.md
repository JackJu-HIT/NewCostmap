# NewCostmap
这是一个基于ROS改的costmap地图，去除了ROS相关代码，在Linux 下使用C++对costmap代码进行构建编译。

# NewCostmap/costmap_2d/build

cd NewCostmap/costmap_2d/build
cmake ..
make


代码运行成功时，如下所示。

这是原来的地图:
![originalcharmap](originalcharmap.png)


这是膨胀后的地图:
![map_info_Rotate_char_map](map_info_Rotate_char_map.png)




