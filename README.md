## Global Map of Travel Time to School

### 样例数据

在`sample-data`文件夹下，包括了加拿大爱德华王子岛的Open Street Map道路、学校点、矢量边界、DEM、由DEM计算出来的坡度、坡度重分类后的行走速度，及MSVC++和G++版本的计算结果。所有矢量数据在`shp`子文件夹下。

### 数据预处理

在`pretreat`文件夹中，包括：

- 道路速度赋值：`assign_road_speed.py`，将按`OSM-RoadSpeed.xlsx`给定道路速度；
- 按行政区边界做人口掩膜：`pop_mask.py`；
- 合并SRTM雷达高程、重采样到与人口栅格一致，并计算坡度：`dem_pipeline.py`；
- 由坡度重分类到栅格内的行走速度：`slope-resample.py`；
- 按行政区边界裁剪计算结果，并进行压缩：`clip_and_zip.py`。

由于SRTM雷达高程不在样例数据中，不能直接运行`dem_pipeline.py`，但在样例数据中给出了加拿大爱德华王子岛的DEM和坡度。

### MSVC++版本

在`MSVC++ Version`文件夹下，内含一个名为`TTTS`的Microsoft<sup>®</sup> Visual Studio解决方案。注意修改项目的包含目录和库目录，把`boost`和`GDAL`的相应目录添加进去。

### GNU G++版本

编译语句如下：

`g++ main.cxx -std=c++17 -fopenmp -I <Your GDAL include path> -L <Your GDAL lib path> -lgdal -I <Yout boost include path> -o main.out`

