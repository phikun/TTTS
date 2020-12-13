/******
 * 上学时间制图：主函数在此
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.03
 ******/

#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <ctime>
#include <gdal_priv.h>
#include <opencv2/opencv.hpp>

#include "model.hpp"
#include "utility.hpp"
#include "index.hpp"
#include "insert_school.hpp"
#include "dijkstra.hpp"
#include "travel_time.hpp"

using namespace std;

int main()
{
	ios_base::sync_with_stdio(false);
	cin.tie(nullptr); cout.tie(nullptr);

	GDALAllRegister();

	const string school_file = "C:/Users/zgcyx/Desktop/upload/School-WGS84.shp";
	const string roadFile = "C:/Users/zgcyx/Desktop/upload/Road-WGS84-Speed.shp";
	const string speed_field_name = "speed";
	const string population_file = "C:/Users/zgcyx/Desktop/upload/Beijing-RoadMask-ArcGIS.tif";
	const string walk_speed_file = "C:/Users/zgcyx/Desktop/upload/Beijing-WalkSpeed.tif";
	const string output_file = "C:/Users/zgcyx/Desktop/upload/output-Pack-RTree.tif";

	cout << "Hello World!" << endl;
	const auto overall_start_time = clock();
	
	// 构建顶点表
	const auto pair1 = ttts::build_vertex_table(roadFile);
	const auto index2vertex = pair1->first;
	const auto vertex2index = pair1->second;

	// 构建边表
	const auto pair2 = ttts::build_edge_table(roadFile, speed_field_name, vertex2index);
	const auto index2edge = pair2->first;
	const auto edge2index = pair2->second;
	
	// 插入学校点，此函数会修改点表边表的4个索引，【学校点的ID号可能有问题，出现奇怪结果后再去检查！】
	// 【edge2index那里有重边，先不管它，最后扫描3次路网文件消除重边】
	const auto insert_school_start_time = clock();
	const auto school_vertex_index = ttts::strategy::insert_school::solve<ttts::model::point_g>(school_file, index2edge, edge2index, index2vertex, vertex2index);
	const auto insert_school_end_time = clock();
	cout << "Insert School Time: " << fixed << setprecision(3) << (insert_school_end_time - insert_school_start_time) / 1000.0 << endl;
	
	// Dijkstra
	const auto dijkstra_start_time = clock();
	const auto vertex_time = ttts::strategy::dijkstra::solve<ttts::model::point_g>(index2vertex, index2edge, school_vertex_index);
	const auto dijkstra_end_time = clock();
	cout << "Dijkstra Time: " << fixed << setprecision(3) << (dijkstra_end_time - dijkstra_start_time) / 1000.0 << endl;

	// 读tif、计算栅格中心点经纬度、制图
	const auto population_raster = ttts::read_population_raster<ttts::model::point_g>(population_file);	
	const auto walk_speed_raster = ttts::read_walk_speed_raster(walk_speed_file);

	cout << "After read" << endl;
	
	// 对calculate_vector算travel time，直接返回结果矩阵
	const auto travel_time_start_time = clock();
	const auto mat = ttts::strategy::travel_time::solve<ttts::model::point_g>(population_raster, walk_speed_raster, index2vertex, index2edge, vertex_time);
	const auto travel_time_end_time = clock();
	cout << "Travel Time Time: " << fixed << setprecision(3) << (travel_time_end_time - travel_time_start_time) / 1000.0 << endl;
	
	// 将结果保存成GeoTiff
	ttts::write_result_to_geotiff(output_file, mat, population_raster);
	
	delete mat;
	delete walk_speed_raster;
	delete population_raster;
	
	delete vertex_time;
	delete school_vertex_index;
	delete edge2index;
	delete index2edge;
	delete index2vertex;
	delete vertex2index;
	delete pair2;
	delete pair1;

	const auto overall_end_time = clock();
	cout << "Overall Time: " << fixed << setprecision(3) << (overall_end_time - overall_start_time) / 1000.0 << endl;

	system("PAUSE");
	
	return 0;
}
