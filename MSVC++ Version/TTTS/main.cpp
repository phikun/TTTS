/******
 * 上学时间制图：主函数在此
 * 2020.12.16更新：使用Linux风格程序计时，将输入输出文件改成main作为main函数参数传入
 *                把程序计时放到Python调用中再说
 * 2020.01.16更新：最后Dijkstra的时候直接把结果存到外存里
 * 2021.02.05更新：手工实现矩阵类，而非使用cv::Mat_，并重载到()运算符
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.16
 ******/

#include <iostream>
#include <string>
#include <cstring>
#include <utility>
#include <vector>
#include <gdal_priv.h>

#include "global_define.hpp"
#include "model.hpp"
#include "utility.hpp"
#include "index.hpp"
#include "insert_school.hpp"
#include "dijkstra.hpp"
#include "travel_time.hpp"

#pragma warning(disable:4996)

#define int long long

using namespace std;

// 默认的输入输出文件
string school_file = "../../sample-data/shp/OSM-School-WGS84.shp";
string road_file = "../../sample-data/shp/Road-WGS84-Speed.shp";
string speed_field_name = "speed";
string population_file = "../../sample-data/Population-Mask.tif";
string walk_speed_file = "../../sample-data/Walk-Speed.tif";
string output_file = "../../sample-data/Output-MSVC.tif";

void parse_input_params(signed argc, char* argv[])
{
	for (auto i = 1; i < argc; i += 2)
	{
		if (strcmp(argv[i], "-school") == 0 || strcmp(argv[i], "-s") == 0)
			school_file = string(argv[i + 1]);
		if (strcmp(argv[i], "-road") == 0 || strcmp(argv[i], "-r") == 0)
			road_file = string(argv[i + 1]);
		if (strcmp(argv[i], "-population") == 0 || strcmp(argv[i], "-pop") == 0 || strcmp(argv[i], "-p") == 0)
			population_file = string(argv[i + 1]);
		if (strcmp(argv[i], "-dem") == 0 || strcmp(argv[i], "-ws") == 0)
			walk_speed_file = string(argv[i + 1]);
		if (strcmp(argv[i], "-output") == 0 || strcmp(argv[i], "-o") == 0)
			output_file = string(argv[i + 1]);
		if (strcmp(argv[i], "-speed") == 0)
			speed_field_name = string(argv[i + 1]);
	}
}

signed main(signed argc, char* argv[])
{
	parse_input_params(argc, argv);
	ios_base::sync_with_stdio(false);
	cin.tie(nullptr); cout.tie(nullptr);

	GDALAllRegister();

	cout << "Hello World!" << endl;

	// 构建顶点表
	const auto pair1 = ttts::build_vertex_table(road_file);
	const auto index2vertex = pair1->first;
	const auto vertex2index = pair1->second;

	cout << "After build vertexTable" << endl;

	// 构建边表
	const auto pair2 = ttts::build_edge_table(road_file, speed_field_name, vertex2index);
	const auto index2edge = pair2->first;
	const auto edge2index = pair2->second;

	// 插入学校点，此函数会修改点表边表的4个索引，【学校点的ID号可能有问题，出现奇怪结果后再去检查！】
	// 【edge2index那里有重边，先不管它，最后扫描3次路网文件消除重边】
	const auto school_vertex_index = ttts::strategy::insert_school::solve<ttts::model::point_g>(school_file, index2edge, edge2index, index2vertex, vertex2index);
	delete edge2index;  // 2021.01.11修改，早点释放内存
	delete vertex2index;  // 2021.01.11修改，早点释放内存

	// Dijkstra
	const auto vertex_time = ttts::strategy::dijkstra::solve<ttts::model::point_g>(index2vertex, index2edge, school_vertex_index);

	cout << "After First Dijkstra" << endl;  /* Test */

	// 读tif、计算栅格中心点经纬度、制图
	const auto population_raster = ttts::read_population_raster<ttts::model::point_g>(population_file);	 cout << "After Read Population Raster" << endl;  /* Test */
	const auto walk_speed_raster = ttts::read_walk_speed_raster(walk_speed_file);

	cout << "After read" << endl;

	// 对calculate_vector算travel time，直接把结果写到文件里
	ttts::strategy::travel_time::solve<ttts::model::point_g>(output_file, population_raster, walk_speed_raster, index2vertex, index2edge, vertex_time);

	// delete walk_speed_raster;
	delete population_raster;

	delete vertex_time;
	delete school_vertex_index;
	// delete index2edge;    // 2021.01.21修改：直接在travel_time里面释放掉
	// delete index2vertex;  // 2021.01.21修改：直接在travel_time里面释放掉
	delete pair2;
	delete pair1;

	cout << "Finished." << endl;
	
	return 0;
}

#undef int
