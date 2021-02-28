/******
 * ��ѧʱ����ͼ���������ڴ�
 * 2020.12.16���£�ʹ��Linux�������ʱ������������ļ��ĳ�main��Ϊmain������������
 *                �ѳ����ʱ�ŵ�Python��������˵
 * 2020.01.16���£����Dijkstra��ʱ��ֱ�Ӱѽ���浽�����
 * 2021.02.05���£��ֹ�ʵ�־����࣬����ʹ��cv::Mat_�������ص�()�����
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

// Ĭ�ϵ���������ļ�
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

	// ���������
	const auto pair1 = ttts::build_vertex_table(road_file);
	const auto index2vertex = pair1->first;
	const auto vertex2index = pair1->second;

	cout << "After build vertexTable" << endl;

	// �����߱�
	const auto pair2 = ttts::build_edge_table(road_file, speed_field_name, vertex2index);
	const auto index2edge = pair2->first;
	const auto edge2index = pair2->second;

	// ����ѧУ�㣬�˺������޸ĵ��߱��4����������ѧУ���ID�ſ��������⣬������ֽ������ȥ��飡��
	// ��edge2index�������رߣ��Ȳ����������ɨ��3��·���ļ������رߡ�
	const auto school_vertex_index = ttts::strategy::insert_school::solve<ttts::model::point_g>(school_file, index2edge, edge2index, index2vertex, vertex2index);
	delete edge2index;  // 2021.01.11�޸ģ�����ͷ��ڴ�
	delete vertex2index;  // 2021.01.11�޸ģ�����ͷ��ڴ�

	// Dijkstra
	const auto vertex_time = ttts::strategy::dijkstra::solve<ttts::model::point_g>(index2vertex, index2edge, school_vertex_index);

	cout << "After First Dijkstra" << endl;  /* Test */

	// ��tif������դ�����ĵ㾭γ�ȡ���ͼ
	const auto population_raster = ttts::read_population_raster<ttts::model::point_g>(population_file);	 cout << "After Read Population Raster" << endl;  /* Test */
	const auto walk_speed_raster = ttts::read_walk_speed_raster(walk_speed_file);

	cout << "After read" << endl;

	// ��calculate_vector��travel time��ֱ�Ӱѽ��д���ļ���
	ttts::strategy::travel_time::solve<ttts::model::point_g>(output_file, population_raster, walk_speed_raster, index2vertex, index2edge, vertex_time);

	// delete walk_speed_raster;
	delete population_raster;

	delete vertex_time;
	delete school_vertex_index;
	// delete index2edge;    // 2021.01.21�޸ģ�ֱ����travel_time�����ͷŵ�
	// delete index2vertex;  // 2021.01.21�޸ģ�ֱ����travel_time�����ͷŵ�
	delete pair2;
	delete pair1;

	cout << "Finished." << endl;
	
	return 0;
}

#undef int
