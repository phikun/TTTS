/******
 * ��ѧʱ����ͼ���������ڴ�
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

	cout << "Hello World!" << endl;
	
	// ���������
	const auto pair1 = ttts::build_vertex_table(roadFile);
	const auto index2vertex = pair1->first;
	const auto vertex2index = pair1->second;

	// �����߱�
	const auto pair2 = ttts::build_edge_table(roadFile, speed_field_name, vertex2index);
	const auto index2edge = pair2->first;
	const auto edge2index = pair2->second;
	
	// ����ѧУ�㣬�˺������޸ĵ��߱��4����������ѧУ���ID�ſ��������⣬������ֽ������ȥ��飡��
	// ��edge2index�������رߣ��Ȳ����������ɨ��3��·���ļ������رߡ�
	const auto insert_school_start_time = clock();
	const auto school_vertex_index = ttts::strategy::insert_school::solve<ttts::model::point_g>(school_file, index2edge, edge2index, index2vertex, vertex2index);
	const auto insert_school_end_time = clock();
	cout << "Insert School Time: " << fixed << setprecision(3) << (insert_school_end_time - insert_school_start_time) / 1000.0 << endl;
	
	// Dijkstra
	const auto dijkstra_start_time = clock();
	const auto vertex_time = ttts::strategy::dijkstra::solve<ttts::model::point_g>(index2vertex, index2edge, school_vertex_index);
	const auto dijkstra_end_time = clock();
	cout << "Dijkstra Time: " << fixed << setprecision(3) << (dijkstra_end_time - dijkstra_start_time) / 1000.0 << endl;

	// ��tif������դ�����ĵ㾭γ�ȡ���ͼ
	const auto population_raster = ttts::read_population_raster(population_file);	
	const auto walk_speed_raster = ttts::read_walk_speed_raster(walk_speed_file);

	cout << "After read" << endl;
	
	// ����ֱ����դ�����ĵ�ľ�γ��
	ttts::strategy::get_center_coordinate<ttts::model::point_g>(population_raster);

	// ��ȡ��Ҫֱ����travel time��դ�񣬲���ֱ������vector<geometry>
	
	// �����ﻹ��Ҫÿ��դ���Ӧ�����кţ�����Ȱ�travel time����Ҫ����д����˵��
	
	// ��calculate_vector��travel time
	// const auto res = ttts::strategy::travel_time::solve<ttts::model::point_g>(index2vertex);
	const auto res = ttts::strategy::travel_time::solve<ttts::model::point_g>(population_raster, walk_speed_raster, index2vertex, index2edge, vertex_time);

	for (auto i = 0; i < 10; ++i)
		cout << fixed << setprecision(3) << (*res)[i] << endl;

	// ��ֵ�Ǻ���ģ����ڿ��Խ�ͼ����һ��Dijkstra��
	
	delete res;
	delete walk_speed_raster;
	delete population_raster;

	delete vertex_time;
	delete school_vertex_index;
	delete edge2index, index2edge;
	delete index2vertex, vertex2index;
	delete pair2, pair1;
	
	system("PAUSE");
	return 0;
}
