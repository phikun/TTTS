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

#include "model.hpp"
#include "utility.hpp"
#include "index.hpp"
#include "insert_school.hpp"

using namespace std;

int main()
{
	ios_base::sync_with_stdio(false);
	cin.tie(nullptr); cout.tie(nullptr);

	GDALAllRegister();

	const string school_file = "C:/Users/zgcyx/Desktop/upload/School-WGS84.shp";
	const string roadFile = "C:/Users/zgcyx/Desktop/upload/Road-WGS84-Speed.shp";
	const string speed_field_name = "speed";

	cout << "Hello World!" << endl;
	
	// 构建顶点表
	auto pair1 = ttts::build_vertex_table(roadFile);
	auto index2vertex = pair1->first;
	auto vertex2index = pair1->second;

	// 构建边表
	auto pair2 = ttts::build_edge_table(roadFile, speed_field_name, vertex2index);
	auto index2edge = pair2->first;
	auto edge2index = pair2->second;

	auto start_time = clock();
	
	// 测试一下插入学校点
	ttts::strategy::insert_school::solve<ttts::model::point_g>(school_file, index2edge, index2vertex);
	auto end_time = clock();

	cout << "Index Time: " << end_time - start_time << endl;
	
	delete edge2index, index2edge;
	delete index2vertex, vertex2index;
	delete pair2, pair1;
	
	system("PAUSE");
	return 0;
}
