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
#include "utility.hpp"

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

	// 获取学校点
	const auto schools = ttts::read_school_points<ttts::point_g>(school_file);  // 这个输入数据中的学校点是地理坐标

	delete schools;
	delete edge2index, index2edge;
	delete index2vertex, vertex2index;
	delete pair2, pair1;
	
	system("PAUSE");
	return 0;
}
