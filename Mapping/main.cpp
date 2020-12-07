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
	const auto pair1 = ttts::build_vertex_table(roadFile);
	const auto index2vertex = pair1->first;
	const auto vertex2index = pair1->second;

	// 构建边表
	const auto pair2 = ttts::build_edge_table(roadFile, speed_field_name, vertex2index);
	const auto index2edge = pair2->first;
	const auto edge2index = pair2->second;
	
	// 插入学校点，此函数会修改点表边表的4个索引，【学校点的ID号可能有问题，出现奇怪结果后再去检查！】
	// 【edge2index那里有重边，先不管它，最后扫描3次路网文件消除重边】
	const auto school_vertex_index = ttts::strategy::insert_school::solve<ttts::model::point_g>(school_file, index2edge, edge2index, index2vertex, vertex2index);

	// 然后是Dijkstra
	
	delete school_vertex_index;
	delete edge2index, index2edge;
	delete index2vertex, vertex2index;
	delete pair2, pair1;
	
	system("PAUSE");
	return 0;
}
