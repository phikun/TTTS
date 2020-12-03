/******
 * 上学时间制图：主函数在此
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.03
 ******/

#include <iostream>
#include <string>
#include <utility>
#include <ctime>
#include <gdal_priv.h>
#include "utility.hpp"

using namespace std;

int main()
{
	ios_base::sync_with_stdio(false);
	cin.tie(nullptr); cout.tie(nullptr);

	GDALAllRegister();
	
	const string roadFile = "C:/Users/zgcyx/Desktop/upload/Road-WGS84-Speed.shp";
	const string speed_field_name = "speed";
	
	auto pair1 = ttts::build_vertex_table(roadFile);
	auto index2vertex = pair1->first;
	auto vertex2index = pair1->second;

	auto start_time = clock();
	
	auto pair2 = ttts::build_edge_table(roadFile, speed_field_name, vertex2index);
	auto index2edge = pair2->first;
	auto edge2index = pair2->second;

	auto end_time = clock();

	cout << "Edge Time = " << end_time - start_time << endl;
	cout << "Edge Size = " << index2edge->size() << endl;
	cout << (*index2edge)[1].speed << endl;
	
	cout << "Hello World!" << endl;

	delete edge2index, index2edge;
	delete index2vertex, vertex2index;
	delete pair2, pair1;
	
	system("PAUSE");
	return 0;
}
