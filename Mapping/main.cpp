/******
 * 上学时间制图：主函数在此
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.03
 ******/

#include <iostream>
#include <string>
#include <utility>
#include <map>
#include <gdal_priv.h>
#include "utility.hpp"

using namespace std;

int main()
{
	ios_base::sync_with_stdio(false);
	cin.tie(nullptr); cout.tie(nullptr);

	GDALAllRegister();

	const string roadFile = "C:/Users/zgcyx/Desktop/upload/Road.shp";
	auto res = ttts::build_vertex_table(roadFile);
	
	cout << "Hello World!" << endl;

	delete res->first;
	delete res->second;
	
	system("PAUSE");
	return 0;
}
