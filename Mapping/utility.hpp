/******
 * 上学时间制图（C++版本）：常用函数，防止单个文件过大
 * 此文件包括的函数有：
 *   1. 根据路网shapefile构建顶点表
 *   2. 【其他一会再说】
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.03
 *****/

#pragma once

#include <iostream>
#include <string>
#include <utility>
#include <map>
#include <ogrsf_frmts.h>

namespace ttts
{
	/// <summary>
	///		输入路网的shapefile，构建顶点表
	/// </summary>
	inline std::pair<std::map<int, std::pair<double, double> >, std::map<std::pair<double, double>, int> > build_vertex_table(std::string fname)
	{
		std::map<int, std::pair<double, double> > index2vertex;
		std::map<std::pair<double, double>, int> vertex2index;
		
		const auto pDataSet = (GDALDataset*) GDALOpenEx(fname.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
		const auto pLayer = pDataSet->GetLayer(0);

		const auto name = pLayer->GetName();
		std::cout << "Name = " << name << std::endl;

		auto res = std::make_pair(index2vertex, vertex2index);
		return res;
	}
}
