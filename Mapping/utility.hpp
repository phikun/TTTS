/******
 * 上学时间制图（C++版本）：常用函数，防止单个文件过大
 * 此文件包括的函数有：
 *   1. 根据路网shapefile构建顶点表
 *   2. 根据路网shapefile构造边表【此版允许重边，之后扫描3次路网shapefile保留重边的最大速度】
 *   3. 根据学校shapefile构造学校点列表
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.03
 *****/

#pragma once

#include <iostream>
#include <string>
#include <list>
#include <utility>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <ogrsf_frmts.h>

#include "edge.hpp"

namespace ttts
{
	typedef boost::geometry::model::point<double, 2, boost::geometry::cs::geographic<boost::geometry::degree> > point_g;  // 地理坐标的点
	typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_p;  // 投影坐标的点
	
	/// <summary>
	///		输入路网的shapefile，构建顶点表
	/// </summary>
	inline std::pair<boost::unordered_map<int, std::pair<double, double> >*, boost::unordered_map<std::pair<double, double>, int>* >* build_vertex_table(std::string fname)
	{
		auto index2vertex = new boost::unordered_map<int, std::pair<double, double> >();
		auto vertex2index = new boost::unordered_map<std::pair<double, double>, int>();
		boost::unordered_set<std::pair<double, double> > set;
		
		const auto pDataSet = reinterpret_cast<GDALDataset*>(GDALOpenEx(fname.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr));
		const auto pLayer = pDataSet->GetLayer(0);

		// 循环每一个Feature
		pLayer->ResetReading();
		auto pFeature = pLayer->GetNextFeature();

		while (pFeature != nullptr)
		{
			const auto pGeometryRef = pFeature->GetGeometryRef();
			const auto pGeometryType = pGeometryRef->getGeometryType();
			
			if (pGeometryType == wkbLineString)
			{
				const auto pLineString = dynamic_cast<OGRLineString*>(pGeometryRef->clone());
				const auto n_points = pLineString->getNumPoints();
				auto pPoint = new OGRPoint();
				for (auto j = 0; j < n_points; ++j)
				{
					pLineString->getPoint(j, pPoint);
					const auto x = pPoint->getX();
					const auto y = pPoint->getY();
					set.insert(std::make_pair(x, y));
				}
			}

			if (pGeometryType == wkbMultiLineString)
			{
				const auto pMultiLineString = dynamic_cast<OGRMultiLineString*>(pGeometryRef->clone());
				const auto n_lines = pMultiLineString->getNumGeometries();

				for (auto i = 0; i < n_lines; ++i)
				{
					const auto pLineString = dynamic_cast<OGRLineString*>(pMultiLineString->getGeometryRef(i));
					const auto n_points = pLineString->getNumPoints();
					auto pPoint = new OGRPoint();
					for (auto j = 0; j < n_points; ++j)
					{
						pLineString->getPoint(j, pPoint);
						const auto x = pPoint->getX();
						const auto y = pPoint->getY();
						set.insert(std::make_pair(x, y));
					}
				}
			}
			
			pFeature = pLayer->GetNextFeature();
		}

		auto idx = 1;
		for (const auto& point : set)
		{
			index2vertex->insert(std::make_pair(idx, point));
			vertex2index->insert(std::make_pair(point, idx));
			++idx;
		}
		
		auto *res = new std::pair<boost::unordered_map<int, std::pair<double, double> >*, boost::unordered_map<std::pair<double, double>, int>* >();
		res->first = index2vertex;
		res->second = vertex2index;
		
		return res;
	}

	/// <summary>
	///		给定一个OGRLineString，查询顶点表更新弧段
	///		build_edge_table的辅助函数
	/// </summary>
	inline void update_edge_list(OGRLineString* pLineString, boost::unordered_map<std::pair<double, double>, int>* vertex2index, std::list<model::edge>* edge_list, double speed)
	{
		auto n_points = pLineString->getNumPoints();
		auto p1 = new OGRPoint(), p2 = new OGRPoint();
		
		for (auto j = 1; j < n_points; ++j)
		{
			pLineString->getPoint(j - 1, p1);
			pLineString->getPoint(j, p2);
			if (p1 == p2)
				continue;
			
			const auto x1 = p1->getX(), y1 = p1->getY();
			const auto x2 = p2->getX(), y2 = p2->getY();
			edge_list->emplace_back((*vertex2index)[std::make_pair(x1, y1)], (*vertex2index)[std::make_pair(x2, y2)], speed);
		}
	}

	/// <summary>
	///		输入路网的shapefile，并指定道路速度所在的列，构造边表
	///		【此版先完成允许重边的，之后直接对始终点id见索引，再扫描一次shapefile构造没有重边的边表！】
	/// </summary>
	inline std::pair<boost::unordered_map<int, model::edge>*, boost::unordered_map<std::pair<int, int>, int>* >* build_edge_table(std::string fname, std::string speed_field_name, boost::unordered_map<std::pair<double, double>, int>* vertex2index)
	{
		auto index2edge = new boost::unordered_map<int, model::edge>();
		auto edge2index = new boost::unordered_map<std::pair<int, int>, int>();
		auto edge_list = new std::list<model::edge>();

		const auto pDataSet = reinterpret_cast<GDALDataset*>(GDALOpenEx(fname.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr));
		const auto pLayer = pDataSet->GetLayer(0);

		// 循环每一个Feature
		pLayer->ResetReading();
		auto pFeature = pLayer->GetNextFeature();

		while (pFeature != nullptr)
		{
			const auto pGeometryRef = pFeature->GetGeometryRef();
			const auto pGeometryType = pGeometryRef->getGeometryType();
			const auto speed = pFeature->GetFieldAsDouble(speed_field_name.c_str());

			if (pGeometryType == wkbLineString)
			{
				const auto pLineString = dynamic_cast<OGRLineString*>(pGeometryRef->clone());
				update_edge_list(pLineString, vertex2index, edge_list, speed);
			}

			if (pGeometryType == wkbMultiLineString)
			{
				const auto pMultiLineString = dynamic_cast<OGRMultiLineString*>(pGeometryRef->clone());
				const auto n_lines = pMultiLineString->getNumGeometries();

				for (auto i = 0; i < n_lines; ++i)
				{
					const auto pLineString = dynamic_cast<OGRLineString*>(pMultiLineString->getGeometryRef(i));
					update_edge_list(pLineString, vertex2index, edge_list, speed);
				}
			}

			pFeature = pLayer->GetNextFeature();
		}

		auto idx = 1;
		for (const auto& edge : *edge_list)
		{
			index2edge->insert(std::make_pair(idx, edge));
			edge2index->insert(std::make_pair(std::make_pair(edge.from_node, edge.to_node), idx));
			++idx;
		}
		
		auto res = new std::pair<boost::unordered_map<int, model::edge>*, boost::unordered_map<std::pair<int, int>, int>* >();
		res->first = index2edge;
		res->second = edge2index;
		return res;
	}

	/// <summary>
	///		从shapefile文件中读入学校点，返回指向学校点vector的指针
	///		需要事先声明指针学校点是地理坐标系或投影坐标系，并从返回值类型推断出T的类型
	/// </summary>
	template<typename T>
	std::vector<T>* read_school_points(std::string school_file)
	{
		const auto pDataSet = reinterpret_cast<GDALDataset*>(GDALOpenEx(school_file.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr));
		const auto pLayer = pDataSet->GetLayer(0);
		const auto n_schools = static_cast<int>(pLayer->GetFeatureCount());
		auto schools = new std::vector<T>(n_schools);
		
		// 循环每一个Feature
		pLayer->ResetReading();
		auto pFeature = pLayer->GetNextFeature();
		auto ptr = 0;

		while (pFeature != nullptr)
		{
			const auto pGeometryRef = pFeature->GetGeometryRef();
			const auto pGeometryType = pGeometryRef->getGeometryType();

			if (pGeometryType != wkbPoint)  // 学校点应该只可能是Point类型的
				continue;

			const auto pPoint = dynamic_cast<OGRPoint*>(pGeometryRef->clone());
			const auto x = pPoint->getX();
			const auto y = pPoint->getY();
			const auto pnt = T(x, y);
			(*schools)[ptr++] = pnt;
			
			pFeature = pLayer->GetNextFeature();
		}

		return schools;
	}
}
