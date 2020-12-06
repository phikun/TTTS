/******
 * ��ѧʱ����ͼ��C++�汾�������ú�������ֹ�����ļ�����
 * ���ļ������ĺ����У�
 *   1. ����·��shapefile���������
 *   2. ����·��shapefile����߱��˰������رߣ�֮��ɨ��3��·��shapefile�����رߵ�����ٶȡ�
 *   3. ����ѧУshapefile����ѧУ���б�
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
	typedef boost::geometry::model::point<double, 2, boost::geometry::cs::geographic<boost::geometry::degree> > point_g;  // ��������ĵ�
	typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_p;  // ͶӰ����ĵ�
	
	/// <summary>
	///		����·����shapefile�����������
	/// </summary>
	inline std::pair<boost::unordered_map<int, std::pair<double, double> >*, boost::unordered_map<std::pair<double, double>, int>* >* build_vertex_table(std::string fname)
	{
		auto index2vertex = new boost::unordered_map<int, std::pair<double, double> >();
		auto vertex2index = new boost::unordered_map<std::pair<double, double>, int>();
		boost::unordered_set<std::pair<double, double> > set;
		
		const auto pDataSet = reinterpret_cast<GDALDataset*>(GDALOpenEx(fname.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr));
		const auto pLayer = pDataSet->GetLayer(0);

		// ѭ��ÿһ��Feature
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
	///		����һ��OGRLineString����ѯ�������»���
	///		build_edge_table�ĸ�������
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
	///		����·����shapefile����ָ����·�ٶ����ڵ��У�����߱�
	///		���˰�����������رߵģ�֮��ֱ�Ӷ�ʼ�յ�id����������ɨ��һ��shapefile����û���رߵı߱���
	/// </summary>
	inline std::pair<boost::unordered_map<int, model::edge>*, boost::unordered_map<std::pair<int, int>, int>* >* build_edge_table(std::string fname, std::string speed_field_name, boost::unordered_map<std::pair<double, double>, int>* vertex2index)
	{
		auto index2edge = new boost::unordered_map<int, model::edge>();
		auto edge2index = new boost::unordered_map<std::pair<int, int>, int>();
		auto edge_list = new std::list<model::edge>();

		const auto pDataSet = reinterpret_cast<GDALDataset*>(GDALOpenEx(fname.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr));
		const auto pLayer = pDataSet->GetLayer(0);

		// ѭ��ÿһ��Feature
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
	///		��shapefile�ļ��ж���ѧУ�㣬����ָ��ѧУ��vector��ָ��
	///		��Ҫ��������ָ��ѧУ���ǵ�������ϵ��ͶӰ����ϵ�����ӷ���ֵ�����ƶϳ�T������
	/// </summary>
	template<typename T>
	std::vector<T>* read_school_points(std::string school_file)
	{
		const auto pDataSet = reinterpret_cast<GDALDataset*>(GDALOpenEx(school_file.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr));
		const auto pLayer = pDataSet->GetLayer(0);
		const auto n_schools = static_cast<int>(pLayer->GetFeatureCount());
		auto schools = new std::vector<T>(n_schools);
		
		// ѭ��ÿһ��Feature
		pLayer->ResetReading();
		auto pFeature = pLayer->GetNextFeature();
		auto ptr = 0;

		while (pFeature != nullptr)
		{
			const auto pGeometryRef = pFeature->GetGeometryRef();
			const auto pGeometryType = pGeometryRef->getGeometryType();

			if (pGeometryType != wkbPoint)  // ѧУ��Ӧ��ֻ������Point���͵�
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
