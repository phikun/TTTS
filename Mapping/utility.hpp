/******
 * ��ѧʱ����ͼ��C++�汾�������ú�������ֹ�����ļ�����
 * ���ļ������ĺ����У�
 *   1. ����·��shapefile���������
 *   2. ������һ����˵��
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.03
 *****/

#pragma once

#include <iostream>
#include <string>
#include <utility>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <ogrsf_frmts.h>

namespace ttts
{
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

		const auto name = pLayer->GetName();

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

		std::cout << "Size = " << set.size() << std::endl;
		
		auto *res = new std::pair<boost::unordered_map<int, std::pair<double, double> >*, boost::unordered_map<std::pair<double, double>, int>* >();
		res->first = index2vertex;
		res->second = vertex2index;
		
		return res;
	}
}
