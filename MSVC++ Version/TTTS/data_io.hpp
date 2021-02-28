/******
 * ��ѧʱ����ͼ��C++�汾���������������
 * ������ȡshapefile���Ա�ttrs��ʽ�ĵ�·����
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2021.02.23
 *****/

#pragma once

#include <iostream>
#include <fstream>
#include <utility>
#include <cstring>
#include <string>
#include <vector>
#include <list>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

#include <ogrsf_frmts.h>

#include "global_define.hpp"
#include "model.hpp"

#define int long long

namespace ttts
{
	namespace data_io
	{
		constexpr size_t len_id = 4;  // TTRS�ļ���ʶ���ĳ��ȣ�4���ֽڣ�len("TTRS") = 4;
		constexpr char expect_id[] = "TTRS";  // TTRS�ļ��ı�ʶ����TTRS

		/// <summary>
		///     ����ļ�����׺�Ƿ�����������ͬ
		/// </summary> 
		inline bool cmp_suffix(const std::string& file_name, const std::string& expect_suffix)
		{
			const auto offset = expect_suffix.length();
			const auto real_suffix = file_name.substr(file_name.length() - offset, offset);

			return real_suffix == expect_suffix;
		}

		/// <summary>
		///     ���������ļ����Ƿ���.shpΪ��׺
		/// </summary>
		inline bool is_shapefile(const std::string& file_name)
		{
			const auto expect_suffix = std::string(".shp");
			return cmp_suffix(file_name, expect_suffix);
		}

		/// <summary>
		///     ���������ļ��Ƿ����Ա�ttrs�ļ���Ҫƥ���ļ���׺.ttrs�������ļ��鿴ǰ4���ֽ��Ƿ���TTRS
		/// </summary>
		inline bool is_ttrs_file(const std::string& file_name)
		{
			const auto expect_suffix = std::string(".ttrs");
			const auto suffix_res = cmp_suffix(file_name, expect_suffix);
			if (!suffix_res) return false;

			char real_id[len_id + 1];

			std::ifstream fin;
			fin.open(file_name, std::ios::in | std::ios::binary);
			fin.read(real_id, len_id);
			real_id[len_id] = '\0';
			fin.close();

			return strcmp(real_id, expect_id) == 0;
		}

		/// <summary>
		///     ��ȡshapefile��ʽ�ĵ�·���ݣ����춥���
		/// </summary>
		inline std::pair<boost::unordered_map<int, std::pair<double, double> >*, boost::unordered_map<std::pair<double, double>, int>* >* build_vertex_table_shapefile(std::string fname)
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

			size_t idx = 1;
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
		///     ��ȡ�Ա�ttrs��ʽ�ĵ�·���ݣ����춥���
		/// </summary>
		inline std::pair<boost::unordered_map<int, std::pair<double, double> >*, boost::unordered_map<std::pair<double, double>, int>* >* build_vertex_table_ttrs(std::string fname)
		{
			auto index2vertex = new boost::unordered_map<int, std::pair<double, double> >();
			auto vertex2index = new boost::unordered_map<std::pair<double, double>, int>();
			boost::unordered_set<std::pair<double, double> > set;

			std::ifstream fin;
			fin.open(fname, std::ios::in | std::ios::binary);

			fin.seekg(0, std::ios::end);
			const auto end_pos = fin.tellg();

			fin.seekg(len_id, std::ios::beg);  // Skip identifier TTTS

			while (fin.tellg() < end_pos)
			{
				int32_t n_vertex;
				double_t x, y;

				fin.read(reinterpret_cast<char*>(&n_vertex), sizeof(int32_t));
				for (size_t i = 0; i < static_cast<size_t>(n_vertex); ++i)
				{
					fin.read(reinterpret_cast<char*>(&x), sizeof(double_t));
					fin.read(reinterpret_cast<char*>(&y), sizeof(double_t));
					set.insert(std::make_pair(x, y));
				}

				fin.seekg(sizeof(double_t), std::ios::cur);  // ����������ٶ�
			}

			fin.close();

			size_t idx = 1;
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
		///		build_edge_table_shapefile�ĸ�������
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
		///     ��ȡshapefile��ʽ�ĵ�·���ݣ�����߱�
		/// </summary>
		inline std::pair<boost::unordered_map<int, model::edge>*, boost::unordered_map<std::pair<int, int>, int>* >* build_edge_table_shapefile(std::string fname, std::string speed_field_name, boost::unordered_map<std::pair<double, double>, int>* vertex2index)
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

			size_t idx = 1;
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
		///     ��ȡ�Ա�ttrs��ʽ�ĵ�·���ݣ�����߱�
		///     Ϊ�����ȡshapefile�ĺ�������һ�£���·�ٶ������еĲ����Ƕ����
		/// </summary>
		inline std::pair<boost::unordered_map<int, model::edge>*, boost::unordered_map<std::pair<int, int>, int>* >* build_edge_table_ttrs(std::string fname, std::string speed_field_name, boost::unordered_map<std::pair<double, double>, int>* vertex2index)
		{
			std::cout << "Here in function: build_edge_table_ttrs" << std::endl;

			auto index2edge = new boost::unordered_map<int, model::edge>();
			auto edge2index = new boost::unordered_map<std::pair<int, int>, int>();
			auto edge_list = new std::list<model::edge>();

			std::ifstream fin;
			fin.open(fname, std::ios::in | std::ios::binary);

			fin.seekg(0, std::ios::end);
			const auto end_pos = fin.tellg();

			fin.seekg(len_id, std::ios::beg);  // Skip identifier TTTS

			while (fin.tellg() < end_pos)
			{
				int32_t n_vertex;
				double_t x, y, speed;

				fin.read(reinterpret_cast<char*>(&n_vertex), sizeof(int32_t));
				std::vector<std::pair<double_t, double_t> > segs(n_vertex);

				for (size_t i = 0; i < static_cast<size_t>(n_vertex); ++i)
				{
					fin.read(reinterpret_cast<char*>(&x), sizeof(double_t));
					fin.read(reinterpret_cast<char*>(&y), sizeof(double_t));
					segs[i] = std::make_pair(x, y);
				}

				fin.read(reinterpret_cast<char*>(&speed), sizeof(double_t));

				for (size_t i = 1; i < static_cast<size_t>(n_vertex); ++i)
				{
					const auto p1 = segs[i - 1], p2 = segs[i];
					edge_list->emplace_back((*vertex2index)[p1], (*vertex2index)[p2], speed);
				}
			}

			fin.close();

			std::cout << "After Read TTRS File!" << std::endl;

			size_t idx = 1;
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
	}
}

#undef int
