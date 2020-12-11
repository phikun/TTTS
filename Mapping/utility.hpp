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
#include <ogrsf_frmts.h>
#include <opencv2/opencv.hpp>

#include "model.hpp"

namespace ttts
{
	// 获取tif数据矩阵的函数，先把声明放前面
	cv::Mat_<bool>* get_data_matrix(const GDALDataType& data_type, GDALDataset* pDataSet, const int& n_rows, const int& n_cols, const int& n_bands, const double& no_data_value);

	
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

	/// <summary>
	///		构造id到boost::geometry::model::point的索引，用于建立空间索引
	///		对于上学时间>=INF的结点（即到不了学校的结点），不加入它们
	/// </summary>
	template<typename TPoint>
	boost::unordered_map<int, TPoint>* build_index_to_geometry_point(boost::unordered_map<int, std::pair<double, double> >* index2vertex, std::vector<double>* vertex_time)
	{
		const auto res = new boost::unordered_map<int, TPoint>();

		for (const auto& p : *index2vertex)
		{
			const auto index = p.first;
			const auto x = p.second.first;
			const auto y = p.second.second;

			if ((*vertex_time)[index] >= model::INF)
				continue;
			
			const auto pnt = TPoint(x, y);
			res->insert(std::make_pair(index, pnt));
		}

		return res;
	}
	
	/// <summary>
	///		构造id到boost::geometry::model::segment的索引，用于建立空间索引
	///		【实际上build_edge_table】时可直接返回这种索引
	/// </summary>
	template<typename TPoint>
	boost::unordered_map<int, boost::geometry::model::segment<TPoint> >* build_index_to_geometry_edge(boost::unordered_map<int, model::edge>* index2edge, boost::unordered_map<int, std::pair<double, double> >* index2vertex)
	{
		auto res = new boost::unordered_map<int, boost::geometry::model::segment<TPoint> >();

		for (const auto& pair : *index2edge)
		{
			const auto index = pair.first;
			const auto edge_ = pair.second;

			const auto p1x = (*index2vertex)[edge_.from_node].first;
			const auto p1y = (*index2vertex)[edge_.from_node].second;
			const auto p2x = (*index2vertex)[edge_.to_node].first;
			const auto p2y = (*index2vertex)[edge_.to_node].second;
			const auto p1 = TPoint(p1x, p1y);
			const auto p2 = TPoint(p2x, p2y);
			
			auto seg = boost::geometry::model::segment<TPoint>(p1, p2);
			res->insert(std::make_pair(index, seg));
		}
		
		return res;
	}

	/// <summary>
	///		读只有一个波段的Tif，返回数据矩阵、no_data_value、坐标信息
	/// </summary>
	inline model::tif_dataset* read_tif(std::string tif_file)
	{
		auto pDataSet = reinterpret_cast<GDALDataset*>(GDALOpen(tif_file.c_str(), GA_ReadOnly));

		const auto n_bands = pDataSet->GetRasterCount();
		const auto n_cols = pDataSet->GetRasterXSize();
		const auto n_rows = pDataSet->GetRasterYSize();

		if (n_bands != 1)
			throw std::exception("Input TIF file must have 1 band!");

		const auto data_type = pDataSet->GetRasterBand(1)->GetRasterDataType();
		const auto no_data_value = pDataSet->GetRasterBand(1)->GetNoDataValue();

		const auto pTif = new model::tif_dataset();
		pTif->n_rows = n_rows;
		pTif->n_cols = n_cols;
		pTif->no_data_value = no_data_value;
		
		pDataSet->GetGeoTransform(pTif->trans);  // 获取坐标变换的6个参数

		const auto proj = pDataSet->GetProjectionRef();
		pTif->proj = std::string(proj);  // 获取坐标系定义

		pTif->mat = get_data_matrix(data_type, pDataSet, n_rows, n_cols, n_bands, no_data_value);  // 获取数据矩阵【只考虑是否是NoData】
		
		return pTif;
	}

	/// <summary>
	///		从tif中读取数据，返回是否是控制的矩阵，用于read_tif函数调用
	///		【此函数类似于简单工厂】
	/// </summary>
	cv::Mat_<bool>* get_data_matrix(const GDALDataType& data_type, GDALDataset* pDataSet, const int& n_rows, const int& n_cols, const int& n_bands, const double& no_data_value)
	{
		const auto mat = new cv::Mat_<bool>(n_rows, n_cols);
		auto ptr = 0;
		
		if (data_type == GDT_Byte)
		{
			const auto data_b = new unsigned char[n_bands * n_rows * n_cols];
			pDataSet->RasterIO(GF_Read, 0, 0, n_cols, n_rows, data_b, n_cols, n_rows, data_type, n_bands, nullptr, 0, 0, 0);
			const auto fill_value_b = static_cast<unsigned char>(no_data_value);
			for (auto i = 0; i < n_rows; ++i) for (auto j = 0; j < n_cols; ++j) (*mat)(i, j) = fill_value_b != data_b[ptr++];
			return mat;
		}
		if (data_type == GDT_Int16)
		{
			const auto data_d = new short[n_bands * n_rows * n_cols];
			pDataSet->RasterIO(GF_Read, 0, 0, n_cols, n_rows, data_d, n_cols, n_rows, data_type, n_bands, nullptr, 0, 0, 0);
			const auto fill_value_d = static_cast<short>(no_data_value);
			for (auto i = 0; i < n_rows; ++i) for (auto j = 0; j < n_cols; ++j) (*mat)(i, j) = fill_value_d != data_d[ptr++];
			return mat;
		}
		if (data_type == GDT_Int32)
		{
			const auto data_ld = new int[n_bands * n_rows * n_cols];
			pDataSet->RasterIO(GF_Read, 0, 0, n_cols, n_rows, data_ld, n_cols, n_rows, data_type, n_bands, nullptr, 0, 0, 0);
			const auto fill_value_ld = static_cast<int>(no_data_value);
			for (auto i = 0; i < n_rows; ++i) for (auto j = 0; j < n_cols; ++j) (*mat)(i, j) = fill_value_ld != data_ld[ptr++];
			return mat;
		}
		if (data_type == GDT_Float32)
		{
			const auto data_f = new float[n_bands * n_rows * n_cols];
			pDataSet->RasterIO(GF_Read, 0, 0, n_cols, n_rows, data_f, n_cols, n_rows, data_type, n_bands, nullptr, 0, 0, 0);
			const auto fill_value_f = static_cast<float>(no_data_value);
			for (auto i = 0; i < n_rows; ++i) for (auto j = 0; j < n_cols; ++j) (*mat)(i, j) = fill_value_f != data_f[ptr++];
			return mat;
		}
		if (data_type == GDT_Float64)
		{
			const auto data_lf = new double[n_bands * n_rows * n_cols];
			pDataSet->RasterIO(GF_Read, 0, 0, n_cols, n_rows, data_lf, n_cols, n_rows, data_type, n_bands, nullptr, 0, 0, 0);
			const auto fill_value_lf = no_data_value;
			for (auto i = 0; i < n_rows; ++i) for (auto j = 0; j < n_cols; ++j) (*mat)(i, j) = fill_value_lf != data_lf[ptr++];
			return mat;
		}

		// 不属于以上类别，说明对该类型的数据未实现读取方法，抛出异常
		throw std::exception("No implement error: mysterious data type!");
	}
}
