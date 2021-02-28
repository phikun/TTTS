/******
 * 上学时间制图（C++版本）：常用函数，防止单个文件过大
 * 此文件包括的函数有：
 *   1. 根据路网shapefile构建顶点表
 *   2. 根据路网shapefile构造边表【此版允许重边，之后扫描3次路网shapefile保留重边的最大速度】
 *   3. 根据学校shapefile构造学校点列表
 *
 * 2021.01.20 Update: Using #define int long long to prevent large number. If the input GeoTiff has int32 data type, this code may return wrong value! 
 * 2021.02.05 Update: Using self-made ttts::model::matrix::matrix<T> instead of cv::Mat_<T>
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

#include "global_define.hxx"
#include "data_io.hxx"
#include "model.hxx"
#include "strategy.hxx"

#define int long long

namespace ttts
{
	// 获取tif数据矩阵的函数，先把声明放前面
	model::matrix::matrix<bool>* get_data_matrix(const GDALDataType& data_type, GDALDataset* pDataSet, const int& n_rows, const int& n_cols, const int& n_bands, const double& no_data_value);
	
	/// <summary>
	///		输入路网的shapefile，构建顶点表
	/// </summary>
	inline std::pair<boost::unordered_map<int, std::pair<double, double> >*, boost::unordered_map<std::pair<double, double>, int>* >* build_vertex_table(std::string fname)
	{
		auto *res = new std::pair<boost::unordered_map<int, std::pair<double, double> >*, boost::unordered_map<std::pair<double, double>, int>* >();

		if (data_io::is_shapefile(fname))
		{
			std::cout << "Read Shapefile!" << std::endl;
			res = data_io::build_vertex_table_shapefile(fname);
		}
		else if (data_io::is_ttrs_file(fname))
		{
			std::cout << "Read TTRS File!" << std::endl;
			res = data_io::build_vertex_table_ttrs(fname);
		}
		else 
			throw "Unable to open mysterious file format!";

		return res;
	}

	/// <summary>
	///		输入路网的shapefile，并指定道路速度所在的列，构造边表
	///		【此版先完成允许重边的，之后直接对始终点id见索引，再扫描一次shapefile构造没有重边的边表！】
	/// </summary>
	inline std::pair<boost::unordered_map<int, model::edge>*, boost::unordered_map<std::pair<int, int>, int>* >* build_edge_table(std::string fname, std::string speed_field_name, boost::unordered_map<std::pair<double, double>, int>* vertex2index)
	{
		auto res = new std::pair<boost::unordered_map<int, model::edge>*, boost::unordered_map<std::pair<int, int>, int>* >();
            
		if (data_io::is_shapefile(fname))
			res = data_io::build_edge_table_shapefile(fname, speed_field_name, vertex2index);
		else if (data_io::is_ttrs_file(fname))
			res = data_io::build_edge_table_ttrs(fname, speed_field_name, vertex2index);
		else
			throw "Unable to open mysterious file format!";

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
	///		读取人口数据的Tif，需事先将非道路栅格赋成NoData，
	///		这里只保留栅格是否是no_data、返回行列数、no_data_value、坐标等信息
	///		并直接在此函数中计算栅格中心点的XY坐标
	/// </summary>
	template <typename TPoint>
	model::population_dataset* read_population_raster(std::string tif_file)
	{
		auto pDataSet = reinterpret_cast<GDALDataset*>(GDALOpen(tif_file.c_str(), GA_ReadOnly));

		const auto n_bands = pDataSet->GetRasterCount();
		const auto n_cols = pDataSet->GetRasterXSize();
		const auto n_rows = pDataSet->GetRasterYSize();

		if (n_bands != 1)
			throw "Input TIF file must have 1 band!";

		const auto data_type = pDataSet->GetRasterBand(1)->GetRasterDataType();
		const auto no_data_value = pDataSet->GetRasterBand(1)->GetNoDataValue();

		const auto pTif = new model::population_dataset();
		pTif->n_rows = n_rows;
		pTif->n_cols = n_cols;
		pTif->no_data_value = no_data_value;
		
		pDataSet->GetGeoTransform(pTif->trans);  // 获取坐标变换的6个参数

		const auto proj = pDataSet->GetProjectionRef();
		pTif->proj = std::string(proj);  // 获取坐标系定义

		std::cout << "In function: read_population_raster, after basic read." << std::endl;

		pTif->mat = get_data_matrix(data_type, pDataSet, n_rows, n_cols, n_bands, no_data_value);  // 获取数据矩阵【只考虑是否是NoData】
		
		std::cout << "    after get data matrix" << std::endl;

		strategy::get_center_coordinate<TPoint>(pTif);  // 根据地理坐标或投影坐标计算栅格中心点XY坐标

		return pTif;
	}

	/// <summary>
	///		从tif中读取数据，返回是否是控制的矩阵，用于read_tif函数调用
	///		【此函数类似于简单工厂】
	/// </summary>
	model::matrix::matrix<bool>* get_data_matrix(const GDALDataType& data_type, GDALDataset* pDataSet, const int& n_rows, const int& n_cols, const int& n_bands, const double& no_data_value)
	{
		const auto mat = new model::matrix::matrix<bool>(n_rows, n_cols);
		auto ptr = 0LL;
		
		if (data_type == GDT_Byte)
		{
			const auto data_b = new unsigned char[n_bands * n_rows * n_cols];
			pDataSet->RasterIO(GF_Read, 0, 0, n_cols, n_rows, data_b, n_cols, n_rows, data_type, n_bands, nullptr, 0, 0, 0);
			const auto fill_value_b = static_cast<unsigned char>(no_data_value);
			for (auto i = 0; i < n_rows; ++i) for (auto j = 0; j < n_cols; ++j) (*mat)(i, j) = fill_value_b != data_b[ptr++];
			delete[] data_b;
			return mat;
		}
		if (data_type == GDT_Int16)
		{
			const auto data_d = new short[n_bands * n_rows * n_cols];
			pDataSet->RasterIO(GF_Read, 0, 0, n_cols, n_rows, data_d, n_cols, n_rows, data_type, n_bands, nullptr, 0, 0, 0);
			const auto fill_value_d = static_cast<short>(no_data_value);
			for (auto i = 0; i < n_rows; ++i) for (auto j = 0; j < n_cols; ++j) (*mat)(i, j) = fill_value_d != data_d[ptr++];
			delete[] data_d;
			return mat;
		}
		if (data_type == GDT_Int32)
		{
			const auto data_ld = new signed[n_bands * n_rows * n_cols];
			pDataSet->RasterIO(GF_Read, 0, 0, n_cols, n_rows, data_ld, n_cols, n_rows, data_type, n_bands, nullptr, 0, 0, 0);
			const auto fill_value_ld = static_cast<signed>(no_data_value);
			for (auto i = 0; i < n_rows; ++i) for (auto j = 0; j < n_cols; ++j) (*mat)(i, j) = fill_value_ld != data_ld[ptr++];
			delete[] data_ld;
			return mat;
		}
		if (data_type == GDT_Float32)
		{
			std::cout << "Here float32!" << std::endl;
			const auto data_f = new float[n_bands * n_rows * n_cols];
			pDataSet->RasterIO(GF_Read, 0, 0, n_cols, n_rows, data_f, n_cols, n_rows, data_type, n_bands, nullptr, 0, 0, 0);
			const auto fill_value_f = static_cast<float>(no_data_value);
			for (auto i = 0; i < n_rows; ++i) for (auto j = 0; j < n_cols; ++j) (*mat)(i, j) = fill_value_f != data_f[ptr++];
			delete[] data_f;
			return mat;
		}
		if (data_type == GDT_Float64)
		{
			std::cout << "Here float64!" << std::endl;
			const auto data_lf = new double[n_bands * n_rows * n_cols];
			pDataSet->RasterIO(GF_Read, 0, 0, n_cols, n_rows, data_lf, n_cols, n_rows, data_type, n_bands, nullptr, 0, 0, 0);
			const auto fill_value_lf = no_data_value;
			for (auto i = 0; i < n_rows; ++i) for (auto j = 0; j < n_cols; ++j) (*mat)(i, j) = fill_value_lf != data_lf[ptr++];
			delete[] data_lf;
			return mat;
		}

		// 不属于以上类别，说明对该类型的数据未实现读取方法，抛出异常
		throw "No implement error: mysterious data type!";
	}

	/// <summary>
	///		
	/// </summary>
	inline model::speed_dataset* read_walk_speed_raster(std::string tif_file)
	{
		auto pDataSet = reinterpret_cast<GDALDataset*>(GDALOpen(tif_file.c_str(), GA_ReadOnly));

		const auto n_bands = static_cast<int>(pDataSet->GetRasterCount());
		const auto n_cols = static_cast<int>(pDataSet->GetRasterXSize());
		const auto n_rows = static_cast<int>(pDataSet->GetRasterYSize());

		if (n_bands != 1)
			throw "Input TIF file must have 1 band!";

		const auto data_type = pDataSet->GetRasterBand(1)->GetRasterDataType();
		if (data_type != GDT_Float64)
			throw "Walk Speed Raster can only accept double type!";
		
		const auto pTif = new model::speed_dataset();
		pTif->n_rows = n_rows;
		pTif->n_cols = n_cols;

		pDataSet->GetGeoTransform(pTif->trans);  // 获取坐标变换的6个参数

		const auto proj = pDataSet->GetProjectionRef();
		pTif->proj = std::string(proj);  // 获取坐标系定义

		std::cout << "length = " << n_bands * n_rows * n_cols << std::endl;

		const auto data = new double[n_bands * n_rows * n_cols];
		pTif->mat = new model::matrix::matrix<double>(n_rows, n_cols);
		pDataSet->RasterIO(GF_Read, 0, 0, n_cols, n_rows, data, n_cols, n_rows, data_type, n_bands, nullptr, 0, 0, 0);
		auto ptr = 0LL;
		for (auto i = 0; i < n_rows; ++i) for (auto j = 0; j < n_cols; ++j) (*pTif->mat)(i, j) = data[ptr++];

		delete[] data;
		return pTif;
	}

	/// <summary>
	///		将计算结果写入GeoTiff
	///		需传入结果矩阵mat、人口栅格的指针（用于获取行列号及坐标投影信息）
	/// </summary>
	inline void write_result_to_geotiff(std::string file_name, model::matrix::matrix<double>* mat, model::population_dataset* pTif)
	{
		const auto pszFormat = "GTiff";
		const auto pDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
		
		const auto pDataSet = pDriver->Create(file_name.c_str(), pTif->n_cols, pTif->n_rows, 1, GDT_Float64, nullptr);  // 1 -> 1个波段
		pDataSet->SetGeoTransform(pTif->trans);  // 写入坐标变换向量（和输入的人口栅格一样）
		pDataSet->SetProjection(pTif->proj.c_str());  // 写入投影信息（和输入的人口栅格一样）

		// 然后写入数据
		const auto data = new double[pTif->n_rows * pTif->n_cols];
		auto ptr = 0LL;
		for (auto i = 0LL; i < pTif->n_rows; ++i)
			for (auto j = 0LL; j < pTif->n_cols; ++j)
				data[ptr++] = (*mat)(i, j);
		pDataSet->RasterIO(GF_Write, 0, 0, pTif->n_cols, pTif->n_rows, data, pTif->n_cols, pTif->n_rows, GDT_Float64, 1, nullptr, 0, 0, 0, nullptr);
		
		GDALClose(reinterpret_cast<GDALDatasetH*>(pDataSet));
		
		delete[] data;
	}
}

#undef int
