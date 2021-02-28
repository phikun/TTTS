/******
 * ��ѧʱ����ͼ��C++�汾�������ú�������ֹ�����ļ�����
 * ���ļ������ĺ����У�
 *   1. ����·��shapefile���������
 *   2. ����·��shapefile����߱��˰������رߣ�֮��ɨ��3��·��shapefile�����رߵ�����ٶȡ�
 *   3. ����ѧУshapefile����ѧУ���б�
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
#include <utility>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/geometry.hpp>
#include <ogrsf_frmts.h>

#include "global_define.hpp"
#include "data_io.hpp"
#include "model.hpp"
#include "strategy.hpp"

#define int long long

namespace ttts
{
	// ��ȡtif���ݾ���ĺ������Ȱ�������ǰ��
	model::matrix::matrix<bool>* get_data_matrix(const GDALDataType& data_type, GDALDataset* pDataSet, const int& n_rows, const int& n_cols, const int& n_bands, const double& no_data_value);

	/// <summary>
	///		����·����shapefile�����������
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
			throw std::exception("Unable to open mysterious file format!");

		return res;
	}

	/// <summary>
	///		����·����shapefile����ָ����·�ٶ����ڵ��У�����߱�
	///		���˰�����������رߵģ�֮��ֱ�Ӷ�ʼ�յ�id����������ɨ��һ��shapefile����û���رߵı߱���
	/// </summary>
	inline std::pair<boost::unordered_map<int, model::edge>*, boost::unordered_map<std::pair<int, int>, int>* >* build_edge_table(std::string fname, std::string speed_field_name, boost::unordered_map<std::pair<double, double>, int>* vertex2index)
	{
		auto res = new std::pair<boost::unordered_map<int, model::edge>*, boost::unordered_map<std::pair<int, int>, int>* >();

		if (data_io::is_shapefile(fname))
			res = data_io::build_edge_table_shapefile(fname, speed_field_name, vertex2index);
		else if (data_io::is_ttrs_file(fname))
			res = data_io::build_edge_table_ttrs(fname, speed_field_name, vertex2index);
		else
			throw std::exception("Unable to open mysterious file format!");

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

	/// <summary>
	///		����id��boost::geometry::model::point�����������ڽ����ռ�����
	///		������ѧʱ��>=INF�Ľ�㣨��������ѧУ�Ľ�㣩������������
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
	///		����id��boost::geometry::model::segment�����������ڽ����ռ�����
	///		��ʵ����build_edge_table��ʱ��ֱ�ӷ�����������
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
	///		��ȡ�˿����ݵ�Tif�������Ƚ��ǵ�·դ�񸳳�NoData��
	///		����ֻ����դ���Ƿ���no_data��������������no_data_value���������Ϣ
	///		��ֱ���ڴ˺����м���դ�����ĵ��XY����
	/// </summary>
	template <typename TPoint>
	model::population_dataset* read_population_raster(std::string tif_file)
	{
		auto pDataSet = reinterpret_cast<GDALDataset*>(GDALOpen(tif_file.c_str(), GA_ReadOnly));

		const auto n_bands = pDataSet->GetRasterCount();
		const auto n_cols = pDataSet->GetRasterXSize();
		const auto n_rows = pDataSet->GetRasterYSize();

		if (n_bands != 1)
			throw std::exception("Input TIF file must have 1 band!");

		const auto data_type = pDataSet->GetRasterBand(1)->GetRasterDataType();
		const auto no_data_value = pDataSet->GetRasterBand(1)->GetNoDataValue();

		const auto pTif = new model::population_dataset();
		pTif->n_rows = n_rows;
		pTif->n_cols = n_cols;
		pTif->no_data_value = no_data_value;

		pDataSet->GetGeoTransform(pTif->trans);  // ��ȡ����任��6������

		const auto proj = pDataSet->GetProjectionRef();
		pTif->proj = std::string(proj);  // ��ȡ����ϵ����

		std::cout << "In function: read_population_raster, after basic read." << std::endl;

		pTif->mat = get_data_matrix(data_type, pDataSet, n_rows, n_cols, n_bands, no_data_value);  // ��ȡ���ݾ���ֻ�����Ƿ���NoData��

		std::cout << "    after get data matrix" << std::endl;

		strategy::get_center_coordinate<TPoint>(pTif);  // ���ݵ��������ͶӰ�������դ�����ĵ�XY����

		return pTif;
	}

	/// <summary>
	///		��tif�ж�ȡ���ݣ������Ƿ��ǿ��Ƶľ�������read_tif��������
	///		���˺��������ڼ򵥹�����
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

		// �������������˵���Ը����͵�����δʵ�ֶ�ȡ�������׳��쳣
		throw std::exception("No implement error: mysterious data type!");
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
			throw std::exception("Input TIF file must have 1 band!");

		const auto data_type = pDataSet->GetRasterBand(1)->GetRasterDataType();
		if (data_type != GDT_Float64)
			throw std::exception("Walk Speed Raster can only accept double type!");

		const auto pTif = new model::speed_dataset();
		pTif->n_rows = n_rows;
		pTif->n_cols = n_cols;

		pDataSet->GetGeoTransform(pTif->trans);  // ��ȡ����任��6������

		const auto proj = pDataSet->GetProjectionRef();
		pTif->proj = std::string(proj);  // ��ȡ����ϵ����

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
	///		��������д��GeoTiff
	///		�贫��������mat���˿�դ���ָ�루���ڻ�ȡ���кż�����ͶӰ��Ϣ��
	/// </summary>
	inline void write_result_to_geotiff(std::string file_name, model::matrix::matrix<double>* mat, model::population_dataset* pTif)
	{
		const auto pszFormat = "GTiff";
		const auto pDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);

		const auto pDataSet = pDriver->Create(file_name.c_str(), pTif->n_cols, pTif->n_rows, 1, GDT_Float64, nullptr);  // 1 -> 1������
		pDataSet->SetGeoTransform(pTif->trans);  // д������任��������������˿�դ��һ����
		pDataSet->SetProjection(pTif->proj.c_str());  // д��ͶӰ��Ϣ����������˿�դ��һ����

		// Ȼ��д������
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
