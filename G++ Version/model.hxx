/******
 * 上学时间制图（C++版）：空间对象模型的定义，包括：
 *   1. boost::geometry中的地理坐标点和投影左边点
 *   2. 自定义结构体【边】的定义，包括from_node, to_node, 和speed
 *   3. 栅格的数据类型
 * 2021.01.18更新：所有栅格数据集都是方的，所以population_raster的center_lng和center_lat只要一维数组即可
 * 2021.02.05更新：手工实现矩阵，而非使用cv::Mat_，并重载到()运算符
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.11
 ******/

#pragma once
#include <string>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

#include "global_define.hxx"
#include "matrix.hxx"

#define int long long

namespace ttts
{
	namespace model
	{
		typedef boost::geometry::model::point<double, 2, boost::geometry::cs::geographic<boost::geometry::degree> > point_g;  // 地理坐标的点
		typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_p;  // 投影坐标的点

		const int trans_length = 6;
		const int offset = 10;  // center_lngs和center_lats数组的buffer
		const double INF = 1E308;

		struct edge
		{
			int from_node;
			int to_node;
			double speed;

			edge(int _f = 0, int _t = 0, double _s = 0.0) : from_node(_f), to_node(_t), speed(_s) { }
		};

		struct population_dataset
		{
			int n_rows, n_cols;  // 行列数
			std::string proj;  // 坐标系定义
			double trans[trans_length];  // 坐标变换的6个参数
			double no_data_value;  // 空值
			matrix::matrix<bool>* mat;  // 数据矩阵，【这里只关心数据输入数据是否是no_data，所以直接指定bool型】
			// matrix::matrix<double>* center_lngs;  // 栅格中心点的经度或X坐标，取决于地理坐标系或投影坐标系，2021.01.18取消
			// matrix::matrix<double>* center_lats;  // 栅格中心点的纬度或Y坐标，取决于地理坐标系或投影坐标系，2021.01.18取消
			double* center_lngs;  // 2021.01.18更新：用一维数组表示栅格中心点的经纬度
			double* center_lats;  // 2021.01.18更新：用一维数组表示栅格中心点的经纬度
			
			population_dataset()
			{
				this->n_rows = this->n_cols = 0;
				this->proj = std::string("");
				for (auto& trans_ : this->trans) trans_ = 0;
				this->no_data_value = -1;
				this->mat = nullptr;
				this->center_lngs = nullptr; this->center_lats = nullptr;
			}
			~population_dataset()
			{
				delete this->mat;
				delete[] this->center_lngs;
				delete[] this->center_lats;
			}
		};

		struct speed_dataset
		{
			int n_rows, n_cols;  // 行列数
			std::string proj;  // 坐标系定义
			double trans[trans_length];  // 坐标变换的6个参数
			matrix::matrix<double>* mat;  // 数据矩阵，只能是double类型
			speed_dataset()
			{
				this->n_rows = this->n_cols = 0;
				this->proj = std::string("");
				for (auto& trans_ : this->trans) trans_ = 0;
				this->mat = nullptr;
			}
			~speed_dataset()
			{
				delete this->mat;
			}
		};
	}
}

#undef int
