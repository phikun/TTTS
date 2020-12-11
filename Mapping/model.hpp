/******
 * 上学时间制图（C++版）：空间对象模型的定义，包括：
 *   1. boost::geometry中的地理坐标点和投影左边点
 *   2. 自定义结构体【边】的定义，包括from_node, to_node, 和speed
 *   3. 栅格的数据类型
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.11
 ******/

#pragma once
#include <string>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <opencv2/opencv.hpp>

namespace ttts
{
	namespace model
	{
		typedef boost::geometry::model::point<double, 2, boost::geometry::cs::geographic<boost::geometry::degree> > point_g;  // 地理坐标的点
		typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_p;  // 投影坐标的点

		const int trans_length = 6;
		
		struct edge
		{
			int from_node;
			int to_node;
			double speed;

			edge(int _f = 0, int _t = 0, double _s = 0.0) : from_node(_f), to_node(_t), speed(_s) { }
		};

		struct tif_dataset
		{
			int n_rows, n_cols;  // 行列数
			std::string proj;  // 坐标系定义
			double trans[trans_length];  // 坐标变换的6个参数
			double no_data_value;  // 空值
			cv::Mat_<bool>* mat;  // 数据矩阵，【这里只关心数据输入数据是否是no_data，所以直接指定bool型】
		};
	}
}
