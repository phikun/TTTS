/******
 * 上学时间制图（C++版）：空间对象模型的定义，包括：
 *   1. boost::geometry中的地理坐标点和投影左边点
 *   2. 自定义结构体【边】的定义，包括from_node, to_node, 和speed
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.07
 ******/

#pragma once
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

namespace ttts
{
	namespace model
	{
		typedef boost::geometry::model::point<double, 2, boost::geometry::cs::geographic<boost::geometry::degree> > point_g;  // 地理坐标的点
		typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_p;  // 投影坐标的点
		
		struct edge
		{
			int from_node;
			int to_node;
			double speed;

			edge(int _f = 0, int _t = 0, double _s = 0.0) : from_node(_f), to_node(_t), speed(_s) { }
		};
	}
}
