/******
 * 上学时间制图（C++版）：最近点对的求解策略
 * 定义get_nearest_point函数，对地理坐标的点和投影坐标的点分别实现
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.07
 ******/

#pragma once

#include <iostream>
#include <cmath>
#include "model.hpp"
#include <boost/geometry.hpp>

namespace ttts
{
	namespace strategy
	{
		typedef long double llf;  // 算球面角时需要用超长浮点数
		
		const llf PI = acos(-1.0L);
		const llf right_angle = PI / 2.0L;  // 90° = PI/2，先算出来防止多次重复计算
		constexpr double eps = 1E-6;  // 浮点数精度
		constexpr llf R = 6371001.00L;  // 地球半径: 6371km

		// 插值求球面最临近点要用到的椭球定义
		const boost::geometry::srs::spheroid<double> spheroid(static_cast<double>(R), static_cast<double>(R));
		const boost::geometry::strategy::line_interpolate::geographic<boost::geometry::strategy::vincenty> str(spheroid);
		
		/// <summary>
		///		计算几何判相等要注意精度
		/// </summary>
		template <typename T>
		int sgn(const T& x) {
			if (fabs(x) < eps) return 0;
			if (x > 0) return 1;
			return -1;
		}

		/// <summary>
		///		给定三点AOB，用球面余弦定理求∠AOB，返回值的单位是【弧度】
		/// </summary>
		inline llf sphere_angle(const model::point_g& A, const model::point_g& O, const model::point_g& B)
		{
			const llf AO = distance(A, O);
			const llf BO = distance(B, O);
			const llf AB = distance(A, B);

			const auto cos_theta = (cos(AB / R) - cos(BO / R) * cos(AO / R)) / (sin(BO / R) * sin(AO / R));
			const auto theta = acos(cos_theta);
			return theta;
		}
		
		/// <summary>
		///		给定点和线段，求线段上距离给定点最近的点，用于insert_school和计算travel_time
		///		这里给出函数模板，此后对地理坐标的点和投影坐标的点特化处理
		/// </summary>
		template <typename TPoint>
		TPoint get_nearest_point(const TPoint& p0, const boost::geometry::model::segment<TPoint>& segment)
		{
			std::cout << "Call function get_nearest_point" << std::endl;
			
			const auto pnt = TPoint(0.0, 0.0);
			return pnt;
		}

		/// <summary>
		///		寻找点到线段最近点函数get_nearest_point对于【地理坐标点】的特化
		///		在球面上到给顶点最近的点不止一个（例如赤道到极点的距离一样），这里返回首个满足要求的点
		/// </summary>
		template <>
		inline model::point_g get_nearest_point<model::point_g>(const model::point_g& p0, const boost::geometry::model::segment<model::point_g>& segment)
		{
			const auto p1 = segment.first;
			const auto p2 = segment.second;

			// 求角∠P0P1P2，若这个角大于等于90°，就认为P1是最近点
			const auto theta012 = sphere_angle(p0, p1, p2);
			if (sgn(theta012 - right_angle) >= 0)
				return p1;

			// 求角∠P0P2P1，若这个角大于等于90°，就认为P2是最近点
			const auto theta021 = sphere_angle(p0, p2, p1);
			if (sgn(theta021 - right_angle) >= 0)
				return p2;

			// 最近点在弧段上，用插值的办法求出最近点
			const llf dist01 = distance(p0, p1);
			const llf distps = distance(p0, segment);
			const auto gamma = acos(cos(dist01 / R) / cos(distps / R));  // 球面勾股定理，求出从P1开始、向P2走需要的弧度
			const auto interp_length = fabs(gamma * R);  // 求出插值长度，保险期间取绝对值

			model::point_g nearest_point;
			line_interpolate(segment, interp_length, nearest_point, str);  // 按距离插值得到垂点
			
			return nearest_point;
		}

		// 然后还可以求投影坐标系下的最近点，现在不需要，就先不弄了
	}
}
