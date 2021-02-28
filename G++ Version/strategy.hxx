/******
 * 上学时间制图（C++版）：最近点对的求解策略
 * 定义get_nearest_point函数，对地理坐标的点和投影坐标的点分别实现
 * 【如果要求平面坐标的最近点，过来改这个文件！】
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.07
 ******/

#pragma once

#include <iostream>
#include <cmath>
#include <algorithm>
#include <boost/geometry.hpp>

#include "global_define.hxx"
#include "model.hxx"

#define int long long

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
		
		// 计算几何点类，顺便当向量用, 用于求笛卡尔坐标系下的垂点
		struct point 
		{
			double x, y;
			point(double _x = 0.0, double _y = 0.0) : x(_x), y(_y) { }
			point operator+ (const point &p1) const { return point(x + p1.x, y + p1.y); }  // 向量加法
			point operator- (const point &p1) const { return point(x - p1.x, y - p1.y); }  // 向量减法
			double operator* (const point &p1) const { return (x * p1.x + y * p1.y); }  // 向量点乘
			double operator^ (const point &p1) const { return (x * p1.y - y * p1.x); }  // 向量叉乘
			double norm() { return sqrt(x * x + y * y); }  // 向量的模
		};

		/// <summary>
		///		计算几何判相等要注意精度
		/// </summary>
		template <typename T>
		int sgn(const T& x)
		{
			if (fabs(x) < eps) return 0;
			if (x > 0) return 1;
			return -1;
		}

		/// <summary>
		/// 	求笛卡尔坐标下点P0到线段P1P2的距离
		///		现在用于平面垂点和所谓的球面垂点
		/// </summary>
		point get_vertical_point_in_descartes(const point &p0, const point& p1, const point& p2) 
		{
			auto v0 = p0 - p1, v1 = p2 - p1;
			auto prj = v0 * v1 / v1.norm();
			if (sgn(prj) == -1)
				return p1;           // 若待求点在线段p1侧，交点即是p1

			v0 = p0 - p2; v1 = p1 - p2;
			prj = v0 * v1 / v1.norm();
			if (sgn(prj) == -1) 
				return p2;           // 若待求点在线段p2侧，交点即是p2

			auto dx = p1.x - p2.x, dy = p1.y - p2.y;  // 平面向量求交点，此时lambda\in[0, 1]
			auto lambda = prj / v1.norm();
			const auto vp = point(p2.x + lambda * dx, p2.y + lambda * dy);  // 垂足在线段上
			return vp;
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
		///		2021.01.03更新：放弃所谓的球面垂点，改求笛卡尔坐标下的垂点
		/// </summary>
		template <>
		inline model::point_g get_nearest_point<model::point_g>(const model::point_g& p0, const boost::geometry::model::segment<model::point_g>& segment)
		{
			const auto p1 = segment.first;
			const auto p2 = segment.second;

			const auto my_p0 = point(p0.get<0>(), p0.get<1>());
			const auto my_p1 = point(p1.get<0>(), p1.get<1>());
			const auto my_p2 = point(p2.get<0>(), p2.get<1>());

			const auto res = get_vertical_point_in_descartes(my_p0, my_p1, my_p2);
			const auto nearest_point = model::point_g(res.x, res.y);

			/* // 求角∠P0P1P2，若这个角大于等于90°，就认为P1是最近点
			const auto theta012 = sphere_angle(p0, p1, p2);
			if (sgn(theta012 - right_angle) >= 0)
				return p1;

			// 求角∠P0P2P1，若这个角大于等于90°，就认为P2是最近点
			const auto theta021 = sphere_angle(p0, p2, p1);
			if (sgn(theta021 - right_angle) >= 0)
				return p2;

			// 最近点在弧段上，用插值的办法求出最近点
			const llf dist01 = distance(p0, p1);
			const llf dist12 = distance(p1, p2);
			const llf distps = distance(p0, segment);
			const auto gamma = acos(cos(dist01 / R) / cos(distps / R));  // 球面勾股定理，求出从P1开始、向P2走需要的弧度
			const auto interp_length = fabs(gamma * R);  // 求出插值长度，保险期间取绝对值

			model::point_g nearest_point;
			line_interpolate(segment, interp_length, nearest_point, str);  // 按距离插值得到垂点 */
			
			return nearest_point;
		}

		// 【然后还可以求投影坐标系下的最近点，现在不需要，就先不弄了】

		/// <summary>
		///		给定一个tif_dataset，按照坐标变换的参数求栅格中心点的经纬度
		///		用于获取栅格中心点的get_center_coordinate函数调用
		///		2021.01.18更新：直接用一维数组记录栅格XY方向的中心经纬度，减少重复计算
		///	</summary>
		inline void calculate_center_lngs_lats(model::population_dataset* pTif)
		{
			std::cout << "Using longitude \\in [-180, 180] Version!" << std::endl;

			if (pTif == nullptr || pTif->mat == nullptr)
				throw "Unable to calculate center longitude and latitude of an emperor's TIFF!";

			if (pTif->trans[2] != 0 || pTif->trans[4] != 0)
				throw "Raster transformation matrix has a sheer parameter, please use more memory!";
			
			const auto xoffset = pTif->trans[1] / 2.0;
			const auto yoffset = pTif->trans[5] / 2.0;

			pTif->center_lngs = new double[pTif->n_cols + model::offset];
			pTif->center_lats = new double[pTif->n_rows + model::offset];

			// Calculate center longitudes
#			pragma omp parallel for num_threads(_NUM_THREADS_)
			for (auto j = 0; j < pTif->n_cols; ++j)
			{
				auto lng = pTif->trans[0] + j * pTif->trans[1] + xoffset;

				while (lng >  180.0) lng -= 360.0;  // 为防止出问题，把栅格中心点经度诱导到[-180, 180]范围内，2021.02.26新加的
				while (lng < -180.0) lng += 360.0;

				pTif->center_lngs[j] = lng;
			}

			// Calculate center latitudes
#			pragma omp parallel for num_threads(_NUM_THREADS_)
			for (auto i = 0; i < pTif->n_rows; ++i)
			{
				const auto lat = pTif->trans[3] + i * pTif->trans[5] + yoffset;
				pTif->center_lats[i] = lat;
			}
			
			/* pTif->center_lngs = new cv::Mat_<double>(pTif->n_rows, pTif->n_cols);
			pTif->center_lats = new cv::Mat_<double>(pTif->n_rows, pTif->n_cols);

#			pragma omp parallel for num_threads(_NUM_THREADS_)
			for (auto i = 0; i < pTif->n_rows; ++i)
				for (auto j = 0; j < pTif->n_cols; ++j)
				{
					const auto lng = pTif->trans[0] + j * pTif->trans[1] + i * pTif->trans[2] + xoffset;
					const auto lat = pTif->trans[3] + j * pTif->trans[4] + i * pTif->trans[5] + yoffset;
					(*pTif->center_lngs)(i, j) = lng;
					(*pTif->center_lats)(i, j) = lat;
				} */
			
		}

		/// <summary>
		///		给定一个tif_dataset，按照坐标变换的参数求栅格中心点坐标
		///		在TPoint中指定是地理坐标或投影坐标，此后做特化处理；
		/// </summary>
		template <typename TPoint>
		void get_center_coordinate(model::population_dataset* pTif)
		{
			std::cout << "Call function get_center_coordinate" << std::endl;
			throw "No implement error!";
		}

		/// <summary>
		///		给定一个tif_dataset，按照坐标变换的参数求栅格中心点坐标
		///		对地理坐标点的特化，直接调用calculate_center_lngs_lats函数即可
		///		【目前不需要完成投影坐标点的特化，那个有需要再说】
		/// </summary>
		template <>
		inline void get_center_coordinate<model::point_g>(model::population_dataset* pTif)
		{
			calculate_center_lngs_lats(pTif);
		}

		// 【然后还可以对投影坐标系下的栅格中心点做投影，现在不需要，就先不弄了】

		/// <summary>
		///		计算Travel Time时，根据最短步行距离构造查询框，分地理坐标的投影坐标特化
		///		【函数参数一会再说】
		/// </summary>
		template <typename TPoint>
		boost::geometry::model::box<TPoint> construct_query_box(const TPoint& p0, const double& mdist, const double& dx, const double& dy)
		{
			std::cout << "Call function construct_query_box" << std::endl;
			throw "No implement error!";
		}

		/// <summary>
		///		根据最短步行距离构造查询框，用于Travel Time调用
		///		【此函数是对于地理坐标的特化，保证查询框不大于栅格大小】
		/// </summary>
		template <>
		inline boost::geometry::model::box<model::point_g> construct_query_box(const model::point_g& p0, const double& mdist, const double& dx, const double& dy)
		{
			const auto x = p0.get<0>();
			const auto y = p0.get<1>();

			const auto delta_lng = std::min(mdist / R * 360.0 / (2.0 * PI), static_cast<llf>(dx));
			const auto delta_lat = std::min(mdist / R * 360.0 / (2.0 * PI), static_cast<llf>(dy));
			
			const auto min_corner = model::point_g(x - delta_lng, y - delta_lat);
			const auto max_corner = model::point_g(x + delta_lng, y + delta_lat);
			const auto query_box = boost::geometry::model::box<model::point_g>(min_corner, max_corner);
			return query_box;
		}

		/// <summary>
		///		根据最短步行距离构造查询框，用于Travel Time调用
		///		【此函数是对于投影坐标的特化，保证查询框不大于栅格大小】
		/// </summary>
		template <>
		inline boost::geometry::model::box<model::point_p> construct_query_box(const model::point_p& p0, const double& mdist, const double& dx, const double& dy)
		{
			const auto x = p0.get<0>();
			const auto y = p0.get<1>();

			const auto delta_x = std::min(mdist, dx);
			const auto delta_y = std::min(mdist, dy);

			const auto min_corner = model::point_p(x - delta_x, y - delta_y);
			const auto max_corner = model::point_p(x + delta_x, y + delta_y);
			const auto query_box = boost::geometry::model::box<model::point_p>(min_corner, max_corner);
			return query_box;
		}
	}
}

#undef int
