/******
 * 上学时间制图（C++版）：计算栅格中心点上学的Travel Time
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.11
 ******/

#pragma once

#include <vector>
#include <utility>
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

#include "utility.hpp"
#include "model.hpp"
#include "strategy.hpp"

namespace ttts { namespace strategy
{
	namespace travel_time
	{
		constexpr double offset = 10.0;  // 开窗查询时向外扩展10m，避免浮点误差
		
		/// <summary>
		///		从tif_dataset的mat中选择需要计算的栅格（即mat值为true的栅格）
		///		将栅格中心点坐标放入vector中
		/// </summary>
		template <typename TPoint>
		std::vector<TPoint>* select_calculate_vector(model::tif_dataset* pTif)
		{
			const auto res = new std::vector<TPoint>();

			for (auto i = 0; i < pTif->n_rows; ++i)
				for (auto j = 0; j < pTif->n_cols; ++j)
					if ((*pTif->mat)(i, j) == true)
						res->push_back(TPoint((*pTif->center_lngs)(i, j), (*pTif->center_lats)(i, j)));

			return res;
		}

		/// <summary>
		///		求得一个栅格中心点上学的Travel Time，这里限制【必须在当前栅格中步行】
		/// </summary>
		template <typename TPoint, typename TStar>
		double solve_one_raster_center(const TPoint& p0,  /* 待求栅格中心点 */
									   const double& wspeed,  /* 当前栅格的步行速度，从DEM中获取 */
									   boost::geometry::index::rtree<std::pair<TPoint, int>, TStar>* vertex_rteee,  /* 道路结点的空间索引 */
									   boost::geometry::index::rtree<std::pair<boost::geometry::model::segment<TPoint>, int>, TStar>* segment_rtree,  /* 道路线段的空间索引 */
									   boost::unordered_map<int, std::pair<double, double> >* index2vertex,  /* 用于建立临时结点 */
									   boost::unordered_map<int, model::edge>* index2edge,  /* 用于查找道路始终点ID号*/
									   std::vector<double>* vertex_time,  /* 路网结点上学时间 */
									   const double& dx, const double& dy  /* 栅格在X和Y方向的半长度 */
									  )
		{
			// Step1: 找一个最近点
			std::vector<std::pair<TPoint, int> > nearest_points;
			vertex_rteee->query(boost::geometry::index::nearest(p0, 1), std::back_inserter(nearest_points));
			auto nearest_point = nearest_points[0].first;  // 到待查点最近的路网结点
			auto id = nearest_points[0].second;  // 最近路网结点的编号

			// Step2: 求该点经过最近点的上学时间，并乘以步行速度确定窗口大小，开窗查询
			const auto mtime = boost::geometry::distance(p0, nearest_point) / wspeed + (*vertex_time)[id];  // 从最近点上学的时间
			const auto mdist = mtime * wspeed + offset;  // 按最近点上学来算，最远步行长度，用于开窗查询
			const auto query_box = construct_query_box(p0, mdist, dx, dy);
			
			std::vector<std::pair<boost::geometry::model::segment<TPoint>, int> > selected_segments;
			segment_rtree->query(boost::geometry::index::intersects(query_box), std::back_inserter(selected_segments));

			// Step3: 遍历查到的线串，更新最短时间
			auto res = mtime;
			BOOST_FOREACH(const auto& segment_pair, selected_segments)
			{
				const auto seg = segment_pair.first;
				const auto idx = segment_pair.second;
				const auto edge = (*index2edge)[idx];
				
				const auto p1x = (*index2vertex)[edge.from_node].first;
				const auto p1y = (*index2vertex)[edge.from_node].second;
				const auto p2x = (*index2vertex)[edge.to_node].first;
				const auto p2y = (*index2vertex)[edge.to_node].second;

				const auto p1 = TPoint(p1x, p1y);
				const auto p2 = TPoint(p2x, p2y);
				
				auto time = boost::geometry::distance(p0, p1) / wspeed + (*vertex_time)[edge.from_node];
				res = std::min(res, time);  // 直接经过P1去学校

				time = boost::geometry::distance(p0, p2) / wspeed + (*vertex_time)[edge.to_node];
				res = std::min(res, time);  // 直接经过P2去学校

				// 然后是经过垂点去学校
				const auto vp = get_nearest_point(p0, seg);
				const auto distps = boost::geometry::distance(p0, vp);
				const auto distv1 = boost::geometry::distance(vp, p1);
				const auto distv2 = boost::geometry::distance(vp, p2);
				time = distps / wspeed + distv1 / edge.speed + (*vertex_time)[edge.from_node];
				res = std::min(res, time);
				time = distps / wspeed + distv2 / edge.speed + (*vertex_time)[edge.to_node];
				res = min(res, time);
			}
			
			return res;
		}

		/// <summary>
		///		循环计算每个栅格中心点的上学时间
		/// </summary>
		template <typename TPoint, typename TStar>
		std::vector<double>* calculate_travel_times(std::vector<TPoint>* raster_centers,  /* 待求栅格中心点的变长数组 */
													std::vector<double>* raster_speeds,  /* 当前栅格的步行速度的变长数组，从DEM中获取 */
													boost::geometry::index::rtree<std::pair<TPoint, int>, TStar>* vertex_rteee,  /* 道路结点的空间索引 */
													boost::geometry::index::rtree<std::pair<boost::geometry::model::segment<TPoint>, int>, TStar>* segment_rtree,  /* 道路线段的空间索引 */
													boost::unordered_map<int, std::pair<double, double> >* index2vertex,  /* 用于建立临时结点 */
													boost::unordered_map<int, model::edge>* index2edge,  /* 用于查找道路始终点ID号*/
													std::vector<double>* vertex_time,  /* 路网结点上学时间 */
													const double& dx, const double& dy  /* 栅格在X和Y方向的半长度 */
												   )
		{
			assert(raster_centers->size() == raster_speeds->size());
			
			const auto res = new std::vector<double>(static_cast<int>(raster_centers->size()));

#			pragma omp parallel for
			for (auto i = 0; i < static_cast<int>(raster_centers->size()); ++i)
			{
				const auto travel_time = solve_one_raster_center((*raster_centers)[i], (*raster_speeds)[i], vertex_rteee, segment_rtree, index2vertex, index2edge, vertex_time, dx, dy);
				(*res)[i] = travel_time;
			}
			
			return res;
		}
		
		/// <summary>
		///		求Travel Time函数，各种此步操作在该函数中汇总
		/// </summary>
		template <typename TPoint>
		std::vector<double>* solve(model::tif_dataset* population_raster,  /* 人口栅格 */
								   model::tif_dataset* speed_raster,  /* 道路速度栅格，相当于DEM */
								   boost::unordered_map<int, std::pair<double, double> >* index2vertex,  /* 用于建立临时结点 */
								   boost::unordered_map<int, model::edge>* index2edge,  /* 用于查找道路始终点ID号*/
								   std::vector<double>* vertex_time  /* 路网结点上学时间 */
								  )
		{
			// 1. 对道路顶点建立索引
			const auto index2geometry_vertex = build_index_to_geometry_point<TPoint>(index2vertex, vertex_time);
			const auto vertex_rtree = index::build_rtree(index2geometry_vertex);

			// 2. 对道路线段建索引
			const auto index2geometry_segment = build_index_to_geometry_edge<TPoint>(index2edge);
			const auto segment_rtree = index::build_rtree(index2geometry_segment);

			std::cout << "After build indexes" << std::endl;

			// 3. 获取需要计算的栅格中心点
			const auto raster_centers = select_calculate_vector<model::point_g>(population_raster);

			// 4. 需要计算的栅格的速度
			// 【明天再说】
			// 【这里还需要每个栅格对应的行列号，具体等把travel time的主要函数写完再说】

			// 5. 给出栅格在X和Y方向的半长度
			const auto dx = fabs(population_raster->trans[1]) / 2.0;
			const auto dy = fabs(population_raster->trans[5]) / 2.0;
			std::cout << "dx = " << dx << std::endl;
			std::cout << "dy = " << dy << std::endl;
			
			// 3. 循环计算每个栅格中心点的Travel Time
			const auto res = calculate_travel_times(raster_centers, , vertex_rtree, segment_rtree, index2vertex, index2edge, vertex_time, dx, dy);

			delete raster_centers;
			delete segment_rtree;
			delete index2geometry_segment;
			delete vertex_rtree;
			delete index2geometry_vertex;

			const auto res = new std::vector<double>();
			return res;
		}
	}	
}
}
