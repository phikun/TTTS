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
		///		将栅格中心点坐标放入vector中；并将待计算栅格行列号写入rows和lats数组中
		/// </summary>
		template <typename TPoint>
		std::vector<TPoint>* select_calculate_vector(model::population_dataset* pTif, std::vector<int>* rows, std::vector<int>* cols)
		{
			if (pTif->center_lngs == nullptr || pTif->center_lats == nullptr)
				throw std::exception("Emperor's Center Coordinates!");
			
			const auto res = new std::vector<TPoint>();
			rows->clear(); cols->clear();

			for (auto i = 0; i < pTif->n_rows; ++i)
				for (auto j = 0; j < pTif->n_cols; ++j)
				{
					if ((*pTif->mat)(i, j) == false)
						continue;

					res->push_back(TPoint((*pTif->center_lngs)(i, j), (*pTif->center_lats)(i, j)));
					rows->push_back(i);
					cols->push_back(j);
				}

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
			// auto res = mtime;
			auto res = model::INF;  // 保守估计，里面可能因为浮点数精度找不到点，因此初始时间应该很大
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
				res = std::min(res, time);
			}
			
			return res;
		}

		/// <summary>
		///		获取待计算栅格的速度
		/// </summary>
		inline std::vector<double>* get_raster_speed(model::speed_dataset* speed_raster, std::vector<int>* rows, std::vector<int>* cols)
		{
			assert(rows->size() == cols->size());

			const auto res = new std::vector<double>(static_cast<int>(rows->size()));

			for (auto i = 0; i < static_cast<int>(rows->size()); ++i)
				(*res)[i] = (*speed_raster->mat)((*rows)[i], (*cols)[i]);

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
		///		计算行列号到图中ID号的索引，用于建图
		/// </summary>
		inline int rc2id(const int& row, const int& col, const int& n_cols)
		{
			return row * n_cols + col + 1;  // 最后+1是要空出开始结点
		}

		/// <summary>
		///		计算图中ID到到行列号的索引，用于写入结果
		/// </summary>
		inline std::pair<int, int> id2rc(const int& id, const int& n_cols)
		{
			const auto tmp = id - 1;  // 一开始-1是要空出开始结点
			const auto row = tmp / n_cols;
			const auto col = tmp % n_cols;
			return std::make_pair(row, col);
		}
		
		/// <summary>
		///		对输入栅格建图，对研究区内的栅格连接其八邻域网格
		///		这里的edge是ttts::strategy::dijkstra中的edge，包括下一个结点及其通行时间
		/// </summart>
		template <typename TPoint>
		std::vector<dijkstra::edge>* build_graph(model::population_dataset* pop, model::speed_dataset* speed)
		{
			const auto n_nodes = pop->n_rows * pop->n_cols;
			const auto res = new std::vector<dijkstra::edge>[n_nodes + dijkstra::offset];
			std::cout << "n_nodes = " << n_nodes << std::endl;

			const auto n_neighbors = 8;  // 考虑八邻域
			const int di[n_neighbors] = { -1, -1, 0, 1, 1, 1, 0, -1 };  // 八邻域行号变化
			const int dj[n_neighbors] = { 0, 1, 1, 1, 0, -1, -1, -1 };  // 八邻域列号变化

			std::cout << "Before Build Graph" << std::endl;
			
#			pragma omp parallel for
			for (auto i = 0; i < pop->n_rows; ++i)
				for (auto j = 0; j < pop->n_cols; ++j)
				{	
					const auto now_id = rc2id(i, j, pop->n_cols);
					const auto p1 = TPoint((*pop->center_lngs)(i, j), (*pop->center_lats)(i, j));
					const auto speed1 = (*speed->mat)(i, j);
					
					for (auto k = 0; k < n_neighbors; ++k)
					{
						const auto ii = i + di[k];
						const auto jj = j + dj[k];
						if (ii < 0 || ii >= pop->n_rows || jj < 0 || jj >= pop->n_cols)
							continue;

						const auto next_id = rc2id(ii, jj, pop->n_cols);
						const auto p2 = TPoint((*pop->center_lngs)(ii, jj), (*pop->center_lats)(ii, jj));
						const auto speed2 = (*speed->mat)(ii, jj);
						
						const auto dist = boost::geometry::distance(p1, p2) / 2.0;
						const auto time = dist / speed1 + dist / speed2;  // 通行时间是两个栅格通行时间的平均
						
						res[now_id].emplace_back(next_id, time);
					}
				}			
			
			return res;
		}

		/// <summary>
		///		构造超级源点到所有道路点的出边，用于Dijkstra算法初始化
		/// </summary>
		inline void construct_edge_from_source_node(std::vector<dijkstra::edge>* edge_table, std::vector<double>* travel_time, std::vector<int>* center_rows, std::vector<int>* center_cols, const int& n_cols)
		{
			assert(center_rows->size() == center_cols->size());

			for (auto i = 0; i < static_cast<int>(travel_time->size()); ++i)
			{
				const auto id = rc2id((*center_rows)[i], (*center_cols)[i], n_cols);
				const auto time = (*travel_time)[i];
				edge_table[0].emplace_back(id, time);  // 直接给边表加到上学时间最短的结点
				edge_table[id].emplace_back(0, time);  // 维护双向图的本质，不加也行
			}
		}
		
		/// <summary>
		///		把Dijkstra算法的运行结果写回栅格，用于输出运行结果
		/// </summary>
		inline cv::Mat_<double>* construct_result_matrix(std::vector<double>* travel_time, const int& n_rows, const int& n_cols)
		{
			const auto n_nodes = n_rows * n_cols;
			const auto res = new cv::Mat_<double>(n_rows, n_cols);

			for (auto i = 1; i <= n_nodes; ++i)  // 0号结点是开始结点，不用管它
			{
				const auto pair_ = id2rc(i, n_cols);
				const auto row = pair_.first;
				const auto col = pair_.second;
				(*res)(row, col) = (*travel_time)[i];
			}
			
			return res;
		}
		
		/// <summary>
		///		求Travel Time函数，各种此步操作在该函数中汇总
		///		此步直接返回对栅格跑Dijkstra后的矩阵，然后包main函数中写入tif
		/// </summary>
		template <typename TPoint>
		cv::Mat_<double>* solve(model::population_dataset* population_raster,  /* 人口栅格 */
								   model::speed_dataset* speed_raster,  /* 道路速度栅格，相当于DEM */
								   boost::unordered_map<int, std::pair<double, double> >* index2vertex,  /* 用于建立临时结点 */
								   boost::unordered_map<int, model::edge>* index2edge,  /* 用于查找道路始终点ID号*/
								   std::vector<double>* vertex_time  /* 路网结点上学时间 */
								  )
		{
			// 1. 对道路顶点建立索引
			const auto index2geometry_vertex = build_index_to_geometry_point<TPoint>(index2vertex, vertex_time);
			const auto vertex_rtree = index::build_rtree(index2geometry_vertex);

			// 2. 对道路线段建索引
			const auto index2geometry_segment = build_index_to_geometry_edge<TPoint>(index2edge, index2vertex);
			const auto segment_rtree = index::build_rtree(index2geometry_segment);

			std::cout << "After build indexes" << std::endl;

			// 3. 获取需要计算的栅格中心点，这里还应该返回对应栅格中心点的经纬度
			const auto rows = new std::vector<int>();
			const auto cols = new std::vector<int>();
			const auto raster_centers = select_calculate_vector<model::point_g>(population_raster, rows, cols);

			// 4. 提取待求栅格的速度
			const auto raster_speed = get_raster_speed(speed_raster, rows, cols);

			// 5. 给出栅格在X和Y方向的半长度
			const auto dx = fabs(population_raster->trans[1]) / 2.0;
			const auto dy = fabs(population_raster->trans[5]) / 2.0;
			
			// 6. 循环计算每个栅格中心点的Travel Time
			const auto res = calculate_travel_times(raster_centers, raster_speed, vertex_rtree, segment_rtree, index2vertex, index2edge, vertex_time, dx, dy);

			std::cout << "After Calculate real Travel Time" << std::endl;
			
			// 7. 建图，再跑一次Dijkstra
			const auto edge_table = build_graph<TPoint>(population_raster, speed_raster);
			construct_edge_from_source_node(edge_table, res, rows, cols, population_raster->n_cols);
			const auto travel_times = dijkstra::calculate(population_raster->n_rows * population_raster->n_cols, edge_table);
			
			// 8. 根据Dijkstra的运行结果写回矩阵
			const auto mat = construct_result_matrix(travel_times, population_raster->n_rows, population_raster->n_cols);
			
			delete travel_times;
			delete[] edge_table;
			delete res;
			delete raster_speed;
			delete raster_centers;
			delete rows, cols;
			delete segment_rtree;
			delete index2geometry_segment;
			delete vertex_rtree;
			delete index2geometry_vertex;
			
			return mat;
		}
	}	
}
}
