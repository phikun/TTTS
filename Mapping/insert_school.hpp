/******
 * 上学时间制图（C++版）：将学校插入路网中
 * 在此文件（命名空间：ttts::strategy::solve）中包括以下函数：
 *   1. 查询到给定点最近的线段ID号
 *   2. 求直角坐标或球面坐标下的最近点
 *   3. 将学校点插入路网
 ******/

#pragma once

#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <boost/unordered_map.hpp>

#include "utility.hpp"
#include "model.hpp"
#include "index.hpp"
#include "strategy.hpp"

namespace ttts { namespace strategy
{
	namespace insert_school
	{
		/// <summary>
		///		查询最临近线段的ID号，用于插入学校
		///		返回的vector中最临近点的顺序与输入学校vector的顺序一致
		/// </summary>
		template <typename TPoint, typename TStar>
		std::vector<int>* find_nearest_segment(std::vector<TPoint>* school_points, boost::geometry::index::rtree<std::pair<boost::geometry::model::segment<TPoint>, int>, TStar>* rtree)
		{
			const auto n_schools = static_cast<int>(school_points->size());
			const auto res = new std::vector<int>(n_schools);

#			pragma omp parallel for
			for (auto i = 0; i < n_schools; ++i)
			{
				const auto school = (*school_points)[i];
				std::vector<std::pair<boost::geometry::model::segment<TPoint>, int> > query_result;

				rtree->query(boost::geometry::index::nearest(school, 1), std::back_inserter(query_result));
				const auto nearest_idx = query_result[0].second;
				(*res)[i] = nearest_idx;
			}

			return res;
		}
		
		/// <summary>
		///		根据学校坐标和最近线段ID号循环调用ttts::strategy中的寻找最近点函数，
		///		获取到每个学校最临近的道路点坐标
		/// </summary>
		template <typename TPoint>
		std::vector<TPoint>* find_nearest_points(std::vector<TPoint>* school_points, boost::unordered_map<int, boost::geometry::model::segment<TPoint> >* index2geometry_segment, std::vector<int>* nearest_segment_index)
		{
			assert(school_points->size() == nearest_segment_index->size());
			
			auto res = new std::vector<TPoint>(school_points->size());

#			pragma omp parallel for
			for (auto i = 0; i < static_cast<int>(school_points->size()); ++i)
			{
				const auto p0 = (*school_points)[i];
				const auto segment = (*index2geometry_segment)[(*nearest_segment_index)[i]];
				(*res)[i] = get_nearest_point(p0, segment);
			}

			return res;
		}

		/// <summary>
		///		将学校插到道路上，更新道路和结点的索引
		/// </summary>
		template <typename TPoint>
		std::vector<int>* update_indexes(std::vector<int>* nearest_segment_idx, std::vector<TPoint>* nearest_points, boost::unordered_map<int, model::edge>* index2edge, boost::unordered_map<std::pair<int, int>, int>* edge2index, boost::unordered_map<int, std::pair<double, double> >* index2vertex, boost::unordered_map<std::pair<double, double>, int>* vertex2index)
		{
			const auto res = new std::vector<int>(nearest_points->size());

			auto vertex_index = static_cast<int>(index2vertex->size()) + 1;  // 首个学校点的ID号是目前index2vertex中最大ID号+1
			auto edge_index = static_cast<int>(index2edge->size()) + 1;  // 首个道路点的ID号是目前index2vertex中最大ID号+1

			for (auto i = 0; i < static_cast<int>(nearest_points->size()); ++i)
			{
				const auto& vp = (*nearest_points)[i];
				const auto& x = vp.get<0>();
				const auto& y = vp.get<1>();
				
				const auto coordinate_pair = std::make_pair(x, y);
				const auto find_vertex = vertex2index->find(coordinate_pair);
				
				if (find_vertex != vertex2index->end())  // 如果最近点在道路结点中，直接返回道路结点ID
				{
					(*res)[i] = find_vertex->second;
					continue;
				}

				// 如果最近点不在道路结点中，加点加边
				index2vertex->insert(std::make_pair(vertex_index, coordinate_pair));
				vertex2index->insert(std::make_pair(coordinate_pair, vertex_index));

				const auto& vp_idx = (*nearest_segment_idx)[i];
				const auto& from_node_idx = (*index2edge)[vp_idx].from_node;
				const auto& to_node_idx = (*index2edge)[vp_idx].to_node;
				const auto& speed = (*index2edge)[vp_idx].speed;
				index2edge->insert(std::make_pair(edge_index, model::edge(vertex_index, from_node_idx, speed)));
				edge2index->insert(std::make_pair(std::make_pair(vertex_index, from_node_idx), edge_index));
				++edge_index;
				index2edge->insert(std::make_pair(edge_index, model::edge(vertex_index, to_node_idx, speed)));
				edge2index->insert(std::make_pair(std::make_pair(vertex_index, to_node_idx), edge_index));
				++edge_index;
				
				(*res)[i] = vertex_index++;  // 指定当前点的ID是学校，此后点的ID更新
			}
			
			return res;
		}
		
		/// <summary>
		///		插入学校函数，各种此步操作在该函数中汇总
		/// </summary>
		template <typename TPoint, typename TStar = boost::geometry::index::rstar<16, 4> >
		std::vector<int>* solve(const std::string& school_file, boost::unordered_map<int, model::edge>* index2edge, boost::unordered_map<std::pair<int, int>, int>* edge2index, boost::unordered_map<int, std::pair<double, double> >* index2vertex, boost::unordered_map<std::pair<double, double>, int>* vertex2index)
		{
			// 1. 读取学校shapefile，获取学校点
			const auto school_points = read_school_points<TPoint>(school_file);
			 
			// 2. 对当前道路建空间索引
			const auto index2geometry_segment = build_index_to_geometry_edge<TPoint>(index2edge, index2vertex);
			const auto rtree = index::build_rtree(index2geometry_segment);
			
			// 3. 求每个学校点最临近的道路ID，【到目前位置测试正确】
			const auto nearest_idx = find_nearest_segment(school_points, rtree);
			
			// 4. 学校点到道路的最近点
			const auto nearest_points = find_nearest_points(school_points, index2geometry_segment, nearest_idx);
			
			// 5. 最近点对的返回结果加点加边
			auto res = update_indexes(nearest_idx, nearest_points, index2edge, edge2index, index2vertex, vertex2index);

			delete nearest_points;
			delete nearest_idx;
			delete rtree;
			delete index2geometry_segment;
			delete school_points;

			return res;
		}
	}
}
}
