/******
 * 上学时间制图（C++版）：将学校插入路网中
 * 在此文件（命名空间：ttts::strategy::solve）中包括以下函数：
 *   1. 查询到给定点最近的线段ID号
 *   2. 求直角坐标或球面坐标下的最近点
 *   3. 将学校点插入路网
 ******/

#pragma once

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
			const auto n_schools = school_points->size();
			auto res = new std::vector<int>(n_schools);

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
		
		// 插入学校函数，各种此步操作在该函数中汇总
		template <typename TPoint, typename TStar = boost::geometry::index::rstar<16, 4> >
		void solve(const std::string& school_file, boost::unordered_map<int, model::edge>* index2edge, boost::unordered_map<int, std::pair<double, double> >* index2vertex)
		{
			// 1. 读取学校shapefile，获取学校点
			const auto school_points = read_school_points<TPoint>(school_file);
			 
			// 2. 对当前道路建空间索引
			const auto index2geometry_segment = build_index_to_geometry_edge<TPoint>(index2edge, index2vertex);
			const auto rtree = index::build_vertex_rtree(index2geometry_segment);
			
			// 3. 求每个学校点最临近的道路ID，【到目前位置测试正确】
			const auto nearest_idx = find_nearest_segment(school_points, rtree);
			
			// 4. 学校点到道路的最近点
			const auto nearest_points = find_nearest_points(school_points, index2geometry_segment, nearest_idx);

			// 5. 最近点对的返回结果加点加边

			delete nearest_points;
			delete nearest_idx;
			delete rtree;
			delete index2geometry_segment;
			delete school_points;
		}
	}
}
}
