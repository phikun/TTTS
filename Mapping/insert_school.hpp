/******
 * ��ѧʱ����ͼ��C++�棩����ѧУ����·����
 * �ڴ��ļ��������ռ䣺ttts::strategy::solve���а������º�����
 *   1. ��ѯ��������������߶�ID��
 *   2. ��ֱ����������������µ������
 *   3. ��ѧУ�����·��
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
		///		��ѯ���ٽ��߶ε�ID�ţ����ڲ���ѧУ
		///		���ص�vector�����ٽ����˳��������ѧУvector��˳��һ��
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
		///		����ѧУ���������߶�ID��ѭ������ttts::strategy�е�Ѱ������㺯����
		///		��ȡ��ÿ��ѧУ���ٽ��ĵ�·������
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
		
		// ����ѧУ���������ִ˲������ڸú����л���
		template <typename TPoint, typename TStar = boost::geometry::index::rstar<16, 4> >
		void solve(const std::string& school_file, boost::unordered_map<int, model::edge>* index2edge, boost::unordered_map<int, std::pair<double, double> >* index2vertex)
		{
			// 1. ��ȡѧУshapefile����ȡѧУ��
			const auto school_points = read_school_points<TPoint>(school_file);
			 
			// 2. �Ե�ǰ��·���ռ�����
			const auto index2geometry_segment = build_index_to_geometry_edge<TPoint>(index2edge, index2vertex);
			const auto rtree = index::build_vertex_rtree(index2geometry_segment);
			
			// 3. ��ÿ��ѧУ�����ٽ��ĵ�·ID������Ŀǰλ�ò�����ȷ��
			const auto nearest_idx = find_nearest_segment(school_points, rtree);
			
			// 4. ѧУ�㵽��·�������
			const auto nearest_points = find_nearest_points(school_points, index2geometry_segment, nearest_idx);

			// 5. �����Եķ��ؽ���ӵ�ӱ�

			delete nearest_points;
			delete nearest_idx;
			delete rtree;
			delete index2geometry_segment;
			delete school_points;
		}
	}
}
}
