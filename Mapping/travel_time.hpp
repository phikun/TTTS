/******
 * ��ѧʱ����ͼ��C++�棩������դ�����ĵ���ѧ��Travel Time
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
		constexpr double offset = 10.0;  // ������ѯʱ������չ10m�����⸡�����
		
		/// <summary>
		///		��tif_dataset��mat��ѡ����Ҫ�����դ�񣨼�matֵΪtrue��դ��
		///		��դ�����ĵ��������vector��
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
		///		���һ��դ�����ĵ���ѧ��Travel Time���������ơ������ڵ�ǰդ���в��С�
		/// </summary>
		template <typename TPoint, typename TStar>
		double solve_one_raster_center(const TPoint& p0,  /* ����դ�����ĵ� */
									   const double& wspeed,  /* ��ǰդ��Ĳ����ٶȣ���DEM�л�ȡ */
									   boost::geometry::index::rtree<std::pair<TPoint, int>, TStar>* vertex_rteee,  /* ��·���Ŀռ����� */
									   boost::geometry::index::rtree<std::pair<boost::geometry::model::segment<TPoint>, int>, TStar>* segment_rtree,  /* ��·�߶εĿռ����� */
									   boost::unordered_map<int, std::pair<double, double> >* index2vertex,  /* ���ڽ�����ʱ��� */
									   boost::unordered_map<int, model::edge>* index2edge,  /* ���ڲ��ҵ�·ʼ�յ�ID��*/
									   std::vector<double>* vertex_time,  /* ·�������ѧʱ�� */
									   const double& dx, const double& dy  /* դ����X��Y����İ볤�� */
									  )
		{
			// Step1: ��һ�������
			std::vector<std::pair<TPoint, int> > nearest_points;
			vertex_rteee->query(boost::geometry::index::nearest(p0, 1), std::back_inserter(nearest_points));
			auto nearest_point = nearest_points[0].first;  // ������������·�����
			auto id = nearest_points[0].second;  // ���·�����ı��

			// Step2: ��õ㾭����������ѧʱ�䣬�����Բ����ٶ�ȷ�����ڴ�С��������ѯ
			const auto mtime = boost::geometry::distance(p0, nearest_point) / wspeed + (*vertex_time)[id];  // ���������ѧ��ʱ��
			const auto mdist = mtime * wspeed + offset;  // ���������ѧ���㣬��Զ���г��ȣ����ڿ�����ѯ
			const auto query_box = construct_query_box(p0, mdist, dx, dy);
			
			std::vector<std::pair<boost::geometry::model::segment<TPoint>, int> > selected_segments;
			segment_rtree->query(boost::geometry::index::intersects(query_box), std::back_inserter(selected_segments));

			// Step3: �����鵽���ߴ����������ʱ��
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
				res = std::min(res, time);  // ֱ�Ӿ���P1ȥѧУ

				time = boost::geometry::distance(p0, p2) / wspeed + (*vertex_time)[edge.to_node];
				res = std::min(res, time);  // ֱ�Ӿ���P2ȥѧУ

				// Ȼ���Ǿ�������ȥѧУ
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
		///		ѭ������ÿ��դ�����ĵ����ѧʱ��
		/// </summary>
		template <typename TPoint, typename TStar>
		std::vector<double>* calculate_travel_times(std::vector<TPoint>* raster_centers,  /* ����դ�����ĵ�ı䳤���� */
													std::vector<double>* raster_speeds,  /* ��ǰդ��Ĳ����ٶȵı䳤���飬��DEM�л�ȡ */
													boost::geometry::index::rtree<std::pair<TPoint, int>, TStar>* vertex_rteee,  /* ��·���Ŀռ����� */
													boost::geometry::index::rtree<std::pair<boost::geometry::model::segment<TPoint>, int>, TStar>* segment_rtree,  /* ��·�߶εĿռ����� */
													boost::unordered_map<int, std::pair<double, double> >* index2vertex,  /* ���ڽ�����ʱ��� */
													boost::unordered_map<int, model::edge>* index2edge,  /* ���ڲ��ҵ�·ʼ�յ�ID��*/
													std::vector<double>* vertex_time,  /* ·�������ѧʱ�� */
													const double& dx, const double& dy  /* դ����X��Y����İ볤�� */
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
		///		��Travel Time���������ִ˲������ڸú����л���
		/// </summary>
		template <typename TPoint>
		std::vector<double>* solve(model::tif_dataset* population_raster,  /* �˿�դ�� */
								   model::tif_dataset* speed_raster,  /* ��·�ٶ�դ���൱��DEM */
								   boost::unordered_map<int, std::pair<double, double> >* index2vertex,  /* ���ڽ�����ʱ��� */
								   boost::unordered_map<int, model::edge>* index2edge,  /* ���ڲ��ҵ�·ʼ�յ�ID��*/
								   std::vector<double>* vertex_time  /* ·�������ѧʱ�� */
								  )
		{
			// 1. �Ե�·���㽨������
			const auto index2geometry_vertex = build_index_to_geometry_point<TPoint>(index2vertex, vertex_time);
			const auto vertex_rtree = index::build_rtree(index2geometry_vertex);

			// 2. �Ե�·�߶ν�����
			const auto index2geometry_segment = build_index_to_geometry_edge<TPoint>(index2edge);
			const auto segment_rtree = index::build_rtree(index2geometry_segment);

			std::cout << "After build indexes" << std::endl;

			// 3. ��ȡ��Ҫ�����դ�����ĵ�
			const auto raster_centers = select_calculate_vector<model::point_g>(population_raster);

			// 4. ��Ҫ�����դ����ٶ�
			// ��������˵��
			// �����ﻹ��Ҫÿ��դ���Ӧ�����кţ�����Ȱ�travel time����Ҫ����д����˵��

			// 5. ����դ����X��Y����İ볤��
			const auto dx = fabs(population_raster->trans[1]) / 2.0;
			const auto dy = fabs(population_raster->trans[5]) / 2.0;
			std::cout << "dx = " << dx << std::endl;
			std::cout << "dy = " << dy << std::endl;
			
			// 3. ѭ������ÿ��դ�����ĵ��Travel Time
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
