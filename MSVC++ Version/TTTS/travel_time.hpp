/******
 * ��ѧʱ����ͼ��C++�棩������դ�����ĵ���ѧ��Travel Time
 * 2021.01.16���£��Ȱѽ������д����棬ÿ��������ж�ȡ��Ӧ�����������ԣ����ڴ��еĽ��һ��������������ڴ������20��
 * 2021.02.05���£��Ա���ttts::model::matrix::matrix<T>������ʹ��cv::Mat_<T>
 * 2021.02.06���£�ȡ����դ�����ĵ��ٶȵ�vector��ÿ��ֱ�Ӵ�speed_raster�ж�����
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.11
 ******/

#pragma once

#include <queue>
#include <vector>
#include <utility>
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

#include "global_define.hpp"
#include "utility.hpp"
#include "model.hpp"
#include "strategy.hpp"
#include "matrix.hpp"

#define int long long

namespace ttts { namespace strategy
{
	namespace travel_time
	{
		constexpr double offset = 10.0;  // ������ѯʱ������չ10m�����⸡�����

		/// <summary>
		///		��tif_dataset��mat��ѡ����Ҫ�����դ�񣨼�matֵΪtrue��դ��
		///		��դ�����ĵ��������vector�У�����������դ�����к�д��rows��lats������
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

					res->push_back(TPoint(pTif->center_lngs[j], pTif->center_lats[i]));
					rows->push_back(i);
					cols->push_back(j);
				}

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
			// auto res = mtime;
			auto res = model::INF;  // ���ع��ƣ����������Ϊ�����������Ҳ����㣬��˳�ʼʱ��Ӧ�úܴ�
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
				res = std::min(res, time);
			}

			return res;
		}

		/// <summary>
		///		��ȡ������դ����ٶ�
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

#			pragma omp parallel for num_threads(_NUM_THREADS_)
			for (auto i = 0; i < static_cast<int>(raster_centers->size()); ++i)
			{
				const auto travel_time = solve_one_raster_center((*raster_centers)[i], (*raster_speeds)[i], vertex_rteee, segment_rtree, index2vertex, index2edge, vertex_time, dx, dy);
				(*res)[i] = travel_time;
			}

			return res;
		}

		/// <summary>
		///		�������кŵ�ͼ��ID�ŵ����������ڽ�ͼ
		/// </summary>
		inline int rc2id(const int& row, const int& col, const int& n_cols)
		{
			return row * n_cols + col + 1;  // ���+1��Ҫ�ճ���ʼ���
		}

		/// <summary>
		///		����ͼ��ID�������кŵ�����������д����
		/// </summary>
		inline std::pair<int, int> id2rc(const int& id, const int& n_cols)
		{
			const auto tmp = id - 1;  // һ��ʼ-1��Ҫ�ճ���ʼ���
			const auto row = tmp / n_cols;
			const auto col = tmp % n_cols;
			return std::make_pair(row, col);
		}

		/// <summary>
		///		��Dijkstra�㷨�����н��д��դ������������н��
		/// </summary>
		inline model::matrix::matrix<double>* construct_result_matrix(std::vector<double>* travel_time, const int& n_rows, const int& n_cols)
		{
			const auto n_nodes = n_rows * n_cols;
			const auto res = new model::matrix::matrix<double>(n_rows, n_cols);

#			pragma omp parallel for num_threads(_NUM_THREADS_)
			for (auto i = 1LL; i <= n_nodes; ++i)  // 0�Ž���ǿ�ʼ��㣬���ù���
			{
				const auto pair_ = id2rc(i, n_cols);
				const auto row = pair_.first;
				const auto col = pair_.second;
				(*res)(row, col) = (*travel_time)[i];
			}

			return res;
		}

		/// <summary>
		///		�ֿ����ʵ�ļ�����д����棬�������Dijkstra�㷨Step1����
		/// </summary>
		inline void outer_dijkstra_block_init(GDALDataset* p_dataset, model::population_dataset* pop, std::vector<double>* travel_time, std::vector<int>* center_rows, std::vector<int>* center_cols)
		{
			// 1. �ȶ�д��dijkstra::INF
			const auto p_band = p_dataset->GetRasterBand(1);
			const auto rows = pop->n_rows, cols = pop->n_cols;
			const auto arr = new double[cols];
			for (auto j = 0; j < cols; ++j)
				arr[j] = dijkstra::INF;
			for (auto i = 0; i < rows; ++i)
				p_band->RasterIO(GF_Write, 0, i, cols, 1, arr, cols, 1, GDT_Float64, 0, 0, nullptr);
			delete[] arr;

			// 2. ����֪�ĸĳ��Ѿ��������������ȥ����ʵ������������
			const auto temp = new double[1];  // ����RasterIOд���ݵ���ʱ����
			for (auto i = 0LL; i < static_cast<int>(travel_time->size()); ++i)
			{
				const auto row = (*center_rows)[i];
				const auto col = (*center_cols)[i];
				temp[0] = (*travel_time)[i];
				p_band->RasterIO(GF_Write, col, row, 1, 1, temp, 1, 1, GDT_Float64, 0, 0, nullptr);
			}
			delete[] temp;

			p_dataset->FlushCache();
		}

		/// <summary>
		///		ֱ�Ӱѽ���浽����Dijkstra�㷨
		/// </summary>
		template <typename TPoint>
		void dijkstra_outer_memory(const std::string& output_name, model::population_dataset* pop, model::speed_dataset* speed, std::vector<double>* travel_time, std::vector<int>* center_rows, std::vector<int>* center_cols)
		{
			assert(center_rows->size() == center_cols->size());
			assert(travel_time->size() == center_rows->size());

			// ���忼��դ��������Dijkstra�㷨�е�ȫ�ֱ���
			const auto temp = new double[1];  // ÿ�ΰ���������copy������
			const auto n_vertex = pop->n_rows * pop->n_cols;
			std::vector<bool> vis(n_vertex + dijkstra::offset, false);
			std::cout << "n_vertex = " << n_vertex << std::endl;
			std::cout << "New Heap!" << std::endl;

			// �����������
			const auto psz_format = "GTiff";
			const auto p_driver = GetGDALDriverManager()->GetDriverByName(psz_format);
			const auto p_dataset = p_driver->Create(output_name.c_str(), pop->n_cols, pop->n_rows, 1, GDT_Float64, nullptr);  // 1 -> 1������
			p_dataset->SetGeoTransform(pop->trans);  // д������任��������������˿�դ��һ����
			p_dataset->SetProjection(pop->proj.c_str());  // д��ͶӰ��Ϣ����������˿�դ��һ����
			const auto p_band = p_dataset->GetRasterBand(1);  // ��һ�����ε�ָ�룬������д����

			// Step1: �ֿ�д�����еĽ������Ϊ�򵥣��ȶ�д��dijkstra::INF��Ȼ�������������ĸ��ˡ�
			outer_dijkstra_block_init(p_dataset, pop, travel_time, center_rows, center_cols);

			std::cout << "Finished Outer Dijkstra Step1: Init" << std::endl;

			// Step2: ���ݼ�������ʼ�����ȶ���
			std::priority_queue<dijkstra::edge> heap;
			for (auto i = 0; i < static_cast<int>(travel_time->size()); ++i)
			{
				const auto id = rc2id((*center_rows)[i], (*center_cols)[i], pop->n_cols);
				const auto ttime = (*travel_time)[i];
				if (ttime > 1E300)
					continue;  // 2021.02.06���£���Щԭ����INF�Ĳ�����
				heap.push(dijkstra::edge(id, ttime));
			}

			std::cout << "Finished Outer Dijkstra Step2: Build Heap" << std::endl;

			// Step3: ������Dijkstra�㷨
			while (!heap.empty())
			{
				const auto current_edge = heap.top(); heap.pop();
				const auto current_vertex = current_edge.to;
				const auto current_time = current_edge.weight;
				if (vis[current_vertex]) continue;
				vis[current_vertex] = true;

				const auto rc_pair = id2rc(current_vertex, pop->n_cols);
				const auto row = rc_pair.first;
				const auto col = rc_pair.second;
				// const auto p1 = TPoint((*pop->center_lngs)(row, col), (*pop->center_lats)(row, col));
				const auto p1 = TPoint(pop->center_lngs[col], pop->center_lats[row]);
				const auto speed1 = (*speed->mat)(row, col);

				// ����һ��һ������ĿǰЧ�ʻ����ԣ���������
				const auto n_neighbors = 8;  // ���ǰ�����
				const int di[n_neighbors] = { -1, -1, 0, 1, 1, 1, 0, -1 };  // �������кű仯
				const int dj[n_neighbors] = { 0, 1, 1, 1, 0, -1, -1, -1 };  // �������кű仯

				for (auto k = 0; k < n_neighbors; ++k)
				{
					const auto ii = row + di[k];
					const auto jj = col + dj[k];
					if (ii < 0 || ii >= pop->n_rows || jj < 0 || jj >= pop->n_cols)
						continue;

					// if ((*pop->mat)(ii, jj) == false)  // ������դ�����о����⣬������
					// 	continue;

					const auto next_id = rc2id(ii, jj, pop->n_cols);
					// const auto p2 = TPoint((*pop->center_lngs)(ii, jj), (*pop->center_lats)(ii, jj));
					const auto p2 = TPoint(pop->center_lngs[jj], pop->center_lats[ii]);
					const auto speed2 = (*speed->mat)(ii, jj);
					const auto dist = boost::geometry::distance(p1, p2) / 2.0;
					const auto local_time = dist / speed1 + dist / speed2;  // ͨ��ʱ��������դ��ͨ��ʱ���ƽ��
					const auto new_time = current_time + local_time;

					p_band->RasterIO(GF_Read, jj, ii, 1, 1, temp, 1, 1, GDT_Float64, 0, 0, nullptr);

					if (!vis[next_id] && new_time < temp[0])
					{
						temp[0] = new_time;
						p_band->RasterIO(GF_Write, jj, ii, 1, 1, temp, 1, 1, GDT_Float64, 0, 0, nullptr);
						heap.push(dijkstra::edge(next_id, new_time));
					}
				}
			}

			p_dataset->FlushCache();
			delete[] temp;
		}

		/// <summary>
		///		ʡ�ڴ��Dijkstra�������Ƚ�ͼ��ÿ�η��ʵ�ĳ��դ����ڿ������İ�����
		///		��solve����ִ�е���һ��ʱ���ȳ���malloc(2 * sizeof(graph))���ڴ棬��malloc�ɹ���ʹ���ڴ�汾�����������
		///		Inputs: pop��speed�ֱ����˿ں��ٶ�դ��Ҫ��֤���Сһ��
		///		        travel_time��center_rows��center_cols���ϸ�������դ���Travel Time���������ȶ��еĳ�ʼ��
		/// </summary>
		template <typename TPoint>
		std::vector<double>* dijkstra_save_memory(model::population_dataset* pop, model::speed_dataset* speed, std::vector<double>* travel_time, std::vector<int>* center_rows, std::vector<int>* center_cols)
		{
			assert(center_rows->size() == center_cols->size());
			assert(travel_time->size() == center_rows->size());

			const auto n_neighbors = 8;  // ���ǰ�����
			const int di[n_neighbors] = { -1, -1, 0, 1, 1, 1, 0, -1 };  // �������кű仯
			const int dj[n_neighbors] = { 0, 1, 1, 1, 0, -1, -1, -1 };  // �������кű仯

			const auto n_vertex = pop->n_rows * pop->n_cols;
			const auto time = new std::vector<double>(n_vertex + dijkstra::offset, dijkstra::INF);
			std::vector<bool> vis(n_vertex + dijkstra::offset, false);
			(*time)[0] = 0.0; vis[0] = true; // ����Դ��

			std::cout << "n_vertex = " << n_vertex << std::endl;
			std::cout << "New Heap!" << std::endl;

			// Step1: �����ܵ����դ���ʼ�����ȶ���
			std::priority_queue<dijkstra::edge> heap;
			// heap.push(dijkstra::edge(0, 0.0));
			for (auto i = 0; i < static_cast<int>(travel_time->size()); ++i)
			{
				const auto id = rc2id((*center_rows)[i], (*center_cols)[i], pop->n_cols);
				const auto ttime = (*travel_time)[i];
				(*time)[id] = ttime;
				if (ttime > 1E300)
					continue;  // 2021.02.06���£���Щԭ����INF�Ĳ�����
				heap.push(dijkstra::edge(id, ttime));
			}

			// Step2: ʡ�ڴ��Dijkstra�㷨
			while (!heap.empty())
			{
				const auto current_edge = heap.top(); heap.pop();
				const auto current_vertex = current_edge.to;
				if (vis[current_vertex]) continue;
				vis[current_vertex] = true;

				const auto rc_pair = id2rc(current_vertex, pop->n_cols);
				const auto row = rc_pair.first;
				const auto col = rc_pair.second;
				// const auto p1 = TPoint((*pop->center_lngs)(row, col), (*pop->center_lats)(row, col));
				const auto p1 = TPoint(pop->center_lngs[col], pop->center_lats[row]);
				const auto speed1 = (*speed->mat)(row, col);

				for (auto k = 0; k < n_neighbors; ++k)
				{
					const auto ii = row + di[k];
					const auto jj = col + dj[k];
					if (ii < 0 || ii >= pop->n_rows || jj < 0 || jj >= pop->n_cols)
						continue;

					// if ((*pop->mat)(ii, jj) == false)  // ������դ�����о����⣬������
					// 	continue;

					const auto next_id = rc2id(ii, jj, pop->n_cols);
					// const auto p2 = TPoint((*pop->center_lngs)(ii, jj), (*pop->center_lats)(ii, jj));
					const auto p2 = TPoint(pop->center_lngs[jj], pop->center_lats[ii]);
					const auto speed2 = (*speed->mat)(ii, jj);
					const auto dist = boost::geometry::distance(p1, p2) / 2.0;
					const auto local_time = dist / speed1 + dist / speed2;  // ͨ��ʱ��������դ��ͨ��ʱ���ƽ��
					const auto new_time = (*time)[current_vertex] + local_time;

					if (!vis[next_id] && new_time < (*time)[next_id])
					{
						(*time)[next_id] = new_time;
						heap.push(dijkstra::edge(next_id, new_time));
					}
				}
			}

			return time;
		}

		/// <summary>
		///		��Travel Time���������ִ˲������ڸú����л���
		///		�˲�ֱ�ӷ��ض�դ����Dijkstra��ľ���Ȼ���main������д��tif
		/// </summary>
		template <typename TPoint>
		void solve(std::string output_name,  /* ���դ����ļ��� */
			model::population_dataset* population_raster,  /* �˿�դ�� */
			model::speed_dataset* speed_raster,  /* ��·�ٶ�դ���൱��DEM */
			boost::unordered_map<int, std::pair<double, double> >* index2vertex,  /* ���ڽ�����ʱ��� */
			boost::unordered_map<int, model::edge>* index2edge,  /* ���ڲ��ҵ�·ʼ�յ�ID��*/
			std::vector<double>* vertex_time,  /* ·�������ѧʱ�� */
			bool outer_memory = false  /* �Ƿ�ǿ��ʹ����棬���ڼ���ڴ�������Ƿ�һ�� */
		)
		{
			// 1. �Ե�·���㽨������
			const auto index2geometry_vertex = build_index_to_geometry_point<TPoint>(index2vertex, vertex_time);
			const auto vertex_rtree = index::build_rtree(index2geometry_vertex);
			delete index2geometry_vertex;

			// 2. �Ե�·�߶ν�����
			const auto index2geometry_segment = build_index_to_geometry_edge<TPoint>(index2edge, index2vertex);
			const auto segment_rtree = index::build_rtree(index2geometry_segment);
			delete index2geometry_segment;

			std::cout << "After build indexes" << std::endl;

			// 3. ��ȡ��Ҫ�����դ�����ĵ㣬���ﻹӦ�÷��ض�Ӧդ�����ĵ�ľ�γ��
			const auto rows = new std::vector<int>();
			const auto cols = new std::vector<int>();
			const auto raster_centers = select_calculate_vector<model::point_g>(population_raster, rows, cols);

			// 4. ��ȡ����դ����ٶ�
			const auto raster_speed = get_raster_speed(speed_raster, rows, cols);  // 2021.02.06ȡ��

			// 5. ����դ����X��Y����İ볤��
			const auto dx = fabs(population_raster->trans[1]) / 2.0;
			const auto dy = fabs(population_raster->trans[5]) / 2.0;

			// 6. ѭ������ÿ��դ�����ĵ��Travel Time
			const auto res = calculate_travel_times(raster_centers, raster_speed, vertex_rtree, segment_rtree, index2vertex, index2edge, vertex_time, dx, dy);

			std::cout << "After Calculate real Travel Time" << std::endl;

			// 2021.01.21���£�����֮��R���Ͳ���Ҫ�ˣ����Ը�delete��
			// 2021.01.21���£���ʵindex2vertex��index2edge����һ��Ҳ�ò����ˣ���delete����֮������һ��Dijkstra�ĵ�����һ���ļ�����
			delete segment_rtree; delete vertex_rtree;
			delete index2vertex; delete index2edge;
			delete raster_speed;  // 2021.02.06��ǰ��ʡ�ڴ棬��Dijkstra֮ǰ�ͷ�
			delete raster_centers;  // 2021.02.06��ǰ��ʡ�ڴ棬��Dijkstra֮ǰ�ͷ�

			// ����malloc(2 * sizeof(graph))���ڴ棬��malloc�ɹ���ʹ�á�dijkstra_save_memory�����������ڴ棻����ʹ�á�dijksrta_outer_memory�������������
			const auto n_vertex = population_raster->n_rows * population_raster->n_cols;
			const auto p = new (std::nothrow) double[2 * n_vertex];
			if (outer_memory || p == nullptr)
			{
				if (p != nullptr) delete[] p;

				// ������Dijkstra��ʵ����������Dijkstra�㷨����û�ã�
				std::cout << "Using Outer Dijkstra, Please Wait . . ." << std::endl;
				dijkstra_outer_memory<TPoint>(output_name, population_raster, speed_raster, res, rows, cols);
				delete speed_raster;
			}
			else
			{
				// ���ڴ��Dijkstra
				delete[] p;
				std::cout << "Using Inner Dijkstra" << std::endl;
				const auto travel_times = dijkstra_save_memory<TPoint>(population_raster, speed_raster, res, rows, cols); std::cout << "Finished Last Dijkstra!" << std::endl;
				const auto mat = construct_result_matrix(travel_times, population_raster->n_rows, population_raster->n_cols);
				delete travel_times;

				// ��ʵ������Ӧ�ÿ��԰�pop_raster��walk_speed_raster�ͷŵ��ˡ�����
				// delete population_raster;
				delete speed_raster;
				std::cout << "After Delete Speed Raster!" << std::endl;

				write_result_to_geotiff(output_name, mat, population_raster);
				std::cout << "After Write to GeoTiff!" << std::endl;

				delete mat;
				// delete travel_times;
			}

			delete res;
			// delete raster_speed;  // 2021.02.06ȡ��
			// delete raster_centers;  // 2021.02.06��ǰ
			delete rows; delete cols;
			// delete segment_rtree;  // 2021.01.21�޸ģ������һ��Dijkstra֮ǰ�����ͷŵ�
			// delete vertex_rtree;   // 2021.01.21�޸ģ������һ��Dijkstra֮ǰ�����ͷŵ�
		}
	}
}
}

#undef int
