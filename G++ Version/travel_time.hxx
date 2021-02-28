/******
 * 上学时间制图（C++版）：计算栅格中心点上学的Travel Time
 * 2021.01.16更新：先把结果矩阵写到外存，每次用外存中读取相应变量；经测试，和内存中的结果一样，北京区域比内存的慢了20秒
 * 2021.02.05更新：自编了ttts::model::matrix::matrix<T>，不再使用cv::Mat_<T>
 * 2021.02.06更新：取消了栅格中心点速度的vector，每次直接从speed_raster中读出来
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

#include "global_define.hxx"
#include "utility.hxx"
#include "model.hxx"
#include "strategy.hxx"
#include "matrix.hxx"

#define int long long

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
				throw "Emperor's Center Coordinates!";
			
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

#			pragma omp parallel for num_threads(_NUM_THREADS_)
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
		///		把Dijkstra算法的运行结果写回栅格，用于输出运行结果
		/// </summary>
		inline model::matrix::matrix<double>* construct_result_matrix(std::vector<double>* travel_time, const int& n_rows, const int& n_cols)
		{
			const auto n_nodes = n_rows * n_cols;
			const auto res = new model::matrix::matrix<double>(n_rows, n_cols);

#			pragma omp parallel for num_threads(_NUM_THREADS_)
			for (auto i = 1LL; i <= n_nodes; ++i)  // 0号结点是开始结点，不用管它
			{
				const auto pair_ = id2rc(i, n_cols);
				const auto row = pair_.first;
				const auto col = pair_.second;
				(*res)(row, col) = (*travel_time)[i];
			}
			
			return res;
		}

		/// <summary>
		///		分块把真实的计算结果写入外存，用于外存Dijkstra算法Step1调用
		/// </summary>
		inline void outer_dijkstra_block_init(GDALDataset* p_dataset, model::population_dataset* pop, std::vector<double>* travel_time, std::vector<int>* center_rows, std::vector<int>* center_cols)
		{			
			// 1. 先都写成dijkstra::INF
			const auto p_band = p_dataset->GetRasterBand(1);
			const auto rows = pop->n_rows, cols = pop->n_cols;
			const auto arr = new double[cols];
			for (auto j = 0; j < cols; ++j) 
				arr[j] = dijkstra::INF;
			for (auto i = 0; i < rows; ++i)
				p_band->RasterIO(GF_Write, 0, i, cols, 1, arr, cols, 1, GDT_Float64, 0, 0, nullptr);
			delete[] arr;
			
			// 2. 把已知的改成已经算出来的逐个填进去【其实这样很慢！】
			const auto temp = new double[1];  // 用于RasterIO写数据的临时数组
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
		///		直接把结果存到外存的Dijkstra算法
		/// </summary>
		template <typename TPoint>
		void dijkstra_outer_memory(const std::string& output_name, model::population_dataset* pop, model::speed_dataset* speed, std::vector<double>* travel_time, std::vector<int>* center_rows, std::vector<int>* center_cols)
		{
			assert(center_rows->size() == center_cols->size());
			assert(travel_time->size() == center_rows->size());

			// 定义考虑栅格八邻域的Dijkstra算法中的全局变量
			const auto temp = new double[1];  // 每次把外存的数据copy到这里
			const auto n_vertex = pop->n_rows * pop->n_cols;
			std::vector<bool> vis(n_vertex + dijkstra::offset, false);
			std::cout << "n_vertex = " << n_vertex << std::endl;
			std::cout << "New Heap!" << std::endl;

			// 创建结果矩阵
			const auto psz_format = "GTiff";
			const auto p_driver = GetGDALDriverManager()->GetDriverByName(psz_format);
			const auto p_dataset = p_driver->Create(output_name.c_str(), pop->n_cols, pop->n_rows, 1, GDT_Float64, nullptr);  // 1 -> 1个波段
			p_dataset->SetGeoTransform(pop->trans);  // 写入坐标变换向量（和输入的人口栅格一样）
			p_dataset->SetProjection(pop->proj.c_str());  // 写入投影信息（和输入的人口栅格一样）
			const auto p_band = p_dataset->GetRasterBand(1);  // 第一个波段的指针，用它读写数据
			
			// Step1: 分块写入已有的结果，【为简单，先都写成dijkstra::INF，然后再逐个把算过的改了】
			outer_dijkstra_block_init(p_dataset, pop, travel_time, center_rows, center_cols);
			
			std::cout << "Finished Outer Dijkstra Step1: Init" << std::endl;

			// Step2: 根据计算结果初始化优先队列
			std::priority_queue<dijkstra::edge> heap;
			for (auto i = 0; i < static_cast<int>(travel_time->size()); ++i)
			{
				const auto id = rc2id((*center_rows)[i], (*center_cols)[i], pop->n_cols);
				const auto ttime = (*travel_time)[i];
				if (ttime > 1E300)
					continue;  // 2021.02.06更新：那些原本是INF的不进堆
				heap.push(dijkstra::edge(id, ttime));
			}
			
			std::cout << "Finished Outer Dijkstra Step2: Build Heap" << std::endl;

			// Step3: 用外存的Dijkstra算法
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

				// 暴力一个一个读，目前效率还可以，就先这样
				const auto n_neighbors = 8;  // 考虑八邻域
				const int di[n_neighbors] = { -1, -1, 0, 1, 1, 1, 0, -1 };  // 八邻域行号变化
				const int dj[n_neighbors] = { 0, 1, 1, 1, 0, -1, -1, -1 };  // 八邻域列号变化

				for (auto k = 0; k < n_neighbors; ++k)
				{
					const auto ii = row + di[k];
					const auto jj = col + dj[k];
					if (ii < 0 || ii >= pop->n_rows || jj < 0 || jj >= pop->n_cols)
						continue;

					// if ((*pop->mat)(ii, jj) == false)  // 若相邻栅格在研究区外，不连边
					// 	continue;
					
					const auto next_id = rc2id(ii, jj, pop->n_cols);
					// const auto p2 = TPoint((*pop->center_lngs)(ii, jj), (*pop->center_lats)(ii, jj));
					const auto p2 = TPoint(pop->center_lngs[jj], pop->center_lats[ii]);
					const auto speed2 = (*speed->mat)(ii, jj);
					const auto dist = boost::geometry::distance(p1, p2) / 2.0;
					const auto local_time = dist / speed1 + dist / speed2;  // 通行时间是两个栅格通行时间的平均
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
		///		省内存的Dijkstra，不事先建图，每次访问到某个栅格后在考虑它的八邻域
		///		在solve函数执行到这一步时，先尝试malloc(2 * sizeof(graph))的内存，若malloc成功，使用内存版本，否则用外存
		///		Inputs: pop和speed分别是人口和速度栅格，要保证其大小一致
		///		        travel_time、center_rows、center_cols是严格计算出的栅格的Travel Time，用于优先队列的初始化
		/// </summary>
		template <typename TPoint>
		std::vector<double>* dijkstra_save_memory(model::population_dataset* pop, model::speed_dataset* speed, std::vector<double>* travel_time, std::vector<int>* center_rows, std::vector<int>* center_cols)
		{
			assert(center_rows->size() == center_cols->size());
			assert(travel_time->size() == center_rows->size());

			const auto n_neighbors = 8;  // 考虑八邻域
			const int di[n_neighbors] = { -1, -1, 0, 1, 1, 1, 0, -1 };  // 八邻域行号变化
			const int dj[n_neighbors] = { 0, 1, 1, 1, 0, -1, -1, -1 };  // 八邻域列号变化

			const auto n_vertex = pop->n_rows * pop->n_cols;
			const auto time = new std::vector<double>(n_vertex + dijkstra::offset, dijkstra::INF);
			std::vector<bool> vis(n_vertex + dijkstra::offset, false);
			(*time)[0] = 0.0; vis[0] = true; // 超级源点

			std::cout << "n_vertex = " << n_vertex << std::endl;
			std::cout << "New Heap!" << std::endl;

			// Step1: 根据能到达的栅格初始化优先队列
			std::priority_queue<dijkstra::edge> heap;
			// heap.push(dijkstra::edge(0, 0.0));
			for (auto i = 0; i < static_cast<int>(travel_time->size()); ++i)
			{
				const auto id = rc2id((*center_rows)[i], (*center_cols)[i], pop->n_cols);
				const auto ttime = (*travel_time)[i];
				(*time)[id] = ttime;
				if (ttime > 1E300)
					continue;  // 2021.02.06更新：那些原本是INF的不进堆
				heap.push(dijkstra::edge(id, ttime));
			}

			// Step2: 省内存的Dijkstra算法
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

					// if ((*pop->mat)(ii, jj) == false)  // 若相邻栅格在研究区外，不连边
					// 	continue;

					const auto next_id = rc2id(ii, jj, pop->n_cols);
					// const auto p2 = TPoint((*pop->center_lngs)(ii, jj), (*pop->center_lats)(ii, jj));
					const auto p2 = TPoint(pop->center_lngs[jj], pop->center_lats[ii]);
					const auto speed2 = (*speed->mat)(ii, jj);
					const auto dist = boost::geometry::distance(p1, p2) / 2.0;
					const auto local_time = dist / speed1 + dist / speed2;  // 通行时间是两个栅格通行时间的平均
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
		///		求Travel Time函数，各种此步操作在该函数中汇总
		///		此步直接返回对栅格跑Dijkstra后的矩阵，然后包main函数中写入tif
		/// </summary>
		template <typename TPoint>
		void solve(std::string output_name,  /* 输出栅格的文件名 */
								model::population_dataset* population_raster,  /* 人口栅格 */
								model::speed_dataset* speed_raster,  /* 道路速度栅格，相当于DEM */
								boost::unordered_map<int, std::pair<double, double> >* index2vertex,  /* 用于建立临时结点 */
								boost::unordered_map<int, model::edge>* index2edge,  /* 用于查找道路始终点ID号*/
								std::vector<double>* vertex_time,  /* 路网结点上学时间 */
								bool outer_memory = false  /* 是否强制使用外存，用于检查内存外存结果是否一致 */
							   )
		{
			// 1. 对道路顶点建立索引
			const auto index2geometry_vertex = build_index_to_geometry_point<TPoint>(index2vertex, vertex_time);
			const auto vertex_rtree = index::build_rtree(index2geometry_vertex);
			delete index2geometry_vertex;

			// 2. 对道路线段建索引
			const auto index2geometry_segment = build_index_to_geometry_edge<TPoint>(index2edge, index2vertex);
			const auto segment_rtree = index::build_rtree(index2geometry_segment);
			delete index2geometry_segment;

			std::cout << "After build indexes" << std::endl;

			// 3. 获取需要计算的栅格中心点，这里还应该返回对应栅格中心点的经纬度
			const auto rows = new std::vector<int>();
			const auto cols = new std::vector<int>();
			const auto raster_centers = select_calculate_vector<model::point_g>(population_raster, rows, cols);

			// 4. 提取待求栅格的速度
			const auto raster_speed = get_raster_speed(speed_raster, rows, cols);  // 2021.02.06取消

			// 5. 给出栅格在X和Y方向的半长度
			const auto dx = fabs(population_raster->trans[1]) / 2.0;
			const auto dy = fabs(population_raster->trans[5]) / 2.0;
			
			// 6. 循环计算每个栅格中心点的Travel Time
			const auto res = calculate_travel_times(raster_centers, raster_speed, vertex_rtree, segment_rtree, index2vertex, index2edge, vertex_time, dx, dy);

			std::cout << "After Calculate real Travel Time" << std::endl;

			// 2021.01.21更新：算完之后R树就不需要了，可以给delete掉
			// 2021.01.21更新：其实index2vertex、index2edge到这一步也用不上了，先delete掉，之后把最后一次Dijkstra的单独出一个文件来做
			delete segment_rtree; delete vertex_rtree;
			delete index2vertex; delete index2edge;
			delete raster_speed;  // 2021.02.06提前：省内存，在Dijkstra之前释放
			delete raster_centers;  // 2021.02.06提前：省内存，在Dijkstra之前释放

			// 尝试malloc(2 * sizeof(graph))的内存，若malloc成功，使用【dijkstra_save_memory】函数，用内存；否则，使用【dijksrta_outer_memory】函数，用外存
			const auto n_vertex = population_raster->n_rows * population_raster->n_cols;
			const auto p = new (std::nothrow) double[2 * n_vertex];
			if (outer_memory || p == nullptr)
			{
				if (p != nullptr) delete[] p;
				
				// 用外存的Dijkstra，实际上用外存的Dijkstra算法基本没用！
				std::cout << "Using Outer Dijkstra, Please Wait . . ." << std::endl;
				dijkstra_outer_memory<TPoint>(output_name, population_raster, speed_raster, res, rows, cols);
				delete speed_raster;
			}
			else
			{
				// 用内存的Dijkstra
				delete[] p;
				std::cout << "Using Inner Dijkstra" << std::endl;
				const auto travel_times = dijkstra_save_memory<TPoint>(population_raster, speed_raster, res, rows, cols); std::cout << "Finished Last Dijkstra!" << std::endl;
				const auto mat = construct_result_matrix(travel_times, population_raster->n_rows, population_raster->n_cols);
				delete travel_times;
				
				// 事实上这里应该可以把pop_raster和walk_speed_raster释放掉了。。。
				// delete population_raster;
				delete speed_raster;
				std::cout << "After Delete Speed Raster!" << std::endl;

				write_result_to_geotiff(output_name, mat, population_raster);
				std::cout << "After Write to GeoTiff!" << std::endl;
				
				delete mat;
				// delete travel_times;
			}
			
			delete res;
			// delete raster_speed;  // 2021.02.06取消
			// delete raster_centers;  // 2021.02.06提前
			delete rows; delete cols;
			// delete segment_rtree;  // 2021.01.21修改：在最后一次Dijkstra之前把它释放掉
			// delete vertex_rtree;   // 2021.01.21修改：在最后一次Dijkstra之前把它释放掉
		}
	}	
}
}

#undef int
