/******
 * 上学时间制图（C++版）：用Dijkstra算法求道路结点的最短上学时间
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.07
 ******/

#pragma once

#include <vector>
#include <queue>
#include <boost/unordered_map.hpp>
#include <boost/geometry.hpp>

#include "global_define.hpp"
#include "model.hpp"

#define int long long

namespace ttts { namespace strategy
{
	namespace dijkstra
	{
		constexpr int offset = 10;  // 防数组越界，给出边表多开10个元素的位置
		constexpr double INF = 1E308;  // 上学时间初值，一个很大的数

		/// <summary>
		///		在Dijkstra算法的堆中使用的边类，包括出边和权重
		///		这与ttts::model中定义的edge不同！建图时注意加命名空间区分！
		/// </summary>
		struct edge
		{
			int to;
			double weight;
			explicit edge(const int _t = 0, const double _w = 0.0) : to(_t), weight(_w) { }
		};

		inline bool operator< (const edge &e1, const edge &e2)
		{
			return e1.weight > e2.weight;
		}

		/// <summary>
		///		建图函数，包括路网结点间的双向边、及超级源点到学校点的无代价边
		///		返回std::vector<edge>*，表示vector的数组（即每个顶点的出边），用delete[]释放
		/// </summary>
		template <typename TPoint>
		std::vector<edge>* build_graph(boost::unordered_map<int, std::pair<double, double> >* index2vertex, boost::unordered_map<int, model::edge>* index2edge, std::vector<int>* school_indexes)
		{
			const auto edge_table = new std::vector<edge>[static_cast<int>(index2vertex->size()) + offset];

			// 构造双向路网
			for (const auto& edge_pair : *index2edge)
			{
				const auto& road_edge = edge_pair.second;
				const auto& u = road_edge.from_node;
				const auto& v = road_edge.to_node;
				const auto& speed = road_edge.speed;

				const auto x0 = (*index2vertex)[u].first;
				const auto y0 = (*index2vertex)[u].second;
				const auto x1 = (*index2vertex)[v].first;
				const auto y1 = (*index2vertex)[v].second;
				const auto p0 = TPoint(x0, y0);
				const auto p1 = TPoint(x1, y1);
				const auto time = boost::geometry::distance(p0, p1) / speed;

				edge_table[u].emplace_back(v, time);
				edge_table[v].emplace_back(u, time);
			}

			// 构造超级源点到所有学校的无代价边
			for (const auto& school : *school_indexes)
			{
				edge_table[0].emplace_back(school, 0.0);
				edge_table[school].emplace_back(0, 0.0);
			}

			return edge_table;
		}

		/// <summary>
		///		用Dijkstra算法求解最短路的主要函数
		///		返回std::vector<double>*，表示指向double型变长数组的指针，用delete释放
		///		2020.12.12更新：增加了初值的参数，用于travel_time直接调用
		/// </summary>
		inline std::vector<double>* calculate(const int& n_vertex, std::vector<edge>* edge_table)
		{
			std::vector<bool> vis(n_vertex + offset, false);
			const auto time = new std::vector<double>(n_vertex + offset, INF);;
			(*time)[0] = 0.0;

			std::priority_queue<edge> heap;
			heap.push(edge(0, 0.0));  // 初始加边，指向编号为0的超级源点！

			while (!heap.empty()) {
				const auto current_edge = heap.top(); heap.pop();
				const auto current_vertex = current_edge.to;
				if (vis[current_vertex]) continue;
				vis[current_vertex] = true;
				for (auto i = 0; i < static_cast<int>(edge_table[current_vertex].size()); ++i) {
					const auto next_vertex = edge_table[current_vertex][i].to;
					const auto new_time = (*time)[current_vertex] + edge_table[current_vertex][i].weight;
					if (!vis[next_vertex] && new_time < (*time)[next_vertex]) {
						(*time)[next_vertex] = new_time;
						heap.push(edge(next_vertex, new_time));
					}
				}
			}

			return time;
		}

		/// <summary>
		///		Dijkstra算法求道路结点上学时间，各种此步操作在该函数中汇总
		/// </summary>
		template <typename TPoint>
		std::vector<double>* solve(boost::unordered_map<int, std::pair<double, double> >* index2vertex, boost::unordered_map<int, model::edge>* index2edge, std::vector<int>* school_indexes)
		{
			const auto edge_table = build_graph<TPoint>(index2vertex, index2edge, school_indexes);
			const auto res = calculate(static_cast<int>(index2vertex->size()), edge_table);

			delete[] edge_table;
			return res;
		}
	}
}
}

#undef int
