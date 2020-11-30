// 上学时间制图：Dijkstra算法求最短路，用于Python脚本调用
// 输入：点数、边数、学校数、学校ID号列表、边列表（每个元素是起点编号、终点编号和时间）
// 输出：每个路网结点的上学时间字典
// 2020.11.02更新：增加了init函数，把所有变量全部置零，用于循环重复调用
// 2020.11.09更新：使用Python的ctypes模块导入，修改输入输出均为C支持的数组形式，而非Python列表元组字典
// 2020.11.27更新：修改输入参数，在此部分中用地理坐标求距离，再计算通行时间

// Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
// Date: 2020.11.27

// 神仙加速指令:
#pragma GCC optimize("O2")
#pragma GCC optimize("O3")
#pragma GCC target("avx")
#pragma GCC optimize("Ofast")
#pragma GCC optimize("inline")
#pragma GCC optimize("-fgcse")
#pragma GCC optimize("-fgcse-lm")
#pragma GCC optimize("-fipa-sra")
#pragma GCC optimize("-ftree-pre")
#pragma GCC optimize("-ftree-vrp")
#pragma GCC optimize("-fpeephole2")
#pragma GCC optimize("-ffast-math")
#pragma GCC optimize("-fsched-spec")
#pragma GCC optimize("unroll-loops")
#pragma GCC optimize("-falign-jumps")
#pragma GCC optimize("-falign-loops")
#pragma GCC optimize("-falign-labels")
#pragma GCC optimize("-fdevirtualize")
#pragma GCC optimize("-fcaller-saves")
#pragma GCC optimize("-fcrossjumping")
#pragma GCC optimize("-fthread-jumps")
#pragma GCC optimize("-funroll-loops")
#pragma GCC optimize("-freorder-blocks")
#pragma GCC optimize("-fschedule-insns")
#pragma GCC optimize("inline-functions")
#pragma GCC optimize("-ftree-tail-merge")
#pragma GCC optimize("-fschedule-insns2")
#pragma GCC optimize("-fstrict-aliasing")
#pragma GCC optimize("-fstrict-overflow")
#pragma GCC optimize("-falign-functions")
#pragma GCC optimize("-fcse-follow-jumps")
#pragma GCC optimize("-fsched-interblock")
#pragma GCC optimize("-fpartial-inlining")
#pragma GCC optimize("no-stack-protector")
#pragma GCC optimize("-freorder-functions")
#pragma GCC optimize("-findirect-inlining")
#pragma GCC optimize("-fhoist-adjacent-loads")
#pragma GCC optimize("-frerun-cse-after-loop")
#pragma GCC optimize("inline-small-functions")
#pragma GCC optimize("-finline-small-functions")
#pragma GCC optimize("-ftree-switch-conversion")
#pragma GCC optimize("-foptimize-sibling-calls")
#pragma GCC optimize("-fexpensive-optimizations")
#pragma GCC optimize("inline-functions-called-once")
#pragma GCC optimize("-fdelete-null-pointer-checks")

#include <cstring>
#include <vector>
#include <queue>
#include <unordered_map>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

using namespace std;
namespace bg = boost::geometry;

typedef bg::model::point<double, 2, bg::cs::geographic<bg::degree> > point;

struct Edge 
{
	int to;
	double weight;
	Edge(int _t = 0, double _w = 0.0) : to(_t), weight(_w) { }
};

bool operator< (const Edge &e1, const Edge &e2) 
{
	return e1.weight > e2.weight;
}

int n, m, s;  // 点数、边数、学校数
const int maxn = 6000010;  // 先假最多6E6的结点，再多的话改源代码重新编译
const double INF = 1E308;
vector<Edge> edges[maxn];
unordered_map<int, point> index2vertex;

double path[maxn];
bool vis[maxn];

// 把所有变量全部置零，用于循环中重复使用；2020.11.02更新
void init()
{
	n = m = s = 0;
	index2vertex.clear();
	for (auto i = 0; i < maxn; ++i)
		edges[i].clear();
	memset(path, 0, sizeof path);
	memset(vis, 0, sizeof vis);
}

// 2020.11.27添加：根据顶点的ID号和坐标构造顶点表
void build_vertex_table(int n_vertex, int* vidxes, double* vcoords)
{
	const int block_size = 2;

	for (auto i = 0; i < n_vertex; ++i)
	{
		auto offset = i * block_size;
		int idx = vidxes[i];
		double lng = vcoords[offset];
		double lat = vcoords[offset + 1];
		index2vertex[idx] = point(lng, lat);
	}
}

// 2020.11.09更新：修改了输入参数形式: int* _schools -> 学校结点id号的数组
//                                   int* _edges -> 各边的始终点id号，块长为2
//                              double*  _speeds -> 各边的速度
// 2020.11.27更新：输入各边的速度，用index2vertex查找顶点坐标，并在此函数中求通行时间
void build_graph(int& _v, int& _e, int& _s, int* _schools, int* _edges, double* _speeds)
{
	n = _v; m = _e; s = _s;

	// 构造超级源点到学校的无代价的边
	for (auto i = 0; i < s; ++i)
	{
		auto si = _schools[i];
		edges[0].emplace_back(si, 0.0);
		edges[si].emplace_back(0, 0.0);
	}

	// 构造路网
    const int block_size = 2;
	for (auto i = 0; i < m; ++i)
	{
		auto offset = i * block_size;
		const int u = _edges[offset];
		const int v = _edges[offset + 1];
		const double speed = _speeds[i];
		const double weight = bg::distance(index2vertex[u], index2vertex[v]) / speed;  // distance的单位是米
		edges[u].emplace_back(v, weight);  // 双向边
		edges[v].emplace_back(u, weight);
	}
}

void solve()
{
	memset(vis, 0, sizeof vis);
	for (auto i = 0; i < maxn; ++i)
		path[i] = INF;
	path[0] = 0.0;

	priority_queue<Edge> heap;
	heap.push(Edge(0, 0));  // 初始加边，指向编号为0的超级源点！
	Edge thisone;
	int now, next;
	double newpath;

	while (!heap.empty()) {
		thisone = heap.top(); heap.pop();
		now = thisone.to;
		if (vis[now]) continue;
		vis[now] = true;;
		for (int i = 0; i < static_cast<int>(edges[now].size()); ++i) {
			next = edges[now][i].to;
			newpath = path[now] + edges[now][i].weight;
			if (!vis[next] && newpath < path[next]) {
				path[next] = newpath;
				heap.push(Edge(next, newpath));
			}
		}
	}
}

// 2020.11.09更新：修改了输出参数形式: 返回自所有结点的上学时间数组，索引等到Python再建
double* build_result()
{
	double *res = new double[n + 1];
	for (auto i = 1; i <= n; ++i)
		res[i] = path[i];
	return res;
} 

extern "C" 
{
    double* dijkstra(int _v, int _e, int _s, int *schools, int *edges, double *speeds, int *vidxes, double *vcoords)
    {
        init();
		build_vertex_table(_v, vidxes, vcoords);
        build_graph(_v, _e, _s, schools, edges, speeds);
        solve();
        auto res = build_result();
        return res;
    }

    void dispose(double *arr)
    {
        delete[] arr;  // prevent MEMORY LEAK!
    }
}
