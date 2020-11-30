// 上学时间制图：用boost::geometry::index建空间索引加速
// 2020.10.31更新：每条边具有速度（从index2edge字典中读入，在speed[]数组中存储）
// 2020.11.02更新：增加了init函数，把所有变量置零，用于循环调用
// 2020.11.10更新：使用Python的ctypes模块导入，修改输入输出均为C支持的数组形式，而非Python列表元组字典
// 2020.11.16更新：用OpenMP做并行优化
//
// Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
// Date: 2020.11.10

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

#include <cmath>
#include <utility>
#include <unordered_map>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/foreach.hpp>

using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef long double llf;  // 球面余弦定理可能遇到两个很小的数相乘做分母，写成long double试一下

constexpr double eps = 1E-8;
constexpr double INF = 1E308;
constexpr int maxn = 6000010;  // 先假定线段最多6E6，再多了改这个文件

constexpr double wspeed = 83.0;   // 步行速度，83m/min
constexpr double offset = 100.0;  // 开窗查询是的buffer，比最长步行距离长50m
constexpr double PI = acos(-1.0);  // 圆周率，用于弧度转十进制度: *360/2pi
constexpr double R = 6371001.00;  // 地球半径: 6371km

typedef bg::model::point<double, 2, bg::cs::geographic<bg::degree> > point;  // 用经纬度表示的点的坐标
typedef bg::model::segment<point> segment;
typedef bg::model::box<point> box;
typedef pair<point, int> pvalue;  // R-Tree索引中道路结点对象
typedef pair<segment, int> svalue;  // R-Tree索引中线段对象

unordered_map<int, point> index2vertex;  // 路网结点表
unordered_map<int, double> vertexTime;  // 路网结点上学时间表
int nedges;  // 边数，即边表文件的行数
int segs[maxn], sege[maxn];  // 线段的始点和终点编号
double speed[maxn];  // 每条边的速度，用index2edge字典中读入

bgi::rtree<pvalue, bgi::rstar<16, 4>> prtree;  // 点对象的R-Tree索引
bgi::rtree<svalue, bgi::rstar<16, 4>> srtree;  // 线串对象的R-Tree索引


// 把所有变量置零，用于循环使用
void init()
{
	index2vertex.clear();
	vertexTime.clear();
	nedges = 0;
	memset(segs, 0, sizeof segs);
	memset(sege, 0, sizeof sege);
	memset(speed, 0, sizeof speed);

	prtree.clear();
	srtree.clear();
}


// 自己写的：计算几何判相等要注意精度
int sgn(double x) {
	if (fabs(x) < eps) return 0;
	if (x > 0) return 1;
	return -1;
}


// 根据传进来的index2vertex字典建立C++11的unordered_map字典
// 由ID号映射到boost::geometry::model::point【注意这里的point不是自己写的！】
// 2020.11.10更新：把上学时间的赋值也放在此函数中
// 2020.11.10更新：修改了输入参数形式: int n -> id2vert
//                              int *ids -> 结点的id号数组
//                        double *coords -> 按[(lng, lat)]组织的结点坐标的数组，块长为2
//                         doublt *times -> 结点上学时间的数组
void build_vertex_table(int n_vertext, int *ids, double *coords, double *times)
{
    const int block_size = 2;

	for (auto i = 0; i < n_vertext; ++i)
	{
		auto offset = i * block_size;
        int idx = ids[i];
		double x = coords[offset];
		double y = coords[offset + 1];
        vertexTime[idx] = times[i];  // 把结点上学时间赋值也放在这里

		if (vertexTime[idx] >= INF)
			continue;
		
		index2vertex[idx] = point(x, y);  // 把点对象插入到索引表中
		prtree.insert(make_pair(point(x, y), idx));  // 把点对象插入到空间索引中
	}
}


// 根据传进来的index2edge字典建立线段始终点数组
// 2020.10.31更新：传入字典形式为{idx: ((start, end), speed)}，把线段速度填入speed[]数组中
// 2020.11.10更新：修改了输入参数形式: int n -> 线段数量
//                            int *verts -> 按[(start, end)]组织的线段始终点的id号数组, 块长为2
//                       double * speeds -> 线段速度数组
void build_segment_table(int n_edges, int *verts, double *speeds) 
{
	const int block_size = 2;
    nedges = n_edges;

	for (auto i = 0; i < nedges; ++i)
	{
		auto offset = i * block_size;
		segs[i] = verts[offset];
		sege[i] = verts[offset + 1];
		speed[i] = speeds[i];  // 这条线段的速度，2020.10.31新加的

		// 把线串插入到空间索引中
		segment seg(index2vertex[segs[i]], index2vertex[sege[i]]);
		srtree.insert(make_pair(seg, i));
	}
}


// 自己写的，给定三点AOB，用球面余弦定理求∠AOB，单位弧度
llf sphere_angle(const point& A, const point& O, const point& B)
{
	const llf AO = bg::distance(A, O);
	const llf BO = bg::distance(B, O);
	const llf AB = bg::distance(A, B);

	const llf cos_theta = (static_cast<llf>(cos(AB / R)) - static_cast<llf>(cos(BO / R)) * static_cast<llf>(cos(AO / R))) / 
		(static_cast<llf>(sin(BO / R)) * static_cast<llf>(sin(AO / R)));
	const llf theta = acos(cos_theta);
	return theta;
}


// 给定一点，求该点上学的最短时间
// 2020.11.28更新：直接用地理坐标系求
double calculate_school_travel_time(const point &p) {	
	// Step1: 找一个最近点
	vector<pvalue> nps;
	prtree.query(bgi::nearest(p, 1), back_inserter(nps));
	auto np = nps[0].first;  // 到待查点最近的路网结点
	auto id = nps[0].second;  // 最近路网结点的编号
	auto mtime = bg::distance(p, np) / wspeed + vertexTime[id];  // 从最近点上学的时间
	auto mdist = mtime * wspeed + offset;  // 按最近点上学来算，最远步行长度，用于开窗查询

	// Step2: 求该点经过最近点的上学时间，并乘以步行速度确定窗口大小，开窗查询
	const auto theta = mdist / R * 360.0 / (2.0 * PI);
	const auto min_corner = point(p.get<0>() - theta, p.get<1>() - theta);
	const auto max_corner = point(p.get<0>() + theta, p.get<1>() + theta);
	const auto query_box = box(min_corner, max_corner);

	vector<svalue> segps;
	srtree.query(bgi::intersects(query_box), back_inserter(segps));

	// Step3: 遍历查到的线串，更新最短时间
	auto res = mtime;
	double time;
	BOOST_FOREACH(const auto& segp, segps)
	{
		auto seg = segp.first;
		auto idx = segp.second;
		auto vertex_flag = false;  // 标记最近点是否在线段端点取得
		const auto p1 = index2vertex[segs[idx]];
		const auto p2 = index2vertex[sege[idx]];

		// 直接经过P1去学校
		time = bg::distance(p, p1) / wspeed + vertexTime[segs[idx]];
		res = min(res, time);

		// 直接经过P2去学校
		time = bg::distance(p, p2) / wspeed + vertexTime[sege[idx]];
		res = min(res, time);

		// 若垂点不在弧段上，直接跳过
		/* const auto theta012 = sphere_angle(p, p1, p2);
		const auto theta021 = sphere_angle(p, p2, p1);
		if (sgn(theta012 - PI / 2.0) >= 0 || sgn(theta021 - PI / 2.0) >= 0)
			continue;

		// 垂点在弧段上，球面勾股定理and插值求垂点
		const double dist01 = bg::distance(p, p1);
		const double distps = bg::distance(p, segment(p1, p2));
		const double gamma = acos(static_cast<llf>(cos(dist01 / R)) / static_cast<llf>(cos(distps / R)));  // 球面勾股定理
		const double interp_length = gamma * R;

		point vp;
		bg::srs::spheroid<double> spheroid(R, R);
		bg::strategy::line_interpolate::geographic<bg::strategy::vincenty> str(spheroid);
		bg::line_interpolate(segment(p1, p2), interp_length, vp, str);  // 按距离插值得到垂点

		const double distv1 = bg::distance(vp, p1);
		const double distv2 = bg::distance(vp, p2);
		time = distps / wspeed + distv1 / speed[idx] + vertexTime[segs[idx]];
		res = min(res, time);
		time = distps / wspeed + distv2 / speed[idx] + vertexTime[sege[idx]];
		res = min(res, time); */
	}

	return res;
}


// 解析传入的栅格中心点列表，循环求得各点的上学时间
// 2020.11.10更新：修改了输入输出形式: int n_center_points -> 待求栅格中心点的数量
//                                      double *coords -> 按[(lng, lat)]组织的栅格中心点经纬度数组，块长为2
// 【行列号在Python里面弄，不传到C++里】
double* solve(int n_center_points, double *coords)
{
    const int block_size = 2;
	double *res = new double[n_center_points];

#   pragma omp parallel for
	for (auto i = 0; i < n_center_points; ++i)
	{
        auto offset = i * block_size;
		double x = coords[offset];
		double y = coords[offset + 1];

		auto pnt = point(x, y);
		auto dist = calculate_school_travel_time(pnt);
		res[i] = dist;
	}

	return res;
}


// 主函数
// 2020.11.10更新：修改了输入输出形式: int n_vertex -> 结点数
//                                     int *ids -> 结点的id号数组
//                               double *coords -> 按[(lng, lat)]组织的结点坐标的数组，块长为2
//                                doublt *times -> 结点上学时间的数组
//                                  int n_edges -> 边数
//                                   int *verts -> 按[(start, end)]组织的线段始终点的id号数组, 块长为2
//                              double * speeds -> 线段速度数组
//                          int n_center_points -> 待求栅格中心点的数量
//                        double *center_coords -> 按[(lng, lat)]组织的栅格中心点经纬度数组，块长为2
double* travel_time_main(int n_vertex, int *ids, double *coords, double *times, int n_edges, int *verts, double *speeds, int n_center_points, double *center_coords)
{
	init();
	build_vertex_table(n_vertex, ids, coords, times);
	build_segment_table(n_edges, verts, speeds);
	auto res = solve(n_center_points, center_coords);
	return res;
}


extern "C" 
{
    double* travel_time(int n_vertex, int *ids, double *coords, double *times, int n_edges, int *verts, double *speeds, int n_center_points, double *center_coords)
    {
        return travel_time_main(n_vertex, ids, coords, times, n_edges, verts, speeds, n_center_points, center_coords);
    }

    void dispose(double *arr) 
    {
        delete[] arr;  // prevent MEMORY LEAK!
    }
}
