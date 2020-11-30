// 上学时间制图：把学校插入到路网中，其实线段最近就可以了！
// 2020.11.02更新：把所有变量全部置零，用于循环更新
// 2020.11.09更新：使用Python的ctypes模块导入，修改输入输出均为C支持的数组形式，而非Python列表元组字典
// 2020.11.27更新：把投影坐标系改成地理坐标系，并直接使用点到线段的距离

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

#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/index/rtree.hpp>

using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

const int maxn = 6000010;

typedef bg::model::point<double, 2, bg::cs::geographic<bg::degree> > point;  // 以度为单位的地理坐标系
typedef bg::model::segment<point> segment;
typedef pair<segment, int> value;  // R-Tree中的索引对象，包括几何形状和它的ID号

bgi::rtree<value, bgi::rstar<16, 4> > rtree;

// 把所有变量全部置零，用于多次使用
void init() 
{
	rtree.clear();
}

// 根据传进来的index2edge字典建立线串空间索引
// 2020.11.09更新：修改了输入参数形式: int n-> 线段数量
//                         int *indexes -> 线段id号数组,
//                       double *coords -> 按(lng1, lat1, lng2, lat2)组织的线段端点坐标
void build_segment_index(int n, int *indexes, double *coords) 
{
	const int block_size = 4;  // 每条线段在coords数组中占的长度

	for (auto i = 0; i < n; ++i)
	{
		auto offset = i * block_size;
		int idx = indexes[i];  // 获取线段的id号
		double x1 = coords[offset];
		double y1 = coords[offset + 1];
		double x2 = coords[offset + 2];
		double y2 = coords[offset + 3];

		auto p1 = point(x1, y1), p2 = point(x2, y2);
		auto seg = segment(p1, p2);

		rtree.insert(make_pair(seg, idx));  // 把线对象插到空间索引中
	}
}

// 空间查询获取到待查点最近线段的ID号，似乎并行失败
int get_nearest_idx(double x, double y)
{
	auto p0 = point(x, y);
	vector<value> query_results;
	rtree.query(bgi::nearest(p0, 1), back_inserter(query_results)); 
	return query_results[0].second;
}

// 求到学校最近的线段id号
// schools列表的形式是：[(lng, lat)]
// 2020.11.09更新：修改了输入输出参数形式: int n -> 学校数量
//                           double *coords -> 按(lng, lat)组织的学校坐标
// 2020.11.27更新：直接用线段的空间索引求最短
int* solve(int n, double *coords)
{
	const int block_size = 2;

	int *res = new int[n];

#   pragma omp parallel for
	for (auto i = 0; i < n; ++i)
	{
		auto offset = i * block_size;
		double x = coords[offset];
		double y = coords[offset + 1];

		res[i] = get_nearest_idx(x, y);
	}
	
	return res;
}

extern "C" 
{
	// 主函数：根据学校经纬度和线段表查询距离学校最近的线段
	int* find_nearest_segment(int n_school, double *school_coords, int n_segment, int *segment_indexes, double *segment_coords)
	{
		init();
		build_segment_index(n_segment, segment_indexes, segment_coords);
		auto res = solve(n_school, school_coords);
		return res;
	}

	void dispose(int *arr)
	{
		delete[] arr;  // prevent MEMORY LEAK!
	}
}
