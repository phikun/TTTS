/******
 * 上学时间制图（C++版）：建立空间索引
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.06
 ******/

#pragma once

#include <utility>
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace ttts
{
	namespace index
	{
		/// <summary>
		///		构造指定类型的边的空间索引
		///		需要指定几何对象的类型TGeometry、R树结点的参数TStar，默认的TStar是(16, 4)
		///		空间索引的形式是(geometry, index)，要求index是int型
		/// </summary>
		template<typename TGeometry, typename TStar = boost::geometry::index::rstar<16, 4> >
		boost::geometry::index::rtree<std::pair<TGeometry, int>, TStar>* build_rtree(boost::unordered_map<int, TGeometry>* index2geometry)
		{
			// 暴力逐个插入：
			/* auto rtree = new boost::geometry::index::rtree<std::pair<TGeometry, int>, TStar>();

			for (const auto& pair : *index2geometry)
			{
				const auto index = pair.first;
				const auto geo = pair.second;

				auto value = std::make_pair(geo, index);
				rtree->insert(value);
			} */

			// 用迭代器逐个插入
			auto vec = std::vector<std::pair<TGeometry, int> >(static_cast<int>(index2geometry->size()));
			auto ptr = 0;
			for (const auto& pair : *index2geometry)
			{
				const auto index = pair.first;
				const auto geo = pair.second;
				vec[ptr++] = std::make_pair(geo, index);
			}

			const auto rtree = new boost::geometry::index::rtree<std::pair<TGeometry, int>, TStar>(vec.begin(), vec.end());
			
			return rtree;
		}
	}
}
