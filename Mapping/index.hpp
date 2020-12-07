/******
 * ��ѧʱ����ͼ��C++�棩�������ռ�����
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.06
 ******/

#pragma once

#include "utility.hpp"
#include <boost/unordered_map.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace ttts
{
	namespace index
	{
		/// <summary>
		///		����ָ�����͵ıߵĿռ�����
		///		��Ҫָ�����ζ��������TGeometry��R�����Ĳ���TStar��Ĭ�ϵ�TStar��(16, 4)
		///		�ռ���������ʽ��(geometry, index)��Ҫ��index��int��
		/// </summary>
		template<typename TGeometry, typename TStar = boost::geometry::index::rstar<16, 4> >
		boost::geometry::index::rtree<std::pair<TGeometry, int>, TStar>* build_vertex_rtree(boost::unordered_map<int, TGeometry>* index2geometry)
		{
			auto rtree = new boost::geometry::index::rtree<std::pair<TGeometry, int>, TStar>();

			for (const auto& pair : *index2geometry)
			{
				const auto index = pair.first;
				const auto geo = pair.second;

				auto value = std::make_pair(geo, index);
				rtree->insert(value);
			}
			
			return rtree;
		}
	}
}
