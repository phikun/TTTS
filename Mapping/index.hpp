/******
 * ��ѧʱ����ͼ��C++�棩�������ռ�����
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
		///		����ָ�����͵ıߵĿռ�����
		///		��Ҫָ�����ζ��������TGeometry��R�����Ĳ���TStar��Ĭ�ϵ�TStar��(16, 4)
		///		�ռ���������ʽ��(geometry, index)��Ҫ��index��int��
		/// </summary>
		template<typename TGeometry, typename TStar = boost::geometry::index::rstar<16, 4> >
		boost::geometry::index::rtree<std::pair<TGeometry, int>, TStar>* build_rtree(boost::unordered_map<int, TGeometry>* index2geometry)
		{
			// ����������룺
			/* auto rtree = new boost::geometry::index::rtree<std::pair<TGeometry, int>, TStar>();

			for (const auto& pair : *index2geometry)
			{
				const auto index = pair.first;
				const auto geo = pair.second;

				auto value = std::make_pair(geo, index);
				rtree->insert(value);
			} */

			// �õ������������
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
