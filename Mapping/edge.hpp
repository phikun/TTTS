/******
 * ��ѧʱ����ͼ��C++�棩���ߵĶ��壬����from_node, to_node, ��speed
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.03
 ******/

#pragma once

namespace ttts
{
	namespace model
	{
		struct edge
		{
			int from_node;
			int to_node;
			double speed;

			edge(int _f = 0, int _t = 0, double _s = 0.0) : from_node(_f), to_node(_t), speed(_s) { }
		};
	}
}
