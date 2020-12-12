/******
 * ��ѧʱ����ͼ��C++�棩���ռ����ģ�͵Ķ��壬������
 *   1. boost::geometry�еĵ���������ͶӰ��ߵ�
 *   2. �Զ���ṹ�塾�ߡ��Ķ��壬����from_node, to_node, ��speed
 *   3. դ�����������
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.11
 ******/

#pragma once
#include <string>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <opencv2/opencv.hpp>

namespace ttts
{
	namespace model
	{
		typedef boost::geometry::model::point<double, 2, boost::geometry::cs::geographic<boost::geometry::degree> > point_g;  // ��������ĵ�
		typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_p;  // ͶӰ����ĵ�

		const int trans_length = 6;
		const double INF = 1E308;
		
		struct edge
		{
			int from_node;
			int to_node;
			double speed;

			edge(int _f = 0, int _t = 0, double _s = 0.0) : from_node(_f), to_node(_t), speed(_s) { }
		};

		struct population_dataset
		{
			int n_rows, n_cols;  // ������
			std::string proj;  // ����ϵ����
			double trans[trans_length];  // ����任��6������
			double no_data_value;  // ��ֵ
			cv::Mat_<bool>* mat;  // ���ݾ��󣬡�����ֻ�����������������Ƿ���no_data������ֱ��ָ��bool�͡�
			cv::Mat_<double>* center_lngs;  // դ�����ĵ�ľ��Ȼ�X���꣬ȡ���ڵ�������ϵ��ͶӰ����ϵ
			cv::Mat_<double>* center_lats;  // դ�����ĵ��γ�Ȼ�Y���꣬ȡ���ڵ�������ϵ��ͶӰ����ϵ
			population_dataset()
			{
				this->n_rows = this->n_cols = 0;
				this->proj = std::string("");
				for (auto& trans_ : this->trans) trans_ = 0;
				this->no_data_value = -1;
				this->mat = nullptr;
				this->center_lngs = this->center_lats = nullptr;
			}
			~population_dataset()
			{
				delete this->mat;
				delete this->center_lngs;
				delete this->center_lats;
			}
		};

		struct speed_dataset
		{
			int n_rows, n_cols;  // ������
			std::string proj;  // ����ϵ����
			double trans[trans_length];  // ����任��6������
			cv::Mat_<double>* mat;  // ���ݾ���ֻ����double����
			speed_dataset()
			{
				this->n_rows = this->n_cols = 0;
				this->proj = std::string("");
				for (auto& trans_ : this->trans) trans_ = 0;
				this->mat = nullptr;
			}
			~speed_dataset()
			{
				delete this->mat;
			}
		};
	}
}
