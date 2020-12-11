/******
 * ��ѧʱ����ͼ��C++�棩�������Ե�������
 * ����get_nearest_point�������Ե�������ĵ��ͶӰ����ĵ�ֱ�ʵ��
 * �����Ҫ��ƽ�����������㣬����������ļ�����
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.07
 ******/

#pragma once

#include <iostream>
#include <cmath>
#include <algorithm>
#include <boost/geometry.hpp>

#include "model.hpp"

namespace ttts
{
	namespace strategy
	{
		typedef long double llf;  // �������ʱ��Ҫ�ó���������
		
		const llf PI = acos(-1.0L);
		const llf right_angle = PI / 2.0L;  // 90�� = PI/2�����������ֹ����ظ�����
		constexpr double eps = 1E-6;  // ����������
		constexpr llf R = 6371001.00L;  // ����뾶: 6371km

		// ��ֵ���������ٽ���Ҫ�õ���������
		const boost::geometry::srs::spheroid<double> spheroid(static_cast<double>(R), static_cast<double>(R));
		const boost::geometry::strategy::line_interpolate::geographic<boost::geometry::strategy::vincenty> str(spheroid);
		
		/// <summary>
		///		���㼸�������Ҫע�⾫��
		/// </summary>
		template <typename T>
		int sgn(const T& x) {
			if (fabs(x) < eps) return 0;
			if (x > 0) return 1;
			return -1;
		}

		/// <summary>
		///		��������AOB�����������Ҷ������AOB������ֵ�ĵ�λ�ǡ����ȡ�
		/// </summary>
		inline llf sphere_angle(const model::point_g& A, const model::point_g& O, const model::point_g& B)
		{
			const llf AO = distance(A, O);
			const llf BO = distance(B, O);
			const llf AB = distance(A, B);

			const auto cos_theta = (cos(AB / R) - cos(BO / R) * cos(AO / R)) / (sin(BO / R) * sin(AO / R));
			const auto theta = acos(cos_theta);
			return theta;
		}
		
		/// <summary>
		///		��������߶Σ����߶��Ͼ������������ĵ㣬����insert_school�ͼ���travel_time
		///		�����������ģ�壬�˺�Ե�������ĵ��ͶӰ����ĵ��ػ�����
		/// </summary>
		template <typename TPoint>
		TPoint get_nearest_point(const TPoint& p0, const boost::geometry::model::segment<TPoint>& segment)
		{
			std::cout << "Call function get_nearest_point" << std::endl;
			
			const auto pnt = TPoint(0.0, 0.0);
			return pnt;
		}

		/// <summary>
		///		Ѱ�ҵ㵽�߶�����㺯��get_nearest_point���ڡ���������㡿���ػ�
		///		�������ϵ�����������ĵ㲻ֹһ����������������ľ���һ���������ﷵ���׸�����Ҫ��ĵ�
		/// </summary>
		template <>
		inline model::point_g get_nearest_point<model::point_g>(const model::point_g& p0, const boost::geometry::model::segment<model::point_g>& segment)
		{
			const auto p1 = segment.first;
			const auto p2 = segment.second;

			// ��ǡ�P0P1P2��������Ǵ��ڵ���90�㣬����ΪP1�������
			const auto theta012 = sphere_angle(p0, p1, p2);
			if (sgn(theta012 - right_angle) >= 0)
				return p1;

			// ��ǡ�P0P2P1��������Ǵ��ڵ���90�㣬����ΪP2�������
			const auto theta021 = sphere_angle(p0, p2, p1);
			if (sgn(theta021 - right_angle) >= 0)
				return p2;

			// ������ڻ����ϣ��ò�ֵ�İ취��������
			const llf dist01 = distance(p0, p1);
			const llf distps = distance(p0, segment);
			const auto gamma = acos(cos(dist01 / R) / cos(distps / R));  // ���湴�ɶ��������P1��ʼ����P2����Ҫ�Ļ���
			const auto interp_length = fabs(gamma * R);  // �����ֵ���ȣ������ڼ�ȡ����ֵ

			model::point_g nearest_point;
			line_interpolate(segment, interp_length, nearest_point, str);  // �������ֵ�õ�����
			
			return nearest_point;
		}

		// ��Ȼ�󻹿�����ͶӰ����ϵ�µ�����㣬���ڲ���Ҫ�����Ȳ�Ū�ˡ�

		/// <summary>
		///		����һ��tif_dataset����������任�Ĳ�����դ�����ĵ�ľ�γ��
		///		���ڻ�ȡդ�����ĵ��get_center_coordinate��������
		///	</summary>
		inline void calculate_center_lngs_lats(model::tif_dataset* pTif)
		{
			if (pTif == nullptr || pTif->mat == nullptr)
				throw std::exception("Unable to calculate center longitude and latitude of an emperor's TIFF!");

			const auto xoffset = pTif->trans[1] / 2.0;
			const auto yoffset = pTif->trans[5] / 2.0;

			pTif->center_lngs = new cv::Mat_<double>(pTif->n_rows, pTif->n_cols);
			pTif->center_lats = new cv::Mat_<double>(pTif->n_rows, pTif->n_cols);

#			pragma omp parallel for
			for (auto i = 0; i < pTif->n_rows; ++i)
				for (auto j = 0; j < pTif->n_cols; ++j)
				{
					const auto lng = pTif->trans[0] + j * pTif->trans[1] + i * pTif->trans[2] + xoffset;
					const auto lat = pTif->trans[3] + j * pTif->trans[4] + i * pTif->trans[5] + yoffset;
					(*pTif->center_lngs)(i, j) = lng;
					(*pTif->center_lats)(i, j) = lat;
				}
			
		}

		/// <summary>
		///		����һ��tif_dataset����������任�Ĳ�����դ�����ĵ�����
		///		��TPoint��ָ���ǵ��������ͶӰ���꣬�˺����ػ�����
		/// </summary>
		template <typename TPoint>
		void get_center_coordinate(model::tif_dataset* pTif)
		{
			std::cout << "Call function get_center_coordinate" << std::endl;
			throw std::exception("No implement error!");
		}

		/// <summary>
		///		����һ��tif_dataset����������任�Ĳ�����դ�����ĵ�����
		///		�Ե����������ػ���ֱ�ӵ���calculate_center_lngs_lats��������
		///		��Ŀǰ����Ҫ���ͶӰ�������ػ����Ǹ�����Ҫ��˵��
		/// </summary>
		template <>
		inline void get_center_coordinate<model::point_g>(model::tif_dataset* pTif)
		{
			calculate_center_lngs_lats(pTif);
		}

		// ��Ȼ�󻹿��Զ�ͶӰ����ϵ�µ�դ�����ĵ���ͶӰ�����ڲ���Ҫ�����Ȳ�Ū�ˡ�

		/// <summary>
		///		����Travel Timeʱ��������̲��о��빹���ѯ�򣬷ֵ��������ͶӰ�����ػ�
		///		����������һ����˵��
		/// </summary>
		template <typename TPoint>
		boost::geometry::model::box<TPoint> construct_query_box(const TPoint& p0, const double& mdist, const double& dx, const double& dy)
		{
			std::cout << "Call function construct_query_box" << std::endl;
			throw std::exception("No implement error!");
		}

		/// <summary>
		///		������̲��о��빹���ѯ������Travel Time����
		///		���˺����Ƕ��ڵ���������ػ�����֤��ѯ�򲻴���դ���С��
		/// </summary>
		template <>
		inline boost::geometry::model::box<model::point_g> construct_query_box(const model::point_g& p0, const double& mdist, const double& dx, const double& dy)
		{
			const auto x = p0.get<0>();
			const auto y = p0.get<1>();

			const auto delta_lng = std::min(mdist / R * 360.0 / (2.0 * PI), static_cast<llf>(dx));
			const auto delta_lat = std::min(mdist / R * 360.0 / (2.0 * PI), static_cast<llf>(dy));
			
			const auto min_corner = model::point_g(x - delta_lng, y - delta_lat);
			const auto max_corner = model::point_g(x + delta_lng, y + delta_lat);
			const auto query_box = boost::geometry::model::box<model::point_g>(min_corner, max_corner);
			return query_box;
		}

		/// <summary>
		///		������̲��о��빹���ѯ������Travel Time����
		///		���˺����Ƕ���ͶӰ������ػ�����֤��ѯ�򲻴���դ���С��
		/// </summary>
		template <>
		inline boost::geometry::model::box<model::point_p> construct_query_box(const model::point_p& p0, const double& mdist, const double& dx, const double& dy)
		{
			const auto x = p0.get<0>();
			const auto y = p0.get<1>();

			const auto delta_x = std::min(mdist, dx);
			const auto delta_y = std::min(mdist, dy);

			const auto min_corner = model::point_p(x - delta_x, y - delta_y);
			const auto max_corner = model::point_p(x + delta_x, y + delta_y);
			const auto query_box = boost::geometry::model::box<model::point_p>(min_corner, max_corner);
			return query_box;
		}
	}
}
