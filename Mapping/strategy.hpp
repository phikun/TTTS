/******
 * ��ѧʱ����ͼ��C++�棩�������Ե�������
 * ����get_nearest_point�������Ե�������ĵ��ͶӰ����ĵ�ֱ�ʵ��
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2020.12.07
 ******/

#pragma once

#include <iostream>
#include <cmath>
#include "model.hpp"
#include <boost/geometry.hpp>

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

		// Ȼ�󻹿�����ͶӰ����ϵ�µ�����㣬���ڲ���Ҫ�����Ȳ�Ū��
	}
}
