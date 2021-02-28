/******
 * Global Map of Travel Time to School: 手动实现的Matrix类，代替cv::Mat_
 *   此文件中包含了matrix类的定义和实现
 *
 * Author: Chen Kuo (201711051122@mail.bnu.edu.cn)
 * Date: 2021.02.05
 ******/

#pragma once

#include <iostream>
#include <cstring>
#include <cstdlib>

#define int long long

namespace ttts { namespace model
{
	namespace matrix
	{
		template <typename T>
		class matrix
		{
		public:
			explicit matrix(int n_rows, int n_cols);
			explicit matrix(const matrix& m);
			~matrix() { delete[] this->mat_; }

			T& operator() (const int i, const int j) const { return get_element_address(i, j); }

		private:
			int n_rows_, n_cols_;
			T** mat_ = nullptr;

			T& get_element_address(int i, int j) const;

			friend std::ostream& operator<< (std::ostream& os, matrix<T>& m)
			{
				os << "Matrix: size = (" << m.n_rows_ << ", " << m.n_cols_ << ")" << std::endl;
				for (auto i = 0LL; i < m.n_rows_; ++i)
				{
					for (auto j = 0LL; j < m.n_cols_; ++j)
						os << m(i, j) << "\t";
					os << std::endl;
				}

				return os;
			}
		};

		template <typename T>
		matrix<T>::matrix(int n_rows, int n_cols)
		{
			delete[] this->mat_;

			std::cout << "New Matrix!, size = (" << n_rows << ", " << n_cols << ")" << std::endl;

			if (n_rows < 0 || n_cols < 0)
				throw std::runtime_error("Invalid range in function matrix::matrix()!");

			this->n_rows_ = n_rows;
			this->n_cols_ = n_cols;

			this->mat_ = new T*[this->n_rows_];
			for (auto i = 0LL; i < this->n_rows_; ++i)
			{
				this->mat_[i] = new (std::nothrow) T[this->n_cols_];
				if (this->mat_[i] == nullptr)
				{
					std::cout << "Wrong! i = " << i << ", NO Enough memory!" << std::endl;
					std::exit(-1);
				}

				memset(this->mat_[i], 0, sizeof(T) * this->n_cols_);
			}

			std::cout << "Here OK!" << std::endl;
		}

		template <typename T>
		matrix<T>::matrix(const matrix& m)
		{
			delete[] this->mat_;

			this->n_rows_ = m.n_rows_;
			this->n_cols_ = m.n_cols_;

			this->mat_ = new T*[this->n_rows_];
			for (auto i = 0LL; i < this->n_rows_; ++i)
			{
				this->mat_[i] = new T[this->n_cols_];
				for (auto j = 0LL; j < this->n_cols_; ++j)
					this->mat_[i][j] = m.mat_[i][j];
			}
		}

		template <typename T>
		T& matrix<T>::get_element_address(int i, int j) const
		{
			if (i < 0 || i >= this->n_rows_ || j < 0 || j >= this->n_cols_)
				throw std::runtime_error("Invalid range in matrix::get_element()!");

			return this->mat_[i][j];
		}
	}
}
}

#undef int
