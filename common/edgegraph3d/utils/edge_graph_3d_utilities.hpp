/*  SPDX-License-Identifier: GPL-3.0-or-later

    Copyright (C) 2022  Povilas Kanapickas <povilas@radix.lt>
    Copyright (C) 2018 Andrea Bignoli (andrea.bignoli@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.
*/

#pragma once

#include <edgegraph3d/utils/globals/global_defines.hpp>
#include <edgegraph3d/sfm_data.h>
#include <edgegraph3d/types.hpp>
#include <common/vector2d.h>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace sanescan::edgegraph3d {

#define PLG_EXTENSION ".plg"

// return pair of index of min and max value
template<typename T>
std::pair<int,int> get_min_max(const std::vector<T>& vals) {
	if(vals.empty())
        throw new std::invalid_argument("Empty vector for minmax");

	int min_index = 0;
	T min = vals[0];

	int max_index = 0;
	T max = vals[0];

	for(int i= 0; i<vals.size();i++) {
		const T &v = vals[i];
		if(v < min) {
			min = v;
			min_index = i;
		}
        if(v > max) {
			max = v;
            max_index = i;
        }
	}

    return {min_index, max_index};
}

cv::Matx34f convert_glm_mat4_to_cv_Mat34(const Mat4f &glm_mat);

cv::Matx44f convert_glm_mat4_to_cv_Mat(const Mat4f &glm_mat);

inline cv::Point2f convert_glm_vec2_to_cv_Point2f(const Vec2f &glm_vec)
{
    cv::Point2f out;
    out.x = glm_vec[0];
    out.y = glm_vec[1];
    return out;
}

std::string remove_path(std::string s);
std::string remove_extension(std::string s);

std::string remove_path_and_exception(std::string s);

void convert_glm_vec2_to_cv_Mat21(const Vec2f &glm_vec, cv::Mat &out);

std::vector<cv::Mat> copy_imgs(const std::vector<cv::Mat> &imgs);

int parse_images(const char *images_folder, const SfMDataWrapper &sfm_data,
                 std::vector<cv::Mat> &imgs);

std::vector<Vec4f> convert_Vec4f_vec4(const std::vector<cv::Vec4f> &cur_edges);

void convert_from_glm_vec3_to_cv_Mat(const Vec3f &v, cv::Mat &vcv);

void convert_from_glm_mat3_to_cv_Mat3f(const Mat3f &m, cv::Mat &mcv);

std::set<int> find_points_on_both_images(const std::vector<std::set<int>> &points_on_images,
                                         int img_i, int img_j);

std::vector<std::set<int>> get_point_sets_on_images(const SfMDataWrapper &sfmd);

std::vector<Vec2f> get_vector_of_2d_coordinates_of_point_on_image(const SfMDataWrapper &sfmd, int img_id,
                                                                  const std::vector<int> &point_ids);

Vec2f get_2d_coordinates_of_point_on_image(const SfMDataWrapper &sfmd, int img_id, int point_id);

// Get color of 2D point on image using bilinear interpolation
cv::Vec3b getColorSubpix(const cv::Mat& img, cv::Point2f pt);

// Returns color of given point, by averaging color of all its 2D observations
cv::Scalar get_refpoint_mixed_color(const SfMDataWrapper &sfmd, const std::vector<cv::Mat> &imgs,
                                    ulong refpoint_id);

template<typename T>
void print_Mat(cv::Mat m) {
	for(int i= 0;i<m.rows;i++) {
		for(int j= 0;j<m.cols;j++)
            std::cout << m.at<T>(i,j) << "\t";
        std::cout << "\n";
	}
}
std::vector<cv::Vec4f> convert_vec4_Vec4f(const std::vector<Vec4f> &cur_edges);

void print_vec2(const Vec2f &p2d);

void print_vec4(const Vec4f &p4d);

void print_vector_vec2(const std::vector<Vec2f> &p2ds);

template<typename T>
void print_vector(const std::vector<T>& v) {
    std::cout << "[";
	if(v.size() > 0) {
        std::cout << v[0];
		for(int i=1;i<v.size();i++) {
            std::cout << ",";
            std::cout << v[i];
		}
	}
    std::cout << "]";
}

float avg_vec_float(const std::vector<float> &v);

void release_vector_Mat(std::vector<cv::Mat> &matvec);

std::vector<cv::Mat> copy_vector_Mat(std::vector<cv::Mat> &matvec);

bool glm_vec2_equal(const Vec2f &v1, const Vec2f &v2);
bool vec_glm_vec2_equal(const std::vector<Vec2f> &v1, const std::vector<Vec2f> &v2);

bool glm_vec3_equal(const Vec3f &v1, const Vec3f &v2);
bool vec_glm_vec3_equal(const std::vector<Vec3f> &v1, const std::vector<Vec3f> &v2);

// check if two vector of vec2 are equal but reversed
bool vec_glm_vec2_equal_inv(const std::vector<Vec2f> &v1, const std::vector<Vec2f> &v2);
bool vec_glm_vec3_equal_inv(const std::vector<Vec3f> &v1, const std::vector<Vec3f> &v2);

std::ostream &operator<< (std::ostream &out, const Vec2f &vec);
std::ostream &operator<< (std::ostream &out, const Vec3f &vec);
std::ostream &operator<< (std::ostream &out, const Vec4f &vec);

template<typename T>
std::ostream &operator<< (std::ostream &out, const std::vector<T> &vec)
{
	out << "[";
	if(vec.size() > 0) {
		out << vec[0];
		for(ulong i=1; i < vec.size(); i++)
			out << "," << vec[i];
	}
	out << "]";
	return out;
}

template<typename T1,typename T2>
std::ostream &operator<< (std::ostream &out, const std::pair<T1,T2> &s)
{
	return out << "(" << s.first << "," << s.second << ")";
}


template<typename T>
std::ostream &operator<< (std::ostream &out, const std::set<T> &s)
{
	out << "{";
	if(s.size() > 0) {
		int i= 0;
		for(const auto &e : s)
			if(i==0) {
				out << e;
				i++;
			} else
				out << "," << e;
	}
	out << "}";
	return out;
}

template<typename T>
bool is_in(const std::set<T> &s, const T &e) {
	return s.find(e) != s.end();
}

template<typename A, typename B>
std::vector<std::pair<A,B>> zip_pair(const std::vector<A> &a, const std::vector<B> &b) {
    std::vector<std::pair<A,B>> res;
    for (int i= 0; i < std::min(a.size(),b.size()); i++) {
        res.push_back(std::make_pair(a[i],b[i]));
    }
	return res;
}

// reorder A and B according to B's values
template<typename A, typename B>
void reorder_pair_of_vector(std::vector<A> &a, std::vector<B> &b) {
    std::vector<std::pair<A,B>> vab = zip_pair(a,b);

    std::sort(vab.begin(), vab.end(),
              [](const auto& a, const auto& b){ return a.second < b.second; });

	for(int i= 0; i < vab.size(); i++) {
		a[i] = vab[i].first;
		b[i] = vab[i].second;
	}
}

Mat4f get_rt4x4(const Mat3f &r, const Vec3f &t);

template<typename T>
T vec_max(const std::vector<T>& v) {
	if(v.size() == 0)
		std::invalid_argument("vec_max : zero-sized vector!");
	T maxe = v[0];
	for(const auto &e : v)
		maxe = maxe >= e ? maxe : e;

	return maxe;
}

std::ostream &operator<<(std::ostream &out, const Mat3f &m);
std::ostream &operator<<(std::ostream &out, const Mat4f &m);

template<typename T>
T** create_2D_array(unsigned long nrows, unsigned long ncols) {
	T** ptr = new T*[nrows];  // allocate pointers
	T* pool = new T[nrows * ncols];  // allocate pool
	for (unsigned i = 0; i < nrows; ++i, pool += ncols)
		ptr[i] = pool;
	return ptr;
}

template<typename T>
void delete_2D_array(T** arr) {
	delete[] arr[0];  // remove the pool
	delete[] arr;     // remove the pointers
}

void delete_2D_Mat_array(Vector2D<Mat3f>& arr, unsigned long nrows, unsigned long ncols);

float floor_or_upper_if_close(float v);
bool is_m_multiple_of_n_float(float m, float n);

std::pair<long, long> get_2dmap_cell_from_coords(float cell_dim, const Vec2f &coords);

template<typename T1>
cv::Vec<T1, 3> convert_vec3_Vec3(Vec3f &v) {
	return cv::Vec<T1, 3>(v[0],v[1],v[2]);
}

template<typename T1>
std::vector<cv::Vec<T1, 3>> convert_vectorvec3_vectorVec3(const std::vector<Vec3f> &v) {
    std::vector<cv::Vec<T1, 3>> vV;
	for(const auto &ve: v)
		vV.push_back(convert_vec3_Vec3<T1>(ve));
	return vV;
}

std::string compute_plg_path(const char *out_images_folder, const SfMDataWrapper &sfm_data, int img_id,
                             const cv::String &s);

std::string compute_img_path(char *out_images_folder, const SfMDataWrapper &sfm_data, int img_id,
                             const cv::String &s);

void write_img(char *out_images_folder, const SfMDataWrapper &sfm_data, int img_id,
               const cv::Mat &img, const cv::String &s);

int write_images(const std::string &out_images_folder, const SfMDataWrapper &sfm_data,
                 std::vector<cv::Mat> &imgs, const cv::String &s);

int write_images(const std::string &out_images_folder, const SfMDataWrapper &sfm_data,
                 std::vector<cv::Mat> &imgs);

std::ostream &operator<<(std::ostream &out,
                         const std::pair<std::vector<int>,std::vector<ulong>> &p);

} // namespace sanescan::edgegraph3d
