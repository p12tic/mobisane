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

#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>

#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <stddef.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>

#include <edgegraph3d/sfm_data.h>

#include <edgegraph3d/utils/geometric_utilities.hpp>

namespace sanescan::edgegraph3d {

void print_vec2(const Vec2f &p2d) {
    std::cout << "[" << p2d[0] << "," << p2d[1] << "]";
}

void print_vec4(const Vec4f &p4d) {
    std::cout << "[" << p4d[0] << "," << p4d[1] << "," << p4d[2] << "," << p4d[3] << "]";
}

void print_vector_vec2(const std::vector<Vec2f> &p2ds) {
    std::cout << "[";
	if(p2ds.size() > 0) {
		print_vec2(p2ds[0]);
		for(int i=1;i<p2ds.size();i++) {
		    std::cout << ",";
			print_vec2(p2ds[i]);
		}
	}
    std::cout << "]";
}

float avg_vec_float(const std::vector<float> &v) {
	if(v.empty())
		return 0;

	float avg= 0;
	for(const auto el: v)
		avg += el;
	avg /= v.size();

	return avg;
}

std::vector<cv::Mat> copy_vector_Mat(std::vector<cv::Mat> &matvec) {
    std::vector<cv::Mat> vm;
	for(auto m : matvec)
		vm.push_back(m.clone());
	return vm;
}

void release_vector_Mat(std::vector<cv::Mat> &matvec) {
	for(auto &m : matvec)
		m.release();
	matvec.clear();
}

void print_glmmat3(Mat3f m) {
	int r = 3;
	int c = 3;

    std::cout << "[";
	for(int i= 0; i < r; i++) {
        std::cout << "\t" << m(i, 0);
		for(int j=1; j < c; j++)
            std::cout << ",\t" << m(i, j);
		if(i < r-1)
		    std::cout << "\n";
	}
    std::cout << "]\n";
}

void print_glmmat4(Mat4f m) {
	int r = 4;
	int c = 4;

    std::cout << "[";
	for(int i= 0; i < r; i++) {
        std::cout << "\t" << m(i, 0);
		for(int j=1; j < c; j++)
            std::cout << ",\t" << m(i, j);
		if(i < r-1)
		    std::cout << "\n";
	}
    std::cout << "]\n";
}

std::vector<cv::Mat> copy_imgs(const std::vector<cv::Mat> &imgs) {
    std::vector<cv::Mat> res;
	for(const auto &i: imgs)
		res.push_back(i.clone());
	return res;
}

std::vector<Vec4f> convert_Vec4f_vec4(const std::vector<cv::Vec4f> &cur_edges) {
    std::vector<Vec4f> glm_edges;

	for (auto& v: cur_edges)
		glm_edges.push_back(Vec4f(v[0],v[1],v[2],v[3]));

	return glm_edges;
}

std::vector<cv::Vec4f> convert_vec4_Vec4f(const std::vector<Vec4f> &cur_edges) {
    std::vector<cv::Vec4f> edges_Vec4f;

	for (auto& e: cur_edges) {
        cv::Vec4f v(e[0],e[1],e[2],e[3]);

		edges_Vec4f.push_back(v);
	}

	return edges_Vec4f;
}

cv::Matx34f convert_glm_mat4_to_cv_Mat34(const Mat4f &glm_mat)
{
    cv::Matx34f res;
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 4; col++) {
            res(row, col) = glm_mat(row, col);
		}
	}
    return res;
}

cv::Matx44f convert_glm_mat4_to_cv_Mat(const Mat4f &glm_mat)
{
    cv::Matx44f res;
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
            res(row, col) = glm_mat(row, col);
		}
	}
    return res;
}

// http://stackoverflow.com/questions/13299409/how-to-get-the-image-pixel-at-real-locations-in-opencv
cv::Vec3b getColorSubpix(const cv::Mat& img, cv::Point2f pt)
{
    assert(!img.empty());
    assert(img.channels() == 3);

    int x = (int)pt.x;
    int y = (int)pt.y;

    int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
    int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
    int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
    int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);

    float a = pt.x - (float)x;
    float c = pt.y - (float)y;

    uchar b = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[0] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[0] * a) * (1.f - c)
                           + (img.at<cv::Vec3b>(y1, x0)[0] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[0] * a) * c);
    uchar g = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[1] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[1] * a) * (1.f - c)
                           + (img.at<cv::Vec3b>(y1, x0)[1] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[1] * a) * c);
    uchar r = (uchar)cvRound((img.at<cv::Vec3b>(y0, x0)[2] * (1.f - a) + img.at<cv::Vec3b>(y0, x1)[2] * a) * (1.f - c)
                           + (img.at<cv::Vec3b>(y1, x0)[2] * (1.f - a) + img.at<cv::Vec3b>(y1, x1)[2] * a) * c);

    return cv::Vec3b(b, g, r);
}

// Returns color of given point, by averaging color of all its 2D observations
cv::Scalar get_refpoint_mixed_color(const SfMDataWrapper &sfmd, const std::vector<cv::Mat> &imgs,
                                    ulong refpoint_id)
{
    cv::Scalar out_col(0,0,0);
    for (const auto& observation : sfmd.landmarks_[refpoint_id].observations) {
        out_col += cv::Scalar(getColorSubpix(imgs[observation.view_id],
                              convert_glm_vec2_to_cv_Point2f(observation.x)));
    }

    auto num_observations = sfmd.landmarks_[refpoint_id].observations.size();
    return cv::Scalar(out_col[0] / float(num_observations),
                      out_col[1] / float(num_observations),
                      out_col[2] / float(num_observations));
}

cv::Point2f convert_glm_vec2_to_cv_Point2f(const Vec2f &glm_vec) {
	cv::Point2f out;
	out.x = glm_vec[0];
	out.y = glm_vec[1];
	return out;
}

cv::Point3f convert_glm_vec3_to_cv_Point3f(const Vec3f &glm_vec) {
	return cv::Point3f(glm_vec[0],glm_vec[1],glm_vec[2]);
}

void convert_glm_vec2_to_cv_Point2f(const Vec2f &glm_vec, cv::Point2f &out) {
	out.x = glm_vec[0];
	out.y = glm_vec[1];
}

void convert_glm_vec2_to_cv_Mat21(const Vec2f &glm_vec, cv::Mat &out) {
	out = cv::Mat(2, 1, CV_64F);
	out.at<double>(0, 0) = glm_vec[0];
	out.at<double>(1, 0) = glm_vec[1];
}

std::string remove_path(std::string s) {
	std::size_t found = s.find_last_of("/\\");
	return s.substr(found+1);
}

std::string remove_extension(std::string s) {
	std::size_t found = s.find_last_of(".");
	return s.substr(0,found);
}


std::string remove_path_and_exception(std::string s) {
	std::size_t found = s.find_last_of("/\\");
	size_t lastindex = s.find_last_of(".");
	return s.substr(found+1,lastindex);
}

int parse_images(const char *images_folder, const SfMDataWrapper &sfm_data, std::vector<cv::Mat> &imgs) {
    cv::Mat img;
    std::string img_path;
	imgs.clear();

    for(auto camera : sfm_data.camerasList_) {
          img_path = images_folder + remove_path(camera.pathImage);
          img = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);

		  if(!img.data) {
              std::cout << ("Could not read \"" + img_path + "\"\n");
			  return -1;
		  }

		  imgs.push_back(img);
	}

	return 0;
}

std::set<int> find_points_on_both_images(const std::vector<std::set<int>> &points_on_images,
                                         int img_i, int img_j) {
    const std::set<int> &s_i = points_on_images[img_i];
    const std::set<int> &s_j = points_on_images[img_j];
    std::set<int> points_on_both;
	set_intersection(s_i.begin(),s_i.end(),s_j.begin(),s_j.end(),
		                  std::inserter(points_on_both,points_on_both.begin()));
	return points_on_both;
}

std::vector<std::set<int>> get_point_sets_on_images(const SfMDataWrapper &sfmd)
{
    std::vector<std::set<int>> points_on_images;
    std::set<int> cur_points;

    for(const auto &landmark: sfmd.landmarks_) {
		cur_points.clear();
        for (const auto& observation : landmark.observations) {
            cur_points.insert(observation.view_id);
        }
		points_on_images.push_back(cur_points);
	}

	return points_on_images;
}

Vec2f get_2d_coordinates_of_point_on_image(const SfMDataWrapper &sfmd, int img_id, int point_id)
{
    for(int i= 0; i < sfmd.landmarks_[point_id].observations.size(); i++) {
        if(sfmd.landmarks_[point_id].observations[i].view_id == img_id) {
            return sfmd.landmarks_[point_id].observations[i].x;
        }
    }

    throw std::runtime_error("Could not find coords of observaion point");
}

std::vector<Vec2f>
    get_vector_of_2d_coordinates_of_point_on_image(const SfMDataWrapper &sfmd, int img_id,
                                                   const std::vector<int> &point_ids)
{
    std::vector<Vec2f> result;

	for(const auto point_id : point_ids) {
        result.push_back(get_2d_coordinates_of_point_on_image(sfmd, img_id, point_id));
	}

	return result;
}

void convert_from_glm_vec3_to_cv_Mat(const Vec3f &v, cv::Mat &vcv) {
	vcv.at<float>(0,0)=v[0];
	vcv.at<float>(1,0)=v[1];
	vcv.at<float>(2,0)=v[2];
}

void convert_from_glm_mat3_to_cv_Mat3f(const Mat3f &m, cv::Mat &mcv) {
    mcv.at<float>(0,0)=m(0, 0);
    mcv.at<float>(0,1)=m(0, 1);
    mcv.at<float>(0,2)=m(0, 2);
    mcv.at<float>(1,0)=m(1, 0);
    mcv.at<float>(1,1)=m(1, 1);
    mcv.at<float>(1,2)=m(1, 2);
    mcv.at<float>(2,0)=m(2, 0);
    mcv.at<float>(2,1)=m(2, 1);
    mcv.at<float>(2,2)=m(2, 2);
}

bool glm_vec2_equal(const Vec2f &v1, const Vec2f &v2) {
	return v1 == v2;
}

bool vec_glm_vec2_equal(const std::vector<Vec2f> &v1, const std::vector<Vec2f> &v2) {
	if(v1.size() != v2.size())
		return false;

	for(ulong i = 0; i < v1.size(); i++)
		if(!glm_vec2_equal(v1[i],v2[i]))
			return false;

	return true;
}

bool glm_vec3_equal(const Vec3f &v1, const Vec3f &v2) {
	return v1 == v2;
}

bool vec_glm_vec3_equal(const std::vector<Vec3f> &v1, const std::vector<Vec3f> &v2) {
	if(v1.size() != v2.size())
		return false;

	for(ulong i = 0; i < v1.size(); i++)
		if(!glm_vec3_equal(v1[i],v2[i]))
			return false;

	return true;
}

// check if two vector of vec2 are equal but reversed
bool vec_glm_vec2_equal_inv(const std::vector<Vec2f> &v1, const std::vector<Vec2f> &v2) {
	if(v1.size() != v2.size())
		return false;

	for(ulong i = 0; i < v1.size(); i++)
		if(!glm_vec2_equal(v1[i],v2[v1.size()-i-1]))
			return false;

	return true;
}

bool vec_glm_vec3_equal_inv(const std::vector<Vec3f> &v1, const std::vector<Vec3f> &v2) {
	if(v1.size() != v2.size())
		return false;

	for(ulong i = 0; i < v1.size(); i++)
		if(!glm_vec3_equal(v1[i],v2[v1.size()-i-1]))
			return false;

	return true;
}

std::ostream &operator<< (std::ostream &out, const Vec2f &vec) {
    out << "("
        << vec.x() << ", " << vec.y()
        << ")";

    return out;
}

std::ostream &operator<< (std::ostream &out, const Vec3f &vec) {
    out << "("
        << vec.x() << ", " << vec.y() << ", "<< vec.z()
        << ")";

    return out;
}

std::ostream &operator<< (std::ostream &out, const Vec4f &vec) {
    out << "("
        << vec.x() << ", " << vec.y() << ", "<< vec.z() << ", "<< vec.w()
        << ")";

    return out;
}

Mat4f get_rt4x4(const Mat3f &r, const Vec3f &t) {
    Mat4f res;
    res << r(0, 0), r(0, 1), r(0, 2),t(0),
           r(1, 0), r(1, 1), r(1, 2),t(1),
           r(2, 0), r(2, 1), r(2, 2),t(2),
           0.0,	0.0,	0.0,	1.0;
    return res;
}

std::ostream &operator<<(std::ostream &out, const Mat3f &m) {
    return out << "[" << m(0, 0) << "," << m(0, 1) << "," << m(0, 2) << ";"
               << m(1, 0) << "," << m(1, 1) << "," << m(1, 2) << ";"
               << m(2, 0) << "," << m(2, 1) << "," << m(2, 2) << "]";
}

std::ostream &operator<<(std::ostream &out, const Mat4f &m) {
    return out << "[" << m(0, 0) << "," << m(0, 1) << "," << m(0, 2) << ","  << m(0, 3) << ";"
               << m(1, 0) << "," << m(1, 1) << "," << m(1, 2) << "," << m(1, 3) << ";"
               << m(2, 0) << "," << m(2, 1) << "," << m(2, 2) << "," << m(2, 3) << ";"
               << m(3, 0) << "," << m(3, 1) << "," << m(3, 2) << "," << m(3, 3) << "]";
}

#define SMALL_FLOAT_EPSILON 0.001

float floor_or_upper_if_close(float v) {
	if(ceil(v) - v < SMALL_FLOAT_EPSILON)
		return ceil(v);
	else
		return floor(v);
}

bool is_m_multiple_of_n_float(float m, float n) {
	float div = m / n;
	float mul = floor_or_upper_if_close(div) * n;
	if(abs(m - mul) < SMALL_FLOAT_EPSILON)
		return true;
	else
		return false;
}

std::pair<long, long> get_2dmap_cell_from_coords(float cell_dim, const Vec2f &coords)
{
    return {std::floor(coords.x() / cell_dim), std::floor(coords.y() / cell_dim)};
}

std::string compute_plg_path(const char *out_images_folder, const SfMDataWrapper &sfm_data, int img_id,
                             const cv::String &s) {
    return out_images_folder + s + remove_path_and_exception(sfm_data.camerasList_[img_id].pathImage) + PLG_EXTENSION;
}

std::string compute_img_path(const char *out_images_folder, const SfMDataWrapper &sfm_data, int img_id,
                             const cv::String &s) {
    return out_images_folder + s + remove_path(sfm_data.camerasList_[img_id].pathImage);
}

void write_img(const std::string &out_images_folder, const SfMDataWrapper &sfm_data, int img_id,
               const cv::Mat &img, const cv::String &s)
{
    std::string img_path = compute_img_path(out_images_folder.c_str(),sfm_data,img_id,s);
    std::cout << "Writing to " << img_path << "\n";
    cv::imwrite(img_path,img);
}

int write_images(const std::string &out_images_folder, const SfMDataWrapper &sfm_data, std::vector<cv::Mat> &imgs, const cv::String &s) {

    std::cout << "Saving images:\n";

	for(int i= 0; i < imgs.size(); i++)
		write_img(out_images_folder,sfm_data,i,imgs[i],s);

	return 0;
}

int write_images(const std::string &out_images_folder, const SfMDataWrapper &sfm_data, std::vector<cv::Mat> &imgs) {
	return write_images(out_images_folder, sfm_data, imgs, "");
}

std::ostream &operator<< (std::ostream &out, const std::pair<std::vector<int>, std::vector<ulong>> &p)
{
	out << "{" << p.first << "," << p.second << "}";
	return out;
}

} // namespace sanescan::edgegraph3d
