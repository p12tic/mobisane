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

#include <edgegraph3d/utils/drawing_utilities.hpp>

#include <opencv2/core/cvdef.h>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <stdlib.h>     /* srand, rand */
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>

#include <edgegraph3d/sfm_data.h>
#include <edgegraph3d/utils/geometric_utilities.hpp>
#include <edgegraph3d/io/input/convert_edge_images_pixel_to_segment.hpp>

namespace sanescan::edgegraph3d {

void draw_point(cv::Mat &img, const cv::Point2f &p, const cv::Scalar &color, int draw_radius) {
	if(draw_radius == 1)
        img.at<cv::Vec3b>((int) p.y,(int) p.x) = cv::Vec3b(color[0],color[1],color[2]);
	else
		cv::circle(img,p,draw_radius,color,DRAW_FILLED_CIRCLE);
}

void draw_point_glm(cv::Mat &img, const Vec2f &p, const cv::Scalar &color, int draw_radius) {
    draw_point(img, cv::Point2f(p[0],p[1]), color, draw_radius);
}

void draw_point(cv::Mat &img, const cv::Point2f &p, int draw_radius) {
    cv::Scalar color = generate_random_color();
	if(draw_radius == 1)
        img.at<cv::Vec3b>((int) p.y,(int) p.x) = cv::Vec3b(color[0],color[1],color[2]);
	else
		cv::circle(img,p,draw_radius,color,DRAW_FILLED_CIRCLE);
}

void draw_point_glm(cv::Mat &img, const Vec2f &p, int draw_radius) {
    draw_point(img, cv::Point2f(p[0],p[1]), draw_radius);
}

void draw_points_glm(cv::Mat &img, const std::vector<Vec2f> &ps, int draw_radius) {
	for(const auto &p: ps)
		draw_point_glm(img, p, draw_radius);
}

void draw_points_glm(cv::Mat &img, const std::vector<Vec2f> &ps, const cv::Scalar &color, int draw_radius) {
	for(const auto &p: ps)
		draw_point_glm(img, p, color, draw_radius);
}

void draw_points_glm(cv::Mat &img, const std::vector<Vec2f> &ps,
                     const std::vector<cv::Scalar> &colors, int draw_radius)
{
    for(int i= 0; i < ps.size();i++) {
        const Vec2f p = ps[i];
		const cv::Scalar &color = colors[i];
		draw_point_glm(img, p, color, draw_radius);
	}
}

void draw_reference_point_glm(cv::Mat &img, const Vec2f &p, const cv::Scalar &color) {
    draw_point(img, cv::Point2f(p[0],p[1]), color, DRAW_REFERENCE_POINT_RADIUS);
}

void draw_segment_on_img(cv::Mat &img, const Vec4f &segm, const cv::Scalar &color) {
    cv::line(img, cv::Point2f(segm[0],segm[1]), cv::Point2f(segm[2],segm[3]), color);
}

void draw_segment_on_img(cv::Mat &img, const Vec4f &segm, const cv::Scalar &color, float thickness) {
    cv::line(img, cv::Point2f(segm[0],segm[1]), cv::Point2f(segm[2],segm[3]), color, thickness);
}

std::vector<Vec2f> convert_vec_plgpoint_to_vec2(
        const std::vector<PolylineGraphPoint2D> &starting_intersections_plgp)
{
    std::vector<Vec2f> res;
	for(const auto &si : starting_intersections_plgp)
		res.push_back(si.plp.coords);
	return res;
}

std::vector<std::vector<std::vector<Vec2f>>> convert_vecvecvec_plgpoint_to_vecvecvec2(
        const std::vector< std::vector<std::vector<PolylineGraphPoint2D>>> &detected_correspondent_intersections_plgp)
{
    std::vector<std::vector<std::vector<Vec2f>>> res;
	for(const auto &cor : detected_correspondent_intersections_plgp) {
        std::vector<std::vector<Vec2f>> cur_res;
		for(const auto &ccor : cor)
			cur_res.push_back(convert_vec_plgpoint_to_vec2(ccor));
		res.push_back(cur_res);
	}
	return res;
}

void draw_refpoint_on_imgs(std::vector<cv::Mat> &imgs,const SfMDataWrapper &sfmd, int point_id,
                           const cv::Scalar &point_color)
{
    for (const auto& observation : sfmd.landmarks_[point_id].observations) {
        auto cam_id = observation.view_id;
        auto p_cur2dcoords = observation.x;
		draw_reference_point_glm(imgs[cam_id], p_cur2dcoords, point_color);
	}
}

void draw_refpoint_on_imgs(std::vector<cv::Mat> &imgs,const SfMDataWrapper &sfmd, int point_id) {
	draw_refpoint_on_imgs(imgs,sfmd, point_id, generate_random_color());
}

cv::Scalar generate_random_color()
{
	return cv::Scalar(rand() % 256,rand() % 256,rand() % 256);
}

std::vector<cv::Scalar> generate_random_colors(int size)
{
    std::vector<cv::Scalar> result;
    for(int i= 0;i<size;i++)
		result.push_back(generate_random_color());
	return result;
}

/**
 * Images are arranged in a squared display, with no scaling.
 */
cv::Mat getUnifiedSquareImage(const std::vector<cv::Mat>& imgs, int image_type)
{
    float nImgs=imgs.size();
    int   rows=ceil(sqrt(nImgs));     // You can set this explicitly
    int   cols=ceil(nImgs/rows); // You can set this explicitly
    cv::Size totalSize(cols*imgs[0].cols,rows*imgs[0].rows);
    return getUnifiedSquareImage(imgs, totalSize, image_type);
}

/**
 * Images are arranged in a squared display, of the specified size.
 */
cv::Mat getUnifiedSquareImage(const std::vector<cv::Mat>& imgs, const cv::Size &totalSize,
                              int image_type)
{
    cv::Mat scaledImg;
    float nImgs=imgs.size();
    int   rows=ceil(sqrt(nImgs));     // You can set this explicitly
    int   cols=ceil(nImgs/rows); // You can set this explicitly
    cv::Size cellSize(totalSize.width / cols, totalSize.height / rows);

    cv::Size scaledSize;
    double fx,fy,scale;

    int resultImgW=cellSize.width*cols;
    int resultImgH=cellSize.height*rows;

    cv::Mat resultImg = cv::Mat::zeros(resultImgH,resultImgW,image_type);
    int ind= 0;

    for(int i= 0;i<rows;i++)
    {
        for(int j= 0;j<cols;j++)
        {
            if(ind<imgs.size())
            {
				fx = float(cellSize.width) / imgs[ind].cols;
				fy = float(cellSize.height) / imgs[ind].rows;
				scale = fx < fy ? fx : fy;
				scaledSize.width = imgs[ind].cols * scale;
				scaledSize.height = imgs[ind].rows * scale;

                resize(imgs[ind], scaledImg, scaledSize, 0, 0, cv::INTER_LINEAR);


				int cell_row=i*cellSize.height;
				int cell_col=j*cellSize.width;

                scaledImg.copyTo(resultImg(cv::Range(cell_row,cell_row+scaledImg.rows),
                                           cv::Range(cell_col,cell_col+scaledImg.cols)));
            }
            ind++;
        }
    }

    return resultImg;
}

void draw_segments_on_image_rnd_color(cv::Mat &img, const std::vector<Vec4f> &segments) {
    cv::Scalar rnd_color = generate_random_color();
	draw_segments_on_image(img, segments,rnd_color);
}

void draw_segments_on_image(cv::Mat &img, const std::vector<Vec4f> &segments, const cv::Scalar color) {
	for(const auto &s : segments)
		draw_segment_on_img(img,s,color);
}

cv::Mat get_black_image(const cv::Mat &img) {
    cv::Mat bim = cv::Mat(img.rows,img.cols,CV_8UC3);
	bim.setTo(cv::Scalar(0,0,0));
	return bim;
}

cv::Mat draw_MultiColorComponents_PolyLineGraph_simplified(const cv::Mat &img, const PolyLineGraph2D & plg) {
    std::vector<std::vector<Vec4f>> vec_segments = plg.get_segments_grouped_by_component();
    cv::Mat outimg = get_black_image(img);
	for(const auto &segments : vec_segments)
		draw_segments_on_image_rnd_color(outimg,segments);

    draw_points_glm(outimg, plg.get_nodes_with_loops_coords(), cv::Scalar(255,255,255), 2);

    draw_points_glm(outimg, plg.get_hub_nodes_coords(), cv::Scalar(255,0,0), 1);
    draw_points_glm(outimg, plg.get_extreme_nodes_coords(), cv::Scalar(0,255,0), 1);
    draw_points_glm(outimg, plg.get_loopnodes_coords(), cv::Scalar(0,0,255), 1);

	return outimg;
}

std::vector<cv::Mat> draw_plgs(const std::vector<cv::Mat> &imgs,
                               const std::vector<PolyLineGraph2DHMapImpl> &plgs)
{
	std::vector<cv::Mat> res;

    for(int i= 0; i < imgs.size(); i++) {
		const PolyLineGraph2DHMapImpl &plg = plgs[i];
		const cv::Mat &img = imgs[i];
		res.push_back(draw_MultiColorComponents_PolyLineGraph_simplified(img, plg));
	}

	return res;
}

cv::Mat draw_overlay_MultiColorComponents_PolyLineGraph_simplified(cv::Mat &img,
                                                                   const PolyLineGraph2D & plg) {
    std::vector<std::vector<Vec4f>> vec_segments = plg.get_segments_grouped_by_component();
    std::cout << "PLG Connected components: " << vec_segments.size() << std::endl;
	for(const auto &segments : vec_segments)
		draw_segments_on_image_rnd_color(img,segments);

    draw_points_glm(img, plg.get_nodes_with_loops_coords(), cv::Scalar(255,255,255), 2);

    draw_points_glm(img, plg.get_hub_nodes_coords(), cv::Scalar(255,0,0), 1);
    draw_points_glm(img, plg.get_extreme_nodes_coords(), cv::Scalar(0,255,0), 1);
    draw_points_glm(img, plg.get_loopnodes_coords(), cv::Scalar(0,0,255), 1);

	return img;
}

cv::Mat draw_MultiColorPolyLines_PolyLineGraph_simplified(const cv::Mat &img,
                                                          const PolyLineGraph2D & plg)
{
    std::vector<std::vector<Vec4f>> vec_segments = plg.get_segments_grouped_by_polyline();
    cv::Mat outimg = get_black_image(img);
	for(const auto &segments : vec_segments)
		draw_segments_on_image_rnd_color(outimg,segments);
	return outimg;
}

cv::Mat draw_polyline_graph_simplified(const cv::Mat &img, const PolyLineGraph2D & plg,
                                       const cv::Scalar &color)
{
    cv::Mat outimg = get_black_image(img);
	draw_PolyLineGraph_simplified_overlay(outimg, plg, color); 
	return outimg;
}

void draw_PolyLineGraph_simplified_overlay(cv::Mat &img, const PolyLineGraph2D & plg,
                                           const cv::Scalar &color)
{
    std::vector<Vec4f> segments = plg.get_segments_list();
	draw_segments_on_image(img,segments,color);
}

void draw_polyline_matches(std::vector<cv::Mat> &imgs,
                           const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                           const std::vector<std::vector<std::set<ulong>>> &pl_matches)
{
	for(const auto &pl_match : pl_matches) {
		const cv::Scalar col = generate_random_color();

        for(int i= 0; i < plgs.size(); i++)
            for(auto jt=pl_match[i].begin(); jt!=pl_match[i].end(); jt++)
				draw_segments_on_image(imgs[i], plgs[i].polylines[*jt].get_segments_list(),col);
	}
}

void draw_sfmd_points_overwrite(std::vector<cv::Mat> &out_imgs,
                                const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                const SfMDataWrapper &sfm_data,
                                ulong start_from_point,
                                const std::string& out_folder,
                                const cv::String &s)
{
    for(int point_id=start_from_point; point_id < sfm_data.landmarks_.size(); point_id++)
		draw_refpoint_on_imgs(out_imgs,sfm_data, point_id, generate_random_color());

	write_images(out_folder, sfm_data, out_imgs, s);
}

std::vector<cv::Mat> draw_plgs_bw(std::vector<cv::Mat> &imgs,
                                  const std::vector<PolyLineGraph2DHMapImpl> &plgs)
{
	std::vector<cv::Mat> out_imgs;
    for(int i= 0; i < plgs.size(); i++)
        out_imgs.push_back(draw_polyline_graph_simplified(imgs[i],plgs[i], cv::Scalar(255,255,255)));
	return out_imgs;
}

void draw_sfmd_points(std::vector<cv::Mat> &imgs, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                      const SfMDataWrapper &sfm_data, const std::string& out_folder,
                      const cv::String &s)
{
	draw_sfmd_points(imgs, plgs, sfm_data, 0, out_folder, s);
}

void draw_sfmd_points(std::vector<cv::Mat> &imgs, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                      const SfMDataWrapper &sfm_data, ulong start_from_point,
                      const std::string& out_folder, const cv::String &s)
{
	std::vector<cv::Mat> out_imgs_orig_bg = copy_vector_Mat(imgs);
    for(int i= 0; i < plgs.size(); i++)
        draw_PolyLineGraph_simplified_overlay(out_imgs_orig_bg[i],plgs[i], cv::Scalar(0,0,0));
	draw_sfmd_points_overwrite(out_imgs_orig_bg, plgs, sfm_data, start_from_point, out_folder, s);
}

void draw_sfmd_points_plgs(std::vector<cv::Mat> &imgs,
                           const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                           const SfMDataWrapper &sfm_data, const std::string& out_folder,
                           const cv::String &s)
{
	draw_sfmd_points_plgs(imgs, plgs, sfm_data, 0, out_folder, s);
}

void draw_sfmd_points_plgs(std::vector<cv::Mat> &imgs,
                           const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                           const SfMDataWrapper &sfm_data, ulong start_from_point,
                           const std::string& out_folder, const cv::String &s)
{
	std::vector<cv::Mat> out_imgs_plgs = draw_plgs_bw(imgs, plgs);
	draw_sfmd_points_overwrite(out_imgs_plgs, plgs, sfm_data, start_from_point, out_folder, s);
}

} // namespace sanescan::edgegraph3d
