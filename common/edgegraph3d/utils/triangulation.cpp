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

#include "triangulation.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <utility>

#include <edgegraph3d/sfm_data.h>
#include <edgegraph3d/matching/plg_matching/plg_matching.hpp>
#include <edgegraph3d/utils/edge_graph_3d_utilities.hpp>

namespace sanescan::edgegraph3d {

int em_point2D3DJacobian(const std::vector<cv::Matx44d> &cameras, const cv::Matx31d &cur3Dpoint,
                         cv::Mat &J, cv::Matx33d &hessian)
{

  int numMeasures = cameras.size();
  cv::Matx41d cur3DPointHomog;

  cur3DPointHomog(0, 0) = cur3Dpoint(0, 0);
  cur3DPointHomog(1, 0) = cur3Dpoint(1, 0);
  cur3DPointHomog(2, 0) = cur3Dpoint(2, 0);
  cur3DPointHomog(3, 0) = 1.0;

  J = cv::Mat(2 * numMeasures, 3, CV_64FC1);  //2 rows for each point: one for x, the other for y
  hessian = cv::Mat(3, 3, CV_64FC1);

  for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
    cv::Matx41d curReproj = cameras[curMeas] * cur3DPointHomog;
    double xH = curReproj(0, 0);
    double yH = curReproj(1, 0);
    double zH = curReproj(2, 0);
    double p00 = cameras[curMeas](0, 0);
    double p01 = cameras[curMeas](0, 1);
    double p02 = cameras[curMeas](0, 2);
    double p10 = cameras[curMeas](1, 0);
    double p11 = cameras[curMeas](1, 1);
    double p12 = cameras[curMeas](1, 2);
    double p20 = cameras[curMeas](2, 0);
    double p21 = cameras[curMeas](2, 1);
    double p22 = cameras[curMeas](2, 2);

    //d(P*X3D)/dX
    J.at<double>(2 * curMeas, 0) = (p00 * zH - p20 * xH) / (zH * zH);
    J.at<double>(2 * curMeas + 1, 0) = (p10 * zH - p20 * yH) / (zH * zH);

    //d(P*X3D)/dY
    J.at<double>(2 * curMeas, 1) = (p01 * zH - p21 * xH) / (zH * zH);
    J.at<double>(2 * curMeas + 1, 1) = (p11 * zH - p21 * yH) / (zH * zH);

    //d(P*X3D)/dZ
    J.at<double>(2 * curMeas, 2) = (p02 * zH - p22 * xH) / (zH * zH);
    J.at<double>(2 * curMeas + 1, 2) = (p12 * zH - p22 * yH) / (zH * zH);
  }

  hessian = (cv::Mat)(J.t() * J);
  double d;
  d = cv::determinant(hessian);
  if (d < 0.00001) {
    //// printf("doh");
    return -1;
  } else {
    return 1;
  }
}

int em_GaussNewton(const std::vector<cv::Matx44d> &cameras, const std::vector<cv::Point2f> &points,
                   const cv::Point3d &init3Dpoint,
  cv::Point3d &optimizedPoint) {

  int numMeasures = points.size();
  cv::Mat r = cv::Mat(numMeasures * 2, 1, CV_64F);
  cv::Matx31d curEstimate3DPoint;
  cv::Matx41d curEstimate3DPointH;

  curEstimate3DPoint(0, 0) = init3Dpoint.x;
  curEstimate3DPoint(1, 0) = init3Dpoint.y;
  curEstimate3DPoint(2, 0) = init3Dpoint.z;

  cv::Mat J;
  cv::Matx33d H;
  double last_mse = 0;

  /************NB maxGaussNewtonIteration needs to be fixed ***********/
  // number of iterations set to 30
  for (int i = 0; i < 30; i++) {

    double mse = 0;
        //compute residuals
    for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
      curEstimate3DPointH(0, 0) = curEstimate3DPoint(0, 0);
      curEstimate3DPointH(1, 0) = curEstimate3DPoint(1, 0);
      curEstimate3DPointH(2, 0) = curEstimate3DPoint(2, 0);
      curEstimate3DPointH(3, 0) = 1.0;

      cv::Matx41d cur2DpositionH = cameras[curMeas] * curEstimate3DPointH;

      r.at<double>(2 * curMeas, 0) =
              ((points[curMeas].x - cur2DpositionH(0, 0) / cur2DpositionH(2, 0)));

      mse += r.at<double>(2 * curMeas, 0) * r.at<double>(2 * curMeas, 0);

      r.at<double>(2 * curMeas + 1, 0) =
              ((points[curMeas].y - cur2DpositionH(1, 0) / cur2DpositionH(2, 0)));

      mse += r.at<double>(2 * curMeas + 1, 0) * r.at<double>(2 * curMeas + 1, 0);
#ifdef DEBUG_OPTIMIZATION_VERBOSE
      std::cout<<"CurMeas: "<<curMeas<<std::endl<<"curEstimate3DPointH="<< curEstimate3DPointH.t()<<std::endl;
      std::cout<<"CurCam"<<cameras[curMeas]<<std::endl;
      std::cout << "cur2DpositionH: "<<cur2DpositionH(0, 0) / cur2DpositionH(2, 0)
                << ", "<<cur2DpositionH(1, 0)/cur2DpositionH(2, 0)<<std::endl;
      std::cout<<"points[curMeas]: "<<points[curMeas]<<std::endl;
      std::cout<<"residual on x: "<<r.at<double>(2 * curMeas, 0)<<std::endl;
      std::cout<<"residual on y: "<<r.at<double>(2 * curMeas + 1 , 0)<<std::endl;
      std::cout<<std::endl;
#endif
    }

// if the error is very low, it  ends the function
    if (abs(mse / (numMeasures * 2) - last_mse) < 0.0000005) {
      break;
    }
    last_mse = mse / (numMeasures * 2);

    if (em_point2D3DJacobian(cameras, curEstimate3DPoint, J, H) == -1)
      return -1;
#ifdef DEBUG_OPTIMIZATION_VERBOSE
    std::cout<<"J: "<<J<<std::endl;
    std::cout<<"H: "<<H<<std::endl;
#endif

    curEstimate3DPoint = (cv::Mat)(curEstimate3DPoint + (H.inv() * J.t() * r));

#ifdef DEBUG_OPTIMIZATION
    // printf("%d %f\n", i, last_mse);
#endif
  }
  if (last_mse < 9/*3 pixels*/) {
    optimizedPoint.x = curEstimate3DPoint(0, 0);
    optimizedPoint.y = curEstimate3DPoint(1, 0);
    optimizedPoint.z = curEstimate3DPoint(2, 0);
    return 1;
  } else {
    return -1;
  }
}

void em_estimate3Dpositions(const SfMDataWrapper &sfmd,
                            const std::vector<Vec2f> &selected_2d_reprojections_coords,
                            const std::vector<int> &selected_2d_reprojections_ids,
                            Vec3f &triangulated_point, bool &valid)
{
    const std::pair<int, int> min_max_ind = get_min_max(
			selected_2d_reprojections_ids);

	/*
	 * Note: I decided to use the minimum and maximum indices for the selection
	 * of the first and last cameras used for initial triangulation. This is done with the
	 * assumption that the camera rays between those cameras form a bigger
	 * angle than the other possible camera pairs. The reason for this is that
	 * I suspect that having wider angle between the cameras used for initialization,
	 * will lead to a better initial result.
	 */

	int firstCamIdx, lastCamIdx;
	cv::Point3d triangulated3DPointInit, triangulated3DPoint;
	cv::Vec4f triangulated3DPointInitTemp;
    std::vector<cv::Point2f> firstPositionVec, lastPositionVec;
	cv::Point2f firstPoint, lastPoint;
    std::vector<cv::Matx44d> curCams;
	std::vector<cv::Point2f> curPoints;

	firstCamIdx = selected_2d_reprojections_ids[min_max_ind.first];
	lastCamIdx = selected_2d_reprojections_ids[min_max_ind.second];

    auto firstCam = convert_glm_mat4_to_cv_Mat34(sfmd.camerasList_[firstCamIdx].cameraMatrix);
    auto lastCam = convert_glm_mat4_to_cv_Mat34(sfmd.camerasList_[lastCamIdx].cameraMatrix);

	convert_glm_vec2_to_cv_Point2f(
			selected_2d_reprojections_coords[min_max_ind.first], firstPoint);
	convert_glm_vec2_to_cv_Point2f(selected_2d_reprojections_coords[min_max_ind.second],
			lastPoint);

	firstPositionVec.push_back(firstPoint);
	lastPositionVec.push_back(lastPoint);

	cv::triangulatePoints(firstCam, lastCam, firstPositionVec, lastPositionVec,
			triangulated3DPointInitTemp);

	triangulated3DPointInit.x = triangulated3DPointInitTemp[0]
			/ triangulated3DPointInitTemp[3];
	triangulated3DPointInit.y = triangulated3DPointInitTemp[1]
			/ triangulated3DPointInitTemp[3];
	triangulated3DPointInit.z = triangulated3DPointInitTemp[2]
			/ triangulated3DPointInitTemp[3];

	//Pack the information to start the Gauss Newton algorithm
	for (int i = 0; i < selected_2d_reprojections_ids.size(); i++) {
		int camIdx = selected_2d_reprojections_ids[i];
        curCams.push_back(convert_glm_mat4_to_cv_Mat(sfmd.camerasList_[camIdx].cameraMatrix));

		cv::Point2f curPoint;
		convert_glm_vec2_to_cv_Point2f(
					selected_2d_reprojections_coords[i], curPoint);
		curPoints.push_back(curPoint);
	}

	int resGN = em_GaussNewton(curCams, curPoints, triangulated3DPointInit, triangulated3DPoint);

	if(resGN != -1) {
		triangulated_point[0] = triangulated3DPoint.x;
		triangulated_point[1] = triangulated3DPoint.y;
		triangulated_point[2] = triangulated3DPoint.z;
		valid = true;
	} else
		valid = false;
}

void em_estimate3Dpositions(const SfMDataWrapper &sfmd,
                            const std::vector<PolylineGraphPoint2D> &selected_2d_reprojections_coords,
                            const std::vector<int> &selected_2d_reprojections_ids,
                            Vec3f &triangulated_point, bool &valid)
{
    const std::pair<int, int> min_max_ind = get_min_max(
			selected_2d_reprojections_ids);

	/*
	 * Note: I decided to use the minimum and maximum indices for the selection
	 * of the first and last cameras used for initial triangulation. This is done with the
	 * assumption that the camera rays between those cameras form a bigger
	 * angle than the other possible camera pairs. The reason for this is that
	 * I suspect that having wider angle between the cameras used for initialization,
	 * will lead to a better initial result.
	 */

	int firstCamIdx, lastCamIdx;
	cv::Point3d triangulated3DPointInit, triangulated3DPoint;
	cv::Vec4f triangulated3DPointInitTemp;
    std::vector<cv::Point2f> firstPositionVec, lastPositionVec;
    std::vector<cv::Matx44d> curCams;
	std::vector<cv::Point2f> curPoints;

	firstCamIdx = selected_2d_reprojections_ids[min_max_ind.first];
	lastCamIdx = selected_2d_reprojections_ids[min_max_ind.second];

    auto firstCam = convert_glm_mat4_to_cv_Mat34(sfmd.camerasList_[firstCamIdx].cameraMatrix);
    auto lastCam = convert_glm_mat4_to_cv_Mat34(sfmd.camerasList_[lastCamIdx].cameraMatrix);

    auto firstPoint = convert_glm_vec2_to_cv_Point2f(selected_2d_reprojections_coords[min_max_ind.first].plp.coords);
    auto lastPoint = convert_glm_vec2_to_cv_Point2f(selected_2d_reprojections_coords[min_max_ind.second].plp.coords);

	firstPositionVec.push_back(firstPoint);
	lastPositionVec.push_back(lastPoint);

	cv::triangulatePoints(firstCam, lastCam, firstPositionVec, lastPositionVec,
			triangulated3DPointInitTemp);

	triangulated3DPointInit.x = triangulated3DPointInitTemp[0]
			/ triangulated3DPointInitTemp[3];
	triangulated3DPointInit.y = triangulated3DPointInitTemp[1]
			/ triangulated3DPointInitTemp[3];
	triangulated3DPointInit.z = triangulated3DPointInitTemp[2]
			/ triangulated3DPointInitTemp[3];

	//Pack the information to start the Gauss Newton algorithm
	for (int i = 0; i < selected_2d_reprojections_ids.size(); i++) {
		int camIdx = selected_2d_reprojections_ids[i];
        curCams.push_back(convert_glm_mat4_to_cv_Mat(sfmd.camerasList_[camIdx].cameraMatrix));
        curPoints.push_back(convert_glm_vec2_to_cv_Point2f(selected_2d_reprojections_coords[i].plp.coords));
	}

	int resGN = em_GaussNewton(curCams, curPoints, triangulated3DPointInit, triangulated3DPoint);

	if(resGN != -1) {
		triangulated_point[0] = triangulated3DPoint.x;
		triangulated_point[1] = triangulated3DPoint.y;
		triangulated_point[2] = triangulated3DPoint.z;
		valid = true;
	} else
		valid = false;
}

bool compatible_new_observation_to_3Dpositions(const SfMDataWrapper &sfmd,
                                               const Pglp3dPointMatches &current_point,
                                               const Vec2f new_coords, int new_viewpoint_id,
                                               Vec3f &triangulated_point)
{
	bool valid;
    em_add_new_observation_to_3Dpositions(sfmd, current_point, new_coords, new_viewpoint_id,
                                          triangulated_point, valid);
	return valid;
}

bool compatible_new_observation_to_3Dpositions(const SfMDataWrapper &sfmd,
                                               const Pglp3dPointMatches &current_point,
                                               const PolylineGraphPoint2D &new_plgp,
                                               int new_viewpoint_id, Vec3f &triangulated_point)
{
    return compatible_new_observation_to_3Dpositions(sfmd, current_point, new_plgp.plp.coords,
                                                     new_viewpoint_id, triangulated_point);
}

void em_add_new_observation_to_3Dpositions(const SfMDataWrapper &sfmd,
                                           const Pglp3dPointMatches &current_point,
                                           const Vec2f new_coords,
                                           int new_viewpoint_id,
                                           Vec3f &triangulated_point, bool &valid)
{
	/*
	 * Note: I decided to use the minimum and maximum indices for the selection
	 * of the first and last cameras used for initial triangulation. This is done with the
	 * assumption that the camera rays between those cameras form a bigger
	 * angle than the other possible camera pairs. The reason for this is that
	 * I suspect that having wider angle between the cameras used for initialization,
	 * will lead to a better initial result.
	 */

	cv::Point3d triangulated3DPointInit, triangulated3DPoint;
	cv::Vec4f triangulated3DPointInitTemp;
    std::vector<cv::Point2f> firstPositionVec, lastPositionVec;
	cv::Point2f firstPoint, lastPoint;
    std::vector<cv::Matx44d> curCams;
	std::vector<cv::Point2f> curPoints;

    triangulated3DPointInit.x = current_point.pos[0];
    triangulated3DPointInit.y = current_point.pos[1];
    triangulated3DPointInit.z = current_point.pos[2];

	//Pack the information to start the Gauss Newton algorithm
    for (int i = 0; i < current_point.reprojection_ids.size(); i++) {
        int camIdx = current_point.reprojection_ids[i];
        curCams.push_back(convert_glm_mat4_to_cv_Mat(sfmd.camerasList_[camIdx].cameraMatrix));

		cv::Point2f curPoint;
        convert_glm_vec2_to_cv_Point2f(current_point.reprojected_coords[i].plp.coords, curPoint);
		curPoints.push_back(curPoint);
	}

	int camIdx = new_viewpoint_id;
    curCams.push_back(convert_glm_mat4_to_cv_Mat(sfmd.camerasList_[camIdx].cameraMatrix));

	cv::Point2f curPoint;
	convert_glm_vec2_to_cv_Point2f(new_coords,curPoint);
	curPoints.push_back(curPoint);

	int resGN = em_GaussNewton(curCams, curPoints, triangulated3DPointInit, triangulated3DPoint);

	if(resGN != -1) {
		triangulated_point[0] = triangulated3DPoint.x;
		triangulated_point[1] = triangulated3DPoint.y;
		triangulated_point[2] = triangulated3DPoint.z;
		valid = true;
	} else
		valid = false;
}


void em_add_new_observation_to_3Dpositions(const SfMDataWrapper &sfmd,
                                           const Vec3f &current_point_coords,
                                           const std::vector<Vec2f> &current_point_observation_coords,
                                           const std::vector<int> &current_point_observation_ids,
                                           const Vec2f new_coords, int new_viewpoint_id,
                                           Vec3f &triangulated_point, bool &valid)
{
	/*
	 * Note: I decided to use the minimum and maximum indices for the selection
	 * of the first and last cameras used for initial triangulation. This is done with the
	 * assumption that the camera rays between those cameras form a bigger
	 * angle than the other possible camera pairs. The reason for this is that
	 * I suspect that having wider angle between the cameras used for initialization,
	 * will lead to a better initial result.
	 */

	cv::Point3d triangulated3DPointInit, triangulated3DPoint;
	cv::Vec4f triangulated3DPointInitTemp;
    std::vector<cv::Point2f> firstPositionVec, lastPositionVec;
    cv::Point2f firstPoint, lastPoint;
    std::vector<cv::Matx44d> curCams;
	std::vector<cv::Point2f> curPoints;

	triangulated3DPointInit.x = current_point_coords[0];
	triangulated3DPointInit.y = current_point_coords[1];
	triangulated3DPointInit.z = current_point_coords[2];

	//Pack the information to start the Gauss Newton algorithm
	for (int i = 0; i < current_point_observation_ids.size(); i++) {
		int camIdx = current_point_observation_ids[i];
        curCams.push_back(convert_glm_mat4_to_cv_Mat(sfmd.camerasList_[camIdx].cameraMatrix));

		cv::Point2f curPoint;
		convert_glm_vec2_to_cv_Point2f(
				current_point_observation_coords[i], curPoint);
		curPoints.push_back(curPoint);
	}

	int camIdx = new_viewpoint_id;
    curCams.push_back(convert_glm_mat4_to_cv_Mat(sfmd.camerasList_[camIdx].cameraMatrix));

	cv::Point2f curPoint;
	convert_glm_vec2_to_cv_Point2f(new_coords,curPoint);
	curPoints.push_back(curPoint);

	int resGN = em_GaussNewton(curCams, curPoints, triangulated3DPointInit, triangulated3DPoint);

	if(resGN != -1) {
		triangulated_point[0] = triangulated3DPoint.x;
		triangulated_point[1] = triangulated3DPoint.y;
		triangulated_point[2] = triangulated3DPoint.z;
		valid = true;
	} else
		valid = false;
}

void compute_3d_point_coords(const SfMDataWrapper &sfmd,
        const std::vector<Vec2f> &selected_2d_reprojections_coords,
        const std::vector<int> &selected_2d_reprojections_ids,
        Vec3f &new_point_data,
		bool &valid) {
	valid = false;

	if (selected_2d_reprojections_coords.size() >= 2) {
		// Compute 3D point
		em_estimate3Dpositions(sfmd, selected_2d_reprojections_coords,
				selected_2d_reprojections_ids, new_point_data,
				valid);
	}
}

void compute_3d_point(const SfMDataWrapper &sfmd,
        const std::vector<PolylineGraphPoint2D> &selected_2d_reprojections_coords,
        const std::vector<int> &selected_2d_reprojections_ids,
        Pglp3dPointMatches &new_point_data,
		bool &valid) {
	valid = false;

	if (selected_2d_reprojections_coords.size() >= 2) {
		em_estimate3Dpositions(sfmd, selected_2d_reprojections_coords,
                selected_2d_reprojections_ids, new_point_data.pos,
				valid);
		if (valid) {
            new_point_data.reprojected_coords = selected_2d_reprojections_coords;
            new_point_data.reprojection_ids = selected_2d_reprojections_ids;
		}
	}
}

void compute_unique_potential_3d_points_3views_plg_following_newpoint_compatibility(
        const SfMDataWrapper &sfmd,
        const Vector2D<Mat3f>& all_fundamental_matrices,
        const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        const EpipolarCorrespondences3ViewRef &all_2d_reprojections_3views,
        const ReprojectionIds3View &all_2d_reprojections_ids_3views,
        Pglp3dPointMatchesWithSides &new_point, bool &valid) {
	valid = true;
	bool found = false;

    Pglp3dPointMatches potential_new_point;

	bool direction1_valid, direction2_valid;
    std::vector<Pglp3dPointMatches> valid_points_direction2,valid_points_direction1;
    std::vector<ulong> directions1, directions2;

	bool tvalid;
    std::vector<int> views;
    views.push_back(all_2d_reprojections_ids_3views.a);
    views.push_back(all_2d_reprojections_ids_3views.b);
    views.push_back(all_2d_reprojections_ids_3views.c);

    for(const auto &plgp0 : all_2d_reprojections_3views.a)
        for(const auto &plgp1 : all_2d_reprojections_3views.b)
            for(const auto &plgp2 : all_2d_reprojections_3views.c) {
                Vec3f triangulated_point;
                std::vector<PolylineGraphPoint2D> new_plgps;
				new_plgps.push_back(plgp0);
				new_plgps.push_back(plgp1);
				new_plgps.push_back(plgp2);

				em_estimate3Dpositions(sfmd, new_plgps, views, triangulated_point, tvalid);

			    std::vector<int> views_cpy = views;
				if(tvalid) {
					// Try following PLG from the new point

                    potential_new_point = {triangulated_point, new_plgps, views_cpy};

                    if(compatible_new_plg_point(sfmd, all_fundamental_matrices, plgs,
                                                potential_new_point,
                                                directions1, direction1_valid, valid_points_direction1,
                                                directions2, direction2_valid, valid_points_direction2))
                    {
						if(found) {
							// Another point was already found, break
							valid = false;
							return;
						} else {
							found = true;
                            new_point = {valid_points_direction1, directions1,
                                         potential_new_point,
                                         valid_points_direction2, directions2};
						}
					}
				}
			}

	if(!found)
		valid=false;
}

void compute_findfirst_potential_3d_points_3views_plg_following_newpoint_compatibility(
        const SfMDataWrapper &sfmd,Vector2D<Mat3f>& all_fundamental_matrices,
        const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        const EpipolarCorrespondences3ViewRef &all_2d_reprojections_3views,
        const ReprojectionIds3View &all_2d_reprojections_ids_3views,
        Pglp3dPointMatchesWithSides &new_point, bool &valid) {
	valid = false;

    Pglp3dPointMatches potential_new_point;

	bool direction1_valid, direction2_valid;
    std::vector<Pglp3dPointMatches> valid_points_direction2,valid_points_direction1;
    std::vector<ulong> directions1, directions2;

	bool tvalid;
    std::vector<int> views;
    views.push_back(all_2d_reprojections_ids_3views.a);
    views.push_back(all_2d_reprojections_ids_3views.b);
    views.push_back(all_2d_reprojections_ids_3views.c);

    for(const auto &plgp0 : all_2d_reprojections_3views.a)
        for(const auto &plgp1 : all_2d_reprojections_3views.b)
            for(const auto &plgp2 : all_2d_reprojections_3views.c) {
                Vec3f triangulated_point;
                std::vector<PolylineGraphPoint2D> new_plgps;
				new_plgps.push_back(plgp0);
				new_plgps.push_back(plgp1);
				new_plgps.push_back(plgp2);

				em_estimate3Dpositions(sfmd, new_plgps, views, triangulated_point, tvalid);

			    std::vector<int> views_cpy = views;
				if(tvalid) {
                    potential_new_point = {triangulated_point, new_plgps, views_cpy};

                    if(compatible_new_plg_point(sfmd, all_fundamental_matrices, plgs,
                                                potential_new_point, directions1, direction1_valid,
                                                valid_points_direction1, directions2, direction2_valid,
                                                valid_points_direction2)) {
						valid = true;
                        new_point = {valid_points_direction1, directions1,
                                     potential_new_point,
                                     valid_points_direction2, directions2};
						return;
					}
				}
			}
}

bool expand_point_to_other_view(const SfMDataWrapper &sfmd,
                                const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                const Vector2D<Mat3f>& all_fundamental_matrices,
                                int starting_plg_id, int other_plg_id,
                                const std::vector<PolylineGraphPoint2D> &epipolar_correspondences,
                                Pglp3dPointMatchesWithSides &current_p3d_with_sides)
{
	for(auto &epc : epipolar_correspondences)
		// try to add epc to current p3d
		if(add_view_to_3dpoint_and_sides_plgp_matches(sfmd, all_fundamental_matrices, plgs, current_p3d_with_sides, other_plg_id, epc))
			return true; // No adding more than one correspondence per image
	return false;
}

void expand_point_to_other_views(
        const SfMDataWrapper &sfmd,
        const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        const Vector2D<Mat3f>& all_fundamental_matrices,
        int starting_plg_id,
        const std::vector<std::vector<PolylineGraphPoint2D>> &epipolar_correspondences,
        int selected_views[],
        Pglp3dPointMatchesWithSides &current_p3d_with_sides) {

	for(int i= 0; i < selected_views[0]; i++)
        expand_point_to_other_view(sfmd, plgs, all_fundamental_matrices, starting_plg_id, i,
                                   epipolar_correspondences[i], current_p3d_with_sides);

	for(int i=selected_views[0]+1; i < selected_views[1]; i++)
        expand_point_to_other_view(sfmd, plgs, all_fundamental_matrices, starting_plg_id, i,
                                   epipolar_correspondences[i], current_p3d_with_sides);

	for(int i=selected_views[1]+1; i < selected_views[2]; i++)
        expand_point_to_other_view(sfmd, plgs, all_fundamental_matrices, starting_plg_id, i,
                                   epipolar_correspondences[i], current_p3d_with_sides);

    for(int i=selected_views[2]+1; i < sfmd.camerasList_.size(); i++)
        expand_point_to_other_view(sfmd, plgs, all_fundamental_matrices, starting_plg_id, i,
                                   epipolar_correspondences[i], current_p3d_with_sides);

}

#define SWITCH_DISABLE_INTERVAL
//#define TEST_SWITCH_EXPANDCENTRALPOINTONLY
void expand_allpoints_to_other_view_using_plmap(const SfMDataWrapper &sfmd,
                                                const std::vector<PolyLineGraph2DHMapImpl> &plgs,
                                                const Vector2D<Mat3f>& all_fundamental_matrices,
                                                int starting_plg_id,
                                                int other_plg_id,
                                                const std::vector<PolylineGraphPoint2D> &epipolar_correspondences,
                                                const PolyLine2DMapSearch &plmap,
                                                std::vector<Pglp3dPointMatches> &cur_p3ds,
                                                std::vector<ulong> &start_dirs,
                                                std::vector<ulong> &end_dirs,
                                                int &original_central_point_index)
{
#if defined(SWITCH_PLG_MATCHING_ADDPOINT_BOTHDIR_ONE)

	bool success;

    std::pair<int,int> central_point_epc_matches;
    std::pair<int,int> central_point_epc_matches_indices;
	bool central_point_epc_matched=false;

	for(auto &epc : epipolar_correspondences) {
		// try to add epc to current p3d
        central_point_epc_matches = add_view_to_3dpoint_and_sides_plgp_matches_vector(
                    sfmd, all_fundamental_matrices, plgs, cur_p3ds, start_dirs, end_dirs,
                    other_plg_id, epc, 0, original_central_point_index, cur_p3ds.size(),
                    central_point_epc_matched);

		if(central_point_epc_matched) {
			if(central_point_epc_matches.first > original_central_point_index) {
				// points added at start
				original_central_point_index = central_point_epc_matches.first;
				central_point_epc_matches_indices.first = 0;
				central_point_epc_matches_indices.second = central_point_epc_matches.first + central_point_epc_matches.second;
			} else {
				central_point_epc_matches_indices.first = original_central_point_index - central_point_epc_matches.first;
				central_point_epc_matches_indices.second = original_central_point_index + central_point_epc_matches.second;
			}
			break;
		}
	}

	ulong cur_pl_id;
	bool valid;

    std::pair<int,int> added_points_start_end;

    Vec2f prev_coords;
    Vec2f start_coords;

	int central_point;

	int cur_interval_end;

#if !defined(TEST_SWITCH_EXPANDCENTRALPOINTONLY)

#if defined(SWITCH_DISABLE_INTERVAL)
	int last_matched=-1;
	for(int cur_point= 0; cur_point < cur_p3ds.size(); cur_point++) {
		if(central_point_epc_matched && cur_point == central_point_epc_matches_indices.first) {
			cur_point = central_point_epc_matches_indices.second; // Skip to after the epc interval
			last_matched = central_point_epc_matches_indices.second;
			continue;
		}

        start_coords = compute_projection(sfmd,other_plg_id,cur_p3ds[cur_point].pos);
		plmap.find_unique_polyline_maybe_in_search_dist(start_coords,cur_pl_id,valid);

		if(valid) {
			central_point = cur_point;

			// Compute PLGP on new view, polyline cur_interval_pl_id
            const Polyline2D &pl = plgs[other_plg_id].polylines[cur_pl_id];
            PolylinePoint2D init_ppl;
			if(pl.compute_distancesq(start_coords,init_ppl) > MAX_3DPOINT_PROJECTIONDISTSQ_EXPANDALLVIEWS)
                return;
            PolylineGraphPoint2D init_plgp(cur_pl_id,init_ppl);

			if(central_point_epc_matched)
                cur_interval_end = central_point <= central_point_epc_matches_indices.first
                        ? central_point_epc_matches_indices.first : cur_p3ds.size();
			else
				cur_interval_end = cur_p3ds.size();

			// Try to add this point, get last matched index
            added_points_start_end = add_view_to_3dpoint_and_sides_plgp_matches_vector(
                        sfmd, all_fundamental_matrices, plgs, cur_p3ds, start_dirs, end_dirs,
                        other_plg_id, init_plgp, last_matched+1, central_point, cur_interval_end,
                        success);

			if(success) {
				if(added_points_start_end.first > central_point) {
					original_central_point_index = added_points_start_end.first;
					// Points were added before start
					cur_point = (added_points_start_end.first+ added_points_start_end.second);
				} else {
					cur_point = central_point + (added_points_start_end.second);
				}
				last_matched = cur_point;
			}	
		}
	}
#else
    int cur_interval_start = 0;
    ulong cur_interval_pl_id = 0;
	for(int cur_point= 0; cur_point < cur_p3ds.size(); cur_point++) {
		if(central_point_epc_matched && cur_point == central_point_epc_matches_indices.first) {
			cur_point = central_point_epc_matches_indices.second; // Skip to after the epc interval

			// Reset interval
			cur_interval_start=cur_point+1;
			continue;
		}

		start_coords = compute_projection(sfmd,other_plg_id,get<0>(cur_p3ds[cur_point]));
		plmap.find_unique_polyline_maybe_in_search_dist(start_coords,cur_pl_id,valid);

		if(!valid || (cur_interval_start != cur_point && cur_interval_pl_id != cur_pl_id)) {
			// Restart interval
			cur_interval_start=cur_point+1;
			//cur_interval_pl_id=cur_pl_id;
		}
		else
		{
			if(cur_interval_start == cur_point) {
				cur_interval_pl_id=cur_pl_id;
			} else if(cur_point - cur_interval_start == 1) {
			// A sequence of 3 valid points has been found => it's time to run plg matching

			central_point = cur_point;

			// Compute PLGP on new view, polyline cur_interval_pl_id
            const Polyline2D &pl = plgs[other_plg_id].polylines[cur_interval_pl_id];
            PolylinePoint2D init_ppl;
			if(pl.compute_distancesq(prev_coords,init_ppl) > MAX_3DPOINT_PROJECTIONDISTSQ_EXPANDALLVIEWS)
                return;
            PolylineGraphPoint2D init_plgp(cur_interval_pl_id,init_ppl);

			if(central_point_epc_matched)
                cur_interval_end = central_point <= central_point_epc_matches_indices.first
                        ? central_point_epc_matches_indices.first : cur_p3ds.size();
			else
				cur_interval_end = cur_p3ds.size();

			// Try to add this point, get last matched index
            added_points_start_end = add_view_to_3dpoint_and_sides_plgp_matches_vector(
                        sfmd, all_fundamental_matrices, plgs, cur_p3ds, start_dirs, end_dirs,
                        other_plg_id, init_plgp, cur_interval_start, central_point,
                        cur_interval_end,success);

			if(added_points_start_end.first > central_point) {
				original_central_point_index = added_points_start_end.first;
				// Points were added before start
				cur_point = (added_points_start_end.first+ added_points_start_end.second);
			} else
				cur_point = central_point + (added_points_start_end.second);

			// Reset interval
			cur_interval_start = cur_point + 1;
		}

		prev_coords = start_coords;
	}
	}
#endif
#else
	if(!central_point_epc_matched) {
		int cur_point = original_central_point_index;

		start_coords = compute_projection(sfmd,other_plg_id,get<0>(cur_p3ds[cur_point]));
		plmap.find_unique_polyline_maybe_in_search_dist(start_coords,cur_pl_id,valid);

		if(valid) {
			// A sequence of 3 valid points has been found => it's time to run plg matching

			central_point = cur_point;

			// Compute PLGP on new view, polyline cur_interval_pl_id
            const Polyline2D &pl = plgs[other_plg_id].polylines[cur_pl_id];
            PolylinePoint2D init_ppl;
			if(pl.compute_distancesq(start_coords,init_ppl) > MAX_3DPOINT_PROJECTIONDISTSQ_EXPANDALLVIEWS)
				return false;
            PolylineGraphPoint2D init_plgp(cur_pl_id,init_ppl);

			cur_interval_end = cur_p3ds.size();

			// Try to add this point, get last matched index
            added_points_start_end = add_view_to_3dpoint_and_sides_plgp_matches_vector(
                        sfmd, all_fundamental_matrices, plgs, cur_p3ds, start_dirs, end_dirs,
                        other_plg_id, init_plgp, 0, central_point, cur_interval_end,success);

			if(added_points_start_end.first > central_point)
				original_central_point_index = added_points_start_end.first;
		}
	}
#endif

#endif
}

void expand_point_to_other_views_expandallviews_vector(
        const SfMDataWrapper &sfmd,
        const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        const Vector2D<Mat3f>& all_fundamental_matrices,
        int starting_plg_id,
        const std::vector<std::vector<PolylineGraphPoint2D>> &epipolar_correspondences,
        int selected_views[],
        const std::vector<PolyLine2DMapSearch> &plmaps,
        std::vector<Pglp3dPointMatches> &cur_p3ds,
        std::vector<ulong> &start_dirs,
        std::vector<ulong> &end_dirs,
        int &central_point_index)
{

	for(int i= 0; i < selected_views[0]; i++)
        expand_allpoints_to_other_view_using_plmap(sfmd, plgs, all_fundamental_matrices,
                                                   starting_plg_id, i, epipolar_correspondences[i],
                                                   plmaps[i], cur_p3ds, start_dirs, end_dirs,
                                                   central_point_index);

	for(int i=selected_views[0]+1; i < selected_views[1]; i++)
        expand_allpoints_to_other_view_using_plmap(sfmd, plgs, all_fundamental_matrices,
                                                   starting_plg_id, i, epipolar_correspondences[i],
                                                   plmaps[i], cur_p3ds, start_dirs, end_dirs,
                                                   central_point_index);

	for(int i=selected_views[1]+1; i < selected_views[2]; i++)
        expand_allpoints_to_other_view_using_plmap(sfmd, plgs, all_fundamental_matrices,
                                                   starting_plg_id, i, epipolar_correspondences[i],
                                                   plmaps[i], cur_p3ds, start_dirs, end_dirs,
                                                   central_point_index);

    for(int i=selected_views[2]+1; i < sfmd.camerasList_.size(); i++)
        expand_allpoints_to_other_view_using_plmap(sfmd, plgs, all_fundamental_matrices,
                                                   starting_plg_id, i, epipolar_correspondences[i],
                                                   plmaps[i], cur_p3ds, start_dirs, end_dirs,
                                                   central_point_index);
}

std::vector<Pglp3dPointMatches>
    compute_3D_point_multiple_views_plg_following_expandallviews_vector(
        const SfMDataWrapper &sfmd, const std::vector<PolyLineGraph2DHMapImpl> &plgs,
        const Vector2D<Mat3f>& all_fundamental_matrices,
        int starting_plg_id,
        const std::vector<std::vector<PolylineGraphPoint2D>> &epipolar_correspondences,
        const std::vector<PolyLine2DMapSearch> &plmaps)
{
    std::vector<Pglp3dPointMatches> res;

    Pglp3dPointMatchesWithSides p3d_with_sides;
	bool valid;

	valid = false;

	int amount_of_non_empty = 0;
	int min_index,max_index;
	min_index = -1;
	max_index = -1;
	for(int i= 0; i < epipolar_correspondences.size();i++)
		if(epipolar_correspondences[i].size()>0) {
			amount_of_non_empty++;
			min_index = min_index != -1 ? min_index : i;
			max_index = i;
		}

	if(amount_of_non_empty < 3)
		return res;

	int cur_relative_index= 0;
	int relative_mid_index = amount_of_non_empty/2;
	int mid_index;
	for(int i= 0; i < epipolar_correspondences.size();i++)
		if(epipolar_correspondences[i].size()>0) {
			if(cur_relative_index == relative_mid_index) {
				mid_index = i;
				break;
			} else
				cur_relative_index++;
		}


	int selected_indexes[3] = {
			min_index,
			starting_plg_id == min_index || starting_plg_id == max_index ? mid_index : starting_plg_id,
			max_index
	};

    std::vector<Pglp3dPointMatchesWithSides> p3ds_with_sides_3views;

    auto all_2d_reprojections_3views =
            EpipolarCorrespondences3ViewRef{epipolar_correspondences[selected_indexes[0]],
                                            epipolar_correspondences[selected_indexes[1]],
                                            epipolar_correspondences[selected_indexes[2]]};

    auto all_2d_reprojections_ids_3views = ReprojectionIds3View{selected_indexes[0],
                                                                selected_indexes[1],
                                                                selected_indexes[2]};

    compute_unique_potential_3d_points_3views_plg_following_newpoint_compatibility(
            sfmd, all_fundamental_matrices, plgs,
            all_2d_reprojections_3views,
            all_2d_reprojections_ids_3views,
            p3d_with_sides,valid);

	if(!valid)
		return res;

    res = Pglp3dPointMatchesWithSides_to_vector(p3d_with_sides);

    int central_point = p3d_with_sides.valid_points_direction1.size();

    expand_point_to_other_views_expandallviews_vector(sfmd, plgs, all_fundamental_matrices,
                                                      starting_plg_id, epipolar_correspondences,
                                                      selected_indexes, plmaps, res,
                                                      p3d_with_sides.directions1,
                                                      p3d_with_sides.directions2,
                                                      central_point);

	return res;
}

void compute_3d_point_coords_combinations(const SfMDataWrapper &sfmd,
        const std::vector<Vec2f> &all_2d_reprojections_coords,
        const std::vector<int> &all_2d_reprojections_ids,
		int min_combinations,
        std::vector<Vec2f> &selected_2d_reprojections_coords,
	    std::vector<int> &selected_2d_reprojections_ids,
	    std::vector<bool> &selected,
        Vec3f &new_point_data,
		bool &valid) {
	valid = false;

	selected.resize(all_2d_reprojections_ids.size());
	std::fill(selected.begin() + min_combinations, selected.end(), false);
    std::fill(selected.begin(), selected.begin() + min_combinations, true);

    do {
    	selected_2d_reprojections_coords.clear();
    	selected_2d_reprojections_ids.clear();

        for (int i = 0; i < all_2d_reprojections_ids.size(); ++i) {
            if (selected[i]) {
            	selected_2d_reprojections_coords.push_back(all_2d_reprojections_coords[i]);
            	selected_2d_reprojections_ids.push_back(all_2d_reprojections_ids[i]);
            }
        }

        compute_3d_point_coords(sfmd, selected_2d_reprojections_coords,
                                selected_2d_reprojections_ids, new_point_data,valid);
    } while (!valid && std::prev_permutation(selected.begin(), selected.end()));

    if(!valid)
    	return;

    Vec3f new_3d_coords;
    bool need_reorder=false;
    for (int i = 0; i < all_2d_reprojections_ids.size(); ++i) {
        if (!selected[i]) {
            em_add_new_observation_to_3Dpositions(sfmd, new_point_data,
                                                  selected_2d_reprojections_coords,
                                                  selected_2d_reprojections_ids,
                                                  all_2d_reprojections_coords[i],
                                                  all_2d_reprojections_ids[i],
                                                  new_3d_coords, valid);
        	if(valid) {
        		selected[i] = true;
        		new_point_data = new_3d_coords;
        		selected_2d_reprojections_coords.push_back(all_2d_reprojections_coords[i]);
        		if(!need_reorder && all_2d_reprojections_ids[i] > selected_2d_reprojections_ids[selected_2d_reprojections_ids.size()-1])
        			need_reorder = true;
        		selected_2d_reprojections_ids.push_back(all_2d_reprojections_ids[i]);
        	}
        }
    }

    valid = true;

    // Re-Order
    if(need_reorder)
    	reorder_pair_of_vector(selected_2d_reprojections_coords,selected_2d_reprojections_ids);
}

void compute_3d_point(const SfMDataWrapper &sfmd,
        const std::vector<Vec2f> &selected_2d_reprojections_coords,
        const std::vector<int> &selected_2d_reprojections_ids,
        ReprejectedPoint3dData &new_point_data,
		bool &valid) {
	valid = false;

	if (selected_2d_reprojections_coords.size() >= 2) {
		em_estimate3Dpositions(sfmd, selected_2d_reprojections_coords,
                selected_2d_reprojections_ids, new_point_data.pos,
				valid);
		if (valid) {
            new_point_data.reprojected_coords = selected_2d_reprojections_coords;
            new_point_data.reprojection_ids = selected_2d_reprojections_ids;
		}
	}
}

} // namespace sanescan::edgegraph3d
