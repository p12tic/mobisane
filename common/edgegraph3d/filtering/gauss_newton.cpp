/*  SPDX-License-Identifier: GPL-3.0-or-later

    Copyright (C) 2022  Povilas Kanapickas <povilas@radix.lt>
    Copyright 2014 Andrea Romanoni

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

#include "gauss_newton.hpp"

#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <edgegraph3d/sfm_data.h>

namespace sanescan::edgegraph3d {

int point2D3DJacobian(const std::vector<cv::Mat> &cameras, const cv::Mat &cur3Dpoint, cv::Mat &J,
                      cv::Mat &hessian)
{
  int numMeasures = cameras.size();
  cv::Mat cur3DPointHomog = cv::Mat(4, 1, CV_32F);

  cur3DPointHomog.at<float>(0, 0) = cur3Dpoint.at<float>(0, 0);
  cur3DPointHomog.at<float>(1, 0) = cur3Dpoint.at<float>(1, 0);
  cur3DPointHomog.at<float>(2, 0) = cur3Dpoint.at<float>(2, 0);
  cur3DPointHomog.at<float>(3, 0) = 1.0;

  J = cv::Mat(2 * numMeasures, 3, CV_32FC1);  //2 rows for each point: one for x, the other for y
  hessian = cv::Mat(3, 3, CV_32FC1);

  for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
    cv::Mat curReproj = cameras[curMeas] * cur3DPointHomog;
    float xH = curReproj.at<float>(0, 0);
    float yH = curReproj.at<float>(1, 0);
    float zH = curReproj.at<float>(2, 0);
    float p00 = cameras[curMeas].at<float>(0, 0);
    float p01 = cameras[curMeas].at<float>(0, 1);
    float p02 = cameras[curMeas].at<float>(0, 2);
    float p10 = cameras[curMeas].at<float>(1, 0);
    float p11 = cameras[curMeas].at<float>(1, 1);
    float p12 = cameras[curMeas].at<float>(1, 2);
    float p20 = cameras[curMeas].at<float>(2, 0);
    float p21 = cameras[curMeas].at<float>(2, 1);
    float p22 = cameras[curMeas].at<float>(2, 2);

    //d(P*X3D)/dX
    J.at<float>(2 * curMeas, 0) = (p00 * zH - p20 * xH) / (zH * zH);
    J.at<float>(2 * curMeas + 1, 0) = (p10 * zH - p20 * yH) / (zH * zH);

    //d(P*X3D)/dY
    J.at<float>(2 * curMeas, 1) = (p01 * zH - p21 * xH) / (zH * zH);
    J.at<float>(2 * curMeas + 1, 1) = (p11 * zH - p21 * yH) / (zH * zH);

    //d(P*X3D)/dZ
    J.at<float>(2 * curMeas, 2) = (p02 * zH - p22 * xH) / (zH * zH);
    J.at<float>(2 * curMeas + 1, 2) = (p12 * zH - p22 * yH) / (zH * zH);
  }

  hessian = J.t() * J;
  float d;
  d = cv::determinant(hessian);
  if (d < 0.0000000001) {
    return -1;
  } else {
    return 1;
  }
}

/**
 * GaussNewton - optimize estimate of a 3D point or mark it as outlier by returning -1
 *
 * cameras : vector of all camera ids viewing the 3D point
 * points : vector of 2D measurements of the 3D point on all cameras viewing it
 */
int GaussNewton(const std::vector<cv::Mat> &cameras, const std::vector<cv::Point2f> &points,
                cv::Point3f init3Dpoint,
    cv::Point3f &optimizedPoint, float gn_max_mse) {
  int numMeasures = points.size();
  cv::Mat r = cv::Mat(numMeasures * 2, 1, CV_32F);

  cv::Mat curEstimate3DPoint = cv::Mat(3, 1, CV_32F);
  cv::Mat curEstimate3DPointH = cv::Mat(4, 1, CV_32F);
  curEstimate3DPoint.at<float>(0, 0) = init3Dpoint.x;
  curEstimate3DPoint.at<float>(1, 0) = init3Dpoint.y;
  curEstimate3DPoint.at<float>(2, 0) = init3Dpoint.z;

  cv::Mat J, H;
  float last_mse = 0;
  int i;
  for (i = 0; i < 30; i++) {

    float mse = 0;
    //compute residuals
    for (int curMeas = 0; curMeas < numMeasures; ++curMeas) {
      curEstimate3DPointH.at<float>(0, 0) = curEstimate3DPoint.at<float>(0, 0);
      curEstimate3DPointH.at<float>(1, 0) = curEstimate3DPoint.at<float>(1, 0);
      curEstimate3DPointH.at<float>(2, 0) = curEstimate3DPoint.at<float>(2, 0);
      curEstimate3DPointH.at<float>(3, 0) = 1.0;
      cv::Mat cur2DpositionH = cameras[curMeas] * curEstimate3DPointH; // MEMORY BUG HERE

      r.at<float>(2 * curMeas, 0) = ((points[curMeas].x - cur2DpositionH.at<float>(0, 0) /
                                                          cur2DpositionH.at<float>(2, 0)));
      mse += r.at<float>(2 * curMeas, 0) * r.at<float>(2 * curMeas, 0);

      r.at<float>(2 * curMeas + 1, 0) = ((points[curMeas].y - cur2DpositionH.at<float>(1, 0) /
                                                              cur2DpositionH.at<float>(2, 0)));
      mse += r.at<float>(2 * curMeas + 1, 0) * r.at<float>(2 * curMeas + 1, 0);
    }
    if (abs(mse / (numMeasures * 2) - last_mse) < 0.0000000005) {
      break;
    }
    last_mse = mse / (numMeasures * 2);

    if (point2D3DJacobian(cameras, curEstimate3DPoint, J, H) == -1) {
      return -1;
    }

    curEstimate3DPoint += H.inv() * J.t() * r;
  }

  if (last_mse < gn_max_mse /*3 pixels*/) {
    optimizedPoint.x = curEstimate3DPoint.at<float>(0, 0);
    optimizedPoint.y = curEstimate3DPoint.at<float>(1, 0);
    optimizedPoint.z = curEstimate3DPoint.at<float>(2, 0);
    return 1;
  } else {
    return -1;
  }
}

void gaussNewtonFiltering(SfMDataWrapper &sfm_data_, std::vector<bool>& inliers, float gn_max_mse) {

  std::cout << "Gauss-Newton filtering with max MSE: " << gn_max_mse << "\n";

  inliers.assign(sfm_data_.landmarks_.size(), false);
  std::vector<cv::Mat> cameras;
  std::vector<cv::Point2f> measures;
  cv::Point3f init3Dpoint;
  cv::Point3f optimizedPoint;

  for (int curPt3D = 0; curPt3D < sfm_data_.landmarks_.size(); curPt3D++) {
    cameras.clear();
    cameras.assign(sfm_data_.landmarks_[curPt3D].observations.size(), cv::Mat());
    for (int curC = 0; curC < sfm_data_.landmarks_[curPt3D].observations.size(); curC++) {
      cameras[curC] = cv::Mat(4, 4, CV_32F);
      for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            auto observation_view_id = sfm_data_.landmarks_[curPt3D].observations[curC].view_id;
          cameras[curC].at<float>(row, col) =
                sfm_data_.camerasList_[observation_view_id].cameraMatrix(row, col);
        }
      }
    }

    measures.clear();
    measures.assign(sfm_data_.landmarks_[curPt3D].observations.size(), cv::Point2f());
    for (int curMeas = 0; curMeas < sfm_data_.landmarks_[curPt3D].observations.size(); curMeas++) {
      measures[curMeas].x = sfm_data_.landmarks_[curPt3D].observations[curMeas].x.x();
      measures[curMeas].y = sfm_data_.landmarks_[curPt3D].observations[curMeas].x.y();
    }

    init3Dpoint.x = sfm_data_.landmarks_[curPt3D].X.x();
    init3Dpoint.y = sfm_data_.landmarks_[curPt3D].X.y();
    init3Dpoint.z = sfm_data_.landmarks_[curPt3D].X.z();

    if (GaussNewton(cameras, measures, init3Dpoint, optimizedPoint, gn_max_mse) != -1) {

      sfm_data_.landmarks_[curPt3D].X.x() = optimizedPoint.x;
      sfm_data_.landmarks_[curPt3D].X.y() = optimizedPoint.y;
      sfm_data_.landmarks_[curPt3D].X.z() = optimizedPoint.z;
      inliers[curPt3D] = true;
    }
  }

}

} // namespace sanescan::edgegraph3d
