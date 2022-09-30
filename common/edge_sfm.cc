/*  SPDX-License-Identifier: GPL-3.0-or-later

    Copyright (C) 2022  Povilas Kanapickas <povilas@radix.lt>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "edge_sfm.h"
#include <common/edgegraph3d/io/input/convert_edge_images_pixel_to_segment.hpp>
#include <aliceVision/camera/Pinhole.hpp>

namespace sanescan {

edgegraph3d::SfMDataWrapper
    edge_sfm_from_av_sfm_data(const std::vector<aliceVision::IndexT>& orig_view_ids,
                              const aliceVision::sfmData::SfMData& sfm_data,
                              const aliceVision::sfmData::Landmarks& landmarks)
{
    edgegraph3d::SfMDataWrapper res;

    std::map<aliceVision::IndexT, int> orig_view_id_to_view_id;

    for (auto orig_view_id : orig_view_ids) {
        const auto& view = sfm_data.getView(orig_view_id);
        auto intrinsic_base = sfm_data.getIntrinsics().at(view.getIntrinsicId());
        auto intrinsic = std::dynamic_pointer_cast<aliceVision::camera::Pinhole>(
                    intrinsic_base);
        auto pose = sfm_data.getPose(view);

        if (!intrinsic) {
            throw std::runtime_error("Unsupported camera intrinsics type");
        }

        edgegraph3d::CameraType camera;
        camera.intrinsics = intrinsic->K().cast<float>();
        camera.rotation = pose.getTransform().rotation().cast<float>();
        camera.translation = pose.getTransform().translation().cast<float>();

        Mat4f k_matrix = Mat4f::Identity();
        k_matrix.block<3, 3>(0, 0) = camera.intrinsics;
        camera.cameraMatrix = k_matrix * pose.getTransform().getHomogeneous().cast<float>();
        camera.center = pose.getTransform().center().cast<float>();
        camera.pathImage = view.getImagePath();
        camera.imageWidth = view.getWidth();
        camera.imageHeight = view.getHeight();
        camera.orig_view_id = orig_view_id;

        auto new_view_id = res.camerasList_.size();
        orig_view_id_to_view_id.emplace(orig_view_id, new_view_id);
        res.camerasList_.push_back(camera);
    }

    std::vector<aliceVision::IndexT> landmark_ids;
    for (const auto& landmark : landmarks) {
        landmark_ids.push_back(landmark.first);
    }
    std::sort(landmark_ids.begin(), landmark_ids.end());

    for (auto landmark_id : landmark_ids) {
        const auto& landmark = landmarks.at(landmark_id);

        edgegraph3d::LandmarkWrapper landmark_wrapper;
        landmark_wrapper.X = landmark.X.cast<float>();

        for (const auto& observation_pair : landmark.observations) {
            auto orig_view_id = observation_pair.first;

            edgegraph3d::ObservationWrapper obs_wrapper;
            obs_wrapper.x = observation_pair.second.x.cast<float>();
            obs_wrapper.view_id = orig_view_id_to_view_id.at(orig_view_id);

            landmark_wrapper.observations.push_back(obs_wrapper);
        }

        res.landmarks_.push_back(landmark_wrapper);
    }

    return res;
}

edgegraph3d::PolyLineGraph2DHMapImpl
    build_polyline_graph_for_boundaries(const std::vector<std::vector<cv::Point>>& edges)
{
    std::uint64_t total_node_count = 0;
    for (const auto& edge : edges) {
        total_node_count += edge.size();
    }

    edgegraph3d::GraphAdjacencySetUndirectedNoType gs{total_node_count};
    std::vector<Vec2f> node_pos;
    node_pos.reserve(total_node_count);

    for (const auto& edge : edges) {
        if (edge.size() < 2) {
            continue;
        }

        std::uint64_t last_node_id = node_pos.size();
        node_pos.push_back({edge.front().x, edge.front().y});

        for (auto it = std::next(edge.begin()); it != edge.end(); ++it) {
            std::uint64_t next_node_id = node_pos.size();
            node_pos.push_back({it->x, it->y});

            gs.add_edge(last_node_id, next_node_id);
            last_node_id = next_node_id;
        }
    }

    auto plg = edgegraph3d::convert_EdgeGraph_to_PolyLineGraph(gs, node_pos);
    plg.optimize();
    return plg;
}

aliceVision::Mat3 get_fundamental_for_views(const aliceVision::sfmData::SfMData& sfm_data,
                                            aliceVision::IndexT view_a_id,
                                            aliceVision::IndexT view_b_id)
{
    const auto& view_a = sfm_data.getView(view_a_id);
    const auto& view_b = sfm_data.getView(view_b_id);
    const auto& transform_a = sfm_data.getPose(view_a).getTransform();
    const auto& transform_b = sfm_data.getPose(view_b).getTransform();
    auto intrinsic_a_base = sfm_data.getIntrinsicsharedPtr(view_a.getIntrinsicId());
    auto intrinsic_b_base = sfm_data.getIntrinsicsharedPtr(view_b.getIntrinsicId());
    auto intrinsic_a = std::dynamic_pointer_cast<aliceVision::camera::Pinhole>(intrinsic_a_base);
    auto intrinsic_b = std::dynamic_pointer_cast<aliceVision::camera::Pinhole>(intrinsic_b_base);
    if (!intrinsic_a || !intrinsic_b) {
        throw std::runtime_error("Unsupported camera intrinsic type for fundamental matrix calc");
    }

    auto P_a = intrinsic_a->getProjectiveEquivalent(transform_a);
    auto P_b = intrinsic_b->getProjectiveEquivalent(transform_b);

    return aliceVision::F_from_P(P_a, P_b);
}

} // namespace sanescan
