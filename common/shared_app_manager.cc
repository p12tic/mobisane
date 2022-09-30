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


    Copyright (c) 2016 AliceVision contributors.
    Copyright (c) 2012 openMVG contributors.
    This Source Code Form is subject to the terms of the Mozilla Public License,
    v. 2.0. If a copy of the MPL was not distributed with this file,
    You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "shared_app_manager.h"
#include "edge_sfm.h"
#include "edge_utils.h"
#include "export_ply.h"
#include "feature_extraction_job.h"
#include "finally.h"
#include "geometry_utils.h"
#include "image_debug_utils.h"
#include "image_utils.h"
#include "parallel_alicevision.h"
#include "time_logger.h"
#include <common/edgegraph3d/io/input/convert_edge_images_pixel_to_segment.hpp>
#include <common/edgegraph3d/plg_edge_manager.hpp>
#include <common/edgegraph3d/filtering/outliers_filtering.hpp>
#include <common/edgegraph3d/matching/consensus_manager/plgpcm_3views_plg_following.hpp>
#include <common/edgegraph3d/matching/plg_matching/polyline_2d_map_search.hpp>
#include <common/edgegraph3d/matching/plg_matching/pipelines.hpp>
#include <common/edgegraph3d/matching/plg_matching/plg_matches_manager.hpp>
#include <common/edgegraph3d/plgs/polyline_graph_3d_hmap_impl.hpp>
#include <common/edgegraph3d/utils/geometric_utilities.hpp>
#include <common/edgegraph3d/utils/data_bundle.hpp>
#include <common/bff/Bff.h>
#include <common/bff/MeshIO.h>
#include <sanescanocr/ocr/ocr_point.h>
#include <aliceVision/image/io.hpp>
#include <aliceVision/imageMatching/ImageMatching.hpp>
#include <aliceVision/matchingImageCollection/matchingCommon.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterType.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilter.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_E_AC.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_F_AC.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_H_AC.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_HGrowing.hpp>
#include <aliceVision/matchingImageCollection/ImagePairListIO.hpp>
#include <aliceVision/matching/io.hpp>
#include <aliceVision/matching/matchesFiltering.hpp>
#include <aliceVision/robustEstimation/estimators.hpp>
#include <aliceVision/sensorDB/parseDatabase.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/sfm/pipeline/sequential/ReconstructionEngine_sequentialSfM.hpp>
#include <aliceVision/sfm/pipeline/structureFromKnownPoses/StructureEstimationFromKnownPoses.hpp>
#include <aliceVision/sfm/sfmFilters.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/system/ParallelismBackend.hpp>
#include <aliceVision/vfs/filesystem.hpp>
#include <aliceVision/vfs/FilesystemManager.hpp>
#include <aliceVision/vfs/FilesystemTreeInMemory.hpp>
#include <taskflow/algorithm/for_each.hpp>
#include <filesystem>
#include <random>

namespace vfs = aliceVision::vfs;

namespace sanescan {

namespace {
    sanescan::OcrBox create_start_area(const sanescan::OcrPoint& point, unsigned size,
                                       unsigned max_x, unsigned max_y)
    {
        return {
            std::clamp<std::int32_t>(point.x - size, 0, max_x),
            std::clamp<std::int32_t>(point.y - size, 0, max_y),
            std::clamp<std::int32_t>(point.x + size, 0, max_x),
            std::clamp<std::int32_t>(point.y + size, 0, max_y)
        };
    }

    vfs::path get_path_for_session(const vfs::path& project_path, std::uint64_t session_id)
    {
        return project_path / ("session_" + std::to_string(session_id));
    }

    vfs::path get_path_for_input_image(const vfs::path& session_path, std::uint64_t image_id)
    {
        return session_path / ("input_" + std::to_string(image_id) + ".uncompressed");
    }

    std::vector<aliceVision::sensorDB::Datasheet> parse_sensor_database(const vfs::path& path)
    {
        if (!vfs::exists(path)) {
            throw std::runtime_error("Sensor database does not exist. Path: " + path.string());
        }

        std::vector<aliceVision::sensorDB::Datasheet> db;

        if (!aliceVision::sensorDB::parseDatabase(path.string(), db)) {
            throw std::runtime_error("Could not parse sensor database");
        }
        return db;
    }

    std::vector<aliceVision::feature::EImageDescriberType> get_describer_types()
    {
        return {
            aliceVision::feature::EImageDescriberType::DSPSIFT
        };
    }

    aliceVision::IndexT get_unused_landmark_id(const aliceVision::sfmData::Landmarks& landmarks)
    {
        if (landmarks.empty()) {
            return 1;
        }
        std::mt19937_64 rng(landmarks.size());
        std::uniform_int_distribution<aliceVision::IndexT> dist;

        aliceVision::IndexT index = dist(rng);
        while (landmarks.find(index) != landmarks.end()) {
            index = dist(rng);
        }
        return index;
    }

} // namespace

struct PhotoData
{
    std::uint64_t image_id = 0;
    aliceVision::IndexT view_id = 0;
    cv::Mat image;
    BoundsDetectionPipeline bounds_pipeline;

    PhotoData() = default;

    // Noncopyable to prevent accidental copies
    PhotoData(const PhotoData&) = delete;
    PhotoData& operator=(const PhotoData&) = delete;
};

struct SharedAppManager::Data
{
    tf::Executor& executor;

    std::mutex task_status_mutex;

    std::mt19937 rng;

    // Contains all tasks running for a single pipeline
    std::atomic<std::uint32_t> running_taskflows_any = 0;
    std::atomic<std::uint32_t> running_taskflows_per_image = 0;

    bool serial_detection_requested = false;

    // All data related to specific photos submitted via submit_photo(). std::shared_ptr is used
    // to allow thread-safe concurrent modification of the array.
    std::vector<std::shared_ptr<PhotoData>> submitted_data;
    std::map<aliceVision::IndexT, std::shared_ptr<PhotoData>> submitted_data_by_view_id;

    // After the initial calculation the landmarks array is moved out of this data structure to
    // sfm_landmarks_* variable below. To use sfm_data in any way after that, appropriate data
    // needs to be copied/moved in.
    aliceVision::sfmData::SfMData sfm_data;

    // Exact landmarks that will be used during OCR
    aliceVision::sfmData::Landmarks sfm_landmarks_exact;
    // Inexact landmarks with less matches. They will be used for auxiliary tasks.
    aliceVision::sfmData::Landmarks sfm_landmarks_inexact;
    // Side edge-based landmarks that are used to delimit the boundaries of the scanned page
    aliceVision::sfmData::Landmarks sfm_landmarks_bounds;

    std::string vfs_root_name = "//mobisane";
    vfs::path vfs_project_path = "//mobisane/alicevision";

    std::uint64_t curr_session_id = 0;
    std::uint64_t next_image_id = 0;

    Options options = Options::NONE;
    int random_seed = 1;

    BoundsDetectionPipeline bounds_pipeline;
    // The following params are used for images submitted via submit_photo() which use different
    // bounds detection pipelines in submitted_data array.
    BoundsDetectionParams photo_bounds_pipeline_params;

    // Results from per-image processing pipeline
    aliceVision::feature::FeaturesPerView features_per_view;
    aliceVision::feature::RegionsPerView regions_per_view;
    // Results from match_images()
    aliceVision::PairSet matched_image_pairs;
    // Results from match_features()
    aliceVision::matching::PairwiseMatches pairwise_putative_matches; // only for debugging
    aliceVision::matching::PairwiseMatches pairwise_geometric_matches; // only for debugging
    aliceVision::matching::PairwiseMatches pairwise_final_matches;
    // Results from compute_object_bounds()
    Vec3 orig_plane_centroid;
    Vec3 orig_plane_normal;
    // rotates from orig_plane_normal to working plane with normal at (0, 0, 1)
    Mat3 orig_plane_rotation_matrix;
    aliceVision::sfmData::Landmarks sfm_landmarks_filtered_horiz;
    std::vector<cv::Point2f> working_plane_2d_bounds;

    // Results from compute_object_mesh()
    // IDs corresponds to landmarks from sfm_landmarks_filtered_horiz
    std::vector<MeshTriangle> mesh_triangles_filtered;

    // Results from unfold_object_mesh()
    aliceVision::sfmData::Landmarks sfm_landmarks_unfolded;

    std::vector<aliceVision::sensorDB::Datasheet> sensor_db;

    // Only single image describer is supported because there is not enough processing power for
    // multiple.
    std::shared_ptr<aliceVision::feature::ImageDescriber> image_describer;

    // Extra debugging data
    std::vector<cv::Mat> edge_match_debug_images;

    Data(tf::Executor& executor) :
        executor{executor},
        rng{std::random_device()()}
    {}

    vfs::path get_path_to_current_session()
    {
        return get_path_for_session(vfs_project_path, curr_session_id);
    }

    vfs::path get_path_to_current_session_features_folder()
    {
        return get_path_to_current_session() / "features";
    }

    vfs::path get_path_to_current_session_feature_matches_folder()
    {
        return get_path_to_current_session() / "feature_matches";
    }

    vfs::path get_path_to_current_session_sfm_folder()
    {
        return get_path_to_current_session() / "sfm";
    }

    Vec3 orig_to_working_plane(const Vec3& point)
    {
        return orig_plane_rotation_matrix * (point - orig_plane_centroid);
    }
};

std::unique_ptr<aliceVision::system::IParallelismBackend> g_alicevision_parallelism_backend;

SharedAppManager::SharedAppManager(tf::Executor& executor) :
    d_{std::make_unique<Data>(executor)}
{
    static bool first_init = true;
    if (first_init) {
        first_init = false;

        g_alicevision_parallelism_backend = std::make_unique<ParallelismBackendTaskflow>(executor);
        aliceVision::system::setCurrentParallelistBackend(*g_alicevision_parallelism_backend);
        vfs::getManager().installTreeAtRoot(d_->vfs_root_name,
                                            std::make_unique<vfs::FilesystemTreeInMemory>());
    }

    vfs::create_directories(d_->vfs_project_path);
    vfs::current_path(d_->vfs_project_path);
    vfs::create_directories(d_->get_path_to_current_session());
    vfs::create_directories(d_->get_path_to_current_session_features_folder());
    vfs::create_directories(d_->get_path_to_current_session_feature_matches_folder());
    vfs::create_directories(d_->get_path_to_current_session_sfm_folder());

    d_->sensor_db = parse_sensor_database(vfs::path(aliceVision::image::getAliceVisionRoot()) /
                                          "share/aliceVision/cameraSensors.db");
    d_->image_describer = aliceVision::feature::createImageDescriber(
                aliceVision::feature::EImageDescriberType::DSPSIFT);
}

SharedAppManager::~SharedAppManager() = default;

void SharedAppManager::set_bounds_detection_params(const BoundsDetectionParams& params)
{
    d_->photo_bounds_pipeline_params = params;
}

void SharedAppManager::set_options(Options options)
{
    d_->options = options;
}

void SharedAppManager::submit_photo(const cv::Mat& rgb_image)
{
    tf::Taskflow taskflow;

    d_->running_taskflows_any++;
    d_->running_taskflows_per_image++;

    auto curr_photo_data = d_->submitted_data.emplace_back(std::make_shared<PhotoData>());
    auto task_image_clone = taskflow.emplace([curr_photo_data, rgb_image]()
    {
        // The image is copied twice: once to a file on vfs and second time for further
        // processing here. It may be possible to optimize this copy out. The cost is
        // relatively small compared to the rest of image processing.
        curr_photo_data->image = rgb_image.clone();
    });

    auto image_id = d_->next_image_id++;
    curr_photo_data->image_id = image_id;

    auto session_path = d_->get_path_to_current_session();
    auto image_path = get_path_for_input_image(session_path, image_id);

    auto sfm_image_id = std::to_string(d_->curr_session_id) + "_" + std::to_string(image_id);

    OIIO::ParamValueList metadata;
    metadata.push_back(OIIO::ParamValue("imageCounter", std::to_string(image_id)));
    metadata.push_back(OIIO::ParamValue("ImageUniqueID", sfm_image_id));
    aliceVision::image::writeImage(image_path.string(), cv_mat_to_image_span(rgb_image),
                                   aliceVision::image::ImageWriteOptions(), metadata);

    aliceVision::sfmData::View sfm_view;
    sfm_view.setImagePath(image_path.string());
    aliceVision::sfmDataIO::updateIncompleteView(sfm_view,
                                                 aliceVision::sfmDataIO::EViewIdMethod::METADATA,
                                                 "");
    sfm_view.setFrameId(image_id);

    aliceVision::sfmDataIO::BuildViewIntrinsicsReport intrinsics_report;

    auto allowed_camera_models = aliceVision::camera::EINTRINSIC::PINHOLE_CAMERA |
            aliceVision::camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL1 |
            aliceVision::camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3 |
            aliceVision::camera::EINTRINSIC::PINHOLE_CAMERA_BROWN |
            aliceVision::camera::EINTRINSIC::PINHOLE_CAMERA_FISHEYE |
            aliceVision::camera::EINTRINSIC::PINHOLE_CAMERA_FISHEYE1;

    auto intrinsic = aliceVision::sfmDataIO::buildViewIntrinsic(
                sfm_view, d_->sensor_db, -1.0, 45, 1.0, 0.0, 0.0,
                aliceVision::camera::EINTRINSIC::PINHOLE_CAMERA, allowed_camera_models,
                aliceVision::sfmDataIO::EGroupCameraFallback::FOLDER, intrinsics_report);

    if (!intrinsic || !intrinsic->isValid()) {
        throw std::runtime_error("Could not build intrinsic");
    }
    sfm_view.addMetadata("AliceVision:useWhiteBalance", "1");

    auto view_id = sfm_view.getViewId();

    curr_photo_data->view_id = view_id;
    d_->submitted_data_by_view_id.emplace(view_id, curr_photo_data);

    d_->sfm_data.getViews().emplace(view_id,
                                    std::make_shared<aliceVision::sfmData::View>(sfm_view));
    d_->sfm_data.getIntrinsics().emplace(sfm_view.getIntrinsicId(), intrinsic);

    taskflow.emplace([this, view_id]()
    {
        const auto& attached_sfm_view = d_->sfm_data.getView(view_id);

        FeatureExtractionJob feature_job;
        feature_job.params.output_path = d_->get_path_to_current_session_features_folder().string();
        feature_job.params.config_preset.gridFiltering = true;
        feature_job.params.config_preset.quality = aliceVision::feature::EFeatureQuality::NORMAL;
        feature_job.params.config_preset.descPreset =
                aliceVision::feature::EImageDescriberPreset::NORMAL;
        feature_job.params.config_preset.contrastFiltering =
                aliceVision::feature::EFeatureConstrastFiltering::GridSort;
        feature_job.params.describer = d_->image_describer;
        feature_job.run(attached_sfm_view);
    });

    auto task_bounds_calc = taskflow.emplace([this, curr_photo_data,
                                             params = d_->photo_bounds_pipeline_params]()
    {
        auto& image = curr_photo_data->image;
        auto& bounds_pipeline = curr_photo_data->bounds_pipeline;
        bounds_pipeline.params = params;

        auto size_x = image.size.p[1];
        auto size_y = image.size.p[0];

        std::vector<sanescan::OcrPoint> initial_points = {
            {size_x / 2, size_y / 2}
        };

        for (const auto& point : initial_points) {
            bounds_pipeline.params.flood_params.start_areas.push_back(
                    create_start_area(point, bounds_pipeline.params.initial_point_area_radius,
                                      size_x, size_y));
        }

        bounds_pipeline.run(image);

        if ((d_->options & PRESERVE_INTERMEDIATE_DATA) == 0) {
            bounds_pipeline.clear_intermediate_data();
        }
    });

    // Bounds detection task will need private image
    task_image_clone.precede(task_bounds_calc);

    d_->executor.run(std::move(taskflow), [this]()
    {
        std::lock_guard lock{d_->task_status_mutex};
        d_->running_taskflows_any--;
        d_->running_taskflows_per_image--;
        maybe_on_photo_tasks_finished();
    });
}

void SharedAppManager::perform_detection()
{
    std::lock_guard lock{d_->task_status_mutex};
    if (d_->serial_detection_requested) {
        throw std::invalid_argument("Detection task is already queued");
    }
    d_->serial_detection_requested = true;
    maybe_on_photo_tasks_finished();
}

void SharedAppManager::wait_for_tasks()
{
    d_->executor.loop_until([&](){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return d_->running_taskflows_any == 0;
    });
}

void SharedAppManager::calculate_bounds_overlay(const cv::Mat& rgb_image, cv::Mat& dst_image)
{
    auto size_x = rgb_image.size.p[1];
    auto size_y = rgb_image.size.p[0];

    std::vector<sanescan::OcrPoint> initial_points = {
        {size_x / 2, size_y / 2}
    };

    for (const auto& point : initial_points) {
        d_->bounds_pipeline.params.flood_params.start_areas.push_back(
                    create_start_area(point, d_->bounds_pipeline.params.initial_point_area_radius,
                                      size_x, size_y));
    }

    d_->bounds_pipeline.run(rgb_image);
    draw_bounds_overlay(rgb_image, dst_image, d_->bounds_pipeline.target_object_mask,
                        d_->bounds_pipeline.params.initial_point_image_shrink,
                        d_->bounds_pipeline.precise_edges);
}

void SharedAppManager::print_debug_info(std::ostream& stream)
{
    stream << "Matched image pairs:\n";
    aliceVision::PairSet matched_image_pairs_set;
    aliceVision::matchingImageCollection::savePairs(stream, d_->matched_image_pairs);

    stream << "Matched image features:\n";
    auto print_matches = [&](const aliceVision::matching::PairwiseMatches& matches, const char* msg)
    {
        for (const auto& match: matches) {
            stream << "    image pair (" << match.first.first << ", " << match.first.second
                   << ") contains " << match.second.getNbAllMatches() << " " << msg << "\n";
        }
    };
    print_matches(d_->pairwise_putative_matches, "putative matches");
    print_matches(d_->pairwise_geometric_matches, "geometric matches");
    print_matches(d_->pairwise_final_matches, "geometric matches after grid filtering");
}

void SharedAppManager::print_debug_images(const std::string& debug_folder_path)
{
    tf::Taskflow taskflow;

    taskflow.emplace([&]()
    {
        int i = 0;
        for (const auto& match : d_->pairwise_final_matches) {
            auto view_a_id = match.first.first;
            auto view_b_id = match.first.second;

            const auto& submitted_data_a = d_->submitted_data_by_view_id.at(view_a_id);
            const auto& submitted_data_b = d_->submitted_data_by_view_id.at(view_b_id);

            auto path = "feature_match_" + std::to_string(submitted_data_a->image_id) + "_" +
                        std::to_string(submitted_data_b->image_id) + ".png";

            write_feature_match_debug_image(debug_folder_path, path,
                                            submitted_data_a->image,
                                            submitted_data_b->image,
                                            view_a_id, view_b_id,
                                            match.second,
                                            d_->features_per_view,
                                            d_->sfm_landmarks_inexact);
        }
    });

    taskflow.emplace([&]()
    {
        for (std::size_t i = 0; i < d_->edge_match_debug_images.size(); ++i) {
            auto& image = d_->edge_match_debug_images[i];
            draw_edges_precise(image, d_->submitted_data[i]->bounds_pipeline.precise_edges);
            write_debug_image(debug_folder_path, "edge_intersections_" + std::to_string(i) + ".png",
                              image);
        }
    });

    taskflow.emplace([&]()
    {
        export_ply(std::filesystem::path(debug_folder_path) / "sfm_only_points.ply",
                   d_->sfm_landmarks_exact, {});

        export_ply(std::filesystem::path(debug_folder_path) / "sfm_only_points_inexact.ply",
                   d_->sfm_landmarks_inexact, {});

        export_ply(std::filesystem::path(debug_folder_path) / "sfm_only_points_filtered_hz.ply",
                   d_->sfm_landmarks_filtered_horiz, {});

        export_ply(std::filesystem::path(debug_folder_path) / "sfm_only_points_filtered_hz_mesh.ply",
                   d_->sfm_landmarks_filtered_horiz, d_->mesh_triangles_filtered);

        write_mesh_debug_2d_image(debug_folder_path, "sfm_mesh_horiz_2d.png",
                                  d_->sfm_landmarks_filtered_horiz, d_->mesh_triangles_filtered);

        export_ply(std::filesystem::path(debug_folder_path) / "sfm_only_points_unfolded_hz.ply",
                   d_->sfm_landmarks_unfolded, {});

        export_ply(std::filesystem::path(debug_folder_path) / "sfm_only_points_unfolded_hz_mesh.ply",
                   d_->sfm_landmarks_unfolded, d_->mesh_triangles_filtered);
    });

    d_->executor.run_and_wait(taskflow);
}

void SharedAppManager::print_debug_images_for_photo(const std::string& debug_folder_path,
                                                    std::size_t index) const
{
    const auto& data = d_->submitted_data.at(index);
    const auto& image = data->image;
    const auto& bp = data->bounds_pipeline;

    write_image_with_mask_overlay(debug_folder_path, "target_object_unfilled.png",
                                  bp.small_for_fill, bp.target_object_unfilled_mask);
    write_image_with_mask_overlay(debug_folder_path, "target_object.png",
                                  bp.small_for_fill, bp.target_object_mask);

    write_image_with_edges(debug_folder_path, "target_object_approx_edges.png",
                           image, bp.edges);
    cv::Mat colored_derivatives_h;
    cv::Mat colored_derivatives_s;
    cv::Mat colored_derivatives_v;
    edge_directional_deriv_to_color(bp.hsv_derivatives, colored_derivatives_h, 0);
    edge_directional_deriv_to_color(bp.hsv_derivatives, colored_derivatives_s, 1);
    edge_directional_deriv_to_color(bp.hsv_derivatives, colored_derivatives_v, 2);
    write_debug_image(debug_folder_path, "target_object_edge_2nd_deriv_h.png",
                      colored_derivatives_h);
    write_debug_image(debug_folder_path, "target_object_edge_2nd_deriv_s.png",
                      colored_derivatives_s);
    write_debug_image(debug_folder_path, "target_object_edge_2nd_deriv_v.png",
                      colored_derivatives_v);

    write_image_with_edges_precise(debug_folder_path, "target_object_precise_edges.png",
                                   image, bp.precise_edges);

    const auto& view = d_->sfm_data.getView(data->view_id);

    const auto& features_type = d_->features_per_view.getData().at(view.getViewId());
    write_features_debug_image(debug_folder_path, "sfm_features.png", features_type, image);
}

void SharedAppManager::maybe_on_photo_tasks_finished()
{
    if (d_->running_taskflows_per_image == 0 && d_->serial_detection_requested)
    {
        d_->running_taskflows_any++;
        d_->serial_detection_requested = false;

        tf::Taskflow taskflow;
        taskflow.emplace([this]()
        {
            serial_detect();
        });
        d_->executor.run(std::move(taskflow), [this]()
        {
            d_->running_taskflows_any--;
        });
    }
}

void SharedAppManager::serial_detect()
{
    match_images();
    load_per_image_data();
    match_features();
    compute_structure_from_motion();
    compute_structure_from_motion_inexact();
    compute_edge_structure_from_motion();
    compute_object_bounds();
    compute_object_mesh();
    unfold_object_mesh();
}

void SharedAppManager::match_images()
{
    TimeLogger time_logger{"match_images()"};
    aliceVision::imageMatching::OrderedPairList matched_image_pairs_list;
    aliceVision::imageMatching::generateAllMatchesInOneMap(d_->sfm_data.getViewsKeys(),
                                                           matched_image_pairs_list);
    d_->matched_image_pairs.clear();
    for (const auto& image_pairs : matched_image_pairs_list) {
        for (const auto& index : image_pairs.second) {
            d_->matched_image_pairs.emplace(image_pairs.first, index);
        }
    }
    if (d_->matched_image_pairs.empty()) {
        throw std::runtime_error("No matched image pairs");
    }
}

void SharedAppManager::load_per_image_data()
{
    TimeLogger time_logger{"load_per_image_data()"};

    std::set<aliceVision::IndexT> filter;

    for (const auto& pair: d_->matched_image_pairs) {
        filter.insert(pair.first);
        filter.insert(pair.second);
    }

    auto features_folder = d_->get_path_to_current_session_features_folder().string();

    auto describer_types = get_describer_types();
    if (!aliceVision::sfm::loadRegionsPerView(d_->regions_per_view, d_->sfm_data,
                                              {features_folder},
                                              describer_types, filter))
    {
        throw std::runtime_error("Invalid regions");
    }
    if (!aliceVision::sfm::loadFeaturesPerView(d_->features_per_view, d_->sfm_data, {features_folder},
                                               describer_types))
    {
        throw std::runtime_error("Could not load features");
    }
}

void SharedAppManager::match_features()
{
    TimeLogger time_logger{"match_features()"};

    using namespace aliceVision::matchingImageCollection;

    auto geometric_estimator = aliceVision::robustEstimation::ERobustEstimator::ACRANSAC;
    auto geometric_error_max = std::numeric_limits<double>::infinity();
    auto nearest_matching_method = aliceVision::matching::EMatcherType::ANN_L2;
    auto distance_ratio = 0.8;
    auto cross_matching = false;
    auto min_required_2d_motion = -1.0;
    auto guided_matching = false;
    auto max_iteration_count = 2048;
    auto use_grid_sort = true;
    auto num_matches_to_keep = 0;

    d_->pairwise_putative_matches.clear();
    d_->pairwise_geometric_matches.clear();
    d_->pairwise_geometric_matches.clear();

    auto geometric_filter_type = EGeometricFilterType::FUNDAMENTAL_MATRIX;
    auto describer_types = get_describer_types();

    auto matcher = createImageCollectionMatcher(nearest_matching_method, distance_ratio,
                                                cross_matching);


    for (auto describer_type : describer_types) {
        matcher->Match(d_->rng, d_->regions_per_view, d_->matched_image_pairs, describer_type,
                       d_->pairwise_putative_matches);
    }

    aliceVision::matching::filterMatchesByMin2DMotion(d_->pairwise_putative_matches,
                                                      d_->regions_per_view,
                                                      min_required_2d_motion);

    if (d_->pairwise_putative_matches.empty()) {
        throw std::runtime_error("No feature matches");
    }

    ALICEVISION_LOG_TRACE("match_features(): End regions matching");

    switch(geometric_filter_type) {
        case aliceVision::matchingImageCollection::EGeometricFilterType::NO_FILTERING: {
            d_->pairwise_geometric_matches = d_->pairwise_putative_matches;
            break;
        }

        case EGeometricFilterType::FUNDAMENTAL_MATRIX: {
            robustModelEstimation(
                d_->pairwise_geometric_matches,
                &d_->sfm_data,
                d_->regions_per_view,
                GeometricFilterMatrix_F_AC(geometric_error_max, max_iteration_count,
                                           geometric_estimator),
                d_->pairwise_putative_matches,
                d_->rng,
                guided_matching);
            break;
        }

        case EGeometricFilterType::FUNDAMENTAL_WITH_DISTORTION: {
            robustModelEstimation(
                d_->pairwise_geometric_matches,
                &d_->sfm_data,
                d_->regions_per_view,
                GeometricFilterMatrix_F_AC(geometric_error_max, max_iteration_count,
                                           geometric_estimator, true),
                d_->pairwise_putative_matches,
                d_->rng,
                guided_matching);
            break;
        }
        case EGeometricFilterType::ESSENTIAL_MATRIX: {
            robustModelEstimation(
                d_->pairwise_geometric_matches,
                &d_->sfm_data,
                d_->regions_per_view,
                GeometricFilterMatrix_E_AC(geometric_error_max, max_iteration_count),
                d_->pairwise_putative_matches,
                d_->rng,
                guided_matching);

            removePoorlyOverlappingImagePairs(d_->pairwise_geometric_matches,
                                              d_->pairwise_putative_matches, 0.3f, 50);
            break;
        }
        case EGeometricFilterType::HOMOGRAPHY_MATRIX: {
            const bool only_guided_matching = true;
            robustModelEstimation(
                d_->pairwise_geometric_matches,
                &d_->sfm_data,
                d_->regions_per_view,
                GeometricFilterMatrix_H_AC(geometric_error_max, max_iteration_count),
                d_->pairwise_putative_matches,
                d_->rng, guided_matching,
                only_guided_matching ? -1.0 : 0.6);
            break;
        }
        case EGeometricFilterType::HOMOGRAPHY_GROWING: {
            robustModelEstimation(
                d_->pairwise_geometric_matches,
                &d_->sfm_data,
                d_->regions_per_view,
                GeometricFilterMatrix_HGrowing(geometric_error_max, max_iteration_count),
                d_->pairwise_putative_matches,
                d_->rng,
                guided_matching);
            break;
        }
    }

    ALICEVISION_LOG_TRACE("match_features(): End geometric matching");

    aliceVision::matching::matchesGridFilteringForAllPairs(d_->pairwise_geometric_matches,
                                                           d_->sfm_data,
                                                           d_->regions_per_view, use_grid_sort,
                                                           num_matches_to_keep,
                                                           d_->pairwise_final_matches);

    ALICEVISION_LOG_TRACE("match_features(): End grid filtering");

    aliceVision::matching::Save(d_->pairwise_final_matches,
                                d_->get_path_to_current_session_feature_matches_folder().string(),
                                "txt", false, "");

    if ((d_->options & PRESERVE_INTERMEDIATE_DATA) == 0) {
        d_->pairwise_putative_matches.clear();
        d_->pairwise_geometric_matches.clear();
    }
}

void SharedAppManager::compute_structure_from_motion()
{
    TimeLogger time_logger{"compute_structure_from_motion()"};

    aliceVision::sfm::ReconstructionEngine_sequentialSfM::Params sfm_params;
    sfm_params.localizerEstimator = aliceVision::robustEstimation::ERobustEstimator::ACRANSAC;
    sfm_params.localizerEstimatorError = std::numeric_limits<double>::infinity();
    sfm_params.minNbObservationsForTriangulation = 3;

    auto describer_types = get_describer_types();

    aliceVision::matching::PairwiseMatches& pairwise_matches = d_->pairwise_final_matches;

    // TODO: investigate whether to set initial pair

    aliceVision::sfm::ReconstructionEngine_sequentialSfM sfm_engine(
                d_->sfm_data, sfm_params,
                d_->get_path_to_current_session_sfm_folder().string(),
                (d_->get_path_to_current_session_sfm_folder() / "sfm_log.html").string());

    sfm_engine.initRandomSeed(d_->random_seed);
    sfm_engine.setFeatures(&d_->features_per_view);
    sfm_engine.setMatches(&pairwise_matches);

    if (!sfm_engine.process()) {
        throw std::runtime_error("Could not run SfM algorithm");
    }

    sfm_engine.getSfMData().addFeaturesFolders(
                {d_->get_path_to_current_session_features_folder().string()});
    sfm_engine.getSfMData().addMatchesFolders(
                {d_->get_path_to_current_session_features_folder().string()});
    sfm_engine.getSfMData().setAbsolutePath(
                {d_->get_path_to_current_session_sfm_folder().string()});

    sfm_engine.retrieveMarkersId();

    // sfm::generateSfMReport(sfm_engine.getSfMData(),
    //                       (d_->get_path_to_current_session_sfm_folder() / "sfm_report.html").string());

    d_->sfm_data = std::move(sfm_engine.getSfMData());
    std::swap(d_->sfm_landmarks_exact, d_->sfm_data.getLandmarks());
}

void SharedAppManager::compute_structure_from_motion_inexact()
{
    TimeLogger time_logger{"compute_structure_from_motion_inexact()"};

    double geometric_error_max = 5.5;

    std::mt19937 rng(1);

    aliceVision::sfm::StructureEstimationFromKnownPoses estimator;
    estimator.match(d_->sfm_data, d_->matched_image_pairs, d_->regions_per_view,
                    geometric_error_max);
    estimator.filter(d_->sfm_data, d_->matched_image_pairs, d_->regions_per_view);
    estimator.triangulate(d_->sfm_data, d_->regions_per_view, rng);
    aliceVision::sfm::RemoveOutliers_AngleError(d_->sfm_data, 2.0);

    std::swap(d_->sfm_data.getLandmarks(), d_->sfm_landmarks_inexact);
}

void SharedAppManager::compute_edge_structure_from_motion()
{
    TimeLogger time_logger{"compute_edge_structure_from_motion()"};

    std::vector<aliceVision::IndexT> orig_view_ids;
    for (const auto& image_data : d_->submitted_data) {
        orig_view_ids.push_back(image_data->view_id);
    }

    // the order of edge_sfm_data.camerasList_ corresponds to views in d_->submitted_data
    auto edge_sfm_data = edge_sfm_from_av_sfm_data(orig_view_ids, d_->sfm_data,
                                                   d_->sfm_landmarks_inexact);
    auto initial_points_count = edge_sfm_data.landmarks_.size();

    std::vector<edgegraph3d::PolyLineGraph2DHMapImpl> graphs;

    for (const auto& image_data : d_->submitted_data) {
        graphs.push_back(build_polyline_graph_for_boundaries(
                             image_data->bounds_pipeline.precise_edges));
    }

    float starting_detection_dist = 400;

    std::vector<edgegraph3d::PolyLine2DMapSearch> pl_maps;
    for (std::size_t i = 0; i < graphs.size(); ++i) {
        auto image_data = d_->submitted_data.at(i);
        pl_maps.push_back(edgegraph3d::PolyLine2DMapSearch(graphs[i], image_data->image.size(),
                                                           starting_detection_dist));
    }

    edgegraph3d::PolyLineGraph3DHMapImpl plg3d;
    edgegraph3d::PLGMatchesManager plgmm(graphs, plg3d);

    Vector2D<Mat3f> fundamental_matrices(d_->sfm_data.getViews().size(),
                                         d_->sfm_data.getViews().size(), {});
    for (int i = 0; i < d_->submitted_data.size(); i++) {
        for (int j = 0; j < d_->submitted_data.size(); j++) {
            if (i == j) {
                fundamental_matrices(i, j).fill(0);
            } else {
                auto fmat = get_fundamental_for_views(d_->sfm_data,
                                                      d_->submitted_data[i]->view_id,
                                                      d_->submitted_data[j]->view_id);
                fundamental_matrices(i, j) = fmat.cast<float>();
            }
        }
    }

    if ((d_->options & COLLECT_DEBUG_INFO) != 0) {
        for (const auto& image_data : d_->submitted_data) {
            d_->edge_match_debug_images.push_back(image_data->image.clone());
        }
    }

    edgegraph3d::PLGEdgeManager em(edge_sfm_data, fundamental_matrices, graphs,
                                   starting_detection_dist,
                                   DETECTION_CORRESPONDENCES_MULTIPLICATION_FACTOR);
    em.set_debug_images(d_->edge_match_debug_images);

    edgegraph3d::PLGPCM3ViewsPLGFollowing consensus_manager(edge_sfm_data,
                                                            fundamental_matrices,
                                                            graphs, pl_maps);
    edgegraph3d::DataBundle mfc;
    mfc.em = &em;
    mfc.cm = &consensus_manager;
    mfc.sfmd = &edge_sfm_data;
    mfc.original_img_size = d_->submitted_data[0]->image.size();
    mfc.all_fundamental_matrices = fundamental_matrices;
    mfc.plgs = graphs;

    // Run edge reconstruction pipeline
    edgegraph3d::edge_reconstruction_pipeline(graphs, edge_sfm_data, &mfc,
                                              plgmm, pl_maps);

    for (std::size_t i = initial_points_count; i < edge_sfm_data.landmarks_.size(); ++i) {
        const auto& landmark_wrapper = edge_sfm_data.landmarks_[i];
        if (landmark_wrapper.observations.empty()) {
            continue;
        }

        aliceVision::sfmData::Landmark landmark;
        landmark.X = landmark_wrapper.X.cast<double>();

        for (std::size_t i_obs = 0; i_obs < landmark_wrapper.observations.size(); ++i_obs) {
            auto& observation_wrapper = landmark_wrapper.observations[i_obs];
            auto view_id = d_->submitted_data.at(observation_wrapper.view_id)->view_id;
            landmark.observations[view_id] =
                    aliceVision::sfmData::Observation(observation_wrapper.x.cast<double>(), 0, 0.0);
        }

        d_->sfm_landmarks_exact[get_unused_landmark_id(d_->sfm_landmarks_exact)] = landmark;
        d_->sfm_landmarks_bounds[get_unused_landmark_id(d_->sfm_landmarks_bounds)] = landmark;
    }
}

void SharedAppManager::compute_object_bounds()
{
    TimeLogger time_logger{"compute_object_bounds()"};

    if (d_->sfm_landmarks_bounds.empty()) {
        throw std::runtime_error("sfm_landmarks_bounds is empty");
    }

    // Calculate parameters to rotate 3D points to a horizontal plane for easier processing.
    // The parameters are such that the plane will fit points in sfm_landmarks_bounds.
    std::vector<Vec3> boundary_points;
    boundary_points.reserve(d_->sfm_landmarks_bounds.size());
    for (const auto& [id, landmark] : d_->sfm_landmarks_bounds) {
        boundary_points.push_back(landmark.X);
    }

    std::tie(d_->orig_plane_centroid, d_->orig_plane_normal) =
            fit_plane_to_points(boundary_points);
    d_->orig_plane_rotation_matrix =
            create_rotation_matrix_from_unit_vectors(d_->orig_plane_normal,
                                                     Vec3({0, 0, 1}));

    // Calculate 2D coordinates of the boundary points on the working plane
    std::vector<cv::Point2f> cv_working_plane_2d_bounds;
    cv_working_plane_2d_bounds.reserve(boundary_points.size());

    for (const auto& point : boundary_points) {
        auto working_point = d_->orig_to_working_plane(point);
        cv_working_plane_2d_bounds.push_back(cv::Point2f(working_point.x(), working_point.y()));
    }

    cv::convexHull(cv_working_plane_2d_bounds, d_->working_plane_2d_bounds);

    // Translate sfm_landmarks_exact to sfm_landmarks_filtered_horiz filtering points outside the
    // scanned object in the process.
    for (const auto& [id, landmark] : d_->sfm_landmarks_exact) {
        auto working_point = d_->orig_to_working_plane(landmark.X);
        cv::Point2f cv_point(working_point.x(), working_point.y());

        if (cv::pointPolygonTest(d_->working_plane_2d_bounds, cv_point, false) >= 0) {
            auto landmark_copy = landmark;
            landmark_copy.X = working_point;
            d_->sfm_landmarks_filtered_horiz.emplace(id, landmark_copy);
        }
    }
}

void SharedAppManager::compute_object_mesh()
{
    TimeLogger time_logger{"compute_object_mesh()"};

    auto [min_coord, max_coord] = minmax_landmark_coords(d_->sfm_landmarks_filtered_horiz);

    // Subdiv2D accepts bounds as integer values. Expand the bounds by 1 to avoid any possible
    // problems caused by precision.
    cv::Rect rect(cv::Point(min_coord.x() - 1, min_coord.y() - 1),
                  cv::Point(max_coord.x() + 1, max_coord.y() + 1));
    Subdiv2D subdiv(rect);

    std::unordered_map<int, aliceVision::IndexT> cv_to_landmark_id_map;
    cv_to_landmark_id_map.reserve(d_->sfm_landmarks_filtered_horiz.size() * 2);

    for (const auto& [id, landmark] : d_->sfm_landmarks_filtered_horiz) {
        // The meshing is being done in 2D. It is assumed that the scanned page is not folded so
        // much that it overlaps.
        int index = 0;
        try {
            index = subdiv.insert(cv::Point2f(landmark.X.x(), landmark.X.y()));
        } catch (cv::Exception ex) {
            ALICEVISION_LOG_WARNING("Failed to insert landmark to mesh: "
                                    << landmark.X.x() << " " << landmark.X.y());
            continue;
        }

        cv_to_landmark_id_map.emplace(index, id);
    }

    int total_edge_ids = subdiv.getEdgeCount() * 4;
    std::vector<bool> visited(total_edge_ids, false);

    for (int i = 4; i < total_edge_ids; i += 2) {
        if (visited[i]) {
            continue;
        }
        int edge_a = i;
        int vertex_a = subdiv.edgeOrg(edge_a, nullptr);
        int edge_b = subdiv.getEdge(edge_a, Subdiv2D::NEXT_AROUND_LEFT);
        int vertex_b = subdiv.edgeOrg(edge_b, nullptr);
        int edge_c = subdiv.getEdge(edge_b, Subdiv2D::NEXT_AROUND_LEFT);
        int vertex_c = subdiv.edgeOrg(edge_c, nullptr);
        visited[edge_a] = true;
        visited[edge_b] = true;
        visited[edge_c] = true;

        // FIXME: cv::Subdiv2D returns non-existing vertex IDs
        if (cv_to_landmark_id_map.find(vertex_a) == cv_to_landmark_id_map.end() ||
            cv_to_landmark_id_map.find(vertex_b) == cv_to_landmark_id_map.end() ||
            cv_to_landmark_id_map.find(vertex_c) == cv_to_landmark_id_map.end())
        {
            continue;
        }

        MeshTriangle triangle{cv_to_landmark_id_map.at(vertex_a),
                              cv_to_landmark_id_map.at(vertex_b),
                              cv_to_landmark_id_map.at(vertex_c)};
        d_->mesh_triangles_filtered.push_back(triangle);
    }
}

void SharedAppManager::unfold_object_mesh()
{
    TimeLogger time_logger{"unfold_object_mesh()"};

    if (d_->sfm_landmarks_filtered_horiz.size() < 2) {
        throw std::runtime_error("Not enough filtered landmarks");
    }

    // load model
    bff::Model model;
    bff::PolygonSoup polygon_soup;

    std::unordered_map<int, aliceVision::IndexT> model_id_to_landmark_id;
    std::unordered_map<aliceVision::IndexT, int> landmark_id_to_model_id;
    std::unordered_set<aliceVision::IndexT> landmark_ids_in_triangles;

    landmark_id_to_model_id.reserve(d_->sfm_landmarks_filtered_horiz.size());
    landmark_ids_in_triangles.reserve(d_->sfm_landmarks_filtered_horiz.size());

    for (const auto& triangle : d_->mesh_triangles_filtered) {
        for (auto landmark_id : triangle.indices) {
            landmark_ids_in_triangles.insert(landmark_id);
        }
    }

    for (const auto& [id, landmark] : d_->sfm_landmarks_filtered_horiz) {
        if (!landmark_ids_in_triangles.contains(id)) {
            continue;
        }
        auto model_id = polygon_soup.positions.size();
        polygon_soup.positions.push_back({landmark.X.x(), landmark.X.y(), landmark.X.z()});
        landmark_id_to_model_id.emplace(id, model_id);
    }

    for (const auto& triangle : d_->mesh_triangles_filtered) {
        for (auto landmark_id : triangle.indices) {
            polygon_soup.indices.push_back(landmark_id_to_model_id.at(landmark_id));
        }
    }

    std::string error;
    if (!bff::MeshIO::read(polygon_soup, model, error)) {
        throw std::runtime_error("While unfolding mesh: " + error);
    }
    if (model.size() != 1) {
        throw std::runtime_error("Got more than one mesh to unfold");
    }
    bff::Mesh& mesh = model[0];
    bff::BFF bff_calculator(mesh);
    bff::DenseMatrix u(bff_calculator.data->bN);
    bff_calculator.flatten(u, true);

    for (const auto& [id, landmark] : d_->sfm_landmarks_filtered_horiz) {
        if (!landmark_id_to_model_id.contains(id)) {
            continue;
        }

        auto model_id = landmark_id_to_model_id.at(id);
        const auto& vertex = mesh.vertices[model_id];
        auto unfolded_pos = vertex.wedge()->uv;

        auto landmark_copy = landmark;
        landmark_copy.X.x() = unfolded_pos.x;
        landmark_copy.X.y() = unfolded_pos.y;
        landmark_copy.X.z() = 0;
        d_->sfm_landmarks_unfolded.emplace(id, landmark_copy);
    }
}

void SharedAppManager::draw_bounds_overlay(const cv::Mat& src_image, cv::Mat& dst_image,
                                           const cv::Mat& object_mask,
                                           unsigned object_mask_shrink,
                                           const std::vector<std::vector<cv::Point>>& precise_edges)
{
    auto size_x = std::min(src_image.size.p[1], dst_image.size.p[1]);
    auto size_y = std::min(src_image.size.p[0], dst_image.size.p[0]);

    for (unsigned y = 0; y < size_y; ++y) {
        cv::Vec4b* row = dst_image.ptr<cv::Vec4b>(y);

        for (unsigned x = 0; x < size_x; ++x) {
            auto mask_y = y / object_mask_shrink;
            auto mask_x = x / object_mask_shrink;

            if (object_mask.at<std::uint8_t>(mask_y, mask_x)) {
                row[x] = cv::Vec4b(128, 128, 255, 128);
            } else {
                row[x] = cv::Vec4b(0, 0, 0, 0);
            }
        }
    }

    for (const auto& edge : precise_edges) {
        draw_polyline_32bit(dst_image, edge, cv::Scalar(255, 255, 255, 255));
    }
}

} // namespace sanescan
