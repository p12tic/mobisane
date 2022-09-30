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

#include "shared_app_manager.h"
#include "edge_utils.h"
#include "feature_extraction_job.h"
#include "finally.h"
#include "image_utils.h"
#include <sanescanocr/ocr/ocr_point.h>
#include <aliceVision/image/io.hpp>
#include <aliceVision/sensorDB/parseDatabase.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/vfs/filesystem.hpp>
#include <aliceVision/vfs/FilesystemManager.hpp>
#include <aliceVision/vfs/FilesystemTreeInMemory.hpp>

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

} // namespace

struct PhotoData
{
    cv::Mat image;
    BoundsDetectionPipeline bounds_pipeline;

    PhotoData() = default;

    // Noncopyable to prevent accidental copies
    PhotoData(const PhotoData&) = delete;
    PhotoData& operator=(const PhotoData&) = delete;
};

struct SharedAppManager::Data
{
    tbb::task_arena& task_arena;

    std::mutex task_status_mutex;

    // Contains all tasks running for a single pipeline
    tbb::task_group pipeline_tasks;
    std::uint32_t running_feature_extraction_tasks = 0;
    std::uint32_t running_bounds_calculation_tasks = 0;

    // All data related to specific photos submitted via submit_photo(). std::shared_ptr is used
    // to allow thread-safe concurrent modification of the array.
    std::vector<std::shared_ptr<PhotoData>> submitted_data;

    aliceVision::sfmData::SfMData sfm_data;
    std::string vfs_root_name = "//mobisane";
    vfs::path vfs_project_path = "//mobisane/alicevision";

    std::uint64_t curr_session_id = 0;
    std::uint64_t next_image_id = 0;

    BoundsDetectionPipeline bounds_pipeline;
    // The following params are used for images submitted via submit_photo() which use different
    // bounds detection pipelines in submitted_data array.
    BoundsDetectionParams photo_bounds_pipeline_params;

    std::vector<aliceVision::sensorDB::Datasheet> sensor_db;

    // Only single image describer is supported because there is not enough processing power for
    // multiple.
    std::shared_ptr<aliceVision::feature::ImageDescriber> image_describer;

    Data(tbb::task_arena& task_arena) : task_arena{task_arena}
    {}

    vfs::path get_path_to_current_session()
    {
        return get_path_for_session(vfs_project_path, curr_session_id);
    }
};

SharedAppManager::SharedAppManager(tbb::task_arena& task_arena) :
    d_{std::make_unique<Data>(task_arena)}
{
    if (!vfs::getManager().getTreeAtRootIfExists(d_->vfs_root_name)) {
        vfs::getManager().installTreeAtRoot(d_->vfs_root_name,
                                            std::make_unique<vfs::FilesystemTreeInMemory>());
    }
    vfs::create_directories(d_->vfs_project_path);
    vfs::current_path(d_->vfs_project_path);
    vfs::create_directories(d_->get_path_to_current_session());

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

void SharedAppManager::submit_photo(const cv::Mat& rgb_image, Options options)
{
    auto curr_photo_data = d_->submitted_data.emplace_back(std::make_shared<PhotoData>());

    tbb::task_group cloning_task_group;
    d_->task_arena.enqueue(cloning_task_group.defer([&]()
    {
        // The image is copied twice: once to a file on vfs and second time for further
        // processing here. It may be possible to optimize this copy out. The cost is
        // relatively small compared to the rest of image processing.
        curr_photo_data->image = rgb_image.clone();
    }));

    auto image_id = d_->next_image_id++;
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
    d_->sfm_data.getViews().emplace(view_id,
                                    std::make_shared<aliceVision::sfmData::View>(sfm_view));
    d_->sfm_data.getIntrinsics().emplace(sfm_view.getIntrinsicId(), intrinsic);

    const auto& attached_sfm_view = d_->sfm_data.getView(view_id);
    started_feature_extraction_task();
    d_->task_arena.enqueue(d_->pipeline_tasks.defer([this, &attached_sfm_view, session_path]()
    {
        auto on_finish = finally([&](){ finished_feature_extraction_task(); });

        FeatureExtractionJob feature_job;
        feature_job.params.session_path = session_path.string();
        feature_job.params.config_preset.gridFiltering = true;
        feature_job.params.config_preset.quality = aliceVision::feature::EFeatureQuality::NORMAL;
        feature_job.params.config_preset.descPreset =
                aliceVision::feature::EImageDescriberPreset::NORMAL;
        feature_job.params.config_preset.contrastFiltering =
                aliceVision::feature::EFeatureConstrastFiltering::GridSort;
        feature_job.params.describer = d_->image_describer;
        feature_job.run(attached_sfm_view);
    }));

    // Bounds detection task will need private image
    cloning_task_group.wait();

    started_bounds_calculation_task();
    d_->task_arena.enqueue(d_->pipeline_tasks.defer([this, curr_photo_data,
                                                     params = d_->photo_bounds_pipeline_params,
                                                     options]()
    {
        auto on_finish = finally([&](){ finished_bounds_calculation_task(); });

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

        if ((options & PRESERVE_INTERMEDIATE_DATA) == 0) {
            bounds_pipeline.clear_intermediate_data();
        }
    }));
}

const cv::Mat& SharedAppManager::get_photo(std::size_t index) const
{
    return d_->submitted_data.at(index)->image;
}

const BoundsDetectionPipeline&
    SharedAppManager::get_bounds_detection_pipeline(std::size_t index) const
{
    return d_->submitted_data.at(index)->bounds_pipeline;
}

void SharedAppManager::wait_for_photo_tasks()
{
    d_->pipeline_tasks.wait();
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

void SharedAppManager::started_feature_extraction_task()
{
    std::lock_guard lock{d_->task_status_mutex};
    d_->running_feature_extraction_tasks++;
}

void SharedAppManager::finished_feature_extraction_task()
{
    std::lock_guard lock{d_->task_status_mutex};
    d_->running_feature_extraction_tasks--;
}

void SharedAppManager::started_bounds_calculation_task()
{
    std::lock_guard lock{d_->task_status_mutex};
    d_->running_bounds_calculation_tasks++;
}

void SharedAppManager::finished_bounds_calculation_task()
{
    std::lock_guard lock{d_->task_status_mutex};
    d_->running_bounds_calculation_tasks--;
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
