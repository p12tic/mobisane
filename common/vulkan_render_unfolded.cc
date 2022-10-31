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


    Portions of this file are based on code that is licensed as follows:

    Copyright (C) by Sascha Willems - www.saschawillems.de
    Copyright (c) 2017 Eric Arnebäck

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#ifdef __APPLE__
#define VK_ENABLE_BETA_EXTENSIONS
#endif

#include "vulkan_render_unfolded.h"
#include "algorithm.h"
#include "geometry_utils.h"
#include "vulkan_utils.h"
#include "vkutils/VulkanBuffer.h"
#include "vkutils/VulkanDevice.h"
#include "vkutils/VulkanInitializers.h"
#include "vkutils/VulkanTools.h"
#include "shaders/render_unfolded_frag.h"
#include "shaders/render_unfolded_vert.h"
#include <vulkan/vulkan.h>
#include <sanescanocr/util/math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <cmath>
#include <unordered_map>

namespace sanescan {

namespace {

Mat3 create_transform_to_render_pos(float scale_x, float scale_y, bool invert_y,
                                    float translate_x, float translate_y,
                                    float rect_angle_rad)
{
    auto angle = rect_angle_rad;

    Mat3 mat_tr;
    mat_tr << 1, 0, translate_x,
              0, 1, translate_y,
              0, 0, 1;

    Mat3 mat_reflect;
    mat_reflect << 1, 0, 0,
               0, (invert_y ? -1 : 1), 0,
               0, 0, 1;

    Mat3 mat_scale;
    mat_scale << scale_x, 0, 0,
               0, scale_y, 0,
               0, 0, 1;

    Mat3 mat_rot;
    mat_rot << std::cos(angle), std::sin(angle), 0,
               -std::sin(angle), std::cos(angle), 0,
               0, 0, 1;

    return mat_scale * mat_reflect * mat_rot * mat_tr;
}

bool should_mirror(const aliceVision::sfmData::Landmarks& landmarks,
                   aliceVision::IndexT view_id)
{
    // The unfolding step produces mirrored meshes sometimes. Later steps will not be able to
    // recover from this problem, thus the image is mirrored if needed during the rendering step.

    // FIXME: this just picks 3 landmarks to determine rotation. Thus the algorithm is very
    // susceptible to invalid landmark locations

    std::vector<std::pair<aliceVision::IndexT, Vec2>> landmark_positions;
    landmark_positions.reserve(landmarks.size());
    for (const auto& [id, landmark] : landmarks) {
        landmark_positions.push_back({id, Vec2(landmark.X.x(), landmark.X.y())});
    }

    auto [min_i, max_i] = minmax_element_i_by_value(landmark_positions.begin(),
                                                    landmark_positions.end(),
                                                    [](const auto& p) { return p.second.y(); });
    auto l1_pos = landmark_positions[min_i];
    auto l2_pos = landmark_positions[max_i];

    Vec2 p1 = l1_pos.second;
    Vec2 p2 = l2_pos.second;

    Vec3 line = line_through_points(p1, p2);

    // Find point farthest away from this line as the third landmark for the triangle.
    auto maxdistance_i = max_element_by_value(landmark_positions.begin(),
                                              landmark_positions.end(),
                                              [&](const auto& p)
    {
        double p_x = p.second.x();
        double p_y = p.second.y();

        // Proper computation should divide by std::hypot(line_a, line_b), but this is a constant
        // factor which does not affect the result of max_element_by_value.
        return std::fabs(line.x() * p_x + line.y() * p_y + line.z());
    });

    auto l3_pos = landmark_positions[maxdistance_i];
    Vec2 p3 = l3_pos.second;

    // Compute the area of the triangle as given by the unfolded landmark positions and also the
    // area of the triangle as given by the input observation positions. If the signs of the
    // triangles are the same, no flipping is needed. Otherwise the coordinates need to be flipped.
    auto area_landmark = signed_triangle_area(p1, p2, p3);
    auto area_observations =
            signed_triangle_area(landmarks.at(l1_pos.first).observations.at(view_id).x,
                                 landmarks.at(l2_pos.first).observations.at(view_id).x,
                                 landmarks.at(l3_pos.first).observations.at(view_id).x);

    return (area_landmark < 0) != (area_observations < 0);
}

} // namespace

RenderingUnfoldedInfo
    build_unfolded_info_for_rendering(const std::vector<aliceVision::IndexT>& view_ids,
                                      const std::vector<cv::Mat>& images,
                                      std::size_t max_size_dimension,
                                      const aliceVision::sfmData::SfMData& sfm_data,
                                      const aliceVision::sfmData::Landmarks& orig_landmarks,
                                      const aliceVision::sfmData::Landmarks& unfolded_landmarks,
                                      const std::vector<MeshTriangle>& triangle_indices)
{
    // Note that unfolded_landmarks has fewer elements due to filtering.
    RenderingUnfoldedInfo res;
    res.images = images;
    if (res.images.empty()) {
        throw std::invalid_argument("No textures supplied");
    }
    res.images_size = images.front().size();

    for (std::size_t i = 1; i < images.size(); ++i) {
        if (images[i].size() != res.images_size) {
            throw std::invalid_argument("All textures must have identical size");
        }
    }

    std::unordered_map<aliceVision::IndexT, std::uint32_t> landmark_id_to_dest_id;
    landmark_id_to_dest_id.reserve(unfolded_landmarks.size());

    // Calculate destination image bounds
    std::vector<cv::Point2f> unfolded_points;
    unfolded_points.reserve(unfolded_landmarks.size());

    std::size_t curr_dest_landmark_index = 0;
    for (const auto& [id, landmark] : unfolded_landmarks) {
        landmark_id_to_dest_id.emplace(id, curr_dest_landmark_index++);

        unfolded_points.push_back(cv::Point2f(landmark.X.x(), landmark.X.y()));
    }

    auto rotated_rect = cv::minAreaRect(unfolded_points);

    if (rotated_rect.size.height >= rotated_rect.size.width) {
        res.render_size.height = max_size_dimension;
        res.render_size.width = max_size_dimension *
                rotated_rect.size.width / rotated_rect.size.height;
    } else {
        res.render_size.width = max_size_dimension;
        res.render_size.height = max_size_dimension *
                rotated_rect.size.height / rotated_rect.size.width;
    }

    float scale_x = 2.0f / rotated_rect.size.width;
    float scale_y = 2.0f / rotated_rect.size.height;

    // Check if output should be mirrored
    bool should_mirror_dest_pos = should_mirror(unfolded_landmarks, view_ids.front());

    // Calculate transform between source unfolded to destination coordinates and apply it.
    auto unfolded_to_dest_transform =
            create_transform_to_render_pos(scale_x, scale_y,
                                           should_mirror_dest_pos,
                                           -rotated_rect.center.x,
                                           -rotated_rect.center.y,
                                           deg_to_rad(rotated_rect.angle));

    res.landmark_positions_in_dest.resize(unfolded_landmarks.size());
    for (const auto& [id, landmark] : unfolded_landmarks) {
        auto dest_id = landmark_id_to_dest_id.at(id);
        Vec3 dest_pos = unfolded_to_dest_transform * Vec3(landmark.X.x(), landmark.X.y(), 1);
        res.landmark_positions_in_dest[dest_id] = cv::Vec2d(dest_pos.x(), dest_pos.y());
    }

    // Remap triangle face indices to destination range
    res.triangle_indices.reserve(triangle_indices.size());
    for (const auto& triangle : triangle_indices) {
        res.triangle_indices.push_back({{{
            static_cast<aliceVision::IndexT>(landmark_id_to_dest_id.at(triangle.indices[0])),
            static_cast<aliceVision::IndexT>(landmark_id_to_dest_id.at(triangle.indices[1])),
            static_cast<aliceVision::IndexT>(landmark_id_to_dest_id.at(triangle.indices[2]))}}});
    }

    // Remap all landmark observations to destination indices. If there is no observation for a
    // particular view, then an observation is invented by projecting an existing observation.
    res.landmark_positions_in_views = cv::Mat_<cv::Vec2d>(unfolded_landmarks.size(),
                                                          view_ids.size());

    auto projection_matrices = get_projection_matrices_for_all_views(sfm_data, view_ids);
    for (const auto& [id, unfolded_landmark]: unfolded_landmarks) {
        const auto& orig_landmark = orig_landmarks.at(id);
        auto dest_id = landmark_id_to_dest_id.at(id);

        for (std::size_t view_i = 0; view_i < view_ids.size(); ++view_i) {
            auto view_id = view_ids[view_i];
            auto obs_it = orig_landmark.observations.find(view_id);

            cv::Vec2d dest_view_pos;
            if (obs_it != orig_landmark.observations.end()) {
                dest_view_pos = cv::Vec2d(obs_it->second.x.x(), obs_it->second.x.y());
            } else {
                Vec3 dest_view_proj =
                        projection_matrices[view_i] * Vec4(orig_landmark.X.x(),
                                                           orig_landmark.X.y(),
                                                           orig_landmark.X.z(), 1.0f);
                dest_view_pos(0) = dest_view_proj.x() / dest_view_proj.z();
                dest_view_pos(1) = dest_view_proj.y() / dest_view_proj.z();
            }
            res.landmark_positions_in_views(dest_id, view_i) = dest_view_pos;
        }
    }

    // Calculate average locations of each triangle face and select view that is nearest each of
    // them.
    auto camera_translations = get_translations_for_all_views(sfm_data, view_ids);
    res.selected_view_per_triangle.reserve(triangle_indices.size());

    for (const auto& triangle : triangle_indices) {
        Vec3 triangle_pos_sum = Vec3::Zero();

        for (std::size_t i = 0; i < 3; ++i) {
            triangle_pos_sum += orig_landmarks.at(triangle.indices[i]).X;
        }

        Vec3 triangle_mean_pos = triangle_pos_sum / 3.0;

        double min_dist = std::numeric_limits<double>::infinity();
        std::size_t min_view_i = 0;
        for (std::size_t view_i = 0; view_i < view_ids.size(); ++view_i) {
            auto cur_dist = (camera_translations[view_i] - triangle_mean_pos).norm();
            if (cur_dist < min_dist) {
                min_dist = cur_dist;
                min_view_i = view_i;
            }
        }

        res.selected_view_per_triangle.push_back(min_view_i);
    }

    return res;
}

class UnfoldedImageRendererVulkan {
private:
    struct Vertex {
        float pos[3] = {};
        float uv[2] = {};
    };

    struct Pixel {
        float r, g, b, a;
    };

    struct VertexUBO {
        glm::mat4 projection;
        glm::mat4 model_view;
        glm::vec4 view_pos;
    };

    bool enable_validation_layers_ = false;

    VkInstance instance_ = {};
    VkDebugReportCallbackEXT debug_report_callback_ = {};
    VkPhysicalDevice physical_device_ = {};
    std::unique_ptr<vks::VulkanDevice> vkdevice_;

    VkPipeline pipeline_ = {};
    VkPipelineLayout pipeline_layout_ = {};
    std::vector<VkShaderModule> shader_modules_;

    VkCommandPool command_pool_ = {};
    VkCommandBuffer command_buffer_;

    VkDescriptorPool descriptor_pool_ = {};
    VkDescriptorSet descriptor_set_ = {};
    VkDescriptorSetLayout descriptor_set_layout_ = {};

    VkPhysicalDeviceMemoryProperties memory_properties_ = {};

    VertexUBO vertex_ubo_data_ = {};
    vks::Buffer vertex_ubo_ = {};
    vks::Buffer vertex_buffer_ = {};
    vks::Buffer index_buffer_ = {};
    std::size_t index_count_ = 0;

    VkSampler src_sampler_ = {};
    VkImage src_image_array_ = {};
    VkImageLayout src_layout_ = {};
    VkDeviceMemory src_device_memory_ = {};
    VkImageView src_image_view_;

    std::vector<const char*> enabled_layers_;
    std::vector<const char*> enabled_extensions_;

    VkQueue queue_ = {};

    std::uint32_t queue_family_index_ = 0;

    std::uint32_t result_width_ = 3200;
    std::uint32_t result_height_ = 2400;

    struct FrameBufferAttachment {
        VkImage image;
        VkDeviceMemory memory;
        VkImageView view;
    };

    FrameBufferAttachment color_attachment_ = {};
    FrameBufferAttachment depth_attachment_ = {};

    VkFormat depth_format_ = {};
    VkRenderPass render_pass_ = {};
    VkFramebuffer framebuffer_ = {};

public:
    cv::Mat run(const RenderingUnfoldedInfo& info)
    {
        result_width_ = info.render_size.width;
        result_height_ = info.render_size.height;

#if MOBISANE_ENABLE_VULKAN_VALIDATION
        enable_validation_layers_ = true;
#endif

        // Initialize vulkan:
        create_instance();
        find_physical_device();
        create_device();
        create_color_attachment();
        create_depth_attachment();
        create_render_pass();
        create_textures(info.images_size, info.images);
        create_uniform_buffers();
        create_descriptor_set_layout();
        create_descriptor_set();
        create_vertex_buffers(info);
        create_compute_pipeline();
        create_command_buffer();

        run_command_buffer(command_buffer_);

        auto res = save_rendered_image();

        cleanup();
        return res;
    }

    static VKAPI_ATTR VkBool32 VKAPI_CALL
        debug_report_callback_fn(VkDebugReportFlagsEXT flags,
                                 VkDebugReportObjectTypeEXT object_type,
                                 std::uint64_t object,
                                 std::size_t location,
                                 std::int32_t message_code,
                                 const char* layer_prefix,
                                 const char* message,
                                 void* user_data)
    {
        std::cout << "[VALIDATION]: " << layer_prefix << ": " << message << std::endl;
        return VK_FALSE;
    }

    void setup_validation_layers()
    {
        std::uint32_t layer_count = 0;
        vkEnumerateInstanceLayerProperties(&layer_count, nullptr);

        std::vector<VkLayerProperties> layer_props(layer_count);
        vkEnumerateInstanceLayerProperties(&layer_count, layer_props.data());

        std::vector<std::string> layer_names;
        for (const auto& props : layer_props) {
            layer_names.push_back(props.layerName);
        }

        for (auto debug_extension_name : {"VK_LAYER_KHRONOS_validation",
                                          "VK_LAYER_LUNARG_standard_validation"})
        {
            auto it = std::find(layer_names.begin(), layer_names.end(), debug_extension_name);
            if (it != layer_names.end()) {
                enabled_layers_.push_back(debug_extension_name);
                break;
            }
        }

        std::uint32_t extension_count;

        vkEnumerateInstanceExtensionProperties(nullptr, &extension_count, nullptr);
        std::vector<VkExtensionProperties> extension_props(extension_count);
        vkEnumerateInstanceExtensionProperties(nullptr, &extension_count, extension_props.data());

        for (VkExtensionProperties prop : extension_props) {
            if (std::strcmp(VK_EXT_DEBUG_REPORT_EXTENSION_NAME, prop.extensionName) == 0) {
                enabled_extensions_.push_back(VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
                break;
            }
        }

#if defined(__APPLE__)
        if (std::find(enabled_extensions_.begin(), enabled_extensions_.end(),
                      VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME) ==
                enabled_extensions_.end())
        {
            enabled_extensions_.push_back(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
        }
#endif

    }

    void create_instance()
    {
        if (enable_validation_layers_) {
            setup_validation_layers();
        }

        VkApplicationInfo application_info = {};
        application_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        application_info.pApplicationName = "Mobisane app";
        application_info.applicationVersion = 0;
        application_info.pEngineName = "mobisane";
        application_info.engineVersion = 0;
        application_info.apiVersion = VK_API_VERSION_1_0;;

        VkInstanceCreateInfo create_info = {};
        create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        create_info.flags = 0;
        create_info.pApplicationInfo = &application_info;
        create_info.enabledLayerCount = enabled_layers_.size();
        create_info.ppEnabledLayerNames = enabled_layers_.data();
        create_info.enabledExtensionCount = enabled_extensions_.size();
        create_info.ppEnabledExtensionNames = enabled_extensions_.data();

        VK_CHECK_RESULT(vkCreateInstance(&create_info, nullptr, &instance_));

        if (enable_validation_layers_) {
            VkDebugReportCallbackCreateInfoEXT createInfo = {};
            createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_REPORT_CALLBACK_CREATE_INFO_EXT;
            createInfo.flags = VK_DEBUG_REPORT_ERROR_BIT_EXT | VK_DEBUG_REPORT_WARNING_BIT_EXT |
                    VK_DEBUG_REPORT_PERFORMANCE_WARNING_BIT_EXT;
            createInfo.pfnCallback = &debug_report_callback_fn;

            auto vkCreateDebugReportCallbackEXT = reinterpret_cast<PFN_vkCreateDebugReportCallbackEXT>(
                        vkGetInstanceProcAddr(instance_, "vkCreateDebugReportCallbackEXT"));
            if (vkCreateDebugReportCallbackEXT) {
                VK_CHECK_RESULT(vkCreateDebugReportCallbackEXT(instance_, &createInfo, nullptr,
                                                               &debug_report_callback_));
            }
        }
    }

    void find_physical_device()
    {
        std::uint32_t device_count = 0;
        vkEnumeratePhysicalDevices(instance_, &device_count, nullptr);
        if (device_count == 0) {
            throw std::runtime_error("No vulkan devices are available");
        }

        std::vector<VkPhysicalDevice> devices(device_count);
        vkEnumeratePhysicalDevices(instance_, &device_count, devices.data());

        // FIXME: Add a way to enable software rasterizer for testing
        physical_device_ = devices.at(0);
        vkGetPhysicalDeviceMemoryProperties(physical_device_, &memory_properties_);
    }

    // Returns the index of a queue family that supports compute operations.
    std::uint32_t get_compute_queue_family_index() {
        std::uint32_t queue_family_count;

        vkGetPhysicalDeviceQueueFamilyProperties(physical_device_, &queue_family_count, nullptr);

        std::vector<VkQueueFamilyProperties> queueFamilies(queue_family_count);
        vkGetPhysicalDeviceQueueFamilyProperties(physical_device_, &queue_family_count,
                                                 queueFamilies.data());

        for (std::uint32_t i = 0; i < queueFamilies.size(); ++i) {
            VkQueueFamilyProperties props = queueFamilies[i];
            if (props.queueCount > 0 && (props.queueFlags & VK_QUEUE_GRAPHICS_BIT)) {
                return i;
            }
        }

        throw std::runtime_error("No Vulkan queue with compute operation bit");
    }

    void create_device()
    {
        VkDeviceQueueCreateInfo queue_create_info = {};
        queue_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queue_family_index_ = get_compute_queue_family_index();
        queue_create_info.queueFamilyIndex = queue_family_index_;
        queue_create_info.queueCount = 1;
        float queue_priorities = 1.0;
        queue_create_info.pQueuePriorities = &queue_priorities;

        VkDeviceCreateInfo device_create_info = {};

        VkPhysicalDeviceFeatures device_features = {};

        std::vector<const char*> device_extensions = {};

#if defined(__APPLE__) && defined(VK_KHR_portability_subset)
        std::uint32_t device_extension_count = 0;
        vkEnumerateDeviceExtensionProperties(physical_device_, nullptr,
                                             &device_extension_count, nullptr);
        if (device_extension_count > 0)
        {
            std::vector<VkExtensionProperties> extensions_props(device_extension_count);
            if (vkEnumerateDeviceExtensionProperties(physical_device_, nullptr,
                                                     &device_extension_count,
                                                     extensions_props.data()) == VK_SUCCESS)
            {
                for (VkExtensionProperties extension : extensions_props)
                {
                    if (std::strcmp(extension.extensionName, VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME) == 0)
                    {
                        device_extensions.push_back(VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
                        break;
                    }
                }
            }
        }
#endif

        device_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        device_create_info.enabledLayerCount = enabled_layers_.size();
        device_create_info.ppEnabledLayerNames = enabled_layers_.data();
        device_create_info.pQueueCreateInfos = &queue_create_info;
        device_create_info.queueCreateInfoCount = 1;
        device_create_info.pEnabledFeatures = &device_features;

        VkDevice device;
        VK_CHECK_RESULT(vkCreateDevice(physical_device_, &device_create_info, nullptr, &device));
        vkdevice_ = std::make_unique<vks::VulkanDevice>(physical_device_, device);

        vkGetDeviceQueue(vkdevice_->device, queue_family_index_, 0, &queue_);

        VkCommandPoolCreateInfo pool_create_info = {};
        pool_create_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        pool_create_info.flags = 0;
        pool_create_info.queueFamilyIndex = queue_family_index_;
        VK_CHECK_RESULT(vkCreateCommandPool(vkdevice_->device, &pool_create_info, nullptr,
                                            &vkdevice_->commandPool));
        depth_format_ = vkdevice_->getSupportedDepthFormat(false);
    }

    std::uint32_t find_memory_type(std::uint32_t memory_type_bits, VkMemoryPropertyFlags properties)
    {
        for (std::uint32_t i = 0; i < memory_properties_.memoryTypeCount; ++i) {
            if ((memory_type_bits & (1 << i)) &&
                ((memory_properties_.memoryTypes[i].propertyFlags & properties) == properties))
                return i;
        }
        return -1;
    }

    void create_textures(const cv::Size& images_size, const std::vector<cv::Mat>& texture_mats)
    {
        for (std::size_t i = 0; i < texture_mats.size(); ++i) {
            if (texture_mats[i].channels() != 3) {
                throw std::invalid_argument("Exactly 3 channels are required");
            }
        }

        std::size_t single_mat_size_bytes = 4 * images_size.width * images_size.height;
        std::size_t total_size_bytes = single_mat_size_bytes * texture_mats.size();
        VkFormat format = VK_FORMAT_R8G8B8A8_UNORM;

        VkBuffer staging_buffer = {};
        VkBufferCreateInfo buffer_create_info = {};
        buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        buffer_create_info.size = total_size_bytes;
        buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        VK_CHECK_RESULT(vkCreateBuffer(vkdevice_->device, &buffer_create_info, nullptr, &staging_buffer));

        VkMemoryRequirements memory_requirements = {};
        vkGetBufferMemoryRequirements(vkdevice_->device, staging_buffer, &memory_requirements);

        VkDeviceMemory staging_memory = {};
        VkMemoryAllocateInfo allocate_info = {};
        allocate_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocate_info.allocationSize = memory_requirements.size;
        allocate_info.memoryTypeIndex = find_memory_type(memory_requirements.memoryTypeBits,
                                                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |
                                                         VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);

        VK_CHECK_RESULT(vkAllocateMemory(vkdevice_->device, &allocate_info, nullptr, &staging_memory));
        VK_CHECK_RESULT(vkBindBufferMemory(vkdevice_->device, staging_buffer, staging_memory, 0));

        {
            std::uint8_t* memory_data_ptr = nullptr;
            VK_CHECK_RESULT(vkMapMemory(vkdevice_->device, staging_memory, 0, memory_requirements.size, 0,
                                        reinterpret_cast<void**>(&memory_data_ptr)));

            cv::Mat temp4ch(images_size.height, images_size.width, CV_8UC4);
            for (std::size_t i = 0; i < texture_mats.size(); ++i) {
                cv::cvtColor(texture_mats[i], temp4ch, cv::COLOR_BGR2BGRA);
                std::memcpy(memory_data_ptr + single_mat_size_bytes * i,
                            temp4ch.ptr(0), single_mat_size_bytes);
            }

            vkUnmapMemory(vkdevice_->device, staging_memory);
        }

        std::vector<VkBufferImageCopy> buffer_copy_regions;

        for (uint32_t i = 0; i < texture_mats.size(); i++) {
            // Setup a buffer image copy structure for the current mip level
            VkBufferImageCopy region = {};
            region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            region.imageSubresource.mipLevel = 0;
            region.imageSubresource.baseArrayLayer = i;
            region.imageSubresource.layerCount = 1;
            region.imageExtent.width = images_size.width;
            region.imageExtent.height = images_size.height;
            region.imageExtent.depth = 1;
            region.bufferOffset = single_mat_size_bytes * i;
            buffer_copy_regions.push_back(region);
        }

        // Create optimal tiled target image on the device
        VkImageCreateInfo image_create_info = {};
        image_create_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        image_create_info.imageType = VK_IMAGE_TYPE_2D;
        image_create_info.format = format;
        image_create_info.mipLevels = 1;
        image_create_info.arrayLayers = texture_mats.size();
        image_create_info.samples = VK_SAMPLE_COUNT_1_BIT;
        image_create_info.tiling = VK_IMAGE_TILING_OPTIMAL;
        image_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        image_create_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        image_create_info.extent.width = images_size.width;
        image_create_info.extent.height = images_size.height;
        image_create_info.extent.depth = 1;
        image_create_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        VK_CHECK_RESULT(vkCreateImage(vkdevice_->device, &image_create_info, nullptr, &src_image_array_));

        vkGetImageMemoryRequirements(vkdevice_->device, src_image_array_, &memory_requirements);

        allocate_info = {};
        allocate_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocate_info.allocationSize = memory_requirements.size;
        allocate_info.memoryTypeIndex = find_memory_type(memory_requirements.memoryTypeBits,
                                                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        VK_CHECK_RESULT(vkAllocateMemory(vkdevice_->device, &allocate_info, nullptr, &src_device_memory_));
        VK_CHECK_RESULT(vkBindImageMemory(vkdevice_->device, src_image_array_, src_device_memory_, 0));

        VkCommandBuffer copy_cmd = vkdevice_->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

        VkImageSubresourceRange subresource_range = {};
        subresource_range.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        subresource_range.baseMipLevel = 0;
        subresource_range.levelCount = 1;
        subresource_range.layerCount = texture_mats.size();


        VkImageMemoryBarrier image_memory_barrier {};
        image_memory_barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        image_memory_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        image_memory_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        image_memory_barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        image_memory_barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        image_memory_barrier.image = src_image_array_;
        image_memory_barrier.subresourceRange = subresource_range;
        image_memory_barrier.srcAccessMask = 0;
        image_memory_barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

        vkCmdPipelineBarrier(copy_cmd,
                             VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                             VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                             0,
                             0, nullptr,
                             0, nullptr,
                             1, &image_memory_barrier);

        vkCmdCopyBufferToImage(copy_cmd,
                               staging_buffer,
                               src_image_array_,
                               VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                               buffer_copy_regions.size(),
                               buffer_copy_regions.data());

        image_memory_barrier = {};
        image_memory_barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        image_memory_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        image_memory_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        image_memory_barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        image_memory_barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        image_memory_barrier.image = src_image_array_;
        image_memory_barrier.subresourceRange = subresource_range;
        image_memory_barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        image_memory_barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

        vkCmdPipelineBarrier(copy_cmd,
                             VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                             VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                             0,
                             0, nullptr,
                             0, nullptr,
                             1, &image_memory_barrier);


        vkdevice_->flushCommandBuffer(copy_cmd, queue_, true);

        VkSamplerCreateInfo sampler_create_info = {};
        sampler_create_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        sampler_create_info.magFilter = VK_FILTER_LINEAR;
        sampler_create_info.minFilter = VK_FILTER_LINEAR;
        sampler_create_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        sampler_create_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        sampler_create_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        sampler_create_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        sampler_create_info.mipLodBias = 0.0f;
        sampler_create_info.maxAnisotropy = 8;
        sampler_create_info.compareOp = VK_COMPARE_OP_NEVER;
        sampler_create_info.minLod = 0.0f;
        sampler_create_info.maxLod = 0.0f;
        sampler_create_info.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        VK_CHECK_RESULT(vkCreateSampler(vkdevice_->device, &sampler_create_info, nullptr, &src_sampler_));

        VkImageViewCreateInfo image_view_create_info = {};
        image_view_create_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        image_view_create_info.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
        image_view_create_info.format = format;
        image_view_create_info.components = { VK_COMPONENT_SWIZZLE_B, VK_COMPONENT_SWIZZLE_G,
                                              VK_COMPONENT_SWIZZLE_R, VK_COMPONENT_SWIZZLE_A };
        image_view_create_info.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
        image_view_create_info.subresourceRange.layerCount = texture_mats.size();
        image_view_create_info.subresourceRange.levelCount = 1;
        image_view_create_info.image = src_image_array_;
        VK_CHECK_RESULT(vkCreateImageView(vkdevice_->device, &image_view_create_info, nullptr, &src_image_view_));

        vkFreeMemory(vkdevice_->device, staging_memory, nullptr);
        vkDestroyBuffer(vkdevice_->device, staging_buffer, nullptr);
    }

    void create_uniform_buffers()
    {
        VK_CHECK_RESULT(vkdevice_->createBuffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                                VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                                &vertex_ubo_,
                                                sizeof(vertex_ubo_data_),
                                                &vertex_ubo_data_));
        VK_CHECK_RESULT(vertex_ubo_.map());

        vertex_ubo_data_.view_pos = glm::vec4(0.0f, 0.0f, 2.5f, 0.0f);
        vertex_ubo_data_.model_view = glm::translate(glm::mat4(1.0f),
                                                     glm::vec3(0.0f, 0.0f, -2.5f));
        vertex_ubo_data_.projection = glm::ortho(-result_width_/2.0f, result_width_/2.0f,
                                                 -result_height_/2.0f, result_height_/2.0f, 0.1f, 260.0f);

        std::memcpy(vertex_ubo_.mapped, &vertex_ubo_data_, sizeof(vertex_ubo_data_));
    }

    void create_descriptor_set_layout()
    {
        std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings =
        {
            // Binding 0 : Vertex shader uniform buffer
            vks::initializers::descriptorSetLayoutBinding(
                VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                VK_SHADER_STAGE_VERTEX_BIT,
                0),
            // Binding 1 : Fragment shader image sampler
            vks::initializers::descriptorSetLayoutBinding(
                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                VK_SHADER_STAGE_FRAGMENT_BIT,
                1)
        };

        VkDescriptorSetLayoutCreateInfo descriptorLayout =
            vks::initializers::descriptorSetLayoutCreateInfo(
                setLayoutBindings.data(),
                static_cast<uint32_t>(setLayoutBindings.size()));

        VK_CHECK_RESULT(vkCreateDescriptorSetLayout(vkdevice_->device, &descriptorLayout,
                                                    nullptr, &descriptor_set_layout_));

        VkPipelineLayoutCreateInfo pPipelineLayoutCreateInfo =
            vks::initializers::pipelineLayoutCreateInfo(
                &descriptor_set_layout_,
                1);
    }

    void create_descriptor_set()
    {
        std::vector<VkDescriptorPoolSize> pool_sizes;

        VkDescriptorPoolSize pool_size = {};
        pool_size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        pool_size.descriptorCount = 1;
        pool_sizes.push_back(pool_size);

        pool_size = {};
        pool_size.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        pool_size.descriptorCount = 1;
        pool_sizes.push_back(pool_size);

        pool_size = {};
        pool_size.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        pool_size.descriptorCount = 1;
        pool_sizes.push_back(pool_size);

        VkDescriptorPoolCreateInfo pool_create_info = {};
        pool_create_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        pool_create_info.maxSets = 1;
        pool_create_info.poolSizeCount = pool_sizes.size();
        pool_create_info.pPoolSizes = pool_sizes.data();

        VK_CHECK_RESULT(vkCreateDescriptorPool(vkdevice_->device, &pool_create_info,
                                               nullptr, &descriptor_pool_));

        VkDescriptorSetAllocateInfo allocInfo =
                vks::initializers::descriptorSetAllocateInfo(descriptor_pool_, &descriptor_set_layout_, 1);
        VK_CHECK_RESULT(vkAllocateDescriptorSets(vkdevice_->device, &allocInfo, &descriptor_set_));

        // Image descriptor for the texture array
        VkDescriptorImageInfo texture_descriptor =
            vks::initializers::descriptorImageInfo(
                src_sampler_,
                src_image_view_,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

        std::vector<VkWriteDescriptorSet> writeDescriptorSets = {
            // Binding 0 : Vertex shader uniform buffer
            vks::initializers::writeDescriptorSet(descriptor_set_, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            0, &vertex_ubo_.descriptor),
            // Binding 1 : Fragment shader cubemap sampler
            vks::initializers::writeDescriptorSet(descriptor_set_, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            1, &texture_descriptor)
        };
        vkUpdateDescriptorSets(vkdevice_->device, writeDescriptorSets.size(), writeDescriptorSets.data(), 0, NULL);
    }

    void create_vertex_buffers(const RenderingUnfoldedInfo& info)
    {
        // Setup vertices for a single uv-mapped quad made from two triangles
        std::vector<Vertex> vertices;
        vertices.reserve(info.landmark_positions_in_dest.size());

        int count = 0;
        for (std::size_t i = 0; i < info.landmark_positions_in_dest.size(); ++i) {
            // FIXME: currently only the first image is used
            std::size_t obs_index = 0;
            const auto& res_pos = info.landmark_positions_in_dest[i];
            const auto& uv_pos = info.landmark_positions_in_views(i, obs_index);

            vertices.push_back({{static_cast<float>(res_pos(0)),
                                 static_cast<float>(res_pos(1)), 0.0f},
                                {static_cast<float>(uv_pos(0)) / info.images_size.width,
                                 static_cast<float>(uv_pos(1)) / info.images_size.height}});

            if (count++ < 10) {
                std::cout << "VERT " << res_pos(0) << " " << res_pos(1) << " " <<
                             static_cast<float>(uv_pos(0)) / info.images_size.width << " " <<
                             static_cast<float>(uv_pos(1)) / info.images_size.height << std::endl;
            }
        }

        // Setup indices
        std::vector<std::uint32_t> indices;
        indices.reserve(info.triangle_indices.size() * 3);
        for (const auto& triangle : info.triangle_indices) {
            indices.push_back(triangle.indices[0]);
            indices.push_back(triangle.indices[1]);
            indices.push_back(triangle.indices[2]);
        }
        index_count_ = static_cast<uint32_t>(indices.size());

        // Create buffers
        // For the sake of simplicity we won't stage the vertex data to the gpu memory
        // Vertex buffer
        VK_CHECK_RESULT(vkdevice_->createBuffer(
            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            &vertex_buffer_,
            vertices.size() * sizeof(Vertex),
            vertices.data()));
        // Index buffer
        VK_CHECK_RESULT(vkdevice_->createBuffer(
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            &index_buffer_,
            indices.size() * sizeof(uint32_t),
            indices.data()));
    }

    VkPipelineShaderStageCreateInfo load_shader(const std::uint8_t* bytes, unsigned size,
                                                VkShaderStageFlagBits stage)
    {
        VkShaderModule shader_module;
        VkShaderModuleCreateInfo shader_module_create_info = {};
        shader_module_create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        shader_module_create_info.pCode = reinterpret_cast<const std::uint32_t*>(bytes);
        shader_module_create_info.codeSize = size;

        VK_CHECK_RESULT(vkCreateShaderModule(vkdevice_->device, &shader_module_create_info,
                                             nullptr, &shader_module));

        VkPipelineShaderStageCreateInfo shader_stage_create_info = {};
        shader_stage_create_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        shader_stage_create_info.stage = stage;
        shader_stage_create_info.module = shader_module;
        shader_stage_create_info.pName = "main";

        shader_modules_.push_back(shader_module);
        return shader_stage_create_info;
    }

    void create_compute_pipeline()
    {
        std::vector<VkPipelineShaderStageCreateInfo> shader_stages;
        shader_stages.push_back(load_shader(render_unfolded_frag_spv, render_unfolded_frag_spv_len,
                                            VK_SHADER_STAGE_FRAGMENT_BIT));
        shader_stages.push_back(load_shader(render_unfolded_vert_spv, render_unfolded_vert_spv_len,
                                            VK_SHADER_STAGE_VERTEX_BIT));

        VkPipelineLayoutCreateInfo layout_create_info = {};
        layout_create_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        layout_create_info.setLayoutCount = 1;
        layout_create_info.pSetLayouts = &descriptor_set_layout_;
        VK_CHECK_RESULT(vkCreatePipelineLayout(vkdevice_->device, &layout_create_info,
                                               nullptr, &pipeline_layout_));

        VkPipelineInputAssemblyStateCreateInfo input_assembly_stage {};
        input_assembly_stage.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        input_assembly_stage.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        input_assembly_stage.flags = 0;
        input_assembly_stage.primitiveRestartEnable = VK_FALSE;


        VkPipelineRasterizationStateCreateInfo rasterization_state {};
        rasterization_state.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        rasterization_state.polygonMode = VK_POLYGON_MODE_FILL;
        rasterization_state.cullMode = VK_CULL_MODE_NONE;
        rasterization_state.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rasterization_state.flags = 0;
        rasterization_state.depthClampEnable = VK_FALSE;
        rasterization_state.lineWidth = 1.0f;


        VkPipelineColorBlendAttachmentState blend_attachment_state {};
        blend_attachment_state.colorWriteMask = 0xf;
        blend_attachment_state.blendEnable = VK_FALSE;


        VkPipelineColorBlendStateCreateInfo color_blend_state {};
        color_blend_state.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        color_blend_state.attachmentCount = 1;
        color_blend_state.pAttachments = &blend_attachment_state;

        VkPipelineDepthStencilStateCreateInfo depth_stencil_state {};
        depth_stencil_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        depth_stencil_state.depthTestEnable = VK_TRUE;
        depth_stencil_state.depthWriteEnable = VK_TRUE;
        depth_stencil_state.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        depth_stencil_state.back.compareOp = VK_COMPARE_OP_ALWAYS;

        VkPipelineViewportStateCreateInfo viewport_state {};
        viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        viewport_state.viewportCount = 1;
        viewport_state.scissorCount = 1;
        viewport_state.flags = 0;

        VkPipelineMultisampleStateCreateInfo multisample_state {};
        multisample_state.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        multisample_state.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        multisample_state.flags = 0;

        std::vector<VkDynamicState> dynamic_state_enables = {
            VK_DYNAMIC_STATE_VIEWPORT,
            VK_DYNAMIC_STATE_SCISSOR
        };

        VkPipelineDynamicStateCreateInfo dynamic_state {};
        dynamic_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamic_state.pDynamicStates = dynamic_state_enables.data();
        dynamic_state.dynamicStateCount = dynamic_state_enables.size();
        dynamic_state.flags = 0;

        // Vertex bindings and attributes
        VkVertexInputBindingDescription vertex_input_binding = {};
        vertex_input_binding.binding = 0;
        vertex_input_binding.stride = sizeof(Vertex);
        vertex_input_binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        std::vector<VkVertexInputAttributeDescription> vertex_input_attrs = {
            { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, pos) },
            { 1, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(Vertex, uv) },
        };

        VkPipelineVertexInputStateCreateInfo vertex_input_state = {};
        vertex_input_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vertex_input_state.vertexBindingDescriptionCount = 1;
        vertex_input_state.pVertexBindingDescriptions = &vertex_input_binding;
        vertex_input_state.vertexAttributeDescriptionCount = vertex_input_attrs.size();
        vertex_input_state.pVertexAttributeDescriptions = vertex_input_attrs.data();

        VkGraphicsPipelineCreateInfo pipeline_create_info = {};
        pipeline_create_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipeline_create_info.layout = pipeline_layout_;
        pipeline_create_info.renderPass = render_pass_;
        pipeline_create_info.flags = 0;
        pipeline_create_info.basePipelineIndex = -1;
        pipeline_create_info.basePipelineHandle = VK_NULL_HANDLE;
        pipeline_create_info.pVertexInputState = &vertex_input_state;
        pipeline_create_info.pInputAssemblyState = &input_assembly_stage;
        pipeline_create_info.pRasterizationState = &rasterization_state;
        pipeline_create_info.pColorBlendState = &color_blend_state;
        pipeline_create_info.pMultisampleState = &multisample_state;
        pipeline_create_info.pViewportState = &viewport_state;
        pipeline_create_info.pDepthStencilState = &depth_stencil_state;
        pipeline_create_info.pDynamicState = &dynamic_state;
        pipeline_create_info.stageCount = shader_stages.size();
        pipeline_create_info.pStages = shader_stages.data();

        VK_CHECK_RESULT(vkCreateGraphicsPipelines(vkdevice_->device, nullptr, 1, &pipeline_create_info,
                                                  nullptr, &pipeline_));
    }

    void create_color_attachment()
    {
        auto format = VK_FORMAT_B8G8R8A8_UNORM;

        VkImageCreateInfo image = vks::initializers::imageCreateInfo();
        image.imageType = VK_IMAGE_TYPE_2D;
        image.format = format;
        image.extent.width = result_width_;
        image.extent.height = result_height_;
        image.extent.depth = 1;
        image.mipLevels = 1;
        image.arrayLayers = 1;
        image.samples = VK_SAMPLE_COUNT_1_BIT;
        image.tiling = VK_IMAGE_TILING_OPTIMAL;
        image.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

        VkMemoryAllocateInfo memAlloc = vks::initializers::memoryAllocateInfo();
        VkMemoryRequirements memReqs;

        VK_CHECK_RESULT(vkCreateImage(vkdevice_->device, &image, nullptr, &color_attachment_.image));
        vkGetImageMemoryRequirements(vkdevice_->device, color_attachment_.image, &memReqs);
        memAlloc.allocationSize = memReqs.size;
        memAlloc.memoryTypeIndex = vkdevice_->getMemoryType(memReqs.memoryTypeBits,
                                                            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        VK_CHECK_RESULT(vkAllocateMemory(vkdevice_->device, &memAlloc, nullptr, &color_attachment_.memory));
        VK_CHECK_RESULT(vkBindImageMemory(vkdevice_->device, color_attachment_.image, color_attachment_.memory, 0));

        VkImageViewCreateInfo colorImageView = vks::initializers::imageViewCreateInfo();
        colorImageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
        colorImageView.format = format;
        colorImageView.subresourceRange = {};
        colorImageView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        colorImageView.subresourceRange.baseMipLevel = 0;
        colorImageView.subresourceRange.levelCount = 1;
        colorImageView.subresourceRange.baseArrayLayer = 0;
        colorImageView.subresourceRange.layerCount = 1;
        colorImageView.image = color_attachment_.image;
        VK_CHECK_RESULT(vkCreateImageView(vkdevice_->device, &colorImageView, nullptr,
                                          &color_attachment_.view));
    }

    void create_depth_attachment()
    {
        VkImageCreateInfo imageCI{};
        imageCI.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageCI.imageType = VK_IMAGE_TYPE_2D;
        imageCI.format = depth_format_;
        imageCI.extent = { result_width_, result_height_, 1 };
        imageCI.mipLevels = 1;
        imageCI.arrayLayers = 1;
        imageCI.samples = VK_SAMPLE_COUNT_1_BIT;
        imageCI.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageCI.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;

        VK_CHECK_RESULT(vkCreateImage(vkdevice_->device, &imageCI, nullptr,
                                      &depth_attachment_.image));
        VkMemoryRequirements memReqs{};
        vkGetImageMemoryRequirements(vkdevice_->device, depth_attachment_.image, &memReqs);

        VkMemoryAllocateInfo memAllloc{};
        memAllloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        memAllloc.allocationSize = memReqs.size;
        memAllloc.memoryTypeIndex = vkdevice_->getMemoryType(memReqs.memoryTypeBits,
                                                             VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        VK_CHECK_RESULT(vkAllocateMemory(vkdevice_->device, &memAllloc, nullptr,
                                         &depth_attachment_.memory));
        VK_CHECK_RESULT(vkBindImageMemory(vkdevice_->device,
                                          depth_attachment_.image, depth_attachment_.memory, 0));

        VkImageViewCreateInfo imageViewCI{};
        imageViewCI.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        imageViewCI.viewType = VK_IMAGE_VIEW_TYPE_2D;
        imageViewCI.image = depth_attachment_.image;
        imageViewCI.format = depth_format_;
        imageViewCI.subresourceRange.baseMipLevel = 0;
        imageViewCI.subresourceRange.levelCount = 1;
        imageViewCI.subresourceRange.baseArrayLayer = 0;
        imageViewCI.subresourceRange.layerCount = 1;
        imageViewCI.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
        // Stencil aspect should only be set on depth + stencil formats (VK_FORMAT_D16_UNORM_S8_UINT..VK_FORMAT_D32_SFLOAT_S8_UINT
        if (depth_format_ >= VK_FORMAT_D16_UNORM_S8_UINT) {
            imageViewCI.subresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
        }
        VK_CHECK_RESULT(vkCreateImageView(vkdevice_->device, &imageViewCI, nullptr,
                                          &depth_attachment_.view));
    }

    void create_render_pass()
    {
        std::array<VkAttachmentDescription, 2> attachments = {};
        // Color attachment
        attachments[0].format = VK_FORMAT_B8G8R8A8_UNORM;
        attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
        attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[0].finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        // Depth attachment
        attachments[1].format = depth_format_;
        attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
        attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentReference colorReference = {};
        colorReference.attachment = 0;
        colorReference.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

        VkAttachmentReference depthReference = {};
        depthReference.attachment = 1;
        depthReference.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkSubpassDescription subpassDescription = {};
        subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpassDescription.colorAttachmentCount = 1;
        subpassDescription.pColorAttachments = &colorReference;
        subpassDescription.pDepthStencilAttachment = &depthReference;
        subpassDescription.inputAttachmentCount = 0;
        subpassDescription.pInputAttachments = nullptr;
        subpassDescription.preserveAttachmentCount = 0;
        subpassDescription.pPreserveAttachments = nullptr;
        subpassDescription.pResolveAttachments = nullptr;

        // Subpass dependencies for layout transitions
        std::array<VkSubpassDependency, 2> dependencies;

        dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[0].dstSubpass = 0;
        dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
        dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

        dependencies[1].srcSubpass = 0;
        dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
        dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

        VkRenderPassCreateInfo renderPassInfo = {};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        renderPassInfo.pAttachments = attachments.data();
        renderPassInfo.subpassCount = 1;
        renderPassInfo.pSubpasses = &subpassDescription;
        renderPassInfo.dependencyCount = static_cast<uint32_t>(dependencies.size());
        renderPassInfo.pDependencies = dependencies.data();

        VK_CHECK_RESULT(vkCreateRenderPass(vkdevice_->device, &renderPassInfo, nullptr, &render_pass_));
    }

    void create_command_buffer()
    {
        VkCommandBufferAllocateInfo allocate_info = {};
        allocate_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocate_info.commandPool = vkdevice_->commandPool;
        allocate_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocate_info.commandBufferCount = 1; // allocate a single command buffer.
        VK_CHECK_RESULT(vkAllocateCommandBuffers(vkdevice_->device, &allocate_info, &command_buffer_));

        VkCommandBufferBeginInfo begin_info = vks::initializers::commandBufferBeginInfo();
        begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        VK_CHECK_RESULT(vkBeginCommandBuffer(command_buffer_, &begin_info));

        vkCmdBindPipeline(command_buffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
        vkCmdBindDescriptorSets(command_buffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_layout_,
                                0, 1, &descriptor_set_, 0, nullptr);

        VkImageView attachments[2];
        attachments[0] = color_attachment_.view;
        attachments[1] = depth_attachment_.view;

        VkClearValue clearValues[2];
        clearValues[0].color = { { 0.025f, 0.025f, 0.025f, 1.0f } };
        clearValues[1].depthStencil = { 1.0f, 0 };

        VkFramebufferCreateInfo framebufferCreateInfo = vks::initializers::framebufferCreateInfo();
        framebufferCreateInfo.renderPass = render_pass_;
        framebufferCreateInfo.attachmentCount = 2;
        framebufferCreateInfo.pAttachments = attachments;
        framebufferCreateInfo.width = result_width_;
        framebufferCreateInfo.height = result_height_;
        framebufferCreateInfo.layers = 1;
        VK_CHECK_RESULT(vkCreateFramebuffer(vkdevice_->device, &framebufferCreateInfo, nullptr,
                                            &framebuffer_));

        VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
        renderPassBeginInfo.renderPass = render_pass_;
        renderPassBeginInfo.renderArea.offset.x = 0;
        renderPassBeginInfo.renderArea.offset.y = 0;
        renderPassBeginInfo.renderArea.extent.width = result_width_;
        renderPassBeginInfo.renderArea.extent.height = result_height_;
        renderPassBeginInfo.clearValueCount = 2;
        renderPassBeginInfo.pClearValues = clearValues;
        renderPassBeginInfo.framebuffer = framebuffer_;

        vkCmdBeginRenderPass(command_buffer_, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

        VkViewport viewport = {};
        viewport.height = (float)result_height_;
        viewport.width = (float)result_width_;;
        viewport.minDepth = (float)0.0f;
        viewport.maxDepth = (float)1.0f;
        vkCmdSetViewport(command_buffer_, 0, 1, &viewport);

        // Update dynamic scissor state
        VkRect2D scissor = {};
        scissor.extent.width = result_width_;
        scissor.extent.height = result_height_;
        vkCmdSetScissor(command_buffer_, 0, 1, &scissor);

        vkCmdBindPipeline(command_buffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);

        vkCmdBindDescriptorSets(command_buffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_layout_,
                                0, 1, &descriptor_set_, 0, nullptr);
        vkCmdBindPipeline(command_buffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);

        VkDeviceSize offsets[1] = { 0 };
        vkCmdBindVertexBuffers(command_buffer_, 0, 1, &vertex_buffer_.buffer, offsets);
        vkCmdBindIndexBuffer(command_buffer_, index_buffer_.buffer, 0, VK_INDEX_TYPE_UINT32);

        vkCmdDrawIndexed(command_buffer_, index_count_, 1, 0, 0, 0);

        vkCmdEndRenderPass(command_buffer_);

        VK_CHECK_RESULT(vkEndCommandBuffer(command_buffer_));
    }

    void run_command_buffer(VkCommandBuffer command_buffer)
    {
        VkSubmitInfo submit_info = {};
        submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit_info.commandBufferCount = 1;
        submit_info.pCommandBuffers = &command_buffer;

        VkFence fence;
        VkFenceCreateInfo fence_create_info = {};
        fence_create_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        fence_create_info.flags = 0;
        VK_CHECK_RESULT(vkCreateFence(vkdevice_->device, &fence_create_info, nullptr, &fence));
        VK_CHECK_RESULT(vkQueueSubmit(queue_, 1, &submit_info, fence));
        VK_CHECK_RESULT(vkWaitForFences(vkdevice_->device, 1, &fence, VK_TRUE, 100000000000));

        vkDestroyFence(vkdevice_->device, fence, nullptr);
    }

    cv::Mat save_rendered_image()
    {
            // Create the linear tiled destination image to copy to and to read the memory from
        VkImageCreateInfo image_create_info(vks::initializers::imageCreateInfo());
        image_create_info.imageType = VK_IMAGE_TYPE_2D;
        image_create_info.format = VK_FORMAT_R8G8B8A8_UNORM;
        image_create_info.extent.width = result_width_;
        image_create_info.extent.height = result_height_;
        image_create_info.extent.depth = 1;
        image_create_info.arrayLayers = 1;
        image_create_info.mipLevels = 1;
        image_create_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        image_create_info.samples = VK_SAMPLE_COUNT_1_BIT;
        image_create_info.tiling = VK_IMAGE_TILING_LINEAR;
        image_create_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        // Create the image
        VkImage dst_image;
        VK_CHECK_RESULT(vkCreateImage(vkdevice_->device, &image_create_info, nullptr, &dst_image));
        // Create memory to back up the image
        VkMemoryRequirements mem_req;
        VkMemoryAllocateInfo mem_alloc_info(vks::initializers::memoryAllocateInfo());
        VkDeviceMemory dst_image_memory;
        vkGetImageMemoryRequirements(vkdevice_->device, dst_image, &mem_req);
        mem_alloc_info.allocationSize = mem_req.size;
        // Memory must be host visible to copy from
        mem_alloc_info.memoryTypeIndex = vkdevice_->getMemoryType(mem_req.memoryTypeBits,
                                                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                                                  VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

        VK_CHECK_RESULT(vkAllocateMemory(vkdevice_->device, &mem_alloc_info, nullptr, &dst_image_memory));
        VK_CHECK_RESULT(vkBindImageMemory(vkdevice_->device, dst_image, dst_image_memory, 0));

        // Do the actual blit from the offscreen image to our host visible destination image
        VkCommandBufferAllocateInfo cmdBufAllocateInfo =
                vks::initializers::commandBufferAllocateInfo(vkdevice_->commandPool,
                                                             VK_COMMAND_BUFFER_LEVEL_PRIMARY, 1);
        VkCommandBuffer copy_cmd;
        VK_CHECK_RESULT(vkAllocateCommandBuffers(vkdevice_->device, &cmdBufAllocateInfo, &copy_cmd));
        VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();
        VK_CHECK_RESULT(vkBeginCommandBuffer(copy_cmd, &cmdBufInfo));

        // Transition destination image to transfer destination layout
        vks::tools::insertImageMemoryBarrier(
            copy_cmd,
            dst_image,
            0,
            VK_ACCESS_TRANSFER_WRITE_BIT,
            VK_IMAGE_LAYOUT_UNDEFINED,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 });

        // colorAttachment.image is already in VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        // and does not need to be transitioned

        VkImageCopy imageCopyRegion{};
        imageCopyRegion.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        imageCopyRegion.srcSubresource.layerCount = 1;
        imageCopyRegion.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        imageCopyRegion.dstSubresource.layerCount = 1;
        imageCopyRegion.extent.width = result_width_;
        imageCopyRegion.extent.height = result_height_;
        imageCopyRegion.extent.depth = 1;

        vkCmdCopyImage(
            copy_cmd,
            color_attachment_.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            dst_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            1,
            &imageCopyRegion);

        // Transition destination image to general layout, which is the required layout
        // for mapping the image memory later on
        vks::tools::insertImageMemoryBarrier(
            copy_cmd,
            dst_image,
            VK_ACCESS_TRANSFER_WRITE_BIT,
            VK_ACCESS_MEMORY_READ_BIT,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VkImageSubresourceRange{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 });

        VK_CHECK_RESULT(vkEndCommandBuffer(copy_cmd));

        run_command_buffer(copy_cmd);

        // Get layout of the image (including row pitch)
        VkImageSubresource subResource{};
        subResource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        VkSubresourceLayout sub_resource_layout;

        vkGetImageSubresourceLayout(vkdevice_->device, dst_image, &subResource, &sub_resource_layout);

        // Map image memory so we can start copying from it
        std::uint8_t* imagedata = nullptr;
        vkMapMemory(vkdevice_->device, dst_image_memory, 0, VK_WHOLE_SIZE, 0, (void**)&imagedata);
        imagedata += sub_resource_layout.offset;

        cv::Mat image_ref(result_height_, result_width_, CV_8UC4, imagedata,
                          sub_resource_layout.rowPitch);
        cv::Mat res;
        cv::cvtColor(image_ref, res, cv::COLOR_RGBA2BGR);

        vkUnmapMemory(vkdevice_->device, dst_image_memory);
        vkFreeMemory(vkdevice_->device, dst_image_memory, nullptr);
        vkDestroyImage(vkdevice_->device, dst_image, nullptr);

        return res;
    }

    void cleanup()
    {
        if (enable_validation_layers_) {
            auto vkDestroyDebugReportCallbackEXT = reinterpret_cast<PFN_vkDestroyDebugReportCallbackEXT>(
                        vkGetInstanceProcAddr(instance_, "vkDestroyDebugReportCallbackEXT"));
            if (vkDestroyDebugReportCallbackEXT == nullptr) {
                throw std::runtime_error("vkDestroyDebugReportCallbackEXT is not present");
            }
            vkDestroyDebugReportCallbackEXT(instance_, debug_report_callback_, nullptr);
        }

        vkDestroyImageView(vkdevice_->device, src_image_view_, nullptr);
        vkDestroyImage(vkdevice_->device, src_image_array_, nullptr);
        vkDestroySampler(vkdevice_->device, src_sampler_, nullptr);
        vkFreeMemory(vkdevice_->device, src_device_memory_, nullptr);

        vkDestroyFramebuffer(vkdevice_->device, framebuffer_, nullptr);

        vertex_buffer_.destroy();
        index_buffer_.destroy();
        vertex_ubo_.destroy();
        vkDestroyImageView(vkdevice_->device, color_attachment_.view, nullptr);
        vkFreeMemory(vkdevice_->device, color_attachment_.memory, nullptr);
        vkDestroyImage(vkdevice_->device, color_attachment_.image, nullptr);
        vkDestroyImageView(vkdevice_->device, depth_attachment_.view, nullptr);
        vkFreeMemory(vkdevice_->device, depth_attachment_.memory, nullptr);
        vkDestroyImage(vkdevice_->device, depth_attachment_.image, nullptr);

        for (auto shader_module : shader_modules_) {
            vkDestroyShaderModule(vkdevice_->device, shader_module, nullptr);
        }
        vkDestroyDescriptorPool(vkdevice_->device, descriptor_pool_, nullptr);
        vkDestroyDescriptorSetLayout(vkdevice_->device, descriptor_set_layout_, nullptr);
        vkDestroyRenderPass(vkdevice_->device, render_pass_, nullptr);
        vkDestroyPipelineLayout(vkdevice_->device, pipeline_layout_, nullptr);
        vkDestroyPipeline(vkdevice_->device, pipeline_, nullptr);
        vkDestroyCommandPool(vkdevice_->device, vkdevice_->commandPool, nullptr);
        vkDestroyDevice(vkdevice_->device, nullptr);
        vkDestroyInstance(instance_, nullptr);

        vkdevice_.reset();
    }
};

cv::Mat render_unfolded_images(const RenderingUnfoldedInfo& info)
{
    // FIXME: this sets up whole Vulkan runtime and could be faster if this is done during
    // app initialization and not on every run.
    UnfoldedImageRendererVulkan renderer;
    return renderer.run(info);
}

} // namespace sanescan
