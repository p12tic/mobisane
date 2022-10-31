# SPDX-License-Identifier: GPL-3.0-or-later
#
# Copyright (C) 2022  Povilas Kanapickas <povilas@radix.lt>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

set(SOURCES
    bounds_detection_params.cc
    bounds_detection_pipeline.cc
    edge_calculator_precise.cc
    export_ply.cc
    longest_line_recognizer.cc
    mean_flood_fill.cc
    feature_extraction_job.cc
    flood_fill_utils.cc
    geometry_utils.cc
    image_debug_utils.cc
    image_utils.cc
    edge_sfm.cc
    edge_utils.cc
    parallel_alicevision.cc
    parallel_tesseract.cc
    segment_calculator_precise.cc
    sfm_refpoints.cc
    shared_app_manager.cc
    vulkan_render_unfolded.cc

    bff/Bff.cpp
    bff/Corner.cpp
    bff/Distortion.cpp
    bff/Edge.cpp
    bff/Face.cpp
    bff/HalfEdge.cpp
    bff/Mesh.cpp
    bff/MeshIO.cpp
    bff/Vertex.cpp

    edgegraph3d/plgs/polyline_2d.cpp
    edgegraph3d/plgs/graph_adjacency_set_undirected_no_type_weighted.cpp
    edgegraph3d/plgs/graph_adjacency_set_no_type.cpp
    edgegraph3d/plgs/polyline_3d.cpp
    edgegraph3d/plgs/polyline_graph_2d_hmap_impl.cpp
    edgegraph3d/plgs/polyline_graph_3d.cpp
    edgegraph3d/plgs/polyline_graph_3d_hmap_impl.cpp
    edgegraph3d/plgs/graph_no_type.cpp
    edgegraph3d/plgs/polyline_graph_2d.cpp
    edgegraph3d/plgs/graph_adjacency_set_undirected_no_type.cpp
    edgegraph3d/utils/edge_graph_3d_utilities.cpp
    edgegraph3d/utils/triangulation.cpp
    edgegraph3d/utils/geometric_utilities.cpp
    edgegraph3d/utils/drawing_utilities.cpp
    edgegraph3d/matching/plg_matching/plg_matching.cpp
    edgegraph3d/matching/plg_matching/polyline_matching.cpp
    edgegraph3d/matching/plg_matching/plg_matches_manager.cpp
    edgegraph3d/matching/plg_matching/polyLine_2d_map.cpp
    edgegraph3d/matching/plg_matching/pipelines.cpp
    edgegraph3d/matching/plg_matching/polyLine_2d_map_search.cpp
    edgegraph3d/matching/plg_matching/plg_matching_from_refpoints.cpp
    edgegraph3d/matching/polyline_matching/polyline_matcher.cpp
    edgegraph3d/matching/consensus_manager/plgp_consensus_manager.cpp
    edgegraph3d/matching/consensus_manager/plgpcm_3views_plg_following.cpp
    edgegraph3d/filtering/gauss_newton.cpp
    edgegraph3d/filtering/outliers_filtering.cpp
    edgegraph3d/filtering/filtering_close_plgps.cpp
    edgegraph3d/plg_edge_manager.cpp
    edgegraph3d/io/input/convert_edge_images_pixel_to_segment.cpp

    vkutils/VulkanAndroid.cpp
    vkutils/VulkanBuffer.cpp
    vkutils/VulkanDebug.cpp
    vkutils/VulkanDevice.cpp
)

set(PUBLIC_HEADERS
    bounds_detection_params.h
    bounds_detection_pipeline.h
    edge_calculator_precise.h
    edge_sfm.h
    edge_utils.h
    export_ply.h
    feature_extraction_job.h
    geometry_utils.h
    flood_fill_utils.h
    image_debug_utils.h
    image_utils.h
    longest_line_recognizer.h
    mean_flood_fill.h
    parallel_alicevision.h
    parallel_tesseract.h
    segment_calculator_precise.h
    sfm_refpoints.h
    shared_app_manager.h
    vulkan_render_unfolded.h

    bff/Bff.h
    bff/Cholesky.h
    bff/Corner.h
    bff/CutIterator.h
    bff/DenseMatrix.h
    bff/Distortion.h
    bff/Edge.h
    bff/Face.h
    bff/HalfEdge.h
    bff/MeshData.h
    bff/Mesh.h
    bff/MeshIO.h
    bff/SparseMatrix.h
    bff/Types.h
    bff/Vector.h
    bff/Vertex.h

    edgegraph3d/plg_edge_manager.hpp
    edgegraph3d/filtering/filtering_close_plgps.hpp
    edgegraph3d/filtering/gauss_newton.hpp
    edgegraph3d/filtering/outliers_filtering.hpp
    edgegraph3d/matching/consensus_manager/plgpcm_3views_plg_following.hpp
    edgegraph3d/matching/consensus_manager/plgp_consensus_manager.hpp
    edgegraph3d/matching/plg_matching/pipelines.hpp
    edgegraph3d/matching/plg_matching/plg_matches_manager.hpp
    edgegraph3d/matching/plg_matching/plg_matching_from_refpoints.hpp
    edgegraph3d/matching/plg_matching/plg_matching.hpp
    edgegraph3d/matching/plg_matching/polyline_2d_map.hpp
    edgegraph3d/matching/plg_matching/polyline_2d_map_search.hpp
    edgegraph3d/matching/plg_matching/polyline_matching.hpp
    edgegraph3d/matching/polyline_matching/polyline_matcher.hpp
    edgegraph3d/plgs/graph_adjacency_list.hpp
    edgegraph3d/plgs/graph_adjacency_set_no_type.hpp
    edgegraph3d/plgs/graph_adjacency_set_undirected_no_type.hpp
    edgegraph3d/plgs/graph_adjacency_set_undirected_no_type_weighted.hpp
    edgegraph3d/plgs/graph.hpp
    edgegraph3d/plgs/graph_no_type.hpp
    edgegraph3d/plgs/polyline_2d.hpp
    edgegraph3d/plgs/polyline_3d.hpp
    edgegraph3d/plgs/polyline_graph_2d_hmap_impl.hpp
    edgegraph3d/plgs/polyline_graph_2d.hpp
    edgegraph3d/plgs/polyline_graph_3d_hmap_impl.hpp
    edgegraph3d/plgs/polyline_graph_3d.hpp
    edgegraph3d/plgs/polyline_graph_point_2d.hpp
    edgegraph3d/plgs/polyline_interval_2d.hpp
    edgegraph3d/plgs/polyline_point_2d.hpp
    edgegraph3d/sfm_data.h
    edgegraph3d/types.hpp
    edgegraph3d/utils/data_bundle.hpp
    edgegraph3d/utils/datatypes.hpp
    edgegraph3d/utils/drawing_utilities.hpp
    edgegraph3d/utils/edge_graph_3d_utilities.hpp
    edgegraph3d/utils/geometric_utilities.hpp
    edgegraph3d/utils/triangulation.hpp
    edgegraph3d/utils/globals/global_defines.hpp

    vkutils/VulkanAndroid.h
    vkutils/VulkanBuffer.h
    vkutils/VulkanDebug.h
    vkutils/VulkanDevice.h
    vkutils/VulkanInitializers.h
)

find_package(OpenCV REQUIRED)
find_package(AliceVision REQUIRED)
find_package(Taskflow REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sanescan REQUIRED)

# FIXME: fix AliceVision config to impor OpenEXR
find_package(OpenEXR REQUIRED)

add_library(mobisanecommon ${SOURCES})

# Disable openmp support
target_compile_definitions(mobisanecommon PRIVATE -DEIGEN_DONT_PARALLELIZE=1)
# Disable <execution> usage in boost. Some implementations use system tbb which then may conflict
# with what version is in 3rdparty/tbb
target_compile_definitions(mobisanecommon PRIVATE -DBOOST_NO_CXX17_HDR_EXECUTION=1)

if(MOBISANE_ENABLE_VULKAN_VALIDATION)
    target_compile_definitions(mobisanecommon PRIVATE -DMOBISANE_ENABLE_VULKAN_VALIDATION=1)
endif()

target_include_directories(mobisanecommon PUBLIC
    $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/common>)

target_link_libraries(mobisanecommon PUBLIC
    ${OpenCV_LIBS}
    aliceVision_numeric
    aliceVision_imageMatching
    aliceVision_matchingImageCollection
    aliceVision_sfm
    aliceVision_vfs
    vulkan
    Sanescan::sanescanocr
)
target_link_directories(mobisanecommon PUBLIC ${CMAKE_PREFIX_PATH}/lib)

target_include_directories(mobisanecommon PUBLIC
    ${CMAKE_INSTALL_PREFIX}/include
    ${OpenCV_INCLUDE_DIRS}
)

install(
    FILES ${PUBLIC_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mobisane
)
