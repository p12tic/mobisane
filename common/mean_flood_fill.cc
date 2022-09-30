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

#include "mean_flood_fill.h"
#include "algorithm.h"
#include <sanescanocr/ocr/ocr_point.h>
#include <boost/container/deque.hpp>
#include <array>
#include <bit>

#if __NEON__
#include <arm_neon.h>
#endif

namespace sanescan {

constexpr std::size_t HISTOGRAM_BUCKET_SIZE = 8;
constexpr std::size_t HISTOGRAM_BUCKET_COUNT = 256 / HISTOGRAM_BUCKET_SIZE;

using FillHistogram = std::array<std::uint32_t, HISTOGRAM_BUCKET_COUNT>;
using DequeBlockOption = typename boost::container::deque_options<boost::container::block_size<1024*32>>::type;
using PointContainer = boost::container::deque<OcrPoint, void, DequeBlockOption>;

struct ResolvedFillLimits {
    std::uint8_t hue_diff = 0;
    std::uint8_t sat_diff = 0;
    std::uint8_t value_diff = 0;
};

ResolvedFillLimits resolve_limits(float max_hue, float max_sat, float max_value)
{
    return {
        static_cast<std::uint8_t>(std::clamp(static_cast<int>(max_hue * 255), 0, 255)),
        static_cast<std::uint8_t>(std::clamp(static_cast<int>(max_sat * 255), 0, 255)),
        static_cast<std::uint8_t>(std::clamp(static_cast<int>(max_value * 255), 0, 255)),
    };
}

std::array<FillHistogram, 3> compute_histograms(const cv::Mat& image, const OcrBox& bounds)
{
    std::array<FillHistogram, 3> histograms = {};

    for (unsigned iy = bounds.y1; iy < bounds.y2; ++iy) {
        const std::uint8_t* row = image.ptr(iy);

        for (unsigned ix = bounds.x1; ix < bounds.x2; ++ix) {
            auto hue = row[3 * ix];
            auto saturation = row[3 * ix + 1];
            auto value = row[3 * ix + 2];

            histograms[0][hue / HISTOGRAM_BUCKET_SIZE]++;
            histograms[1][saturation / HISTOGRAM_BUCKET_SIZE]++;
            histograms[2][value / HISTOGRAM_BUCKET_SIZE]++;
        }
    }
    return histograms;
}

bool is_pixel_colored(unsigned base_h, unsigned base_s, unsigned base_v,
                      unsigned h, unsigned s, unsigned v,
                      const ResolvedFillLimits& limits)
{
    auto h_diff = std::abs(static_cast<int>(base_h) - static_cast<int>(h));
    auto s_diff = std::abs(static_cast<int>(base_s) - static_cast<int>(s));
    auto v_diff = std::abs(static_cast<int>(base_v) - static_cast<int>(v));

    if (v_diff > limits.value_diff) {
        return false;
    }
    if ((s_diff << 8) > base_v * limits.sat_diff) {
        return false;
    }
    if ((h_diff << 8) > base_v * limits.hue_diff) {
        return false;
    }

    return true;
}

void fill_initial_points(PointContainer& next_points,
                         cv::Mat_<std::uint8_t>& colored,
                         const cv::Mat& image,
                         const OcrBox& bounds,
                         const std::array<FillHistogram, 3>& histograms,
                         const ResolvedFillLimits& limits)
{
    auto best_h = max_element_i(histograms[0].begin(), histograms[0].end()) * HISTOGRAM_BUCKET_SIZE;
    auto best_s = max_element_i(histograms[1].begin(), histograms[1].end()) * HISTOGRAM_BUCKET_SIZE;
    auto best_v = max_element_i(histograms[2].begin(), histograms[2].end()) * HISTOGRAM_BUCKET_SIZE;

    for (unsigned iy = bounds.y1; iy < bounds.y2; ++iy) {
        const std::uint8_t* row = image.ptr(iy);

        for (unsigned ix = bounds.x1; ix < bounds.x2; ++ix) {
            auto h = row[3 * ix];
            auto s = row[3 * ix + 1];
            auto v = row[3 * ix + 2];

            if (!is_pixel_colored(best_h, best_s, best_v, h, s, v, limits)) {
                continue;
            }

            colored(iy, ix) = 1;
            next_points.push_back({static_cast<std::int32_t>(ix),
                                   static_cast<std::int32_t>(iy)});
        }
    }
}

cv::Mat mean_flood_fill(const cv::Mat& image, const MeanFloodFillParams& params)
{
    unsigned size_x = image.size.p[1];
    unsigned size_y = image.size.p[0];

    auto initial_limits = resolve_limits(params.max_initial_hue_diff,
                                         params.max_initial_sat_diff,
                                         params.max_initial_value_diff);
    auto limits = resolve_limits(params.max_hue_diff,
                                 params.max_sat_diff,
                                 params.max_value_diff);

    PointContainer next_points;
    cv::Mat_<std::uint8_t> colored(size_y, size_x, static_cast<std::uint8_t>(0));

    for (const auto& start_area : params.start_areas) {
        auto histograms = compute_histograms(image, start_area);
        fill_initial_points(next_points, colored, image, start_area, histograms, initial_limits);
    }

    auto search_size = params.search_size;
    auto border_clearance = params.search_size + params.nofill_border_size;

    while (!next_points.empty()) {
        auto next = next_points.front();
        next_points.pop_front();

        // bounds are enforced when adding to next points
        auto start_x = next.x - search_size;
        auto end_x = next.x + search_size;
        auto start_y = next.y - search_size;
        auto end_y = next.y + search_size;

        std::uint32_t sum_h = 0;
        std::uint32_t sum_s = 0;
        std::uint32_t sum_v = 0;
        unsigned count = 0;

#if __ARM_NEON__
        // FIXME: the code below likely uses A64-specific instructions
        const std::uint8_t end_mask[32] = {
            0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0
        };
        uint8x16_t zero = {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0};

        uint32x4_t v_sum_h = { 0, 0, 0, 0 };
        uint32x4_t v_sum_s = { 0, 0, 0, 0 };
        uint32x4_t v_sum_v = { 0, 0, 0, 0 };
        uint32x4_t v_sum_count = { 0, 0, 0, 0 };

        for (unsigned y = start_y; y < end_y; ++y) {
            const std::uint8_t* row = image.ptr(y);
            const std::uint8_t* colored_row = colored.ptr(y);
            row += 3 * start_x;
            colored_row += start_x;
            std::size_t count = end_x - start_x;

            for (unsigned x = 0; x < count; x += 16) {
                // We can safely read out of bounds because of nofill_border_size, as there are
                // at least one full line at the end of the image.
                uint8x16x3_t v_pixel = vld3q_u8(row);
                uint8x16_t v_pixel_h = v_pixel.val[0];
                uint8x16_t v_pixel_s = v_pixel.val[1];
                uint8x16_t v_pixel_v = v_pixel.val[2];

                uint8x16_t v_colored = vld1q_u8(colored_row);

                unsigned end_mask_offset = x + 16 > count ? x + 16 - count : 0;
                uint8x16_t v_end_mask = vld1q_u8(end_mask + end_mask_offset);

                v_colored = vandq_u8(v_colored, v_end_mask);
                uint8x16_t v_colored_mask = vcgtq_u8(v_colored, zero);

                v_pixel_h = vandq_u8(v_pixel_h, v_colored_mask);
                v_pixel_s = vandq_u8(v_pixel_s, v_colored_mask);
                v_pixel_v = vandq_u8(v_pixel_v, v_colored_mask);

                v_sum_h = vaddq_u32(v_sum_h, vpaddlq_u16(vpaddlq_u8(v_pixel_h)));
                v_sum_s = vaddq_u32(v_sum_s, vpaddlq_u16(vpaddlq_u8(v_pixel_s)));
                v_sum_v = vaddq_u32(v_sum_v, vpaddlq_u16(vpaddlq_u8(v_pixel_v)));
                v_sum_count = vaddq_u32(v_sum_count, vpaddlq_u16(vpaddlq_u8(v_colored)));

                row += 3 * 16;
                colored_row += 16;
            }
        }

        uint64x2_t v_sum64_h = vpaddlq_u32(v_sum_h);
        uint64x2_t v_sum64_s = vpaddlq_u32(v_sum_s);
        uint64x2_t v_sum64_v = vpaddlq_u32(v_sum_v);
        uint64x2_t v_sum64_count = vpaddlq_u32(v_sum_count);

        sum_h = vdupd_laneq_u64(v_sum64_h, 0) + vdupd_laneq_u64(v_sum64_h, 1);
        sum_s = vdupd_laneq_u64(v_sum64_s, 0) + vdupd_laneq_u64(v_sum64_s, 1);
        sum_v = vdupd_laneq_u64(v_sum64_v, 0) + vdupd_laneq_u64(v_sum64_v, 1);
        count = vdupd_laneq_u64(v_sum64_count, 0) + vdupd_laneq_u64(v_sum64_count, 1);
#else
        for (unsigned y = start_y; y < end_y; ++y) {
            const std::uint8_t* row = image.ptr(y);
            const std::uint8_t* colored_row = colored.ptr(y);
            for (unsigned x = start_x; x < end_x; ++x) {
                if (colored_row[x]) {
                    auto h = row[3 * x];
                    auto s = row[3 * x + 1];
                    auto v = row[3 * x + 2];

                    sum_h += h;
                    sum_s += s;
                    sum_v += v;
                    count++;
                }
            }
        }
#endif

        if (count == 0) {
            continue;
        }

        unsigned base_h = sum_h / count;
        unsigned base_s = sum_s / count;
        unsigned base_v = sum_v / count;

        for (unsigned y = start_y; y < end_y; ++y) {
            const std::uint8_t* row = image.ptr(y);
            std::uint8_t* colored_row = colored.ptr(y);

            row += start_x * 3;
            colored_row += start_x;

            auto count_x = end_x - start_x;
            for (unsigned ix8 = 0; ix8 < count_x; ix8 += 8) {
                std::uint64_t colored8;
                std::memcpy(&colored8, colored_row + ix8, sizeof(colored8));
                colored8 *= 255;

                // only works on little endian
                auto right_ones = std::countr_one(colored8);
                while (right_ones != 64) {
                    colored8 |= 0xffull << right_ones;
                    unsigned ix = ix8 + right_ones / 8;

                    auto right_ones_before = right_ones;
                    right_ones = std::countr_one(colored8);

                    if (ix >= count_x) {
                        break;
                    }
                    auto h = row[3 * ix];
                    auto s = row[3 * ix + 1];
                    auto v = row[3 * ix + 2];

                    if (!is_pixel_colored(base_h, base_s, base_v, h, s, v, limits)) {
                        continue;
                    }

                    colored_row[ix] = 1;

                    auto x = start_x + ix;
                    if (x < border_clearance || x >= size_x - border_clearance ||
                        y < border_clearance || y >= size_y - border_clearance)
                    {
                        continue;
                    }

                    next_points.push_back({static_cast<std::int32_t>(x),
                                           static_cast<std::int32_t>(y)});
                }
            }
        }
    }

    return std::move(colored);
}

} // namespace sanescan
