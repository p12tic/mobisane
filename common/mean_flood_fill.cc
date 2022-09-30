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
#include <array>

namespace sanescan {

constexpr std::size_t HISTOGRAM_BUCKET_SIZE = 8;
constexpr std::size_t HISTOGRAM_BUCKET_COUNT = 256 / HISTOGRAM_BUCKET_SIZE;

using FillHistogram = std::array<std::uint32_t, HISTOGRAM_BUCKET_COUNT>;

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

void fill_initial_points(std::vector<OcrPoint>& next_points,
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

    std::vector<OcrPoint> next_points;
    cv::Mat_<std::uint8_t> colored(size_y, size_x, static_cast<std::uint8_t>(0));

    for (const auto& start_area : params.start_areas) {
        auto histograms = compute_histograms(image, start_area);
        fill_initial_points(next_points, colored, image, start_area, histograms, initial_limits);
    }

    auto search_size = params.search_size;
    auto border_clearance = params.search_size + params.nofill_border_size;

    while (!next_points.empty()) {
        auto next = next_points.back();
        next_points.pop_back();

        // bounds are enforced when adding to next points
        auto start_x = next.x - search_size;
        auto end_x = next.x + search_size;
        auto start_y = next.y - search_size;
        auto end_y = next.y + search_size;

        std::uint32_t sum_h = 0;
        std::uint32_t sum_s = 0;
        std::uint32_t sum_v = 0;
        unsigned count = 0;

        for (unsigned y = start_y; y < end_y; ++y) {
            auto* colored_row = colored.ptr(y);
            for (unsigned x = start_x; x < end_x; ++x) {
                if (colored_row[x]) {
                    const std::uint8_t* row = image.ptr(y);
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

        if (count == 0) {
            continue;
        }

        unsigned base_h = sum_h / count;
        unsigned base_s = sum_s / count;
        unsigned base_v = sum_v / count;

        for (unsigned y = start_y; y < end_y; ++y) {
            auto* colored_row = colored.ptr(y);

            for (unsigned x = start_x; x < end_x; ++x) {
                if (colored_row[x]) {
                    continue;
                }

                const std::uint8_t* row = image.ptr(y);
                auto h = row[3 * x];
                auto s = row[3 * x + 1];
                auto v = row[3 * x + 2];

                if (!is_pixel_colored(base_h, base_s, base_v, h, s, v, limits)) {
                    continue;
                }

                colored_row[x] = 1;

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

    return std::move(colored);
}

} // namespace sanescan
