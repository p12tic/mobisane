// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "parallel_tesseract.h"
#include <taskflow/algorithm/for_each.hpp>
#include <stdexcept>

namespace sanescan {

TesseractParallelismBackendTaskflow::TesseractParallelismBackendTaskflow(tf::Executor& executor) :
    executor_{executor}
{}

TesseractParallelismBackendTaskflow::~TesseractParallelismBackendTaskflow() = default;

void TesseractParallelismBackendTaskflow::ParallelForImpl(
        std::int64_t lower_bound, std::int64_t upper_bound,
        const tesseract::ParallelSettings& settings,
        const ParallelForCallback &callback)
{
    if (!settings.IsMultiThreadingEnabled())
    {
        for (std::int64_t i = lower_bound; i < upper_bound; ++i) {
            callback(i, 0);
        }
        return;
    }

    std::exception_ptr exception;

    tf::Taskflow taskflow;
    taskflow.for_each_index(lower_bound, upper_bound, std::int64_t(1), [&](int i)
    {
        try {
            callback(i, executor_.this_worker_id());
        } catch (...) {
            exception = std::current_exception();
        }
    });

    executor_.run_and_wait(taskflow);

    if (exception) {
        std::rethrow_exception(exception);
    }
}

int TesseractParallelismBackendTaskflow::GetMaxThreadCount() const
{
    return executor_.num_workers();
}


} // namespace sanescan
