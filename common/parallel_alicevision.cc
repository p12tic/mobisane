// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "parallel_alicevision.h"
#include <taskflow/algorithm/for_each.hpp>
#include <taskflow/core/executor.hpp>
#include <taskflow/core/taskflow.hpp>
#include <mutex>
#include <stdexcept>

namespace sanescan {

ParallelLoopManagerTaskflow::ParallelLoopManagerTaskflow(std::atomic<std::int64_t>* sharedCounter) :
    _sharedCounter{sharedCounter}
{
}

ParallelLoopManagerTaskflow::~ParallelLoopManagerTaskflow() = default;

void ParallelLoopManagerTaskflow::submit(const std::function<void()>& callback)
{
    // Note that there are no dependent memory operations, thus relaxed ordering is fine.
    auto sharedCounterValue = _sharedCounter->load(std::memory_order_relaxed);
    if (_thisThreadCounter < sharedCounterValue)
    {
        // Current thread is catching up
        _thisThreadCounter++;
        return;
    }

    // The current thread has made farthest progress and could take a task by increasing the
    // shared counter.
    while (!_sharedCounter->compare_exchange_weak(sharedCounterValue, sharedCounterValue + 1,
                                                  std::memory_order_relaxed,
                                                  std::memory_order_relaxed))
    {
        if (_thisThreadCounter < sharedCounterValue)
        {
            // Another thread increased the shared counter in the mean time, which means the
            // current thread is catching up again.
            _thisThreadCounter++;
            return;
        }
    }

    // _sharedCounter has been successfully incremented, do the work
    _thisThreadCounter++;
    callback();
}

ParallelismBackendTaskflow::ParallelismBackendTaskflow(tf::Executor& executor) :
    _executor{executor}
{}

ParallelismBackendTaskflow::~ParallelismBackendTaskflow() = default;

void ParallelismBackendTaskflow::parallelFor(std::int64_t lowerBound, std::int64_t upperBound,
                                             aliceVision::system::ParallelSettings settings,
                                             const ParallelForCallback& callback)
{
    if (!settings.enableMultithreading())
    {
        callback(lowerBound, upperBound, 0);
        return;
    }

    bool anyThreadCount =
            settings.threadCount() == aliceVision::system::ParallelSettings::ANY_THREAD_COUNT;

    std::int64_t maxBlockCount = upperBound - lowerBound;
    int blockCount = anyThreadCount
            ? _executor.num_workers()
            : std::min<int>(_executor.num_workers(), settings.threadCount());

    if (settings.isDynamicScheduling())
    {
        // Dynamic scheduling, the tasks difficulty varies, so they are scheduled dynamically.
        // We achieve this by splitting each block of work items into 4 and letting TBB to
        // do dynamic scheduling.
        blockCount = std::min<std::int64_t>(maxBlockCount, blockCount * 4);
    }
    double itemsPerBlock = static_cast<double>(upperBound - lowerBound) / blockCount;

    tf::Taskflow taskflow;
    taskflow.for_each_index(0, blockCount, 1, [&](int i)
    {
        auto blockLowerBound = lowerBound + static_cast<std::int64_t>(itemsPerBlock * i);
        auto blockUpperBound = lowerBound + static_cast<std::int64_t>(itemsPerBlock * (i + 1));
        if (i == blockCount - 1) {
            blockUpperBound = upperBound;
        }
        callback(blockLowerBound, blockUpperBound, _executor.this_worker_id());
    });

    _executor.run_and_wait(taskflow);
}

void ParallelismBackendTaskflow::parallelLoop(
        aliceVision::system::ParallelSettings settings,
        const std::function<void(aliceVision::system::IParallelLoopManager&)>& callback)
{
    if (!settings.enableMultithreading())
    {
        aliceVision::system::ParallelLoopManagerSingleThread manager;
        callback(manager);
        return;
    }

    if (settings.isDynamicScheduling())
    {
        throw std::invalid_argument("Dynamic scheduling is not supported in parallelLoop()");
    }

    // This will be shared with other threads. Add padding to avoid false sharing.
    constexpr int cacheLineSize = 128;
    struct {
        char pad0[cacheLineSize];
        std::atomic<std::int64_t> sharedCounter;
        char pad1[cacheLineSize];
    } sharedCounterStorage;

    sharedCounterStorage.sharedCounter = 0;

    bool anyThreadCount =
            settings.threadCount() == aliceVision::system::ParallelSettings::ANY_THREAD_COUNT;
    int threadCount = anyThreadCount
            ? _executor.num_workers()
            : std::min<int>(settings.threadCount(), _executor.num_workers());

    std::vector<ParallelLoopManagerTaskflow> loopManagers;
    loopManagers.reserve(threadCount);
    for (int i = 0; i < threadCount; ++i)
    {
        loopManagers.emplace_back(&sharedCounterStorage.sharedCounter);
    }

    tf::Taskflow taskflow;
    taskflow.for_each_index(0, threadCount, 1, [&](int i)
    {
        callback(loopManagers[i]);
    });
    _executor.run_and_wait(taskflow);
}

int ParallelismBackendTaskflow::getMaxThreadCount() const
{
    return _executor.num_workers();
}

} // namespace sanescan
