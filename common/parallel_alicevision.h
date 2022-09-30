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

#pragma once

#include <aliceVision/system/ParallelismBackend.hpp>
#include <taskflow/core/executor.hpp>
#include <atomic>

namespace tf {
    class Executor;
} // namespace tf

namespace sanescan {

constexpr int CACHE_LINE_SIZE = 128;

class ParallelLoopManagerTaskflow : public aliceVision::system::IParallelLoopManager
{
public:
    ParallelLoopManagerTaskflow(std::atomic<std::int64_t>* sharedCounter);
    ~ParallelLoopManagerTaskflow() override;

    void submit(const std::function<void()>& callback) override;
private:
    std::atomic<std::int64_t>* _sharedCounter = nullptr;
    std::int64_t _thisThreadCounter = 0;

    // Avoid false sharing with _thisThreadCounter of other threads.
    char pad[CACHE_LINE_SIZE - sizeof(_sharedCounter) - sizeof(_thisThreadCounter)];
};

class ParallelismBackendTaskflow : public aliceVision::system::IParallelismBackend
{
public:
    ParallelismBackendTaskflow(tf::Executor& executor);
    ~ParallelismBackendTaskflow() override;

    void parallelFor(std::int64_t lowerBound, std::int64_t upperBound,
                     aliceVision::system::ParallelSettings settings,
                     const ParallelForCallback& callback) override;

    void parallelLoop(aliceVision::system::ParallelSettings settings,
                      const std::function<void(aliceVision::system::IParallelLoopManager&)>& callback) override;

    int getMaxThreadCount() const override;

private:
    tf::Executor& _executor;
};

} // namespace sanescan
