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

#include "log_utils.h"
#include <android/log.h>
#include <unistd.h>
#include <cstdio>
#include <thread>

namespace sanescan {

class StreamRedirector
{
public:
    bool is_started() const
    {
        return started_;
    }

    void initialize_and_start(const char* app_name)
    {
        started_ = true;
        app_name_ = app_name;

        std::setvbuf(stdout, 0, _IOLBF, 0);
        std::setvbuf(stderr, 0, _IONBF, 0);

        pipe(pipe_fds_);
        dup2(pipe_fds_[1], 1);
        dup2(pipe_fds_[1], 2);

        std::thread output_thread([this]()
        {
            constexpr std::size_t BUFFER_SIZE = 256;
            char buffer[BUFFER_SIZE];

            ssize_t read_size = 0;
            while ((read_size = read(pipe_fds_[0], buffer, BUFFER_SIZE - 1)) > 0) {
                // Drop newline, if any, as Android adds it by itself..
                if (buffer[read_size - 1] == '\n') {
                    --read_size;
                }
                buffer[read_size] = 0;
                __android_log_write(ANDROID_LOG_DEBUG, app_name_, buffer);
            }
        });
        output_thread.detach();
    }

private:
    bool started_ = false;
    const char* app_name_ = nullptr;
    int pipe_fds_[2] = {};
};

void setup_std_stream_redirection_to_logcat(const char* app_name)
{
    static StreamRedirector redirector;
    if (redirector.is_started()) {
        return;
    }
    redirector.initialize_and_start(app_name);
}

} // namespace sanescan
