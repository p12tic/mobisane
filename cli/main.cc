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

#include <opencv2/imgcodecs.hpp>
#include <boost/program_options.hpp>
#include <iostream>

struct Options {
    static constexpr const char* INPUT_PATH = "input-path";
    static constexpr const char* OUTPUT_PATH = "output-path";
    static constexpr const char* HELP = "help";
};

int main(int argc, char* argv[])
{
    namespace po = boost::program_options;

    std::string input_path;
    std::string output_path;

    po::positional_options_description positional_options_desc;
    positional_options_desc.add(Options::INPUT_PATH, 1);
    positional_options_desc.add(Options::OUTPUT_PATH, 1);

    auto introduction_desc = R"(Usage:
    mobisanecli [OPTION]... [input_path] [output_path]

input_path and output_path options can be passed either as positional or named arguments.
)";

    po::options_description options_desc("Options");

    options_desc.add_options()
            (Options::INPUT_PATH, po::value(&input_path), "the path to the input image")
            (Options::OUTPUT_PATH, po::value(&output_path), "the path to the output PDF file")
            (Options::HELP, "produce this help message")
            ;

    po::variables_map options;
    try {
        po::store(po::command_line_parser(argc, argv)
                      .options(options_desc)
                      .positional(positional_options_desc)
                      .run(),
                  options);
        po::notify(options);
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse options: " << e.what() << "\n";
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "Failed to parse options: unknown error\n";
        return EXIT_FAILURE;
    }

    if (options.count(Options::HELP)) {
        std::cout << introduction_desc << "\n"
                  << options_desc << "\n";
        return EXIT_SUCCESS;
    }

    if (options.count(Options::INPUT_PATH) != 1) {
        std::cerr << "Must specify single input path\n";
        return EXIT_FAILURE;
    }

    if (options.count(Options::OUTPUT_PATH) != 1) {
        std::cerr << "Must specify single output path\n";
        return EXIT_FAILURE;
    }


    try {
        auto image = cv::imread(input_path);
        cv::imwrite(output_path, image);
    } catch (const std::exception& e) {
        std::cerr << "Failed to do OCR: " << e.what() << "\n";
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "Failed to do OCR uknown failure\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
