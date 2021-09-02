#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>

#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if (!is)
        return std::nullopt;

    auto size = is.tellg();
    std::vector<std::byte> contents(size);

    is.seekg(0);
    is.read((char *)contents.data(), size);

    if (contents.empty())
        return std::nullopt;
    return std::move(contents);
}

// Get the coordinates one at a time from stdin & perform error checking.
/// param: coordinate, min & max value.
float UserPrompt(float x = -1.0, int min = 0, int max = 100)
{
    // Loop control.
    unsigned int input_failure = 0;

    do
    {
        std::cout << "Please enter coordinates below [0-100]" << std::endl;
        cin >> x;

        while (cin.fail())
        {
            cin.clear();
            cin.ignore(123, '\n');
            std::cout << '\n'
                      << "Wrong entry! Try again: [" << input_failure << "/3]" << std::endl;
            cin >> x;

            input_failure++;

            if (input_failure == 3)
            {
                std::cout << "Limit exceeded !!!" << std::endl;
                exit(1);
            }
        }
        return x;
    } while ((x < max) && (x > min));

    exit(-1);
}

int main(int argc, const char **argv)
{
    std::string osm_data_file = "";
    if (argc > 1)
    {
        for (int i = 1; i < argc; ++i)
            if (std::string_view{argv[i]} == "-f" && ++i < argc)
                osm_data_file = argv[i];
    }
    else
    {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }

    std::vector<std::byte> osm_data;

    if (osm_data.empty() && !osm_data_file.empty())
    {
        std::cout << "Reading OpenStreetMap data from the following file: " << osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if (!data)
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }

    // Prompt the user and get the coordinates.
    float start_x = UserPrompt();
    float start_y = UserPrompt();
    float end_x = UserPrompt();
    float end_y = UserPrompt();

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    // Build Model.
    RouteModel model{osm_data};

    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface &surface)
                                 { surface.dimensions(surface.display_dimensions()); });
    display.draw_callback([&](io2d::output_surface &surface)
                          { render.Display(surface); });
    display.begin_show();
}
