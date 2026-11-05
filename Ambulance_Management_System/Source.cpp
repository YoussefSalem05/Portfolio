#include "Organizer.h"
#include "Output.h"
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <chrono>

unsigned parseSeed(const std::string &value)
{
    size_t used = 0;
    auto parsed = std::stoul(value, &used);
    if (used != value.size() || parsed > 0xffffffffUL || value.empty() || value[0] == '-')
        throw std::runtime_error("Invalid random seed.");
    return static_cast<unsigned>(parsed);
}

int main(int argc, char *argv[])
{
    try
    {
        bool cli = argc > 1;
        int mode = cli ? 2 : Output::mode();
        if (mode == 0)
            return 0;
        std::string input = "testload.txt", output = "OutputFile.txt";
        unsigned seed =
            static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
        if (cli)
        {
            input = argv[1];
            if (argc > 2)
                output = argv[2];
            if (argc > 3)
                seed = parseSeed(argv[3]);
            if (argc > 4)
                throw std::runtime_error("Usage: AmbulanceSystem [input [output [seed]]]");
        }
        else
        {
            std::string value;
            std::cout << "Input file [testload.txt]: ";
            if (!std::getline(std::cin, value))
                return 0;
            if (!value.empty())
                input = value;
            std::cout << "Output file [OutputFile.txt]: ";
            if (!std::getline(std::cin, value))
                return 0;
            if (!value.empty())
                output = value;
            std::cout << "Random seed [random]: ";
            if (!std::getline(std::cin, value))
                return 0;
            if (!value.empty())
                seed = parseSeed(value);
        }
        if (std::filesystem::absolute(input).lexically_normal() ==
                std::filesystem::absolute(output).lexically_normal() ||
            (std::filesystem::exists(input) && std::filesystem::exists(output) &&
             std::filesystem::equivalent(input, output)))
            throw std::runtime_error("Input and output must be different files.");
        std::cout << "Random seed: " << seed << "\n";
        Organizer org;
        org.Load(input, seed);
        org.run(mode == 1, output);
        if (!cli)
        {
            std::cout << "Press Enter to exit...";
            std::string line;
            std::getline(std::cin, line);
        }
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        if (argc == 1)
        {
            std::cout << "Press Enter to exit...";
            std::string line;
            std::getline(std::cin, line);
        }
        return 1;
    }
}
