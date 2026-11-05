#pragma once
#include <string>
class Organizer;
class Hospital;
class Output
{
  public:
    static int mode();
    static void printhospital(Hospital &h);
    static void printState(Organizer &org);
    static void OutputFile(Organizer &org, const std::string &path);
};
