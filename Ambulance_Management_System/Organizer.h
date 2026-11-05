#pragma once
#include "Hospital.h"
#include "OUTcars.h"
#include <random>
#include <string>

class Output;
class Organizer
{
    friend class Output;
    Hospital *hospitalList = nullptr;
    int totalhospitals = 0;
    LinkedQueue<Patients *> AllRequests, Cancelled, finishedList;
    // Ownership lists ensure every allocation is released exactly once.
    LinkedQueue<Patients *> ownedPatients, ownedCancellations;
    LinkedQueue<Car *> ownedCars;
    OUTcars outCars;
    priQueue<Car *> backCars, checkupList;
    std::mt19937 random;
    int transfer_emerg = 0, cancelledCount = 0, failureCount = 0, timestep = 0;
    void returnFree(Car *c);
    void dispatch(Car *c);
    void step();
    bool complete() const;
    bool hospitalQueuesEmpty() const;
    void assign_to_another_hospital(Patients *p);
    void add_to_finishedlist(Patients *p, int ft);
    void OUTtoBack();
    void BackToFree();
    void processCancellations();
    void assign_patients_to_hospitals();
    void outCarFailureProbability();
    void processCheckups();

  public:
    Organizer() = default;
    Organizer(const Organizer &) = delete;
    Organizer &operator=(const Organizer &) = delete;
    ~Organizer();
    void Load(const std::string &input, unsigned seed);
    void run(bool interactive, const std::string &output);
};
