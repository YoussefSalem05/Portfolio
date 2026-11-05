#include "Hospital.h"
#include <stdexcept>
int Hospital::totalhospitals = 0;
int **Hospital::Hdistance = nullptr;
void Hospital::fillHdistance(std::ifstream &in)
{
    Hdistance = new int *[totalhospitals]();
    for (int i = 0; i < totalhospitals; ++i)
    {
        Hdistance[i] = new int[totalhospitals]();
        for (int j = 0; j < totalhospitals; ++j)
            if (!(in >> Hdistance[i][j]) || Hdistance[i][j] < 0 || Hdistance[i][j] > 1000000)
                throw std::runtime_error("Invalid hospital distance matrix.");
    }
}
void Hospital::cleanupdistance()
{
    if (!Hdistance)
        return;
    for (int i = 0; i < totalhospitals; ++i)
        delete[] Hdistance[i];
    delete[] Hdistance;
    Hdistance = nullptr;
}
Car *Hospital::assign_patient_to_ncar()
{
    if (Np.isEmpty() || NormalCars.isEmpty())
        return nullptr;
    Patients *p = nullptr;
    Car *c = nullptr;
    Np.dequeue(p);
    NormalCars.dequeue(c);
    c->setAssignedpatient(p);
    return c;
}
Car *Hospital::assign_patient_to_scar()
{
    if (Sp.isEmpty() || SpecialCars.isEmpty())
        return nullptr;
    Patients *p = nullptr;
    Car *c = nullptr;
    Sp.dequeue(p);
    SpecialCars.dequeue(c);
    c->setAssignedpatient(p);
    return c;
}
Car *Hospital::assign_emergencypatient()
{
    if (Ep.isEmpty())
        return nullptr;
    Car *c = nullptr;
    // Emergency cases use a normal car first, preserving special cars when possible.
    if (!NormalCars.dequeue(c) && !SpecialCars.dequeue(c))
        return nullptr;
    Patients *p = nullptr;
    int priority = 0;
    Ep.dequeue(p, priority);
    c->setAssignedpatient(p);
    return c;
}
