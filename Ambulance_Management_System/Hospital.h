#pragma once
#include <fstream>
#include "NPQueue.h"
#include "Car.h"
#include "priQueue.h"

// Hospital queues own their nodes. The Organizer owns the pointed-to objects.
class Hospital
{
    int HID, numSCARS, numNCARS;
    LinkedQueue<Car *> NormalCars, SpecialCars;
    NPQueue Np;
    LinkedQueue<Patients *> Sp;
    priQueue<Patients *> Ep;
    int stagedEmergencies = 0;
    static int totalhospitals;
    static int **Hdistance;

  public:
    Hospital(int id = 0, int sc = 0, int nc = 0) : HID(id), numSCARS(sc), numNCARS(nc) {}
    static void settotalhospitals(int n) { totalhospitals = n; }
    static int gettotalhospitals() { return totalhospitals; }
    static void fillHdistance(std::ifstream &in);
    static int getHdistance(int i, int j) { return Hdistance[i][j]; }
    static void cleanupdistance();
    int gethospitalID() const { return HID; }
    int getnumberscars() const { return numSCARS; }
    int getnumberncars() const { return numNCARS; }
    LinkedQueue<Car *> &getNormalCarQueue() { return NormalCars; }
    LinkedQueue<Car *> &getSpecialCarQueue() { return SpecialCars; }
    NPQueue &getNormalpatientQueue() { return Np; }
    LinkedQueue<Patients *> &getSpecialpatientQueue() { return Sp; }
    priQueue<Patients *> &getEmergencypatientQueue() { return Ep; }
    int getEmergencyCount() const { return Ep.getCount() + stagedEmergencies; }
    void setStagedEmergencies(int count) { stagedEmergencies = count; }
    void removeStagedEmergency() { --stagedEmergencies; }
    void addnormalpatient(Patients *p) { Np.enqueue(p); }
    void addspecialpatient(Patients *p) { Sp.enqueue(p); }
    void addemergencypatient(Patients *p, int priority) { Ep.enqueue(p, priority); }
    void addncar(Car *c) { NormalCars.enqueue(c); }
    void addscar(Car *c) { SpecialCars.enqueue(c); }
    Car *assign_patient_to_ncar();
    Car *assign_patient_to_scar();
    Car *assign_emergencypatient();
};
