#include "Output.h"
#include "Organizer.h"
#include <fstream>
#include <iomanip>
#include <stdexcept>
int Output::mode()
{
    std::cout << "\n============================================================\n"
              << "             AMBULANCE MANAGEMENT SYSTEM\n"
              << "============================================================\n"
              << "  1  Interactive - inspect queues at each timestep\n"
              << "  2  Silent      - run and save the final report\n"
              << "  0  Exit\n";
    std::string choice;
    while (true)
    {
        std::cout << "Choose mode: ";
        if (!std::getline(std::cin, choice) || choice == "0")
            return 0;
        if (choice == "1" || choice == "2")
            return choice == "1" ? 1 : 2;
        std::cout << "Please enter 0, 1, or 2.\n";
    }
}
void Output::printhospital(Hospital &h)
{
    std::cout << "\nHospital " << h.gethospitalID() << "\n";
    std::cout << "  EP (" << h.getEmergencypatientQueue().getCount() << "): ";
    h.getEmergencypatientQueue().print();
    std::cout << "\n  SP (" << h.getSpecialpatientQueue().getCount() << "): ";
    h.getSpecialpatientQueue().print();
    std::cout << "\n  NP (" << h.getNormalpatientQueue().getCount() << "): ";
    h.getNormalpatientQueue().print();
    std::cout << "\n  Free SC (" << h.getSpecialCarQueue().getCount() << "): ";
    h.getSpecialCarQueue().print();
    std::cout << "\n  Free NC (" << h.getNormalCarQueue().getCount() << "): ";
    h.getNormalCarQueue().print();
    std::cout << "\n";
}
void Output::printState(Organizer &org)
{
    std::cout << "\n================ TIMESTEP " << org.timestep << " ================\n";
    std::cout << "Future requests (" << org.AllRequests.getCount() << "): ";
    org.AllRequests.print();
    for (int i = 0; i < org.totalhospitals; ++i)
        printhospital(org.hospitalList[i]);
    std::cout << "OUT (" << org.outCars.getCount() << "): ";
    org.outCars.print();
    std::cout << "\nBACK (" << org.backCars.getCount() << "): ";
    org.backCars.print();
    std::cout << "\nCheckup (" << org.checkupList.getCount() << "): ";
    org.checkupList.print();
    std::cout << "\nFinished (" << org.finishedList.getCount() << "): ";
    org.finishedList.print();
    std::cout << "\nCancelled: " << org.cancelledCount << " | Failures: " << org.failureCount
              << "\n";
}
void Output::OutputFile(Organizer &org, const std::string &path)
{
    std::ofstream out(path);
    if (!out)
        throw std::runtime_error("Cannot create report: " + path);
    out << "FT\tPID\tQT\tWT\n";
    // FINISH is already chronological because BACK events are processed in time order.
    auto finished = org.finishedList;
    Patients *p = nullptr;
    double wait = 0;
    while (finished.dequeue(p))
    {
        out << p->get_finish_time() << '\t' << p->getid() << '\t' << p->getrequesttime() << '\t'
            << p->get_wait_time() << '\n';
        wait += p->get_wait_time();
    }
    int np = 0, sp = 0, ep = 0, nc = 0, sc = 0;
    double busy = 0;
    auto patients = org.ownedPatients;
    while (patients.dequeue(p))
    {
        if (p->gettype() == "NP")
            ++np;
        else if (p->gettype() == "SP")
            ++sp;
        else
            ++ep;
    }
    auto cars = org.ownedCars;
    Car *c = nullptr;
    while (cars.dequeue(c))
    {
        busy += c->get_busyTime();
        if (c->gettype() == "NC")
            ++nc;
        else
            ++sc;
    }
    int served = org.finishedList.getCount(), fleet = nc + sc;
    double averageBusy = fleet ? busy / fleet : 0;
    out << std::fixed << std::setprecision(2) << "Patients: " << np + sp + ep << " [NP: " << np
        << ", SP: " << sp << ", EP: " << ep << "]\n"
        << "Served: " << served << "\nCancelled: " << org.cancelledCount
        << "\nHospitals: " << org.totalhospitals << "\nCars: " << fleet << " [SCar: " << sc
        << ", NCar: " << nc << "]\n"
        << "Avg wait time: " << (served ? wait / served : 0) << "\n"
        << "EP not served by home hospital: " << (ep ? 100.0 * org.transfer_emerg / ep : 0) << "%\n"
        << "Avg busy time: " << averageBusy
        << "\nAvg utilization: " << (org.timestep ? 100.0 * averageBusy / org.timestep : 0) << "%\n"
        << "OUT failure events: " << org.failureCount << "\nSimulation timesteps: " << org.timestep
        << "\n";
    out.close();
    if (!out)
        throw std::runtime_error("Could not finish writing report: " + path);
}
