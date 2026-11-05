#include "Organizer.h"
#include "Output.h"
#include <fstream>
#include <limits>
#include <stdexcept>

namespace
{
// Upper bounds keep malformed input from overflowing event times or allocations.
int number(std::ifstream &in, int low, int high, const char *field)
{
    int value;
    if (!(in >> value) || value < low || value > high)
        throw std::runtime_error(std::string("Invalid or missing ") + field + ".");
    return value;
}
int travel(int distance, int speed)
{
    return distance / speed + (distance % speed != 0);
}
// Merge sort validates unique IDs using the project's queues in O(n log n) time.
void sortByID(LinkedQueue<Patients *> &queue)
{
    if (queue.getCount() < 2)
        return;
    LinkedQueue<Patients *> left, right;
    int half = queue.getCount() / 2;
    Patients *p = nullptr;
    while (half--)
    {
        queue.dequeue(p);
        left.enqueue(p);
    }
    while (queue.dequeue(p))
        right.enqueue(p);
    sortByID(left);
    sortByID(right);
    Patients *a = nullptr;
    Patients *b = nullptr;
    while (left.peek(a) && right.peek(b))
    {
        if (a->getid() < b->getid())
            left.dequeue(p);
        else
            right.dequeue(p);
        queue.enqueue(p);
    }
    while (left.dequeue(p))
        queue.enqueue(p);
    while (right.dequeue(p))
        queue.enqueue(p);
}
} // namespace
Organizer::~Organizer()
{
    delete[] hospitalList;
    Hospital::cleanupdistance();
    Patients *p = nullptr;
    Car *c = nullptr;
    while (ownedPatients.dequeue(p))
        delete p;
    while (ownedCancellations.dequeue(p))
        delete p;
    while (ownedCars.dequeue(c))
        delete c;
}
void Organizer::Load(const std::string &input, unsigned seed)
{
    std::ifstream in(input);
    if (!in)
        throw std::runtime_error("Cannot open input file: " + input);
    random.seed(seed);
    totalhospitals = number(in, 1, 100, "hospital count");
    Hospital::settotalhospitals(totalhospitals);
    Car::setspeedSCAR(number(in, 1, 1000000, "special car speed"));
    Car::setspeedNCAR(number(in, 1, 1000000, "normal car speed"));
    Car::setCheckupTimeSCAR(number(in, 0, 1000000, "special car checkup time"));
    Car::setCheckupTimeNCAR(number(in, 0, 1000000, "normal car checkup time"));
    float rate = 0;
    if (!(in >> rate) || !(rate >= 0 && rate <= 1))
        throw std::runtime_error("Failure probability must be from 0 to 1.");
    Car::setFailureRate(rate);
    Hospital::fillHdistance(in);
    hospitalList = new Hospital[totalhospitals];
    for (int i = 0; i < totalhospitals; ++i)
    {
        int sc = number(in, 0, 1000, "special car count"),
            nc = number(in, 0, 1000, "normal car count");
        hospitalList[i] = Hospital(i + 1, sc, nc);
        for (int j = 0; j < sc + nc; ++j)
        {
            bool special = j < sc;
            Car *c = new Car(j + 1, special ? "SC" : "NC", "ready",
                             special ? Car::getspeedSCAR() : Car::getspeedNCAR(), i + 1);
            ownedCars.enqueue(c);
            returnFree(c);
        }
    }
    int count = number(in, 0, 100000, "request count"), previous = 0;
    for (int i = 0; i < count; ++i)
    {
        std::string type;
        in >> type;
        if (type != "EP" && type != "SP" && type != "NP")
            throw std::runtime_error("Unknown patient type.");
        int rt = number(in, 1, 1000000, "request time"),
            id = number(in, 1, 1000000000, "patient ID");
        if (rt < previous)
            throw std::runtime_error("Requests must be sorted by time.");
        previous = rt;
        int hid = number(in, 1, totalhospitals, "hospital ID"),
            distance = number(in, 0, 1000000, "patient distance");
        int severity = type == "EP" ? number(in, 0, 1000000, "severity") : 0;
        Patients *p = new Patients(type, rt, id, hid, distance, severity);
        ownedPatients.enqueue(p);
        AllRequests.enqueue(p);
    }
    auto sorted = ownedPatients;
    sortByID(sorted);
    Patients *previousPatient = nullptr;
    Patients *current = nullptr;
    while (sorted.dequeue(current))
    {
        if (previousPatient && current->getid() == previousPatient->getid())
            throw std::runtime_error("Patient IDs must be unique.");
        previousPatient = current;
    }
    count = number(in, 0, 100000, "cancellation count");
    for (int i = 0; i < count; ++i)
    {
        int ct = number(in, 1, 1000000, "cancellation time"),
            id = number(in, 1, 1000000000, "cancelled patient ID");
        int hid = number(in, 1, totalhospitals, "cancellation hospital ID");
        Patients *p = new Patients(ct, id, hid);
        ownedCancellations.enqueue(p);
        Cancelled.enqueue(p);
    }
    std::string extra;
    if (in >> extra)
        throw std::runtime_error("Unexpected data after cancellations.");
    // Reject configurations that could wait forever for a car type that does not exist.
    auto requests = ownedPatients;
    Patients *p = nullptr;
    while (requests.dequeue(p))
    {
        Hospital &h = hospitalList[p->getHID() - 1];
        bool willCancel = false;
        if (p->gettype() == "NP" && h.getnumberncars() == 0)
        {
            auto cancellations = Cancelled;
            Patients *cancel = nullptr;
            while (cancellations.dequeue(cancel))
                if (cancel->getid() == p->getid() && cancel->getHID() == p->getHID() &&
                    cancel->getcanceltime() >= p->getrequesttime())
                    willCancel = true;
        }
        if ((p->gettype() == "NP" && h.getnumberncars() == 0 && !willCancel) ||
            (p->gettype() == "SP" && h.getnumberscars() == 0) ||
            (p->gettype() == "EP" && ownedCars.isEmpty()))
            throw std::runtime_error("No compatible fleet for patient " +
                                     std::to_string(p->getid()) + ".");
    }
}
bool Organizer::hospitalQueuesEmpty() const
{
    for (int i = 0; i < totalhospitals; ++i)
    {
        Hospital &h = hospitalList[i];
        if (!h.getNormalpatientQueue().isEmpty() || !h.getSpecialpatientQueue().isEmpty() ||
            !h.getEmergencypatientQueue().isEmpty())
            return false;
    }
    return true;
}
bool Organizer::complete() const
{
    return AllRequests.isEmpty() && hospitalQueuesEmpty() && outCars.isEmpty() &&
           backCars.isEmpty() && checkupList.isEmpty();
}
void Organizer::returnFree(Car *c)
{
    c->set_status("ready");
    Hospital &h = hospitalList[c->getHomeHospital() - 1];
    if (c->gettype() == "NC")
        h.addncar(c);
    else
        h.addscar(c);
}
void Organizer::dispatch(Car *c)
{
    Patients *p = c->getAssignedpatient();
    int speed = c->gettype() == "NC" ? Car::getspeedNCAR() : Car::getspeedSCAR();
    int duration = travel(p->getdistance(), speed);
    c->set_assignmenttime(timestep);
    c->set_timetoreach(duration);
    c->set_status("assigned");
    p->set_assignmenttime(timestep);
    p->set_pickuptime(0);
    p->setisPicked(false);
    p->retry = false;
    // Our max-priority queue uses negative timestamps so the earliest event is first.
    outCars.enqueue(c, -(timestep + duration));
}
void Organizer::add_to_finishedlist(Patients *p, int ft)
{
    p->Set_status_to_finished();
    p->set_finish_time(ft);
    finishedList.enqueue(p);
}
void Organizer::OUTtoBack()
{
    Car *c = nullptr;
    int priority = 0;
    while (outCars.peek(c, priority) && -priority <= timestep)
    {
        outCars.dequeue(c, priority);
        Patients *p = c->getAssignedpatient();
        p->setisPicked(true);
        p->set_pickuptime(timestep);
        p->set_Wait_time(timestep - p->getrequesttime());
        c->set_status("loaded");
        backCars.enqueue(c, -(timestep + c->get_timetoreach()));
    }
}
void Organizer::BackToFree()
{
    Car *c = nullptr;
    int priority = 0;
    while (backCars.peek(c, priority) && -priority <= timestep)
    {
        backCars.dequeue(c, priority);
        c->set_busyTime(c->get_busyTime() + timestep - c->get_assignmenttime());
        if (c->getAssignedpatient())
            add_to_finishedlist(c->getAssignedpatient(), timestep);
        c->clearAssignedPatient();
        if (c->getCheckupStatus())
        {
            int duration =
                c->gettype() == "NC" ? Car::getCheckupTimeNCAR() : Car::getCheckupTimeSCAR();
            c->set_assignmenttime(timestep);
            c->set_status("checkup");
            checkupList.enqueue(c, -(timestep + duration));
        }
        else
            returnFree(c);
    }
}
void Organizer::processCheckups()
{
    Car *c = nullptr;
    int priority = 0;
    while (checkupList.peek(c, priority) && -priority <= timestep)
    {
        checkupList.dequeue(c, priority);
        c->set_busyTime(c->get_busyTime() + timestep - c->get_assignmenttime());
        c->setCheckupStatus(false);
        returnFree(c);
    }
}
void Organizer::processCancellations()
{
    int count = Cancelled.getCount();
    Patients *request = nullptr;
    while (count-- && Cancelled.dequeue(request))
    {
        if (request->getcanceltime() > timestep)
        {
            Cancelled.enqueue(request);
            continue;
        }
        auto patients = ownedPatients;
        Patients *p = nullptr;
        while (patients.dequeue(p))
        {
            if (p->getid() != request->getid() || p->getHID() != request->getHID() ||
                p->gettype() != "NP" || p->getcancelstatus() || p->getisPicked() ||
                p->getfinishstatus() || p->getrequesttime() > timestep)
                continue;
            Hospital &h = hospitalList[p->getHID() - 1];
            if (!h.getNormalpatientQueue().cancelRequest(p))
            {
                Car *c = outCars.removeCarByPatientID(p->getid());
                if (!c)
                    break;
                c->clearAssignedPatient();
                c->set_status("returning");
                // An interrupted outbound trip returns over the distance already travelled.
                backCars.enqueue(c, -(timestep + timestep - c->get_assignmenttime()));
            }
            p->Set_status_to_cancelled();
            ++cancelledCount;
            break;
        }
    }
}
void Organizer::assign_to_another_hospital(Patients *p)
{
    int source = p->getHID() - 1, best = -1, shortest = std::numeric_limits<int>::max(),
        nearest = shortest;
    for (int i = 0; i < totalhospitals; ++i)
    {
        if (i == source || hospitalList[i].getnumberncars() + hospitalList[i].getnumberscars() == 0)
            continue;
        int count = hospitalList[i].getEmergencyCount();
        int distance = Hospital::getHdistance(source, i);
        if (count < shortest || (count == shortest && distance < nearest))
        {
            best = i;
            shortest = count;
            nearest = distance;
        }
    }
    if (best < 0)
    {
        hospitalList[source].addemergencypatient(p, p->getseverity());
        return;
    }
    p->setHID(best + 1);
    p->setDistance(p->getdistance() + nearest);
    if (!p->transferred)
    {
        p->transferred = true;
        ++transfer_emerg;
    }
    hospitalList[best].addemergencypatient(p, p->retry ? std::numeric_limits<int>::max()
                                                       : p->getseverity());
}
void Organizer::assign_patients_to_hospitals()
{
    Patients *p = nullptr;
    while (AllRequests.peek(p) && p->getrequesttime() <= timestep)
    {
        AllRequests.dequeue(p);
        Hospital &h = hospitalList[p->getHID() - 1];
        if (p->gettype() == "NP")
            h.addnormalpatient(p);
        else if (p->gettype() == "SP")
            h.addspecialpatient(p);
        else
            h.addemergencypatient(p, p->getseverity());
    }
}
void Organizer::outCarFailureProbability()
{
    if (outCars.isEmpty() || Car::getFailureRate() == 0)
        return;
    if (std::generate_canonical<double, 32>(random) >= Car::getFailureRate())
        return;
    int selected = static_cast<int>(random() % outCars.getCount());
    priQueue<Car *> saved;
    Car *c = nullptr;
    int priority = 0;
    while (outCars.dequeue(c, priority))
    {
        if (selected-- != 0)
        {
            saved.enqueue(c, priority);
            continue;
        }
        Patients *p = c->getAssignedpatient();
        Hospital &h = hospitalList[p->getHID() - 1];
        // Failed patients keep their original request time and go to the front.
        if (p->gettype() == "NP")
            h.getNormalpatientQueue().enqueueFront(p);
        else if (p->gettype() == "SP")
            h.getSpecialpatientQueue().enqueueFront(p);
        else
            h.getEmergencypatientQueue().enqueueFront(p, std::numeric_limits<int>::max());
        p->retry = true;
        c->clearAssignedPatient();
        c->setCheckupStatus(true);
        c->set_status("returning");
        backCars.enqueue(c, -(timestep + timestep - c->get_assignmenttime()));
        ++failureCount;
    }
    while (saved.dequeue(c, priority))
        outCars.enqueue(c, priority);
}
void Organizer::step()
{
    ++timestep;
    // Complete due events before dispatch: returning cars can be used this timestep.
    OUTtoBack();
    BackToFree();
    processCheckups();
    assign_patients_to_hospitals();
    processCancellations();
    // Transfer each unassigned emergency at most once per timestep, avoiding transfer loops.
    LinkedQueue<Patients *> transfers;
    for (int i = 0; i < totalhospitals; ++i)
    {
        Hospital &h = hospitalList[i];
        Car *c = nullptr;
        while ((c = h.assign_emergencypatient()))
            dispatch(c);
        Patients *p = nullptr;
        int priority = 0;
        h.setStagedEmergencies(h.getEmergencypatientQueue().getCount());
        while (h.getEmergencypatientQueue().dequeue(p, priority))
            transfers.enqueue(p);
    }
    Patients *p = nullptr;
    while (transfers.dequeue(p))
    {
        hospitalList[p->getHID() - 1].removeStagedEmergency();
        // Keep a transferred request at its destination until a car is free.
        // Repeated transfers would inflate its distance and could starve it forever.
        if (p->transferred || p->retry)
            hospitalList[p->getHID() - 1].addemergencypatient(
                p, p->retry ? std::numeric_limits<int>::max() : p->getseverity());
        else
            assign_to_another_hospital(p);
    }
    for (int i = 0; i < totalhospitals; ++i)
    {
        Hospital &h = hospitalList[i];
        Car *c = nullptr;
        while ((c = h.assign_emergencypatient()))
            dispatch(c);
        while ((c = h.assign_patient_to_scar()))
            dispatch(c);
        while ((c = h.assign_patient_to_ncar()))
            dispatch(c);
    }
    // Zero-distance requests finish immediately; picked-up patients cannot fail OUT.
    OUTtoBack();
    BackToFree();
    processCheckups();
    outCarFailureProbability();
    BackToFree();
    processCheckups();
}
void Organizer::run(bool interactive, const std::string &output)
{
    if (!interactive)
        std::cout << "Silent mode: simulation started...\n";
    while (!complete())
    {
        if (timestep >= 10000000)
            throw std::runtime_error(
                "Simulation limit reached; lower failure probability or check fleet capacity.");
        step();
        if (interactive)
        {
            Output::printState(*this);
            std::cout << "[Enter] Next timestep   [R] Run to completion   [Q] Stop\n> ";
            std::string choice;
            if (!std::getline(std::cin, choice))
                interactive = false;
            else if (choice == "r" || choice == "R")
                interactive = false;
            else if (choice == "q" || choice == "Q")
            {
                std::cout << "Stopped. No final report written.\n";
                return;
            }
        }
    }
    Output::OutputFile(*this, output);
    std::cout << "Simulation complete at timestep " << timestep
              << ". Served: " << finishedList.getCount() << " | Cancelled: " << cancelledCount
              << " | OUT failures: " << failureCount << "\nReport: " << output << "\n";
}
