#pragma once
#include "priQueue.h"
#include "Car.h"

class OUTcars : public priQueue<Car *>
{
  public:
    Car *removeCarByPatientID(int pid)
    {
        Car *found = nullptr;
        priQueue<Car *> temp;
        Car *c;
        int pri;
        while (dequeue(c, pri))
        {
            if (found == nullptr && c && c->getAssignedpatient() &&
                c->getAssignedpatient()->getid() == pid)
            {
                found = c;
            }
            else
            {
                temp.enqueue(c, pri);
            }
        }
        while (temp.dequeue(c, pri))
            enqueue(c, pri);
        return found;
    }
};
