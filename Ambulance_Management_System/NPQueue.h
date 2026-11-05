#pragma once
#include "LinkedQueue.h"
#include "Patients.h"

class NPQueue : public LinkedQueue<Patients *>
{
  public:
    bool cancelRequest(Patients *target)
    {
        if (target == nullptr)
            return false;
        int pid = target->getid();
        int hid = target->getHID();
        LinkedQueue<Patients *> temp;
        Patients *current = nullptr;
        bool removed = false;
        while (dequeue(current))
        {
            if (!removed && current && current->getid() == pid && current->getHID() == hid)
            {
                removed = true;
                continue; // discard
            }
            temp.enqueue(current);
        }
        while (temp.dequeue(current))
            enqueue(current);
        return removed;
    }
};
