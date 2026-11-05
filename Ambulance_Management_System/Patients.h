#pragma once
#include <string>
#include <ostream>
using namespace std;

class Patients
{
  private:
    int PID, HID;
    // RT: request; PT: pickup; FT: finish; WT: pickup minus request.
    int RT, PT, FT, WT;
    int DST, CT, SVR;
    string type;
    bool finished;
    bool cancelled;
    bool isPicked;
    int assignmentTime;

  public:
    Patients()
        : PID(0), HID(0), RT(0), PT(0), FT(0), WT(0), DST(0), CT(0), SVR(0), type(""),
          finished(false), cancelled(false), isPicked(false), assignmentTime(0)
    {
    }

    Patients(string t, int rt, int pid, int hid, int dst, int svr = -1)
        : PID(pid), HID(hid), RT(rt), PT(0), FT(0), WT(0), DST(dst), CT(0), SVR(svr), type(t),
          finished(false), cancelled(false), isPicked(false), assignmentTime(0)
    {
    }

    Patients(int ct, int pid, int hid)
        : PID(pid), HID(hid), RT(0), PT(0), FT(0), WT(0), DST(0), CT(ct), SVR(0), type("NP"),
          finished(false), cancelled(false), isPicked(false), assignmentTime(0)
    {
    }

    int getid() const { return PID; }
    int getHID() const { return HID; }
    void setHID(int hid) { HID = hid; }
    void setDistance(int distance) { DST = distance; }
    bool getcancelstatus() const { return cancelled; }
    bool getfinishstatus() const { return finished; }
    bool transferred = false; // Count each emergency patient at most once in transfer statistics.
    bool retry = false;       // An OUT failure gives this request the next compatible car.
    int getrequesttime() const { return RT; }
    int get_assignmenttime() const { return assignmentTime; }
    int get_pickuptime() const { return PT; }
    int get_finish_time() const { return FT; }
    int get_wait_time() const { return WT; }
    int getdistance() const { return DST; }
    int getseverity() const { return SVR; }
    int getcanceltime() const { return CT; }
    string gettype() const { return type; }

    void set_assignmenttime(int at) { assignmentTime = at; }
    void set_pickuptime(int pt) { PT = pt; }
    void set_finish_time(int ft) { FT = ft; }
    void set_Wait_time(int wt) { WT = wt; }
    void setisPicked(bool one) { isPicked = one; }
    bool getisPicked() const { return isPicked; }

    void Set_status_to_finished() { finished = true; }
    void Set_status_to_cancelled() { cancelled = true; }

    friend ostream &operator<<(ostream &output, const Patients &p)
    {
        output << p.PID;
        return output;
    }
};
