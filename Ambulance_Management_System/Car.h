#pragma once
#include <string>
#include "Patients.h"
using namespace std;

class Car
{
  private:
    int carID;
    string cartype;
    string status;
    bool CheckupStatus;
    int timetoreach;
    int busytime;
    int homeHospitalIndex;
    Patients *assignedPatient;

    static int speedSCAR;
    static int speedNCAR;
    static int checkupTimeSCAR;
    static int checkupTimeNCAR;
    static float failureRate;

  public:
    Car()
        : carID(0), cartype(""), status("ready"), CheckupStatus(false), timetoreach(0), busytime(0),
          homeHospitalIndex(0), assignedPatient(nullptr)
    {
    }

    Car(int ID, string type, string stat, int speed, int homeHospital)
        : carID(ID), cartype(type), status(stat), CheckupStatus(false), timetoreach(0), busytime(0),
          homeHospitalIndex(homeHospital), assignedPatient(nullptr)
    {
        if (type == "SC")
            speedSCAR = speed;
        else if (type == "NC")
            speedNCAR = speed;
    }

    static void setspeedNCAR(int nnc) { speedNCAR = nnc; }
    static void setspeedSCAR(int snc) { speedSCAR = snc; }
    static int getspeedNCAR() { return speedNCAR; }
    static int getspeedSCAR() { return speedSCAR; }

    int getcarid() const { return carID; }
    string gettype() const { return cartype; }
    string get_status() const { return status; }
    bool getCheckupStatus() const { return CheckupStatus; }
    int get_timetoreach() const { return timetoreach; }
    int get_busyTime() const { return busytime; }
    int getHomeHospital() const { return homeHospitalIndex; }
    Patients *getAssignedpatient() const { return assignedPatient; }
    int get_assignmenttime() const { return AT; }

    void set_status(string s) { status = s; }
    void setCheckupStatus(bool v) { CheckupStatus = v; }
    void set_timetoreach(int t) { timetoreach = t; }
    void set_busyTime(int b) { busytime = b; }
    void setHomeHospital(int h) { homeHospitalIndex = h; }

    void setAssignedpatient(Patients *p) { assignedPatient = p; }
    void clearAssignedPatient() { assignedPatient = nullptr; }

    void set_assignmenttime(int at) { AT = at; }

    static void setCheckupTimeSCAR(int t) { checkupTimeSCAR = t; }
    static void setCheckupTimeNCAR(int t) { checkupTimeNCAR = t; }
    static int getCheckupTimeSCAR() { return checkupTimeSCAR; }
    static int getCheckupTimeNCAR() { return checkupTimeNCAR; }

    static void setFailureRate(float f) { failureRate = f; }
    static float getFailureRate() { return failureRate; }

    friend ostream &operator<<(ostream &output, const Car &c)
    {
        if (c.assignedPatient)
            output << c.cartype << c.carID << "_H" << c.homeHospitalIndex << "_P"
                   << c.assignedPatient->getid();
        else
            output << c.cartype << c.carID << "_H" << c.homeHospitalIndex << "(" << c.status << ")";
        return output;
    }

  private:
    int AT = 0;
};
