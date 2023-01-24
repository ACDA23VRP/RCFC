
#ifndef VRP_TOURVRP_H
#define VRP_TOURVRP_H

#include <iostream>
#include <vector>
#include "model_data.h"

using namespace std;

class tourVRP{
private:
    int             day_;
public:
    vector<int>     tour_;
    int             length_;
    int             capacity_;
    SCIP_Real       obj_;
    tourVRP(){
        length_ = 0;
        capacity_ = 0;
        obj_ = 0;
        day_ = 0;
    }
    tourVRP(
            int length,
            int day
    ): day_(day)
    {
        tour_.resize(length);
        length_ = length;
        capacity_ = 0;
        obj_ = 0.0;
    }
    ~tourVRP()= default;

    void copy(
            tourVRP& tour
    );

    /** returns day of tour */
    int getDay(){
        return day_;
    }

    void setDay(int day){
        day_ = day;
    }

    /** clears the tour of tvrp object */
    void clearTour();

    /** check if correct capacity is stored */
    SCIP_Bool checkCapacity(
        model_data* modelData
    );

    /** check if correct obj is stored */
    SCIP_Bool checkObj(
        model_data* modelData
    );

    /** checks if tour is feasible */
    SCIP_Bool isFeasible(
        model_data* modelData
    );

    /** adds cust at the end of the tour */
    SCIP_RETCODE addEnd(
        model_data* modelData,
        int         cust
    );

    /** adds cust to the cheapest spot */
    SCIP_Bool addNode(
        SCIP*       scip,
        model_data* modelData,
        int         *newpos,
        int         cust
    );

    SCIP_RETCODE setValues(
            model_data* modelData
    );

    friend ostream &operator<<(std::ostream &os, const tourVRP& tvrp) {
        os << "Tour on day "<< tvrp.day_<< '\n';
        os << "Length: " << tvrp.length_ << '\n';
        os << "Costs: " << tvrp.obj_ << '\n';
        os << "Capacity used: " << tvrp.capacity_ << '\n';
        os << "Tour: ";
        for(int u : tvrp.tour_)
            os << u << " ";
        os << '\n';
        return os;
    };
};


#endif //VRP_TOURVRP_H
