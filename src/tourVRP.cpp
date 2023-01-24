
#include "tourVRP.h"

SCIP_Bool tourVRP::isFeasible(
        model_data *modelData
){
    vector<vector<double>>& tr = modelData->travel;
    vector<vector<timeWindow>>& tws = modelData->timeWindows;
    assert(length_ > 0);
    /* initialize for first customer */
    int cap = modelData->demand[tour_[0]];
    if(cap > modelData->max_caps[day_])
        cout << cap << " vs " << modelData->max_caps[day_] << " cust: " << tour_[0]  << " day " << day_ << endl;
    assert(cap <= modelData->max_caps[day_]);
    double time = max((double) tws[tour_[0]][day_].start, tws[0][day_].start + tr[0][tour_[0]]);
    /* check if first customer is reachable within time window */
    if(time > tws[tour_[0]][day_].end + 0.00001)
        return FALSE;
    time += modelData->service[tour_[0]];
//    if(length_ == 9 && obj_ >= 119.22 && obj_ < 119.24 && tour_[0] == 37 && tour_[1] == 36 && day_ == 1)
//        cout << tour_[0] << " departure: " << time << endl;
    /* iterate through the tour */
    for(int i = 1; i < length_; i++)
    {
        cap += modelData->demand[tour_[i]];

        /* check if next customer is reachable within time window and capacity is not exceeded */
        if(time + tr[tour_[i-1]][tour_[i]] > tws[tour_[i]][day_].end + 0.00001 || cap > modelData->max_caps[day_])
        {
//            if(time + tr[tour_[i-1]][tour_[i]] > tws[tour_[i]][day_].end + 0.00001)
//            {
//                cout << "TIME PROBLEM " << time + tr[tour_[i-1]][tour_[i]] << " > " << tws[tour_[i]][day_].end;
//                cout << " capacity: "<< cap << "/" << modelData->max_caps[day_] <<endl;
//            }
//            else
//                cout << "CAPACITY PROBLEM" << endl;
//            cout << "ERROR FOR CUST: " << tour_[i] << endl;
//            cout << time + tr[tour_[i-1]][tour_[i]] << " > " << tws[tour_[i]][day_].end << " or cap: " << cap << endl;
            return FALSE;
        }
        time = max(time + tr[tour_[i-1]][tour_[i]], (double) tws[tour_[i]][day_].start) + modelData->service[tour_[i]];
//        if(length_ == 9 && obj_ >= 119.22 && obj_ < 119.24 && tour_[0] == 37 && tour_[1] == 36 && day_ == 1)
//            cout << tour_[i] << " departure: " << time << " cap: " << cap << "/" << modelData->max_caps[day_] << endl;
    }
    /* back to depot */
    if(time + tr[tour_[length_-1]][0] > tws[0][day_].end + 0.00001)
    {
        return FALSE;
    }
//    cout << "depot departure: " << time + tr[tour_[length_-1]][0] << endl;

    return TRUE;
}

/** try to add cust to the cheapest spot (if possible)
 * @return FALSE if no spot was found
 *         TRUE else */
SCIP_Bool tourVRP::addNode(
        SCIP*       scip,
        model_data* modelData,
        int         *newpos,
        int         cust
) {
    int i;
    tourVRP tmpTour(length_ + 2, day_);
    double extracosts;                              // costs of adding the customer
    double bestcosts = SCIP_DEFAULT_INFINITY;
    int bestpos = -1;
    vector<vector<double>>& tr = modelData->travel;
    vector<vector<timeWindow>>& tws = modelData->timeWindows;

    /* initialize with new customer at first position */
    tmpTour.length_--;
    tmpTour.tour_[0] = cust;
    for(i = 1; i < length_ + 1; i++)
    {
        tmpTour.tour_[i] = tour_[i - 1];
    }
    /* add depot at the end for technical reasons */
    tmpTour.tour_[i] = 0;

    extracosts = tr[0][cust] + tr[cust][tmpTour.tour_[1]] - tr[0][tmpTour.tour_[1]];
    if(SCIPisSumNegative(scip, extracosts - bestcosts))
    {
        /* check for necessary conditions of feasiblity first */
        if(tws[cust][day_].start + modelData->service[cust] <= tws[tmpTour.tour_[1]][day_].end)
        {
            /* check if position is feasible */
            if(tmpTour.isFeasible(modelData))
            {
                bestpos = 0;
                bestcosts = extracosts;
            }
        }
    }

    /* try every other position */
    for(i = 1; i < length_ + 1; i++)
    {
        tmpTour.tour_[i - 1] = tmpTour.tour_[i];
        tmpTour.tour_[i] = cust;

        /* check if the extra costs are the lowest so far */
        extracosts = tr[tmpTour.tour_[i-1]][cust] + tr[cust][tmpTour.tour_[i+1]] - tr[tmpTour.tour_[i-1]][tmpTour.tour_[i+1]];
        if(SCIPisSumNegative(scip, extracosts - bestcosts))
        {
            /* check for necessary conditions first */
            if(tws[cust][day_].end < tws[tmpTour.tour_[i-1]][day_].start + modelData->service[tmpTour.tour_[i-1]])
                break;
            if(tws[cust][day_].start + modelData->service[cust] > tws[tmpTour.tour_[i+1]][day_].end)
                continue;
            /* check if current position is feasible */
            if(tmpTour.isFeasible(modelData))
            {
                bestpos = i;
                bestcosts = extracosts;
            }
        }
    }
    /* if at least one feasible spot was found take the best one */
    if(bestpos >= 0)
    {
        tour_.push_back(-1);
        for(i = length_; i > bestpos; i--)
        {
            tour_[i] = tour_[i-1];
        }
        tour_[i] = cust;
        obj_ += bestcosts;
        length_++;
        capacity_ += modelData->demand[cust];
        if(newpos != nullptr)
            *newpos = bestpos;
        return TRUE;
    }
    return FALSE;
}

void tourVRP::copy(
        tourVRP& tour
){
    tour_.clear();
    for(auto i : tour.tour_)
    {
        tour_.push_back(i);
    }
    length_ = tour.length_;
    capacity_ = tour.capacity_;
    obj_ = tour.obj_;
    day_ = tour.getDay();
}

void tourVRP::clearTour()
{
    assert(length_ > 0);
    obj_ = 0.0;
    length_ = 0;
    tour_.clear();
}

/** check if correct capacity is stored */
SCIP_Bool tourVRP::checkCapacity(
        model_data* modelData
){
    int cap = 0;
    for(auto v : tour_)
    {
        cap += modelData->demand[v];
    }
    return (cap == capacity_);
}

/** check if correct obj is stored */
SCIP_Bool tourVRP::checkObj(
        model_data* modelData
){
    if(length_ == 0)
        return (obj_ == 0);
    double costs = modelData->travel[0][tour_[0]];
    for(int i = 1; i < length_; i++)
    {
        costs += modelData->travel[tour_[i-1]][tour_[i]];
    }
    costs += modelData->travel[tour_[length_-1]][0];
//    cout << "COSTS " << costs << " VS. Obj: " << obj_ << '\n';
    return (fabs(costs - obj_) < 0.000001);
}

/** adds cust at the end of the tour */
SCIP_RETCODE tourVRP::addEnd(
        model_data* modelData,
        int         cust
){
    assert(capacity_ + modelData->demand[cust] <= modelData->max_caps[day_]);

    tour_.push_back(cust);
    capacity_ += modelData->demand[cust];
    if(length_ == 0)
    {
        obj_ += modelData->travel[0][cust];
    }else{
        obj_ += modelData->travel[tour_[length_ - 1]][cust];
    }
    length_++;

    return SCIP_OKAY;
}

SCIP_RETCODE tourVRP::setValues(
        model_data* modelData
){
    int capacity = 0;
    double obj = 0.0;
    obj += modelData->travel[0][tour_[0]];
    obj += modelData->travel[tour_[length_-1]][0];
    capacity += modelData->demand[tour_[0]];

    for(int i = 1; i < length_; i++)
    {
        obj += modelData->travel[tour_[i-1]][tour_[i]];
        capacity += modelData->demand[i];
    }

    obj_ = obj;
    capacity_ = capacity;

    return SCIP_OKAY;
}
