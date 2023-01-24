
#include "tools_vrp.h"
#include <iostream>
#include <vector>
#include "model_data.h"
#include "scip/cons_setppc.h"
#include "probdata_vrp.h"
#include "vardata.h"

/** prepares tour variable to be added to be model */
SCIP_RETCODE add_tour_variable(
        SCIP*                       scip,
        vrp::ProbDataVRP*           probData,
        SCIP_Bool                   isFarkas,
        SCIP_Bool                   isInitial,
        char*                       algoName,
        tourVRP&                    tvrp
){
    char name[SCIP_MAXSTRLEN];
    char str_tmp[SCIP_MAXSTRLEN];
    int i;

    /* create variable name */
    if (!isFarkas)
    {
        (void) SCIPsnprintf(name, SCIP_MAXSTRLEN, "%sRed_%2d: ", algoName, tvrp.getDay());
    } else {
        (void) SCIPsnprintf(name, SCIP_MAXSTRLEN, "%sFar_%2d: ", algoName, tvrp.getDay());
    }
    for (i = 0; i < tvrp.length_; i++)
    {
        (void) SCIPsnprintf(str_tmp, SCIP_MAXSTRLEN, "_%d", tvrp.tour_[i]);
        strcat(name, str_tmp);
    }

    SCIP_CALL(SCIPcreateColumn(scip, probData, name, isInitial, tvrp));

    return SCIP_OKAY;
}

/** returns the costs that occur when exchanging tour[pos] by new_cust */
double getExchangeCosts(
        model_data*             modelData,
        vector<int>&            tour,
        int                     length,
        int                     new_cust,
        int                     pos
){
    double tmp_new, tmp_old;
    vector<vector<double>>& tr = modelData->travel;
    assert(length > 0);
    assert(pos < length);
    /* in-going arc */
    if(pos == 0)
    {
        tmp_new = tr[0][new_cust];
        tmp_old = tr[0][tour[pos]];
    }else
    {
        tmp_new = tr[tour[pos-1]][new_cust];
        tmp_old = tr[tour[pos-1]][tour[pos]];
    }
    /* out-going arc */
    if(pos + 1 == length)
    {
        tmp_new += tr[new_cust][0];
        tmp_old += tr[tour[pos]][0];
    }else
    {
        tmp_new += tr[new_cust][tour[pos+1]];
        tmp_old += tr[tour[pos]][tour[pos+1]];
    }
    return tmp_new - tmp_old;
}


double getDeleteCosts(
        model_data*             modelData,
        vector<int>&            tour,
        int                     length,
        int                     pos
){
    double tmp_old;
    double tmp_new;
    assert(length > 0);
    assert(length > pos);

    /* in-going arc */
    tmp_old = pos == 0 ? modelData->travel[0][tour[0]] : modelData->travel[tour[pos-1]][tour[pos]];

    /* new arc */
    if(pos == 0)
    {
        tmp_old = modelData->travel[0][tour[0]];
        tmp_new = pos == length - 1 ? 0 : modelData->travel[0][tour[pos+1]];
    }else
    {
        tmp_old = modelData->travel[tour[pos-1]][tour[pos]];
        tmp_new = pos == length - 1 ? modelData->travel[tour[pos-1]][0] : modelData->travel[tour[pos-1]][tour[pos+1]];
    }
    /* out-going arc */
    tmp_old += pos == length - 1 ? modelData->travel[tour[pos]][0] : modelData->travel[tour[pos]][tour[pos+1]];

    return tmp_new - tmp_old;
}

SCIP_RETCODE addUnvisitedCustomers(
        SCIP*                   scip,
        model_data*             modelData,
        vector<tourVRP>&        tvrp,
        vector<int>&            vehicleofNode,
        vector<int>&            unvisitedC
){
    int i, old_cust, day;
    double tmp_costs;
    SCIP_Bool success;
    vector<int> newUnvisitedC;
    /* try to add unvisited customers */
    for(auto new_cust : unvisitedC)
    {
        assert(vehicleofNode[new_cust] == -1);
        /* check on each available day */
        success = FALSE;
        for(auto vehicle : modelData->availableVehicles[new_cust])
        {
            day = modelData->dayofVehicle[vehicle];
            vector<int>& tour = tvrp[vehicle].tour_;
            /* try every customer of that tour to exchange with current one */
            for(i = 0; i < tvrp[vehicle].length_; i++)
            {
                if(tvrp[vehicle].capacity_ + modelData->demand[new_cust] - modelData->demand[tour[i]] > modelData->max_caps[day])
                    continue;
                tmp_costs = getExchangeCosts(modelData, tour, tvrp[vehicle].length_, new_cust, i);
                tvrp[vehicle].obj_ += tmp_costs;
                old_cust = tour[i];
                tour[i] = new_cust;
                tvrp[vehicle].capacity_ += modelData->demand[new_cust] - modelData->demand[old_cust];
                if(tvrp[vehicle].isFeasible(modelData))
                {
                    /* find new spot for old_cust */
                    for(auto vehicle2 : modelData->availableVehicles[old_cust])
                    {
                        int day2 = modelData->dayofVehicle[vehicle2];
                        if(tvrp[vehicle2].capacity_ + modelData->demand[old_cust] > modelData->max_caps[day2])
                            continue;
                        if(tvrp[vehicle2].addNode(scip, modelData, nullptr, old_cust))
                        {
                            success = TRUE;
                            vehicleofNode[new_cust] = vehicle;
                            vehicleofNode[old_cust] = vehicle2;
//                            cout << "Customer " << new_cust << " on day " << day << " --> Customer " << old_cust << " on day " << day2 << '\n';
                            break;
                        }
                    }
                }
                if(success)
                    break;
                /* if old_cust could not be added to any tour, undo changes */
                tour[i] = old_cust;
                tvrp[vehicle].obj_ -= tmp_costs;
                tvrp[vehicle].capacity_ += modelData->demand[old_cust] - modelData->demand[new_cust];
            }
            if(success)
                break;
        }
        if(!success)
            newUnvisitedC.push_back(new_cust);
    }
    unvisitedC = newUnvisitedC;
    return SCIP_OKAY;
}

/**
 * As long as there are customers u, v, w such that exchanging v by u, exchanging w by v
 * and giving w a new position in one of the tours yields a better solution,
 * apply this step.
 * */
SCIP_RETCODE twoNodeShift(
        SCIP*                   scip,
        model_data*             modelData,
        vector<tourVRP>&        tvrps,
        vector<int>&            vehicleofnode
)
{
    double bestimprovement;
    int currentnode, node, pos;
    int nC;
    int k;
    SCIP_Bool changed;
    int day1;
    int pos1, newpos;
    double oldobj1, oldobj2, oldobj3;
    double newobj1 = 0.0, newobj2, newobj3;
    double oldtotal, newtotal;

    double bestnewobj1 = 0.0, bestnewobj2 = 0.0;
    int bestnewday1 = -1, bestnewday2 = -1;
    int bestnewpos1 = -1, bestnewpos2 = -1;
    int numimprov = 0;

    double chng1 = 0.0;
    double chng2;
    double chng3;

    assert(modelData != nullptr);
    nC = modelData->nC;

    changed = TRUE;
    while(changed){
        changed = FALSE;
        for (currentnode = 1; currentnode < nC; currentnode++)
        {
            if(vehicleofnode[currentnode] == -1)
                continue;
            bestimprovement = 0.0;
            day1 = vehicleofnode[currentnode];
            assert(day1 >= 0);
            tourVRP tvrpday1(tvrps[day1].length_-1, modelData->dayofVehicle[day1]);
            k = 0;
            oldobj1 = tvrps[day1].obj_;
            for (pos = 0; pos < tvrps[day1].length_; pos++)
            {
                if (tvrps[day1].tour_[pos] == currentnode)
                {
                    k = 1;
                    chng1 = getDeleteCosts(modelData, tvrps[day1].tour_, tvrps[day1].length_, pos);
                    newobj1 = tvrps[day1].obj_ + chng1;
                    tvrpday1.obj_ = newobj1;
                    continue;
                }
                tvrpday1.tour_[pos - k] = tvrps[day1].tour_[pos];
            }
            assert(!SCIPisSumNegative(scip, newobj1));
            tvrps[day1].length_--;
            if (tvrps[day1].length_ == 0)
            {
                assert(SCIPisZero(scip, newobj1));
                newobj1 = 0.0;
            }
            else if(!tvrpday1.isFeasible(modelData))
            {
                assert(FALSE); // should never happen!
            }
            for(auto day2 : modelData->availableVehicles[currentnode])
            {
                for (pos1 = 0; pos1 < tvrps[day2].length_; pos1++) {
                    oldobj2 = tvrps[day2].obj_;
                    if(day2 == day1)
                    {
                        chng2 = getExchangeCosts(modelData, tvrpday1.tour_, tvrps[day1].length_, currentnode, pos1);
                        newobj2 = newobj1 + chng2;
                        node = tvrpday1.tour_[pos1];
                        tvrpday1.tour_[pos1] = currentnode;
                        if (!tvrpday1.isFeasible(modelData))
                        {
                            tvrpday1.tour_[pos1] = node;
                            continue;
                        }
                    }else{
                        chng2 = getExchangeCosts(modelData, tvrps[day2].tour_, tvrps[day2].length_, currentnode, pos1);
                        newobj2 = oldobj2 + chng2;
                        node = tvrps[day2].tour_[pos1];
                        tvrps[day2].tour_[pos1] = currentnode;
                        if (!tvrps[day2].isFeasible(modelData))
                        {
                            tvrps[day2].tour_[pos1] = node;
                            continue;
                        }
                    }

                    for(auto day3 : modelData->availableVehicles[node]){
                        if(day3 == day1)
                        {
                            newobj3 = day1 == day2 ? newobj2 : newobj1;
                        }else if(day3 == day2)
                        {
                            newobj3 = newobj2;
                        }else
                        {
                            newobj3 = tvrps[day3].obj_;
                        }
                        tourVRP tvrpday3(tvrps[day3].length_, modelData->dayofVehicle[day3]);
                        for(pos = 0; pos < tvrpday3.length_; pos++)
                        {
                            if(day3 == day1)
                            {
                                tvrpday3.tour_[pos] = tvrpday1.tour_[pos];
                            }else{
                                tvrpday3.tour_[pos] = tvrps[day3].tour_[pos];
                            }
                        }
                        oldobj3 = tvrps[day3].obj_;
                        tvrpday3.obj_ = newobj3;
                        if(tvrpday3.addNode(scip, modelData, &newpos, node))
                        {
                            chng3 = tvrpday3.obj_ - newobj3;
                            newobj3 = tvrpday3.obj_;
                            if(day1 == day2)
                            {
                                if(day2 == day3)
                                {
                                    oldtotal = oldobj1;
                                    newtotal = newobj3;
                                }else{
                                    oldtotal = oldobj1 + oldobj3;
                                    newtotal = newobj2 + newobj3;
                                }
                            }else if(day1 == day3)
                            {
                                oldtotal = oldobj1 + oldobj2;
                                newtotal = newobj2 + newobj3;
                            }else if(day2 == day3)
                            {
                                oldtotal = oldobj1 + oldobj2;
                                newtotal = newobj1 + newobj3;
                            }else{
                                oldtotal = oldobj1 + oldobj2 + oldobj3;
                                newtotal = newobj1 + newobj2 + newobj3;
                            }
                            assert(SCIPisEQ(scip, newtotal - oldtotal, chng1+chng2+chng3)); // TODO DELETE
                            if(SCIPisSumPositive(scip, -(chng1+chng2+chng3) - bestimprovement))
                            {
                                bestimprovement = -(chng1+chng2+chng3);
                                bestnewpos1 = pos1;
                                bestnewpos2 = newpos;
                                bestnewday1 = day2;
                                bestnewday2 = day3;
                                bestnewobj1 = newobj2;
                                bestnewobj2 = newobj3;
                            }
                        }
                    }
                    if(day1 == day2)
                    {
                        tvrpday1.tour_[pos1] = node;
                    }else{
                        tvrps[day2].tour_[pos1] = node;
                    }
                }
            }
            if(SCIPisSumPositive(scip, bestimprovement))
            {
                changed = TRUE;
                for(pos = 0; pos < tvrps[day1].length_; pos++)
                {
                    tvrps[day1].tour_[pos] = tvrpday1.tour_[pos];
                }
                assert(!SCIPisNegative(scip, newobj1));
                tvrps[day1].tour_.pop_back();
                tvrps[day1].obj_ = newobj1;
                node = tvrps[bestnewday1].tour_[bestnewpos1];
                tvrps[bestnewday1].tour_[bestnewpos1] = currentnode;
                tvrps[bestnewday1].obj_ = bestnewobj1;
                tvrps[bestnewday2].tour_.push_back(0);
                for(pos = tvrps[bestnewday2].length_; pos > bestnewpos2; pos--)
                {
                    tvrps[bestnewday2].tour_[pos] = tvrps[bestnewday2].tour_[pos - 1];
                }
                tvrps[bestnewday2].tour_[bestnewpos2] = node;
                tvrps[bestnewday2].obj_ = bestnewobj2;
                tvrps[bestnewday2].length_++;
                vehicleofnode[currentnode] = bestnewday1;
                vehicleofnode[node] = bestnewday2;
                numimprov++;

                tvrps[day1].capacity_ -= modelData->demand[currentnode];
                tvrps[bestnewday1].capacity_ += modelData->demand[currentnode] - modelData->demand[node];
                tvrps[bestnewday2].capacity_ += modelData->demand[node];

                /* check for wrong calculations */
                assert(tvrps[day1].checkObj(modelData));
                assert(tvrps[bestnewday1].checkObj(modelData));
                assert(tvrps[bestnewday2].checkObj(modelData));

            }else{
                tvrps[day1].length_++;
            }
        }
    }
    return SCIP_OKAY;
}

/** checks if tour violates ng path properties */
bool violatesNGProperty(
        vector<bitset<neighborhood_size>>&  ng_sets,
        vector<bitset<neighborhood_size>>&  ng_DSSR,
        vector<int>&                        tour
){
    bool violates = FALSE;
    bitset<neighborhood_size> ng_memory;
    for(int i = 0; i < (int) tour.size(); i++)
    {
        int u = tour[i];
        if(ng_memory[u])
        {
            violates = TRUE;
            for(int j = i - 1; j >= 0; j++)
            {
                if(tour[j] == u)
                    break;
                assert(ng_sets[tour[j]][u]);
                ng_DSSR[tour[j]][u] = true;
            }
        }
        ng_memory = ng_memory & ng_sets[u];
        ng_memory[u] = true;
    }
    return violates;
}