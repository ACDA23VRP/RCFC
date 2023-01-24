
#include "local_search_pricing.h"
#include "model_data.h"
#include "vardata.h"
#include "pricer_vrp.h"
#include "probdata_vrp.h"
#include "tools_vrp.h"
#include "tourVRP.h"

using namespace std;

static
bool addNodeToTourPricing(
        SCIP*                               scip,
        model_data*                         modelData,
        tourVRP&                            tvrp,
        const vector<vector<bool>>&         isForbidden,
        int                                 cust,
        int                                 *newpos,
        SCIP_Real                           threshold
){
    int i, day = tvrp.getDay();
    tourVRP tmpTour(tvrp.length_ + 2, day);
    double extracosts;                              // costs of adding the customer
    double bestcosts = threshold;
    int bestpos = -1;
    vector<vector<double>>& tr = modelData->travel;
    vector<vector<timeWindow>>& tws = modelData->timeWindows;

    /* initialize with new customer at first position */
    tmpTour.length_--;
    tmpTour.tour_[0] = cust;
    for(i = 1; i < tvrp.length_ + 1; i++)
    {
        tmpTour.tour_[i] = tvrp.tour_[i - 1];
    }
    /* add depot at the end for technical reasons */
    tmpTour.tour_[i] = 0;

    extracosts = tr[0][cust] + tr[cust][tmpTour.tour_[1]] - tr[0][tmpTour.tour_[1]];
    if(SCIPisSumNegative(scip, extracosts - bestcosts))
    {
        /* check for branching decisions */
        if(!isForbidden[0][cust] && !isForbidden[cust][tmpTour.tour_[1]])
        {
            /* check for necessary conditions of feasiblity first */
            if(tws[cust][day].start + modelData->service[cust] <= tws[tmpTour.tour_[1]][day].end)
            {
                /* check if position is feasible */
                if(tmpTour.isFeasible(modelData))
                {
                    bestpos = 0;
                    bestcosts = extracosts;
                }
            }
        }
    }

    /* try every other position */
    for(i = 1; i < tvrp.length_ + 1; i++)
    {
        tmpTour.tour_[i - 1] = tmpTour.tour_[i];
        tmpTour.tour_[i] = cust;
        /* check for branching decisions */
        if(isForbidden[tmpTour.tour_[i-1]][cust] || isForbidden[cust][tmpTour.tour_[i+1]])
            continue;
        /* check if the extra costs are the lowest so far */
        extracosts = tr[tmpTour.tour_[i-1]][cust] + tr[cust][tmpTour.tour_[i+1]] - tr[tmpTour.tour_[i-1]][tmpTour.tour_[i+1]];
        if(SCIPisSumNegative(scip, extracosts - bestcosts))
        {
            /* check for necessary conditions first */
            if(tws[cust][day].end < tws[tmpTour.tour_[i-1]][day].start + modelData->service[tmpTour.tour_[i-1]])
                break;
            if(tws[cust][day].start + modelData->service[cust] > tws[tmpTour.tour_[i+1]][day].end)
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
        tvrp.tour_.push_back(-1);
        for(i = tvrp.length_; i > bestpos; i--)
        {
            tvrp.tour_[i] = tvrp.tour_[i-1];
        }
        tvrp.tour_[i] = cust;
        tvrp.obj_ += bestcosts;
        tvrp.length_++;
        tvrp.capacity_ += modelData->demand[cust];
        if(newpos != nullptr)
            *newpos = bestpos;
        return TRUE;
    }
    return FALSE;
}

/** tries to add customers to the tour if it reduces the current reduced cost */
static
bool extendColumn(
        SCIP*                               scip,
        model_data*                         modelData,
        ObjPricerVRP*                       pricerData,
        bool                                isFarkas,
        tourVRP&                            tvrp,
        const vector<SCIP_Real>&            dualvalues,
//        const vector<pair<int, SCIP_Real>>& toCheck,
        vector<bool>&                       isinTour,
        SCIP_Real                           *lhs
){
    SCIP_Real threshold;
//    int cust;
    bool success = FALSE;

    for(auto cust : modelData->neighbors[0][tvrp.getDay()])
//    for(auto p : toCheck)
    {
        if(!pricerData->timetable_[cust][tvrp.getDay()])
            continue;
//        cust = p.first;
//        assert(pricerData->timetable_[cust][tvrp.getDay()]);
        /* cust must not already be in the tour */
        if(isinTour[cust])
            continue;
        /* only customers with positive dual values can lead to improvement */
        if(!SCIPisSumPositive(scip, dualvalues[cust]))
            continue;
        /* check for capacity condition */
        if(tvrp.capacity_ + modelData->demand[cust] > modelData->max_caps[tvrp.getDay()])
            continue;
        /* only add cust if cheaper than dualvalues[cust] */
        threshold = isFarkas ? SCIP_DEFAULT_INFINITY : dualvalues[cust];
        if(addNodeToTourPricing(scip, modelData, tvrp, pricerData->isForbidden_, cust, nullptr, threshold))
        {
            *lhs += dualvalues[cust];
            isinTour[cust] = TRUE;
            success = TRUE;
        }
    }
    return success;
}

/** tries to exchange a customers of the tour by unused ones to lower the reduced costs */
static
bool shiftColumn(
        SCIP*                               scip,
        model_data*                         modelData,
        ObjPricerVRP*                       pricerData,
        bool                           isFarkas,
        tourVRP&                            tvrp,
        const vector<SCIP_Real>&            dualvalues,
//        const vector<pair<int, SCIP_Real>>& toCheck,
        vector<bool>&                  isinTour,
        SCIP_Real                           *lhs
){
    int exchanged;
    bool success = FALSE;
    int day = tvrp.getDay();
    for(auto cust : modelData->neighbors[0][day])
//    for(auto p : toCheck)
    {
        if(!pricerData->timetable_[cust][day])
            continue;
//        cust = p.first;
        if(isinTour[cust])
            continue;
        /* try every spot in current tour */
        for(int pos = 0; pos < tvrp.length_; pos++)
        {
            if(pos == tvrp.length_ - 1 && !pricerData->toDepot_[cust])
                continue;
            /* do not exchange enforced customers */
            if(pricerData->eC_[tvrp.tour_[pos]] == day)
                continue;
            /* check for capacity condition */
            if(tvrp.capacity_ + modelData->demand[cust] - modelData->demand[tvrp.tour_[pos]] > modelData->max_caps[day])
                continue;
            /* check for forbidden arcs */
            if(pos == 0)
            {
                if(pricerData->isForbidden_[0][cust])
                    continue;
                if(tvrp.length_ > 1 && pricerData->isForbidden_[cust][tvrp.tour_[pos+1]])
                    continue;
            }
            else if(pos == tvrp.length_ - 1)
            {
                if(pricerData->isForbidden_[tvrp.tour_[pos-1]][cust] ||
                   pricerData->isForbidden_[cust][0])
                    continue;
            }else
            {
                if(pricerData->isForbidden_[tvrp.tour_[pos-1]][cust] ||
                   pricerData->isForbidden_[cust][tvrp.tour_[pos+1]])
                    continue;
            }
            /* costs of exchange */
            double costs = getExchangeCosts(modelData, tvrp.tour_, tvrp.length_, cust, pos);
            /* check if this exchange yields an improvement */
            if(SCIPisSumNegative(scip, (!isFarkas)*costs - dualvalues[cust] + dualvalues[tvrp.tour_[pos]]))
            {
                exchanged = tvrp.tour_[pos];
                tvrp.tour_[pos] = cust;
                if(tvrp.isFeasible(modelData))
                {
                    /* update values */
                    tvrp.capacity_ += modelData->demand[cust] - modelData->demand[exchanged];
                    isinTour[cust] = TRUE;
                    isinTour[exchanged] = FALSE;
                    *lhs += dualvalues[cust] - dualvalues[exchanged];
                    tvrp.obj_ += costs;
                    success = TRUE;
                    break;
                }
                tvrp.tour_[pos] = exchanged;
            }
        }
    }

    return success;
}

/** tries to delete customers of the tour to lower the reduced costs */
static
bool decreaseColumn(
        SCIP*                               scip,
        model_data*                         modelData,
        ObjPricerVRP*                       pricerData,
        bool                           isFarkas,
        tourVRP&                            tvrp,
        const vector<SCIP_Real>&            dualvalues,
        vector<bool>&                  isinTour,
        SCIP_Real                           *lhs
){
    int cust, prev, i, j;
    double old_costs, new_costs;
    bool success = FALSE;
    /* start at first customer in tour -> previous is depot */
    prev = 0;
    for(i = 0; i < tvrp.length_; i++)
    {
        cust = tvrp.tour_[i];
        /* skip customer if enforced for that day */
        if(pricerData->eC_[cust] == tvrp.getDay())
        {
            prev = cust;
            continue;
        }
        assert(pricerData->eC_[cust] == -1); // must not be enforced on any other day

        /* skip customer if arc between predecessor and successor is forbidden */
        if(i == tvrp.length_ - 1)
        {
            if(pricerData->isForbidden_[prev][0])
            {
                prev = cust;
                continue;
            }
        }else{
            if(pricerData->isForbidden_[prev][tvrp.tour_[i+1]])
            {
                prev = cust;
                continue;
            }
        }

        /* cust is last in tour */
        if(i == tvrp.length_ - 1)
        {
            old_costs = modelData->travel[prev][cust] + modelData->travel[cust][0];
            new_costs = modelData->travel[prev][0];
        }else
        {
            old_costs = modelData->travel[prev][cust] + modelData->travel[cust][tvrp.tour_[i+1]];
            new_costs = modelData->travel[prev][tvrp.tour_[i+1]];
        }
        /* check if deleting cust yields an improvement */
        if(SCIPisSumNegative(scip, (!isFarkas)*(new_costs - old_costs) + dualvalues[cust]))
        {
            *lhs -= dualvalues[cust];
            isinTour[cust] = FALSE;
            /* delete cust from tour */
            for(j = i+1; j < tvrp.length_; j++)
            {
                tvrp.tour_[j-1] = tvrp.tour_[j];
            }
            tvrp.tour_.pop_back();
            tvrp.length_--;
            tvrp.obj_ += new_costs - old_costs;
            tvrp.capacity_ -= modelData->demand[cust];
            success = TRUE;
            i--;
        }else
        {
            prev = cust;
        }
    }
    return success;

    // TODO: alternative - faster if multiple customers get deleted!
    vector<bool> deleted(tvrp.length_);
    prev = 0;
    int count = 0;
    for(i = 0; i < tvrp.length_; i++)
    {
        cust = tvrp.tour_[i];
        /* skip customer if enforced for that day */
        if(pricerData->eC_[cust] == tvrp.getDay())
        {
            prev = cust;
            continue;
        }
        assert(pricerData->eC_[cust] == -1); // must not be enforced on any other day

        /* skip customer if arc between predecessor and successor is forbidden */
        if(i == tvrp.length_ - 1)
        {
            if(pricerData->isForbidden_[prev][0])
            {
                prev = cust;
                continue;
            }
        }else{
            if(pricerData->isForbidden_[prev][tvrp.tour_[i+1]])
            {
                prev = cust;
                continue;
            }
        }

        /* cust is last in tour */
        if(i == tvrp.length_ - 1)
        {
            old_costs = modelData->travel[prev][cust] + modelData->travel[cust][0];
            new_costs = modelData->travel[prev][0];
        }else
        {
            old_costs = modelData->travel[prev][cust] + modelData->travel[cust][tvrp.tour_[i+1]];
            new_costs = modelData->travel[prev][tvrp.tour_[i+1]];
        }
        /* check if deleting cust yields an improvement */
        if(SCIPisSumNegative(scip, (!isFarkas)*(new_costs - old_costs) + dualvalues[cust]))
        {
            *lhs -= dualvalues[cust];
            isinTour[cust] = FALSE;
            tvrp.obj_ += new_costs - old_costs;
            success = TRUE;
            /* tag cust as deleted */
            deleted[i] = TRUE;
            count++;
        }else
        {
            prev = cust;
        }
    }
    j = 0;
    /* delete all tagged customers */
    for(i = 0; i < tvrp.length_; i++)
    {
        if(deleted[i])
            continue;
        tvrp.tour_[j] = tvrp.tour_[i];
        j++;
    }
    for(i = 0; i < count; i++)
    {
        tvrp.length_--;
        tvrp.tour_.pop_back();
    }
    return success;
}

static
bool investigateColumn(
        SCIP*                       scip,
        ObjPricerVRP*               pricerData,
        model_data*                 modelData,
        bool                        isFarkas,
        SCIP_VAR*                   var,
        const vector<SCIP_Real>&    dualValues,
        vector<tourVRP>&            finalTour
){
    int day;
    vector<pair<int, SCIP_Real>> toCheck;
    vector<bool> isinTour(modelData->nC);
    tourVRP tvrp;
    SCIP_Real lhs;
    bool improvement = FALSE;

    /* get vardata */
    auto* vardata = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));
    if(pricerData->fixedDay_[vardata->tourVrp_.getDay()])
        return false;

    tvrp.copy(vardata->tourVrp_);
    assert(tvrp.isFeasible(modelData));
    day = tvrp.getDay();
    lhs = dualValues[modelData->nC + day];
    /* set lhs and isinTour */
    for(auto u : tvrp.tour_)
    {
        if(!pricerData->timetable_[u][day])
        {
            cout << "NODE: "<<  SCIPnodeGetNumber(SCIPgetCurrentNode(scip)) << endl;
            cout << "u: " << u << " day: " << day << endl;
            cout << SCIPvarGetLPSol(var) << endl;
            SCIP_Node* node = SCIPgetCurrentNode(scip);
            while(SCIPnodeGetNumber(node) != 1)
            {
                cout << SCIPnodeGetNumber(node) << " - ";
                node = SCIPnodeGetParent(node);
            }
            cout << endl;
            SCIPprintVar(scip, var, nullptr);
            assert(false);
        }
        isinTour[u] = TRUE;
        lhs += dualValues[u];
    }

    /* Sort available customers by dualvalue (descending order) */
//    for(auto u : modelData->neighbors[0][day])
//    {
//        if(!pricerData->timetable_[u][day])
//            continue;
//        if(pricerData->eC_[u] == day)
//        {
//            if(!isinTour[u])
//            {
//                cout << day << " c " << u << " val " << SCIPvarGetLPSol(var)  << " red "<< SCIPgetVarRedcost(scip, var) << '\n';
//                SCIPprintVar(scip, var, nullptr);
//                assert(FALSE);
//            }
//        }
//        toCheck.emplace_back(u, dualValues[u]);
//    }
//    sort(toCheck.begin(), toCheck.end(), [](auto &left, auto &right){
//        return left.second > right.second;
//    });
    /** local search */
//    int cap = 0;
//    for(auto u : tvrp.tour_)
//        cap += modelData->demand[u];
//    if(cap != tvrp.capacity_)
//        SCIPprintVar(scip, var, nullptr);
//    assert(cap == tvrp.capacity_);
    /* try to add customers to the tour */
    if(extendColumn(scip, modelData, pricerData, isFarkas or !modelData->minTravel, tvrp,
                           dualValues, isinTour, &lhs))
        improvement = TRUE;
//    cap = 0;
//    for(auto u : tvrp.tour_)
//        cap += modelData->demand[u];
//    assert(cap == tvrp.capacity_);
    assert(tvrp.isFeasible(modelData));
    /* try to exchange a customers of the tour by unused ones to lower the reduced costs */
    if(shiftColumn(scip, modelData, pricerData, isFarkas or !modelData->minTravel, tvrp,
                    dualValues, isinTour, &lhs))
        improvement = TRUE;
//    cap = 0;
//    for(auto u : tvrp.tour_)
//        cap += modelData->demand[u];
//    assert(cap == tvrp.capacity_);
    /* try to delete customers of the tour to lower the reduced costs */
    if(!(isFarkas or !modelData->minTravel)) // NOTE: seems not to be worth it in Farkas pricing
    {
        if(decreaseColumn(scip, modelData, pricerData, isFarkas or !modelData->minTravel,
                          tvrp, dualValues, isinTour, &lhs))
            improvement = TRUE;
    }
//    cap = 0;
//    for(auto u : tvrp.tour_)
//        cap += modelData->demand[u];
//    assert(cap == tvrp.capacity_);
    if(tvrp.length_ == 0)
        return FALSE;
    assert(tvrp.isFeasible(modelData));

    /* the tour has been improved (in terms of reduced costs) */
    if(improvement)
    {
        /* check if reduced costs are negative */
//        TODO: Minimize num of Vehicle
        if(!modelData->minTravel)
            lhs -= 1;
//        if (!SCIPisSumNegative(scip, 1 - lhs))
        if (!SCIPisSumNegative(scip, (!(isFarkas or !modelData->minTravel)) * tvrp.obj_ - lhs))
            return FALSE;
        assert(tvrp.length_ > 0);
        /* check if this tour has already added in this iteration */
        for(auto& t : finalTour)
        {
            if(t.tour_ == tvrp.tour_)
            {
                return FALSE;
            }
        }
        finalTour.emplace_back(tvrp);
        return TRUE;
    }

    return FALSE;
}

/** calculates negative reduced cost tours using local search tools */
SCIP_RETCODE localSearchPricing(
        SCIP*                       scip,
        ObjPricerVRP*               pricerData,
        vrp::ProbDataVRP*           probData,
        const vector<SCIP_Real>&    dualValues,
        bool                        isFarkas
){
    int i;
    int num_found;
    char algoName[] = "localSearchPricing";
    SCIP_Real lpval;
    model_data* modelData = probData->getData();
    vector<tourVRP> new_tours;
//    vector<vector<pair<int, SCIP_Real>>> toCheck(modelData->nDays, vector<pair<int, SCIP_Real>>());

    vector<pair<int, SCIP_Real>> primary_vars; // variables with lp value > 0
    vector<pair<int, SCIP_Real>> backup_vars; //TODO: Maybe include other variables

    double num_v = 0.0;
    for(i = 0; i < probData->nVars_; i++)
    {
        if(SCIPvarGetUbLocal(probData->vars_[i]) < 0.5)
            continue;
        lpval = SCIPvarGetLPSol(probData->vars_[i]);

        if(SCIPisSumPositive(scip, lpval - 1)) // For Farkas Pricing
            continue;

        num_v += lpval;
        if(SCIPisSumPositive(scip, lpval))
        {
            /* only search for elementary tours */
            if(!dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, probData->vars_[i]))->isElementary())
                continue;
            assert(SCIPvarGetUbLocal(probData->vars_[i]) > 0.5);
            primary_vars.emplace_back(i, lpval);
        }else
        {
//            backup_vars.emplace_back(i, SCIPgetVarRedcost(scip, probData->vars_[i]));
        }
    }
//    cout << "NUMBER OF VEHICLE IN FRAC: " << num_v << '\n';
    /* sort vars */
    sort(primary_vars.begin(), primary_vars.end(), [](auto &left, auto &right){
        return left.second > right.second;
    });
//    sort(backup_vars.begin(), backup_vars.end(), [](auto &left, auto &right){
//        return left.second < right.second;
//    });

    /* init customers to check for each day */

    num_found = 0;
    for(i = 0; i < (int) primary_vars.size(); i++)
    {
        int var_index = primary_vars[i].first;
        assert(var_index >= 0 && var_index < probData->nVars_);
        if(investigateColumn(scip, pricerData, modelData, isFarkas, probData->vars_[var_index], dualValues, new_tours))
        {
            num_found++;
        }

        if(num_found == MAX_LOCALSEARCH_TOURS)
            break;
    }
    for(i = 0; i < num_found; i++)
    {
        add_tour_variable(scip, probData, isFarkas, FALSE, algoName, new_tours[i]);
    }

    return SCIP_OKAY;
}