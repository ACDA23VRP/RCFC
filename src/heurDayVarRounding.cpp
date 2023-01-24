
#include "heurDayVarRounding.h"
#include "var_tools.h"
#include "tools_vrp.h"

SCIP_DECL_HEURINITSOL(HeurDayVarRounding::scip_initsol)
{
    model_data* modelData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_))->getData();
    best_ = SCIPinfinity(scip);
    ncalls_ = 0;
    for(int i = 0; i < modelData->nVehicles; i++)
        sol_tours_.emplace_back(tourVRP(0, modelData->dayofVehicle[i]));

    return SCIP_OKAY;
}

/** execution method of primal heuristic */
SCIP_DECL_HEUREXEC(HeurDayVarRounding::scip_exec)
{
//    return SCIP_OKAY;
    /* Do not run during probing */
    if(SCIPinProbing(scip))
        return SCIP_OKAY;
    /* infeasible node */
    if(nodeinfeasible || SCIPisInfinity(scip, SCIPgetLPObjval(scip)))
    {
        *result = SCIP_DIDNOTRUN;
        return SCIP_OKAY;
    }
    /* integral lp-sol or solution of last heuristic call has not been processed */
    if(SCIPisEQ(scip, SCIPgetLPObjval(scip), SCIPgetUpperbound(scip)) || SCIPisNegative(scip, best_ - SCIPgetUpperbound(scip)))
    {
        *result = SCIP_DIDNOTRUN;
        return SCIP_OKAY;
    }
//    cout << "START HEURISTIC! ObjVal: " << SCIPgetLPObjval(scip)  << endl;
    ncalls_++;
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    model_data* modelData = probData->getData();

    bool isintegral = true;
    for(auto* var : probData->vars_)
    {
        if(SCIPisPositive(scip, SCIPvarGetLPSol(var)) && SCIPisSumNegative(scip, SCIPvarGetLPSol(var) - 1))
        {
            isintegral = false;
            break;
        }
    }
    if(isintegral) // TODO: change it?
    {
        *result = SCIP_DIDNOTRUN;
        return SCIP_OKAY;
    }

    vector<tourVRP> tours;
    vector<int> vehicleofNode(modelData->nC, -1);
    vector<pair<int, SCIP_Real>> dayVarVector;
    vector<vector<SCIP_Real>> valOnDay(modelData->nC, vector<SCIP_Real>(modelData->nDays));
    vector<int> numOfDays(modelData->nC);
    vector<int> unvisited;

    int nDayVars = 0;
    int cust, day, vehicle;

    for(vehicle = 0; vehicle < modelData->nVehicles; vehicle++)
    {
        if(sol_tours_[vehicle].length_ > 0)
            sol_tours_[vehicle].clearTour();
        tours.emplace_back(0, modelData->dayofVehicle[vehicle]);
    }

    SCIP_CALL(getVehiAssValues(scip, valOnDay, numOfDays));

    /* transform matrix into vector to sort */
    for(cust = 1; cust < modelData->nC; cust++)
    {
        for(day = 0; day < modelData->nDays; day++)
        {
            if(SCIPisPositive(scip, valOnDay[cust][day]))
            {
                nDayVars++;
                dayVarVector.emplace_back(pair<int, SCIP_Real>{cust + day * modelData->nC, valOnDay[cust][day]});
            }
        }
    }
    /* sort by value (descending) */
    sort(dayVarVector.begin(), dayVarVector.end(), [](auto &left, auto &right){
        return left.second > right.second;
    });

    for(auto pa : dayVarVector)
    {
        day = pa.first / modelData->nC;
        cust = pa.first - day * modelData->nC;
        if(vehicleofNode[cust] >= 0)
            continue;
        for(vehicle = modelData->firstVehicleofday[day]; vehicle < modelData->firstVehicleofday[day+1]; vehicle++)
        {
            if(sol_tours_[vehicle].addNode(scip, modelData, nullptr, cust))
            {
                vehicleofNode[cust] = vehicle;
                break;
            }
        }
        if(vehicleofNode[cust] == -1)
        {
            numOfDays[cust]--;
            if(numOfDays[cust] == 0)
            {
                unvisited.push_back(cust);
            }
        }
    }

    /* not all customer have been added */
    if(!unvisited.empty())
    {
        SCIP_CALL(addUnvisitedCustomers(scip, modelData, sol_tours_, vehicleofNode, unvisited));
    }
    if(unvisited.empty())
    {
        SCIP_CALL(twoNodeShift(scip, modelData, sol_tours_, vehicleofNode));
        double sum = 0.0;
        for(auto& tvrp : sol_tours_)
        {
            if(tvrp.length_ == 0)
                continue;
            sum += tvrp.obj_;
        }

        if(SCIPisSumNegative(scip, sum - SCIPgetPrimalbound(scip)) && SCIPisSumNegative(scip, sum - best_))
        {
            best_ = sum;
//            cout << "FOUND IMPROVEMENT!" << endl;
//            cout << "total costs: " << sum << endl;
        }
    }
//    assert(false);

    return SCIP_OKAY;
}