
#include "probdata_vrp.h"
#include "model_data.h"
#include "initial.h"
#include <iostream>
#include "tools_vrp.h"
#include "tourVRP.h"

using namespace std;

static
int getTimeWindowSum(
        vector<timeWindow> &tws
){
    int sum = 0;
    for(auto tw: tws)
    {
        if(tw.end > 0)
        {
            sum += (tw.end - tw.start);
        }
    }
    return sum;
}

SCIP_RETCODE addInitSolution(
        SCIP*               scip,
        vrp::ProbDataVRP*   probData,
        vector<tourVRP>&    sol_tvrps
){
    char algoName[] = "inputSolution";

    for(auto& t: sol_tvrps)
    {
        assert(t.length_ > 0);
        if(t.obj_ == 0 || t.capacity_ == 0)
        {
            t.setValues(probData->getData());
        }
//        int cap = 0;
//        for(auto u : t.tour_)
//            cap += probData->getData()->demand[u];
//        assert(t.capacity_ == cap);
        SCIP_CALL(add_tour_variable(scip, probData, FALSE, TRUE, algoName, t));
//        cout << t << endl;
        assert(t.isFeasible(probData->getData()));

    }
    cout << "ADDED inital solution!" << endl;

    return SCIP_OKAY;
}

SCIP_RETCODE addEmptyTours(
        SCIP*                   scip,
        vrp::ProbDataVRP*       probData
){
    char algoName[] = "empty";
    for(int day = 0; day < probData->getData()->nDays; day++)
    {
        tourVRP tvrp(0, day);
        SCIP_CALL(add_tour_variable(scip, probData, FALSE, TRUE, algoName, tvrp));
    }

    return SCIP_OKAY;
}

SCIP_RETCODE initialDispatching(
        SCIP*                   scip,
        vrp::ProbDataVRP*       probdata
)
{
    model_data* modelData = probdata->getData();
    char algoName[] = "initialDispatching";
    int cust, day, vehicle;
    int i;
    vector<int> unvisitedC;
    vector<tourVRP> tours;
    vector<int> vehicleofNode(modelData->nC, -1);

    vector<pair<int, int>> sortedCustomers(modelData->nC - 1, pair<int, int>());

    vector<int> sizeofVehicles(modelData->nVehicles);
    vector<int> nDaysofCustomer(modelData->nC);

    for(cust = 1; cust < modelData->nC; cust++)
    {
        sortedCustomers[cust-1] = {cust, getTimeWindowSum(modelData->timeWindows[cust])};
    }
    sort(sortedCustomers.begin(), sortedCustomers.end(), [](auto &left, auto &right){
        return left.second < right.second;
    });

    for(vehicle = 0; vehicle < modelData->nVehicles; vehicle++)
    {
        day = modelData->dayofVehicle[vehicle];
        tours.emplace_back(0, day);
        sizeofVehicles[vehicle] = (int) modelData->neighbors[0][day].size();
    }

    for(i = 0; i < modelData->nC - 1; i++)
    {
        cust = sortedCustomers[i].first;

        assert(cust != 0);
        /* Set up priority day-array for customer */
        vector<pair<int, int>> sortedVehicles;
        for(auto v : modelData->availableVehicles[cust])
        {
            sortedVehicles.emplace_back(v, sizeofVehicles[v]);
        }
        sort(sortedVehicles.begin(), sortedVehicles.end(), [](auto &left, auto &right) {
            return left.second < right.second;
        });

        for(auto& sortedVehicle : sortedVehicles)
        {
            vehicle = sortedVehicle.first;
            day = modelData->dayofVehicle[vehicle];

            if(tours[vehicle].capacity_ + modelData->demand[cust] > modelData->max_caps[day])
                continue;
            /* check if tour[day] stays feasible when we add customer node and add if possible */
            if(tours[vehicle].addNode(scip, modelData, nullptr, cust))
            {
                for(auto v : modelData->availableVehicles[cust])
                {
                    if(v != vehicle)
                        sizeofVehicles[v]--;
                }
                vehicleofNode[cust] = vehicle;
                break;
            }
        }

        if(vehicleofNode[cust] == -1)
        {
//            cout << "could not add customer " << cust << "!\n";
            unvisitedC.push_back(cust);
            for(auto v : modelData->availableVehicles[cust])
                sizeofVehicles[v]--;
//            assert(FALSE);
        }
    }
//    assert(false);

    double sumi = 0.0;
//    for(const auto& t : tours)
//    {
//        unused -= t.length_;
//    }
    if(!unvisitedC.empty())
    {
//        cout << "UNUSED: " << unvisitedC.size() << endl;
//        for(auto u : unvisitedC)
//            cout << u << " (" << vehicleofNode[u] << ")  ";
//        cout << endl;
        SCIP_CALL(addUnvisitedCustomers(scip, modelData, tours, vehicleofNode, unvisitedC));
//        for(auto u : unvisitedC)
//            cout << u << " (" << vehicleofNode[u] << ")  ";
//        cout << endl;
        sumi = 0.0;
        for(auto& t : tours)
        {
            sumi += t.obj_;
            if(t.length_ > 0)
                SCIP_CALL(add_tour_variable(scip, probdata, FALSE, TRUE, algoName, t));
        }
    }

    if(unvisitedC.empty())
    {
        sumi = 0.0;
        for(auto& t : tours)
        {
            assert(t.getDay() < modelData->nDays);
            sumi += t.obj_;
        }
//        cout << "current costs: " << sumi << endl;
        SCIP_CALL(twoNodeShift(scip, modelData, tours, vehicleofNode));

        sumi = 0.0;
        for(auto& t : tours)
        {
            sumi += t.obj_;
            if(t.length_ > 0)
                SCIP_CALL(add_tour_variable(scip, probdata, FALSE, TRUE, algoName, t));
        }
//        cout << "unvisited: " << unused << '\n';
//        cout << "after sum obj: " << sumi << '\n';
    }else
    {
        int old_unused = 0;
        while(old_unused != (int) unvisitedC.size())
        {
            old_unused = (int) unvisitedC.size();
            SCIP_CALL(twoNodeShift(scip, modelData, tours, vehicleofNode));
            SCIP_CALL(addUnvisitedCustomers(scip, modelData, tours, vehicleofNode, unvisitedC));
            sumi = 0.0;
            for(const auto& t : tours)
            {
                sumi += t.obj_;
            }
//            cout << "OBJ: " << sumi << " and unused: " << unused << '\n';
        }
        for(auto& t : tours)
        {
            if(t.length_ > 0)
                SCIP_CALL(add_tour_variable(scip, probdata, FALSE, TRUE, algoName, t));
        }
    }

    return SCIP_OKAY;
}