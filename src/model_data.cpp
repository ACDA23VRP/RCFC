
#include "stdio.h"
#include "iostream"
#include "fstream"
#include "math.h"
#include "scip/scip.h"
#include "probdata_vrp.h"
#include "vardata.h"

#include "model_data.h"
#include "../include/json.hpp"

using json = nlohmann::json;

static
SCIP_RETCODE getTimeWindows(
        model_data*     modelData,
        json::iterator& it,
        int             customer
){
    int day;
    for(auto& it2 : it.value())
    {
        day = it2.value("day", -1);
        modelData->timeWindows[customer][day].day = day;
        modelData->timeWindows[customer][day].start = it2.value("start", 0);
        modelData->timeWindows[customer][day].end = it2.value("end", 0);
        if(modelData->timeWindows[customer][day].end > 0)
        {
            modelData->availableDays[customer].push_back(day);
            for(int i = modelData->firstVehicleofday[day]; i < modelData->firstVehicleofday[day+1]; i++)
                modelData->availableVehicles[customer].push_back(i);
        }

    }
    return SCIP_OKAY;
}

static
SCIP_RETCODE getTravelTimes(
        std::vector<double>&    x,
        std::vector<double>&    y,
        model_data*             modelData
){
    double tmp;
    assert(x.size() == y.size());
    for(int i = 0; i < int(x.size()); i++)
    {
        for(int j = i; j < int(y.size()); j++)
        {
            tmp = sqrt(pow(x[i]-x[j], 2.0) + pow(y[i]-y[j], 2.0));
            tmp = round(tmp * ROUNDING_FACTOR) / ROUNDING_FACTOR;
            modelData->travel[i][j] = tmp;
            modelData->travel[j][i] = tmp;
        }
    }
    return SCIP_OKAY;
}

static
SCIP_RETCODE setNeighborhood(
        model_data*     modelData
){
    double earliest_arrival;
    for(int u = 0; u < modelData->nC; u++)
    {
        for(int day = 0; day < modelData->nDays; day++)
        {
            /* continue if u is not available on the day */
            if(modelData->timeWindows[u][day].end == 0) continue;
            for(int v = u+1; v < modelData->nC; v++)
            {
                /* continue if v is not available on the day */
                if(modelData->timeWindows[v][day].end == 0) continue;
                /* check arc from u to v */
                earliest_arrival = max(modelData->travel[0][u], (double) modelData->timeWindows[u][day].start) + modelData->service[u] + modelData->travel[u][v];
                if(earliest_arrival <= modelData->timeWindows[v][day].end)
                {
                    modelData->adjacency_k[day][u][v] = true;
                    modelData->neighbors[u][day].push_back(v);
                    if(u > 0)
                        modelData->predecessors[v][day].push_back(u);
                }
                /* check arc from v to u */
                earliest_arrival = max(modelData->travel[0][v], (double) modelData->timeWindows[v][day].start) + modelData->service[v] + modelData->travel[v][u];
                if(earliest_arrival <= modelData->timeWindows[u][day].end)
                {
                    modelData->adjacency_k[day][v][u] = true;
                    if(u > 0)
                        modelData->neighbors[v][day].push_back(u);
                    modelData->predecessors[u][day].push_back(v);
                }

            }
        }
    }
    return SCIP_OKAY;
}

static
SCIP_RETCODE setngNeighbors(
        model_data*     modelData,
        int             ng_parameter
){
    for(int i = 1; i < modelData->nC; i++)
    {
        int k = 0;
        vector<pair<int, double>> sortedTravel(modelData->nC - 2, pair<int, double>());
        for(int j = 1; j < modelData->nC; j++)
        {
            if(i != j)
            {
                sortedTravel[j - k - 1]  = {j, modelData->travel[i][j]};
            }else{
                k = 1;
            }
        }
        sort(sortedTravel.begin(), sortedTravel.end(), [](auto &left, auto &right){
            return left.second < right.second;
        });
        for(int j = 0; j < min(ng_parameter, modelData->nC); j++)
        {
            modelData->ng_set[i][sortedTravel[j].first] = true;
        }
    }
    return SCIP_OKAY;
}

SCIP_RETCODE getModelDataFromJson(
        model_data*     modelData,
        char*           input_file,
        int             ng_parameter
){
    /* read file */
    json j;
    std::ifstream is(input_file);

    is >> j;

    /** global instance data */
    int nC = j.value("num_c", -1) + 1;
    int nDays = j.value("num_v", -1);

    /* Save in model data */
    modelData->nC = nC;
    modelData->nDays = nDays;
    modelData->minTravel = false;

    std::cout << "nCustomers: " << nC << " nDays: " << nDays << '\n';

    /** vehicle data */
    modelData->max_caps.resize(nDays);
    modelData->num_v.resize(nDays);
    modelData->firstVehicleofday.resize(nDays+1);

    /** customer data */
    /* service */
    modelData->service.resize(nC);
    /* demand */
    modelData->demand.resize(nC);
    /* coordinates */
    std::vector<double> x(nC);
    std::vector<double> y(nC);
    /* travel times */
    modelData->travel.resize(nC, std::vector<double>(nC));
    /* time windows */
    modelData->timeWindows.resize(nC, std::vector<timeWindow>(nDays));
    /* available days */
    modelData->availableDays.resize(nC, std::vector<int>());
    /* neighbors and predecessors */
    modelData->neighbors.resize(nC, std::vector<std::vector<int>>(nDays));
    modelData->predecessors.resize(nC, std::vector<std::vector<int>>(nDays));
    /* adjacency matrices */
    modelData->adjacency_k.resize(nDays, vector<vector<bool>>(nC, vector<bool>(nC, false)));
    /* ng-neighbors */
    modelData->ng_set.resize(nC, bitset<neighborhood_size>());

    /** read vehicle data from file */
    json vehicle_j = j.value("vehicles", json());
    json::iterator it = vehicle_j.begin();
    int k = 0;
    int num_veh = 0;
    while (it != vehicle_j.end())
    {
        modelData->firstVehicleofday[k] = num_veh;
        modelData->max_caps[k] = it.value()[0];
        modelData->num_v[k] = it.value()[1];
        num_veh += modelData->num_v[k];
        for(int l = 0; l < modelData->num_v[k]; l++)
            modelData->dayofVehicle.push_back(k);
        k++;
        it++;
    }
    modelData->firstVehicleofday[k] = num_veh;
    assert(k == nDays);
    modelData->nVehicles = num_veh;
    modelData->availableVehicles.resize(nC);

    /** read customer (and depot) data from file */
    json cust_j = j.value("customers", json());
    it = cust_j.begin();
    k = 0;
    while (it != cust_j.end()) {
        json::iterator it2 = it->begin();
        assert(it2.key() == "coords");
        x[k] = it2.value()[0];
        y[k] = it2.value()[1];

        it2++;
        assert(it2.key() == "demand");
        modelData->demand[k] = it2.value();
        it2++;
        assert(it2.key() == "service");
        modelData->service[k] = it2.value();

        it2++;
        assert(it2.key() == "windows");
        assert(!it2->empty());
        SCIP_CALL( getTimeWindows(modelData, it2, k) );

        it++;
        k++;
    }
    assert(k == nC);

    /* calculate travel times */
    SCIP_CALL( getTravelTimes(x, y, modelData) );

    /* set neighborhoods and adjacency matrices based on time windows and travel/service times */
    SCIP_CALL( setNeighborhood(modelData) );

    /* set ng-path-neighborhood (k-nearest neighbors) */
    SCIP_CALL( setngNeighbors(modelData, ng_parameter) );

    return SCIP_OKAY;
}

SCIP_RETCODE transformModelData
(
        SCIP*               scip,
        model_data*         modelData_old,
        model_data*         modelData_new,
        vector<int>&        hash_day

){
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip));
    int num_days = (int) SCIPgetPrimalbound(scip);
    std::cout << "\nNEW NUM OF DAYS: " << num_days << '\n';

    /* overwrite data that stays the same */
    modelData_new->travel = modelData_old->travel;
    modelData_new->service = modelData_old->service;
    modelData_new->demand = modelData_old->demand;
    modelData_new->nC = modelData_old->nC;
    modelData_new->max_caps = modelData_old->max_caps;
    modelData_new->num_v = modelData_old->num_v;
    /** get new data */
    modelData_new->nDays = num_days;
    modelData_new->minTravel = true;
    /* time windows */
    modelData_new->timeWindows.resize(modelData_new->nC, std::vector<timeWindow>(num_days));
    /* available days */
    modelData_new->availableDays.resize(modelData_new->nC, std::vector<int>());
    /* neighbors */
    modelData_new->neighbors.resize(modelData_new->nC, std::vector<std::vector<int>>(num_days));

    /* built hash map for days */

    SCIP_Sol* sol = SCIPgetBestSol(scip);
    std::vector<SCIP_Bool> dayisActive(num_days);
    int count = 0;
    for(auto var : probData->vars_)
    {
        if(SCIPgetSolVal(scip, sol, var) > 0.5)
        {
            hash_day[dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var))->getDay()] = count;
            count++;
        }
    }
    for(int cust = 0; cust < modelData_new->nC; cust++)
    {
        for(int day = 0; day < modelData_old->nDays; day++)
        {
            if(hash_day[day] == -1)
                continue;
            if(modelData_old->timeWindows[cust][day].end > 0)
            {
                modelData_new->timeWindows[cust][hash_day[day]] = modelData_old->timeWindows[cust][day];
                modelData_new->availableDays[cust].push_back(hash_day[day]);
                modelData_new->neighbors[cust][hash_day[day]] = modelData_old->neighbors[cust][day];
            }
        }
    }

    return SCIP_OKAY;
}

int getNumOfArcs(
    model_data* modelData
){
    int count = (modelData->nC - 1) * 2; // depot arcs
    for(int cust = 1; cust < modelData->nC; cust++)
    {
        vector<int> hasArc(modelData->nC, 0);
        for(int day = 0; day < modelData->nDays; day++)
        {
            for(auto nb : modelData->neighbors[cust][day])
                hasArc[nb] = 1;
        }
        for(auto nb : hasArc)
            count += nb;
    }
    return count;
}

int getNumOfTW(
    model_data* modelData
){
    int count = 0;
    for(int cust = 1; cust < modelData->nC; cust++)
    {
//        cout << cust << " avail " << (int) modelData->availableDays[cust].size() << ": ";
//        for(auto day : modelData->availableDays[cust])
//            cout << day << " ";
//        cout << '\n';
        count += (int) modelData->availableDays[cust].size();
    }
    return count;
}