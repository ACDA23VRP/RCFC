
#include <iostream>
#include "tsptw.h"
#include "queue"

static
tsp_label* propagateLabel(
    model_data*     modelData,
    tsp_label*      old_label,
    vector<int>&    mapping,
    int             head,
    int             day
){
    tsp_label* new_label;
    double arrivaltime;
    int u = mapping[old_label->last_];
    int v = mapping[head];
    assert(!old_label->visited_[head]);

    arrivaltime = old_label->deptime_ + modelData->travel[u][v];
    arrivaltime = max(arrivaltime, (double) modelData->timeWindows[v][day].start);
    if(arrivaltime > modelData->timeWindows[v][day].end)
    {
        return nullptr;
    }

    new_label = new tsp_label(head, arrivaltime + modelData->service[v], old_label->visited_);
    new_label->visited_[head] = true;

    return new_label;
}

/** checks if a solution to the travelings salesman problem with time windows exists */
bool feasibleTSP(
        model_data*         modelData,
        std::vector<int>&   indexSet,
        int                 day
){
    tsp_label* label;
    tsp_label* next_label;
    int t, new_t;
    bool success = false;
    int nlabels;
    int n = (int) indexSet.size();
    int T = modelData->timeWindows[0][day].end;
    vector<vector<int>> active_custs(T+1, vector<int>());                 /* a list of customers that have a label for each timestep */
    vector<vector<bool>> has_label(T+1, vector<bool>(n, false));    /* indicates if customer has a label for each timestep */
    vector<int> custsToSet(modelData->nC, -1);
//    vector<queue<int>> active_times(n, queue<int>());
    /* mapping from all customers to node set for TSPTW */
    for(t = 0; t < n; t++)
    {
        custsToSet[indexSet[t]] = t;
    }
    indexSet.push_back(0);

    vector<vector<tsp_list>> lists(T+1, vector<tsp_list>(n, tsp_list()));
    /* init label */
    label = new tsp_label(n, 0, vector<bool>(n, false));
    active_custs[0].push_back(0);
    lists[0][0].insert_node(label);
    t = 0;
    nlabels = 1;
    while (nlabels > 0)
    {
        assert(t <= T);
        /* Check if there are active labels at time t */
        if(!active_custs[t].empty())
        {
            /* Propagate for each customer v (in first iteration v = depot) */
            for(auto v : active_custs[t])
            {
                assert(lists[t][v].head_ != nullptr);
                while(lists[t][v].head_ != nullptr)
                {
                    label = lists[t][v].extract_first();
                    nlabels--;
                    /* Propagate label to all unvisited customers */
                    for(int i = 0; i < n; i++)
                    {
                        if(label->visited_[i])
                            continue;
                        next_label = propagateLabel(modelData, label, indexSet, i, day);
                        if(next_label == nullptr)
                            continue;

                        /* Valid label - check if each unvisited customer is still reachable.
                         * If all customers are visited it return false and saves final result in success */
                        if(!next_label->allReachable(modelData, indexSet, &success, day))
                        {
                            delete next_label;
                            if(success)
                            {
//                                cout << "FOUND feasible TSPTW solution!" << endl;
                                return true;
                            }
                            continue;
                        }
                        /* Check for dominance relation of existing labels */
                        // TODO: Dominance
//                        cout << "add label: ";
//                        for(int u = 0; u < n; u++)
//                        {
//                            if(next_label->visited_[u])
//                                cout << indexSet[u] << " ";
//                        }
//                        cout << " last: " << indexSet[next_label->last_] << " time: " << next_label->deptime_ << endl;

                        /* add to corresponding list */
                        nlabels++;
//                        cout << "nlabels: " << nlabels << endl;
                        new_t = (int) next_label->deptime_;
                        lists[new_t][i].insert_node(next_label);
                        if(!has_label[new_t][i])
                        {
                            has_label[new_t][i] = true;
                            active_custs[new_t].push_back(i);
                        }
                    }
                }
            }
        }
        t++;
    }
    return false;
}

/** checks if all unvisited customers are still reachable for that label
 * if all customers are visited, check if we can return to the depot in time -> success */
bool tsp_label::allReachable(
    model_data*     modelData,
    vector<int>&    mapping,
    bool*           success,
    int             day
){
    int u, v;
    bool allvisited = true;
    assert(!(*success));
    u = mapping[last_];
    for(int i = 0; i < (int) visited_.size(); i++)
    {
        if(!visited_[i])
        {
            allvisited = false;
            v = mapping[i];
            if(deptime_ + modelData->travel[u][v] > modelData->timeWindows[v][day].end)
                return false;
        }
    }
    /* Check if we can return to depot in time */
    if(allvisited)
    {
        if(deptime_ + modelData->travel[u][0] < modelData->timeWindows[0][day].end)
        {
            *success = true;
            return false;
        }
    }
    return true;
}

