
#include <cassert>
#include <cstring>
#include <ctime>
#include "iostream"
#include <pthread.h>
#include "vector"
#include "queue"
#include "scip/scip.h"

#include "pricer_vrp.h"
#include "probdata_vrp.h"
#include "labeling_algorithm_vrp.h"
#include "model_data.h"
#include "tools_vrp.h"
#include "tourVRP.h"
#include "label2.h"


static
SCIP_RETCODE getToursFromLabelPairs(
        SCIP*               scip,
        model_data*         modelData,
        vector<pair<pair<LabelNode*, LabelNode*>, double>> &sol_pairs,
        vector<tourVRP>     &finalTours,
        int                 day
){
    double last = 0.0;
    LabelNode* node;
    int count = 0;

    for(auto sol : sol_pairs)
    {
        if(count > 20)
            break;
        if(SCIPisEQ(scip, sol.second, last))
            continue;
        int cap = 0;
        int fw_length = 0;
        node = sol.first.first;
        while(node->label2_->current_ != 0)
        {
            cap += modelData->demand[node->label2_->current_];
            fw_length++;
            node = node->parent_;
        }
        finalTours.emplace_back(tourVRP(fw_length, day));

        /* tour from fw_label */
        node = sol.first.first;
        while (node->label2_->current_ != 0)
        {
            finalTours[count].tour_[--fw_length] = node->label2_->current_;
            node = node->parent_;
        }
        /* tour from bw_label */
        node = sol.first.second;
        while(node->label2_->current_ != 0)
        {
            finalTours[count].tour_.push_back(node->label2_->current_);
            finalTours[count].length_++;
            cap += modelData->demand[node->label2_->current_];
            node = node->parent_;
        }
        finalTours[count].obj_ = sol.first.first->label2_->obj_ + sol.first.second->label2_->obj_ +
                                 modelData->travel[sol.first.first->label2_->current_][sol.first.second->label2_->current_];
        finalTours[count].capacity_ = cap;
        assert(finalTours[count].checkObj(modelData));
        count++;
        last = sol.second;
    }

    return SCIP_OKAY;
}


SCIP_RETCODE generateLabelsBiDir(
        SCIP*               scip,
        model_data*         modelData,
        ObjPricerVRP*       pricerData,
        vector<tourVRP>&    bestTours,
        bool                isFarkas,
        bool                getDayVarRed,
        bool                isHeuristic,
        bool                noDominance,
        int                 day
){
    vector<LabelList*> labelLists_fw(modelData->nC);        /* vector of lists of non-propagated fw-labels */
    vector<LabelList*> propLabelLists_fw(modelData->nC);    /* vector of lists of propagated fw-labels */
    vector<LabelList*> labelLists_bw(modelData->nC);        /* vector of lists of non-propagated bw-labels */
    vector<LabelList*> propLabelLists_bw(modelData->nC);    /* vector of lists of propagated bw-labels */
    int i;
    double start_redcosts;
    queue<int> indexQ;
    Label2* new_label2;
    LabelNode* new_node;

    /* check which step of optimization */
    start_redcosts = -pricerData->dualValues_[modelData->nC + day] + pricerData->nEC_[day] * ENFORCED_PRICE_COLLECTING;
    start_redcosts -= pricerData->dual_nVehicle_;
    if(!modelData->minTravel && !isFarkas)
    {
        start_redcosts += 1;
    }
    for(i = 0; i < modelData->nC; i++)
    {
        labelLists_fw[i] = new LabelList();
        propLabelLists_fw[i] = new LabelList();
        labelLists_bw[i] = new LabelList();
        propLabelLists_bw[i] = new LabelList();
    }
    vector<bool> isInQ(modelData->nC, FALSE);
    bitset<neighborhood_size> emtpy_bitset;
    indexQ.push(0);
    /* initial forwards label */
    new_label2 = new Label2(start_redcosts, 0, 0, pricerData->nnonzSRC_, 0.0, 0.0, emtpy_bitset, emtpy_bitset);
    new_node = new LabelNode(new_label2, start_redcosts);
    labelLists_fw[0]->insert_node(new_node);
    /* initial backwards label */
    new_label2 = new Label2(0.0, 0, 0, pricerData->nnonzSRC_, 0.0, 0.0, emtpy_bitset, emtpy_bitset);
    new_node = new LabelNode(new_label2, 0);
    labelLists_bw[0]->insert_node(new_node);

    if(isHeuristic)
        cout << "heuristic" << endl;
    /* bidirectional labeling algorithm */
//    if(noDominance)
//        cout << "Without Dominance check" << endl;
    while(!indexQ.empty())
    {
        assert(labelLists_fw[indexQ.front()] != nullptr || labelLists_bw[indexQ.front()] != nullptr);
        /* propagate all forwards labels of customer */
        if(labelLists_fw[indexQ.front()]->length_ > 0)
            propagateCustomer(scip, modelData, pricerData, day, propLabelLists_fw, indexQ, labelLists_fw,
                              isInQ, TRUE, getDayVarRed, noDominance);
        /* propagate all backwards labels of customer */
        if(labelLists_bw[indexQ.front()]->length_ > 0)
            propagateCustomer(scip, modelData, pricerData, day, propLabelLists_bw, indexQ, labelLists_bw,
                              isInQ, FALSE, getDayVarRed, noDominance);
        isInQ[indexQ.front()] = FALSE;
        indexQ.pop();
    }
//    for(i = 0; i < modelData->nC; i++)
//    {
//        assert(labelLists_fw[i]->length_ == 0);
//        assert(labelLists_bw[i]->length_ == 0);
//    }

    /* concatenate fw and bw labels */
    vector<pair<pair<LabelNode*, LabelNode*>, double>> sol_pairs;
    SCIP_CALL(concatenateLabels(scip, modelData, pricerData, getDayVarRed, day,
                                propLabelLists_fw, propLabelLists_bw, sol_pairs));

    getToursFromLabelPairs(scip, modelData, sol_pairs, bestTours, day);

    for(i = 0; i < modelData->nC; i++)
    {
        delete labelLists_fw[i];
        delete labelLists_bw[i];
        delete propLabelLists_fw[i];
        delete propLabelLists_bw[i];
    }

    return SCIP_OKAY;
}

SCIP_RETCODE propagateCustomer(
        SCIP*               scip,
        model_data          *modelData,
        ObjPricerVRP        *pricerData,
        int                 day,
        vector<LabelList*>  &propLabelLists,
        queue<int>          &indexQ,
        vector<LabelList*>  &labelLists,
        vector<bool>        &isInQ,
        bool                isFW,
        bool                getDayVarRed,
        bool                noDominance
){
    LabelNode* curr_node;
    LabelNode* last_node;
    LabelNode* new_node;
    int current_index = indexQ.front();
    Label2* new_label2;
    Label2* curr_label2;
    int dom_active;
    int dom_old;
    vector<int> &neighbors = isFW ? pricerData->neighbors_[current_index][day] : pricerData->predecessors_[current_index][day];
    vector<vector<bool>>& timetable = pricerData->atRoot_ ? pricerData->global_timetable_ : pricerData->timetable_;
    while(labelLists[current_index]->length_ > 0)
    {
        curr_node = labelLists[current_index]->extract_first();
        curr_label2 = curr_node->label2_;

        if(getDayVarRed || curr_label2->time_ <= (double) modelData->timeWindows[0][day].end / 2) // TODO: can we make it sharp? - Prob. no!
        {
            last_node = nullptr;
            for(auto neighbor : neighbors)
            {
                assert(timetable[neighbor][day]);

                /* skip neighbor if ng-condition gets violated */
                if(curr_label2->ng_memory_[neighbor])
                    continue;
                if(curr_label2->visitedEC_[neighbor])
                {
                    assert(pricerData->eC_[neighbor] == day);
                    continue;
                }

                new_label2 = label_propagate2(modelData, pricerData, curr_label2, isFW, neighbor, day);

                if(new_label2 == nullptr)
                    continue;

                if(noDominance)
                {
                    dom_active = 0;
                    dom_old = 0;
                }else
                {
                    dom_active = dominance_check2(labelLists, propLabelLists, new_label2, FALSE, pricerData, scip);
                    dom_old = dominance_check2(labelLists, propLabelLists, new_label2, TRUE, pricerData, scip);
                }

                if(dom_old > 0)
                {
                    assert(dom_active >= 0);
                }

                /* label is dominated, no adding to pool */
                if(dom_active == -1 || dom_old == -1)
                {
                    assert(dom_old <= 0);
                    assert(dom_active <= 0);
                    delete new_label2;
                }else
                {
                    assert(dom_active >= 0);
                    new_node = new LabelNode(new_label2, new_label2->red_costs_);
                    labelLists[neighbor]->insert_node(new_node);

                    /* add current customer to index Q */
                    if(!isInQ[new_label2->current_])
                    {
                        isInQ[new_label2->current_] = TRUE;
                        indexQ.push(new_label2->current_);
                    }
                    /* add to family tree */
                    assert(new_node != nullptr);
                    if(last_node == nullptr)
                    {
                        curr_node->child_ = new_node;
                    }else
                    {
                        last_node->nextSib_ = new_node;
                        new_node->prevSib_ = last_node;
                    }
                    new_node->parent_ = curr_node;
                    last_node = new_node;
                }
            }
        }
        curr_node->is_propagated_ = TRUE;
        curr_node->next_ = nullptr;
        curr_node->prev_ = nullptr;
        propLabelLists[current_index]->insert_node(curr_node);
    }
    return SCIP_OKAY;
}

/** Checks the rescource compatiblity of a fw-label and a bw-label */
static
bool isFeasible(
        model_data*         modelData,
        int                 day,
        Label2*             fw_label,
        Label2*             bw_label
){
    /* check for capacity rescource */
    if(fw_label->cap_ + bw_label->cap_ > modelData->max_caps[day])
        return FALSE;
    /* check for time rescource */
    if(fw_label->time_ + bw_label->time_ + modelData->travel[fw_label->current_][bw_label->current_]
        > modelData->timeWindows[0][day].end)
        return FALSE;
    /* check for ng-path rescources */
    if((fw_label->ng_memory_ & bw_label->ng_memory_) != 0)
        return FALSE;
    /* check for visited EC */
    if((fw_label->visitedEC_ & bw_label->visitedEC_) != 0)
        return FALSE;

    /* TODO: check for non-robust-cuts rescources (when implemented) */

    return TRUE;
}

SCIP_RETCODE concatenateLabels(
        SCIP*               scip,
        model_data*         modelData,
        ObjPricerVRP*       pricerData,
        bool                getDayVarRed,
        int                 day,
        vector<LabelList*>  &fw_list,
        vector<LabelList*>  &bw_list,
        vector<pair<pair<LabelNode*, LabelNode*>, double>> &sol_pairs
){
    //TODO: HOT CODE
    LabelNode* fw_node;
    LabelNode* bw_node;
    vector<vector<bool>>& timetable = pricerData->atRoot_ ? pricerData->global_timetable_ : pricerData->timetable_;
    vector<vector<bool>>& isForbidden = pricerData->atRoot_ ? pricerData->global_isForbidden_ : pricerData->isForbidden_;
    double best_bw = SCIP_DEFAULT_INFINITY;
    double best_fw = SCIP_DEFAULT_INFINITY;
    vector<double> best_fw_cust(modelData->nC, SCIP_DEFAULT_INFINITY);
    vector<double> best_bw_cust(modelData->nC, SCIP_DEFAULT_INFINITY);
    double minred = 0.0;
    modelData->neighbors[0][day].push_back(0);
    for(auto i : modelData->neighbors[0][day])
    {
        if(fw_list[i]->length_ > 0)
        {
            if(fw_list[i]->head_->value_ < best_fw_cust[i])
                best_fw_cust[i] = fw_list[i]->head_->value_;
        }
        if(bw_list[i]->length_ > 0)
        {
            if(bw_list[i]->head_->value_ < best_bw_cust[i])
                best_bw_cust[i] = bw_list[i]->head_->value_;
        }
        if(best_fw_cust[i] < best_fw)
            best_fw = best_fw_cust[i];
        if(best_bw_cust[i] < best_bw)
            best_bw = best_bw_cust[i];
    }

    int count = 0;
    for(auto i : modelData->neighbors[0][day])
    {
        if(!timetable[i][day])
            continue;
        double minred_i = SCIP_DEFAULT_INFINITY;

        fw_node = fw_list[i]->head_;
        while (fw_node != nullptr)
        {
            for(auto j : modelData->neighbors[0][day])
            {
                if(!timetable[j][day])
                    continue;
                if(i == j)
                    continue;
                if(isForbidden[i][j])
                    continue;
                double minred_ij = SCIP_DEFAULT_INFINITY;
                bw_node = bw_list[j]->head_;
                while(bw_node != nullptr)
                {
                    /* reduced costs */
                    double rc = bw_node->value_ + fw_node->value_ + pricerData->arcPrices_[i][j];

                    if(!getDayVarRed)
                    {
                        /* if not even this value is negative, we can stop for this i, j combination */
                        if(!SCIPisSumNegative(scip, rc))
                            break;
                    }else {
                        /* if not even this value a better value of arcflow/dayvar-redcosts -> stop */
                        if(SCIPisGE(scip, rc, minred_ij) && SCIPisGE(scip, rc, minred_i))
                            break;
                    }
                    /* include subset row cut influence */
                    for(int c = 0; c < pricerData->nnonzSRC_; c++)
                    {
                        if(SCIPisGE(scip, fw_node->label2_->SRCstate_[c] + bw_node->label2_->SRCstate_[c], 1))
                        {
                            rc -= pricerData->SRC_dualv[c];
                        }
                    }
                    if(getDayVarRed || SCIPisSumNegative(scip, rc))
                    {
                        if(isFeasible(modelData, day, fw_node->label2_, bw_node->label2_))
                        {
                            /* arc fixing */
                            if(rc < minred_ij)
                                minred_ij = rc;
                            /* assignment fixing */
                            if(rc < minred_i)
                                minred_i = rc;

                            if(SCIPisSumNegative(scip, rc))
                            {
                                if(rc < minred)
                                    minred = rc;

                                sol_pairs.emplace_back(pair(fw_node, bw_node), rc);
                                count++;
                                if(count > 10 && !getDayVarRed)
                                    break;
                            }else // getDayVarRed = true - first feasible has the lowest red costs (if there are no cuts)
                            {
                                if(pricerData->nnonzSRC_ == 0)
                                    break;
                            }
                        }
                    }
                    bw_node = bw_node->next_;
                }
                if(minred_ij < pricerData->arcRedCosts_[i][j])
                    pricerData->arcRedCosts_[i][j] = minred_ij;
                if(count > 10 && !getDayVarRed)
                    break;
            }
            fw_node = fw_node->next_;
            if(count > 10 && !getDayVarRed)
                break;
        }
        if(getDayVarRed)
        {
            pricerData->dayVarRedCosts_[i][day] = minred_i;
        }
        if(count > 10 && !getDayVarRed)
            break;
    }
    modelData->neighbors[0][day].pop_back();
    if(getDayVarRed)
    {
        pricerData->dayVarRedCosts_[0][day] = minred;
    }
    sort(sol_pairs.begin(), sol_pairs.end(), [](auto &left, auto &right){
        return left.second < right.second;
    });

    return SCIP_OKAY;
}

static
void *labeling_thread(void *arguments){
    auto* args = static_cast<arg_struct *>(arguments);

    assert(args->modelData != nullptr);

    generateLabelsBiDir(args->scip, args->modelData, args->pricerData, *args->bestTours,
                     args->isFarkas, args->getDayVarRed, args->isHeuristic, args->noDomiance, args->day);
//    generateLabels(args->scip, args->modelData, args->pricerData, args->bestLabels,
//                   args->isFarkas, args->isHeuristic, args->day);
//    printf("Thread for day %d: Ended.\n", args->day);
    return nullptr;
}

SCIP_RETCODE labelingAlgorithmnParallel(
        SCIP*                       scip,
        vrp::ProbDataVRP*           probData,
        ObjPricerVRP*               pricerData,
        bool                        isFarkas,
        bool                        isHeuristic,
        bool                        getDayVarRed
)
{
    assert(probData != nullptr);
    assert(pricerData != nullptr);
    model_data* modelData = probData->getData();
    pthread_t threads[modelData->nDays];
    arg_struct* thread_args;
    int result_code;
    int i;
    char algoName[] = "pricingLabel";

    vector<vector<tourVRP>> tours(probData->getData()->nDays, vector<tourVRP>());
    SCIP_CALL( SCIPallocMemoryArray(scip, &thread_args, modelData->nDays) );
    //create all threads one by one
    for (i = 0; i < modelData->nDays; i++) {
        if(pricerData->fixedDay_[i])
            continue;
        thread_args[i].scip = scip;
        thread_args[i].modelData = modelData;
        thread_args[i].pricerData = pricerData;
        thread_args[i].isFarkas = isFarkas;
        thread_args[i].isHeuristic = isHeuristic;
        thread_args[i].getDayVarRed = getDayVarRed;
        if(SCIPinProbing(scip) && i == pricerData->prop_tourfixing_->tvrp_.getDay())
        {
            thread_args[i].noDomiance = true;
        }else
        {
            thread_args[i].noDomiance = false;
        }
        thread_args[i].day = i;
        thread_args[i].bestTours = &tours[i];
//        thread_args[i].bestTours = vector<tourVRP>();
        result_code = pthread_create(&threads[i], nullptr, labeling_thread, &thread_args[i]);
        assert(!result_code);
    }

    //wait for each thread to complete
    for (i = 0; i < modelData->nDays; i++) {
        if(pricerData->fixedDay_[i])
            continue;
        result_code = pthread_join(threads[i], nullptr);
        assert(!result_code);
    }

    /* add the best labels as tours of each day to the master problem */
    for(i = 0; i < modelData->nDays; i++)
    {
        if(pricerData->fixedDay_[i])
            continue;

        int count = 0;
        for(auto& tvrp : *thread_args[i].bestTours)
        {
            if(SCIPinProbing(scip))
            {
                if(i == pricerData->prop_tourfixing_->tvrp_.getDay() && SCIPisEQ(scip, pricerData->prop_tourfixing_->tvrp_.obj_, tvrp.obj_)
                   && pricerData->prop_tourfixing_->tvrp_.length_ == tvrp.length_)
                {
                    bool isfixtour = true;
                    for(int j = 0; j < tvrp.length_; j++)
                    {
                        if(tvrp.tour_[j] != pricerData->prop_tourfixing_->tvrp_.tour_[j])
                        {
                            isfixtour = false;
                            break;
                        }
                    }
                    if(isfixtour)
                    {
//                            cout << "FOUND FIX TOUR (" << SCIPnodeGetNumber(SCIPgetFocusNode(scip)) << ")" << endl;
                        continue;
                    }
                }
            }
            /* check for ng-path violations */
            if(USE_DSSR)
            {
                if(violatesNGProperty(probData->getData()->ng_set, pricerData->ng_DSSR_, tvrp.tour_))
                    continue;
            }
            assert(tvrp.length_ > 0);
            SCIP_CALL(add_tour_variable(scip, probData, isFarkas, FALSE, algoName, tvrp));

            count++;
            if(count > probData->getData()->nC / 4) // TODO: what is a good criteria?
            {
                break;
            }
        }

    }
    SCIPfreeMemoryArray(scip, &thread_args);

    return SCIP_OKAY;
}


