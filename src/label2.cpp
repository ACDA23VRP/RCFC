
#include "label2.h"
#include "scip/scip.h"
#include "model_data.h"


Label2* label_propagate2(
        model_data*     modelData,
        ObjPricerVRP*   pricerData,
        Label2*         old_label,
        SCIP_Bool       isFW,
        int             end,
        int             day
){
    int capacity, start;
    double arrival, new_time;
    SCIP_Real red_costs;
    double obj;
    double state;
    Label2* new_label;
    bitset<neighborhood_size> ng_memory;
    bool isEnforced = pricerData->eC_[end] == day;

    assert(old_label != nullptr);
    assert(0 <= end && end <= modelData->nC);
    assert(old_label->current_ != end);
    start = old_label->current_;

    /* customer would exceed max capacity */
    capacity = old_label->cap_ + modelData->demand[end];
    if(modelData->max_caps[day] < capacity)
        return nullptr;
    /* update time rescource */
    arrival = old_label->time_ + modelData->travel[start][end];
    if(isFW) /* forward label */
    {
        if(modelData->timeWindows[end][day].end < arrival)
            return nullptr;
        /* start of service */
        new_time = std::max(double(modelData->timeWindows[end][day].start), arrival);
        /* if start of service exceeds the Half way point: return null */
//        if(new_time > (double) modelData->timeWindows[0][day].end / 2)
//            return nullptr;
        /* departure time */
        new_time += modelData->service[end];
    }else /* backwards label */
    {
        arrival += modelData->service[end];
        if(modelData->timeWindows[0][day].end - modelData->timeWindows[end][day].start < arrival)
            return nullptr;
        /* calculate new time (consider possible waiting time) */
        new_time = std::max(double(modelData->timeWindows[0][day].end - modelData->timeWindows[end][day].end), arrival);
        // TODO: check if enough
//        if(new_time > (double) modelData->timeWindows[0][day].end / 2)
//            return nullptr;
    }
    /* set new ng-memory */
    if(USE_DSSR)
        ng_memory = old_label->ng_memory_ & pricerData->ng_DSSR_[end];
    else
        ng_memory = old_label->ng_memory_ & modelData->ng_set[end];
    ng_memory[end] = true;

    /* calculate new reduced costs */
    if(isFW)
    {
        red_costs = old_label->red_costs_ + pricerData->arcPrices_[start][end];
    }else
    {
        red_costs = old_label->red_costs_ + pricerData->arcPrices_[end][start];
    }
    if(isEnforced)
        red_costs -= ENFORCED_PRICE_COLLECTING;

//    red_costs = old_label->red_costs_ - pricerData->dualValues_[end];
//    if(!isFarkas && modelData->minTravel)
//        red_costs += modelData->travel[start][end];

    obj = old_label->obj_ + modelData->travel[start][end];

    new_label = new Label2(red_costs, end, capacity, pricerData->nnonzSRC_, new_time, obj, ng_memory, old_label->visitedEC_);
    if(isEnforced)
        new_label->visitedEC_[end] = true;

    /* update subset row cuts states */
//    if(pricerData->nnonzSRC_ == 18 && day == 0 && start == 0 && end == 30)
//        cout << "start it: " << endl;
    for(int c = 0; c < pricerData->nnonzSRC_; c++)
    {
        assert(SCIPisNegative(pricerData->scip_, pricerData->SRC_dualv[c]));
        assert(pricerData->SRC_para_[c] == 0.5);
        state = old_label->SRCstate_[c];

        if((*pricerData->SRC_Set_[c])[end])
            state += pricerData->SRC_para_[c];
        if(SCIPisGE(pricerData->scip_, state, 1))
        {
            new_label->red_costs_ -= pricerData->SRC_dualv[c];
            state -= 1;
        }
        new_label->SRCstate_[c] = state;
    }

    assert(new_label != nullptr);

    return new_label;
}
