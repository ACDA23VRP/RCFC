
#ifndef __LABELING_ALGORITHMN_VRP__
#define __LABELING_ALGORITHMN_VRP__

#include "scip/scip.h"
#include "labellist.h"
#include "model_data.h"
#include "tourVRP.h"
#include "pricer_vrp.h"
#include "queue"

/* struct to pass arguments for labeling to worker threads */
typedef struct arg_struct {
    SCIP*               scip;
    model_data*         modelData;
    ObjPricerVRP*       pricerData;
    bool                isFarkas;        /**< TRUE for farkas-pricing, FALSE for redcost-pricing */
    bool                isHeuristic;
    bool                getDayVarRed;
    bool                noDomiance;
    int                 day;
    vector<tourVRP>*    bestTours;
} arg_struct;

SCIP_RETCODE generateLabels(
        SCIP*               scip,
        model_data*         modelData,
        ObjPricerVRP*       pricerData,
        LabelList&          bestLabels,
        bool                isFarkas,
        bool                isHeuristic,
        int                 day
);

SCIP_RETCODE labelingAlgorithmnParallel(
        SCIP*                       scip,
        vrp::ProbDataVRP*           probData,
        ObjPricerVRP*               pricerData,
        bool                        isFarkas,
        bool                        isHeuristic,
        bool                        getDayVarRed
);

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
);

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
);

SCIP_RETCODE concatenateLabels(
        SCIP*               scip,
        model_data*         modelData,
        ObjPricerVRP*       pricerData,
        bool                getDayVarRed,
        int                 day,
        vector<LabelList*>  &fw_list,
        vector<LabelList*>  &bw_list,
        vector<pair<pair<LabelNode*, LabelNode*>, double>> &sol_pairs
);

#endif