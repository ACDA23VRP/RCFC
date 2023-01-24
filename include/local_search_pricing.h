
#ifndef VRP_LOCAL_SEARCH_PRICING_H
#define VRP_LOCAL_SEARCH_PRICING_H

#include "scip/scip.h"
#include "labellist.h"
#include "model_data.h"
#include "pricer_vrp.h"

SCIP_RETCODE localSearchPricing(
        SCIP*                       scip,
        ObjPricerVRP*               pricerData,
        vrp::ProbDataVRP*           probData,
        const vector<SCIP_Real>&    dualValues,
        bool                   isFarkas
);


#endif //VRP_LOCAL_SEARCH_PRICING_H
