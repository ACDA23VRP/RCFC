
#ifndef VRP_TOOLS_VRP_H
#define VRP_TOOLS_VRP_H

#include "pricer_vrp.h"

#include <iostream>
#include <vector>

#include "model_data.h"
#include "scip/cons_setppc.h"
#include "scip/cons_linear.h"
#include "probdata_vrp.h"
#include "labeling_algorithm_vrp.h"
#include "labellist.h"
#include "vardata.h"
#include "tourVRP.h"

#define MAX_LOCALSEARCH_TOURS 25

/** prepares tour variable to be added to be model */
SCIP_RETCODE add_tour_variable(
        SCIP*                       scip,
        vrp::ProbDataVRP*           probData,
        SCIP_Bool                   isFarkas,
        SCIP_Bool                   isInitial,
        char*                       algoName,
        tourVRP&                    tvrp
);

double getExchangeCosts(
        model_data*             modelData,
        vector<int>&            tour,
        int                     length,
        int                     new_cust,
        int                     pos
);

double getDeleteCosts(
        model_data*             modelData,
        vector<int>&            tour,
        int                     length,
        int                     pos
);

SCIP_RETCODE addUnvisitedCustomers(
        SCIP*                   scip,
        model_data*             modelData,
        vector<tourVRP>&        tvrp,
        vector<int>&            vehicleofNode,
        vector<int>&            unvisitedC
);

SCIP_RETCODE twoNodeShift(
        SCIP*                   scip,
        model_data*             modelData,
        vector<tourVRP>&        tvrps,
        vector<int>&            vehicleofnode
);

bool violatesNGProperty(
        vector<bitset<neighborhood_size>>&  ng_sets,
        vector<bitset<neighborhood_size>>&  ng_DSSR,
        vector<int>&                        tour
);

#endif //VRP_TOOLS_VRP_H
