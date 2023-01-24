
#ifndef VRP_VAR_TOOLS_H
#define VRP_VAR_TOOLS_H

#include "model_data.h"
#include "vector"
#include "scip/scip.h"
#include "objscip/objscip.h"
#include "vardata.h"
#include "probdata_vrp.h"

using namespace scip;

SCIP_RETCODE getArcFlowValues(
    SCIP*                       scip,
    vector<vector<SCIP_Real>>&  values,
    int*                        narcs
);

SCIP_RETCODE getVehiAssValues(
    SCIP*                       scip,
    vector<vector<SCIP_Real>>&  values,
    vector<int>&                numofDays
);

bool varInRow(
    SCIP_VAR*                   var,
    SCIP_ROW*                   row
);

SCIP_RETCODE SCIPsortNonzeroVars(
    SCIP*                       scip,
    vrp::ProbDataVRP*           probData
);

#endif //VRP_VAR_TOOLS_H
