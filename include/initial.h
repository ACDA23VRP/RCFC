
#include "probdata_vrp.h"
#include "model_data.h"

#ifndef VRP_INITIAL_H
#define VRP_INITIAL_H

SCIP_RETCODE addInitSolution(
        SCIP*               scip,
        vrp::ProbDataVRP*   probData,
        vector<tourVRP>&    sol_tvrps
);

SCIP_RETCODE addEmptyTours(
        SCIP*                   scip,
        vrp::ProbDataVRP*       probData
);

extern
SCIP_RETCODE initialDispatching(
        SCIP*                 scip,
        vrp::ProbDataVRP*     probdata
);

#endif //VRP_INITIAL_H
