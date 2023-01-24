
#include <iostream>

#include "objscip/objscip.h"
#include "branchingrule_vehicle.h"
#include "model_data.h"
#include "probdata_vrp.h"
#include "scip/scip.h"
#include "pricer_vrp.h"
#include "vardata.h"
#include "ConshdlrVehicle.h"
#include "ConshdlrNVehicle.h"

using namespace std;

SCIP_DECL_BRANCHEXECPS(ObjBranchruleVehicle::scip_execlp)
{
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    auto* pricerData = dynamic_cast<ObjPricerVRP*>(SCIPfindObjPricer(scip_, "VRP_Pricer"));
    model_data* modelData = probData->getData();
    double sumvars = 0.0;
    vector<double> daysums(modelData->nDays, 0);
    SCIP_NODE* childupbranch;
    SCIP_NODE* childdownbranch;
    SCIP_CONS* consupbranch;
    SCIP_CONS* consdownbranch;

    assert(probData != nullptr);
    assert(pricerData != nullptr);
    /* no Branching applied to node */
    assert(pricerData->tree_data_[SCIPnodeGetNumber(SCIPgetFocusNode(scip))].branchtype_ == BRANCHTYPE_NOTBRANCHED);

    if(PRINT_BRANCHING_INFORMATION)
    {
        cout << "Node "<< SCIPnodeGetNumber(SCIPgetCurrentNode(scip))
        << " solved, but still fractional - start: NUM VEHICLE BRANCHING!\n";
    }
    for(auto* var : probData->vars_)
    {
        if(SCIPisPositive(scip, SCIPvarGetLPSol(var)))
        {
            auto* vardata = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));
            daysums[vardata->getDay()] += SCIPvarGetLPSol(var);
            sumvars += SCIPvarGetLPSol(var);
        }
    }

    /* First branch on the number of used vehicles */
    if(!SCIPisIntegral(scip, sumvars))
    {
        /* create new nodes */
        SCIP_CALL( SCIPcreateChild(scip, &childdownbranch, 0.0, SCIPgetLocalTransEstimate(scip)) );
        SCIP_CALL( SCIPcreateChild(scip, &childupbranch, 0.0, SCIPgetLocalTransEstimate(scip)) );
        if(true || PRINT_BRANCHING_INFORMATION)
        {
            cout << "SUM OF VARS: " << sumvars << endl;
            cout << "Create Branches with (sum <= " << floor(sumvars) << ") and (sum >= " << ceil(sumvars) << ")" << endl;
        }
        SCIP_CALL(SCIPcreateConsNVehicle(scip, &consdownbranch, "prohibit", floor(sumvars), PROHIBIT, childdownbranch));
        SCIP_CALL(SCIPcreateConsNVehicle(scip, &consupbranch, "enforce", ceil(sumvars), ENFORCE, childupbranch));

        pricerData->tree_data_[SCIPnodeGetNumber(SCIPgetFocusNode(scip))].branchtype_ = BRANCHTYPE_NVEHICLE;
    }else{
        return SCIP_OKAY;
        if(PRINT_BRANCHING_INFORMATION)
        {
            cout << "Number of used Vehicles is integral -> TRY single-VEHICLE BRANCHING" << endl;
        }
        /* Then branch on integral usage of each vehicle */
        double best = 0.5;
        int bestday = -1;
        for(int d = 0; d < modelData->nDays; d++)
        {
            if(SCIPisSumNegative(scip, fabs(daysums[d] - 0.5) - best))
            {
                best = fabs(daysums[d] - 0.5);
                bestday = d;
            }
        }
        if(bestday != -1)
        {
            /* create new nodes */
            SCIP_CALL( SCIPcreateChild(scip, &childdownbranch, 0.0, SCIPgetLocalTransEstimate(scip)) );
            SCIP_CALL( SCIPcreateChild(scip, &childupbranch, 0.0, SCIPgetLocalTransEstimate(scip)) );
            if(PRINT_BRANCHING_INFORMATION)
            {
                cout << "Branch on vehicle " << bestday << " (current value: " << daysums[bestday] << ")" << endl;
            }
            /* create corresponding constraints */
            SCIP_CALL(SCIPcreateConsVehicle(scip, &consdownbranch, "prohibit", bestday, PROHIBIT, childdownbranch));
            SCIP_CALL(SCIPcreateConsVehicle(scip, &consupbranch, "enforce", bestday, ENFORCE, childupbranch));

            pricerData->tree_data_[SCIPnodeGetNumber(SCIPgetFocusNode(scip))].branchtype_ = BRANCHTYPE_VEHICLE;
        }else
        {
            return SCIP_OKAY;
        }
    }

    /* add constraints to nodes */
    SCIP_CALL( SCIPaddConsNode(scip, childdownbranch, consdownbranch, nullptr) );
    SCIP_CALL( SCIPaddConsNode(scip, childupbranch, consupbranch, nullptr) );

    /* release constraints */
    SCIP_CALL( SCIPreleaseCons(scip, &consupbranch) );
    SCIP_CALL( SCIPreleaseCons(scip, &consdownbranch) );

    *result = SCIP_BRANCHED;

//    assert(false);

    return SCIP_OKAY;
}