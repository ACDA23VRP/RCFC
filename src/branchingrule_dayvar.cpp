
#include <iostream>

#include "objscip/objscip.h"
#include "branchingrule_dayvar.h"
#include "model_data.h"
#include "probdata_vrp.h"
#include "scip/scip.h"
#include "pricer_vrp.h"
#include "ConshdlrDayVar.h"
#include "var_tools.h"

using namespace std;

static
SCIP_RETCODE setTreeData(
        SCIP*               scip,
        ObjPricerVRP*       pricerData
){
    long long int currNode = SCIPnodeGetNumber(SCIPgetCurrentNode(scip));
    assert(!pricerData->tree_data_[currNode].visitedChild);
    if(currNode == 1)
    {
        pricerData->tree_data_[currNode].node_timetable_ = pricerData->global_timetable_;
        pricerData->tree_data_[currNode].node_isForbidden_ = pricerData->global_isForbidden_;
    }else
    {
        pricerData->tree_data_[currNode].node_timetable_ = pricerData->timetable_;
        pricerData->tree_data_[currNode].node_isForbidden_ = pricerData->isForbidden_;
    }
    pricerData->tree_data_[currNode].node_ng_DSSR_ = pricerData->ng_DSSR_;
    pricerData->tree_data_[currNode].num_vars = SCIPgetNVars(scip);
    pricerData->tree_data_[currNode].node_fixedDay_ = pricerData->fixedDay_;
    pricerData->tree_data_[currNode].node_varfixing_ = pricerData->varfixing_gap_;

    return SCIP_OKAY;
}


SCIP_DECL_BRANCHEXECPS(ObjBranchruleDayVar::scip_execlp)
{
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    auto* pricerData = dynamic_cast<ObjPricerVRP*>(SCIPfindObjPricer(scip_, "VRP_Pricer"));
    model_data* modelData = probData->getData();
    vector<vector<SCIP_Real>> valonDay(modelData->nC, vector<SCIP_Real>(modelData->nDays));
    vector<int> numofDays(modelData->nC);
    int i, j;
    int best_cust = -1;
    int best_day = -1;
    SCIP_Real cur_val;
    SCIP_Real best_val = 0.5;
    int bestnum = 0;

    SCIP_NODE* childprohibit;
    SCIP_NODE* childenforce;
    SCIP_CONS* consprohibit;
    SCIP_CONS* consenforce;

    assert(probData != nullptr);
    assert(pricerData != nullptr);
    /* no Branching applied to node */
    assert(pricerData->tree_data_[SCIPnodeGetNumber(SCIPgetFocusNode(scip))].branchtype_ == BRANCHTYPE_NOTBRANCHED);

    /* update nodeData */
    SCIP_CALL(setTreeData(scip, pricerData));

    if(PRINT_BRANCHING_INFORMATION)
    {
        cout << "Node "<< SCIPnodeGetNumber(SCIPgetCurrentNode(scip))
        << " solved, but still fractional - start: DAY VAR BRANCHING!\n";
    }
    int count = 0;
//    cout << "-------------------------- NODE"<< SCIPnodeGetNumber(SCIPgetFocusNode(scip)) << " --------------------------" << endl;
//    for(auto var : probData->vars_)
//    {
//        if(SCIPisEQ(scip, 1.0, SCIPvarGetLPSol(var)))
//            count++;
//    }
//    cout << "Number of ModelVars with val 1.0:   " << count << endl;
//
//    count = 0;
//    int narcs;
//    vector<vector<SCIP_Real>> arcweights(modelData->nC, vector<SCIP_Real>(modelData->nC));
//    SCIP_CALL(getArcFlowValues(scip, arcweights, &narcs));
//    for(i = 0; i < modelData->nC; i++)
//    {
//        for(j = i+1; j < modelData->nC; j++)
//        {
//            if(SCIPisEQ(scip, arcweights[i][j], 1.0))
//                count++;
//            if(SCIPisEQ(scip, arcweights[j][i], 1.0))
//                count++;
//        }
//    }
//
//    cout << "Number of ArFloVars with val 1.0:   " << count << endl;

    SCIP_CALL(getVehiAssValues(scip, valonDay, numofDays));

    count = 0;
    for(i = 0; i < modelData->nC; i++)
    {
        for(j = 0; j < modelData->nDays; j++)
        {
            if(SCIPisEQ(scip, 1.0, valonDay[i][j]))
                count++;
            cur_val = fabs(0.5 - valonDay[i][j]);
            if(SCIPisEQ(scip, cur_val, 0.5))
                continue;
            /* day variable must not be 'worse' than current best day variable */
            if(!SCIPisLE(scip, cur_val, best_val))
                continue;
            /* if equal, then compare number of days the customer gets served on */
            if(SCIPisEQ(scip, cur_val, best_val))
            {
                if(numofDays[i] <= bestnum)
                    continue;
            }
            /* new best variable found */
            best_cust = i;
            best_day = j;
            bestnum = numofDays[i];
            best_val = cur_val;
        }
    }
//    cout << "Number of VeAssVars with val 1.0:   " << count << endl;

    if(best_cust == -1)
    {
        return SCIP_OKAY;
    }


    assert(best_cust >= 1 && best_cust <= modelData->nC - 1);
    assert(best_day >= 0 && best_day <= modelData->nDays);
    if(PRINT_BRANCHING_INFORMATION)
    {
        cout << "\tChosen Var: (Cust: " << best_cust << ", day:" << best_day << ") with value: ";
        cout << valonDay[best_cust][best_day] << " (num days: " << bestnum << ")\n";
    }

    /* create new nodes */
    SCIP_CALL( SCIPcreateChild(scip, &childprohibit, 0.0, SCIPgetLocalTransEstimate(scip)) );
    SCIP_CALL( SCIPcreateChild(scip, &childenforce, 0.0, SCIPgetLocalTransEstimate(scip)) );

    /* create corresponding constraints */
    SCIP_CALL( SCIPcreateConsDayVar(scip, &consprohibit, "prohibit", best_cust, best_day, PROHIBIT, childprohibit, TRUE) );
    SCIP_CALL( SCIPcreateConsDayVar(scip, &consenforce, "enforce", best_cust, best_day, ENFORCE, childenforce, TRUE) );

    /* add constraints to nodes */
    SCIP_CALL( SCIPaddConsNode(scip, childprohibit, consprohibit, nullptr) );
    SCIP_CALL( SCIPaddConsNode(scip, childenforce, consenforce, nullptr) );

    /* release constraints */
    SCIP_CALL( SCIPreleaseCons(scip, &consprohibit) );
    SCIP_CALL( SCIPreleaseCons(scip, &consenforce) );

    pricerData->tree_data_[SCIPnodeGetNumber(SCIPgetFocusNode(scip))].branchtype_ = BRANCHTYPE_DAYVAR;

    *result = SCIP_BRANCHED;

//    assert(FALSE);
    return SCIP_OKAY;
}