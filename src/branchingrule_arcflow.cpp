
#include <iostream>

#include "objscip/objscip.h"
#include "branchingrule_arcflow.h"
#include "model_data.h"
#include "probdata_vrp.h"
#include "scip/scip.h"
#include "pricer_vrp.h"
#include "vardata.h"
#include "var_tools.h"
#include "ConshdlrArcflow.h"

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

SCIP_DECL_BRANCHEXECPS(ObjBranchruleArcflow::scip_execlp)
{
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    auto* pricerData = dynamic_cast<ObjPricerVRP*>(SCIPfindObjPricer(scip_, "VRP_Pricer"));
    model_data* modelData = probData->getData();
    vector<vector<SCIP_Real>> arcweights(modelData->nC, vector<SCIP_Real>(modelData->nC));
    vector<SCIP_VAR*> modelvars = probData->vars_;

    int narcs;
    int i, j;
    int start = 0;
    int end = 0;
    SCIP_Real goal_val = 0.5;
    SCIP_Real bestval = goal_val;

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
             << " solved, but still fractional - start: ARC FLOW BRANCHING!\n";
    }
    SCIP_CALL(getArcFlowValues(scip, arcweights, &narcs));

    for(i = 0; i < modelData->nC; i++)
    {
        for(j = 0; j < modelData->nC; j++)
        {
            if(SCIPisSumPositive(scip, arcweights[i][j]) && SCIPisSumPositive(scip, 1 - arcweights[i][j]))
            {
                if(fabs(goal_val - arcweights[i][j]) < bestval)
                {
                    bestval = fabs(goal_val - arcweights[i][j]);
                    start = i;
                    end = j;
                }
            }
        }
    }

    assert(start >= 0 && start <= modelData->nC - 1);
    assert(end >= 0 && end <= modelData->nC - 1);
    if(PRINT_BRANCHING_INFORMATION)
        cout << "\tChosen arc: (" << start << "-" << end << ") with value: " << arcweights[start][end] << '\n';

    /* create new nodes */
    SCIP_CALL( SCIPcreateChild(scip, &childprohibit, 0.0, SCIPgetLocalTransEstimate(scip)) );
    SCIP_CALL( SCIPcreateChild(scip, &childenforce, 0.0, SCIPgetLocalTransEstimate(scip)) );

    /* create corresponding constraints */
    SCIP_CALL( SCIPcreateConsArcFlow(scip, &consprohibit, "prohibit", start, end, PROHIBIT, childprohibit, TRUE) );
    SCIP_CALL( SCIPcreateConsArcFlow(scip, &consenforce, "enforce", start, end, ENFORCE, childenforce, TRUE) );

    /* add constraints to nodes */
    SCIP_CALL( SCIPaddConsNode(scip, childprohibit, consprohibit, nullptr) );
    SCIP_CALL( SCIPaddConsNode(scip, childenforce, consenforce, nullptr) );

    /* release constraints */
    SCIP_CALL( SCIPreleaseCons(scip, &consprohibit) );
    SCIP_CALL( SCIPreleaseCons(scip, &consenforce) );

    pricerData->tree_data_[SCIPnodeGetNumber(SCIPgetFocusNode(scip))].branchtype_ = BRANCHTYPE_ARCFLOW;

    *result = SCIP_BRANCHED;

//    assert(FALSE);
    return SCIP_OKAY;

}