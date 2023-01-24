
#include "prop_varfixing.h"
#include "pricer_vrp.h"
#include "vardata.h"
#include "tourVRP.h"

static
SCIP_RETCODE updateRootRedCosts(
    SCIP*           scip,
    ObjPricerVRP*   pricerData,
    model_data*     modelData
){
    SCIP_Real best_redcosts;
    SCIP_Real best_objval;
    SCIP_Real current_redcosts;
    SCIP_Real current_objval = SCIPgetLocalLowerbound(scip);
//    cout << current_objval << " VS " << SCIPgetLPObjval(scip) << endl;
    assert(SCIPisEQ(scip, current_objval, SCIPgetLPObjval(scip)));

    /* update vehicle assignment variables */
    for (int day = 0; day < modelData->nDays; day++) {
        for (auto cust: modelData->neighbors[0][day]) {
            if (!pricerData->global_timetable_[cust][day])
                continue;
            /* best red costs of the vehicle assignment variable */
            best_redcosts = pricerData->root_dayVarRedCosts_[cust][day];
            best_objval = pricerData->root_dayVarLPObj_[cust][day];
            /* current red costs */
            current_redcosts = pricerData->dayVarRedCosts_[cust][day];
            /* update if current improves the best red costs */
            if(SCIPisGT(scip, current_redcosts + current_objval, best_redcosts + best_objval))
            {
                pricerData->root_dayVarRedCosts_[cust][day] = current_redcosts;
                pricerData->root_dayVarLPObj_[cust][day] = current_objval;
            }
        }
    }

    for (int c1 = 0; c1 < modelData->nC; c1++) {
        for (int c2 = 0; c2 < modelData->nC; c2++) {
            if(c1 == c2 || pricerData->global_isForbidden_[c1][c2])
                continue;
            /* best red costs of the arc flow variable */
            best_redcosts = pricerData->root_arcRedCosts_[c1][c2];
            best_objval = pricerData->root_arcLPObj_[c1][c2];
            /* current red costs */
            current_redcosts = pricerData->arcRedCosts_[c1][c2];
            /* update if current improves the best red costs */
//            cout << current_redcosts + current_objval << " VS " << best_redcosts + best_objval << endl; // TODO INFTY
            if(SCIPisGT(scip, current_redcosts + current_objval, best_redcosts + best_objval))
            {
                pricerData->root_arcRedCosts_[c1][c2] = current_redcosts;
                pricerData->root_arcLPObj_[c1][c2] = current_objval;
            }
        }
    }

    return SCIP_OKAY;
}

static
SCIP_RETCODE varFixingRoot(
    SCIP*           scip,
    ObjPricerVRP*   pricerData,
    model_data*     modelData,
    bool*           success
){
    int count_v = 0;
    int count_a = 0;
    SCIP_Real lhs;
    SCIP_Real cutoffbound = SCIPgetCutoffbound(scip);

    /* check vehicle assignment variables */
    for (int day = 0; day < modelData->nDays; day++) {
        for (auto cust: modelData->neighbors[0][day]) {
            if (!pricerData->global_timetable_[cust][day])
                continue;
            /* check for day assignment variables with high reduced costs */
            lhs = pricerData->root_dayVarRedCosts_[cust][day] + pricerData->root_dayVarLPObj_[cust][day];
            if (SCIPisSumPositive(scip, lhs - cutoffbound)) {
                count_v++;
                *success = true;
                pricerData->global_timetable_[cust][day] = false;
            }
        }
    }
    /* check arc flow variables*/
    for (int c1 = 0; c1 < modelData->nC; c1++) {
        for (int c2 = 0; c2 < modelData->nC; c2++) {
            if(c1 == c2 || pricerData->global_isForbidden_[c1][c2])
                continue;
            lhs = pricerData->root_arcRedCosts_[c1][c2] + pricerData->root_arcLPObj_[c1][c2];
            if (SCIPisSumPositive(scip, lhs - cutoffbound)) {
                count_a++;
                *success = true;
                pricerData->global_isForbidden_[c1][c2] = true;
            }

        }
    }
//    cout << "GlobalFixing: (" << SCIPnodeGetNumber(SCIPgetCurrentNode(scip)) << ") cutoff: " << cutoffbound;
//    cout << " --> DELETED VARS: (d) " << count_v << " (a) " << count_a << endl;
    return SCIP_OKAY;
}

static
SCIP_RETCODE varFixing(
        SCIP*           scip,
        ObjPricerVRP*   pricerData,
        model_data*     modelData,
        bool*           success
){
    int count_v = 0;
    int count_a = 0;
    int day;
    double gap = SCIPgetCutoffbound(scip) - SCIPgetLocalDualbound(scip);
    /* check vehicle assignment variables */
    for (day = 0; day < modelData->nDays; day++) {
        for (auto cust: modelData->neighbors[0][day]) {
            if (!pricerData->timetable_[cust][day])
                continue;
            /* check for day assignment variables that with high reduced costs */
            if (SCIPisSumPositive(scip, pricerData->dayVarRedCosts_[cust][day] - gap)) {
                count_v++;
                *success = true;
                pricerData->timetable_[cust][day] = false;
//                cout << "fix (cust, day): (" << cust << ", " << day << ")" << endl;
            }
        }
    }
    /* check arc flow variables*/
    for (int c1 = 0; c1 < modelData->nC; c1++) {
        for (int c2 = c1 + 1; c2 < modelData->nC; c2++) {
            if (!pricerData->isForbidden_[c1][c2]) {
                if (SCIPisSumPositive(scip, pricerData->arcRedCosts_[c1][c2] - gap)) {
                    count_a++;
                    *success = true;
                    pricerData->isForbidden_[c1][c2] = true;
//                    cout << "fix edge (u, v): (" << c1 << ", " << c2 << ")" << endl;
                }
            }
            if (!pricerData->isForbidden_[c2][c1]) {
                if (SCIPisSumPositive(scip, pricerData->arcRedCosts_[c2][c1] - gap)) {
                    count_a++;
                    *success = true;
                    pricerData->isForbidden_[c2][c1] = true;
//                    cout << "fix edge (u, v): (" << c1 << ", " << c2 << ")" << endl;
                }
            }
        }
    }
//    cout << "LocalFixing: (" << SCIPnodeGetNumber(SCIPgetCurrentNode(scip)) << ") gap: " << gap;
//    cout << " --> DELETED VARS: (d) " << count_v << " (a) " << count_a << endl;

    return SCIP_OKAY;
}

static
SCIP_RETCODE tourVarFixing(
    SCIP*               scip,
    vrp::ProbDataVRP*   probData,
    ObjPricerVRP*       pricerData,
    bool                isGlobal
){
    vector<vector<bool>>& timetable = isGlobal ?  pricerData->global_timetable_ : pricerData->timetable_;
    vector<vector<bool>>& isForbidden = isGlobal ? pricerData->global_isForbidden_ : pricerData->isForbidden_;

    pricerData->fixed_nonzero_ = false;
    SCIP_Bool fixed;
    SCIP_Bool infeasible;
    int cnt = 0;
    for (auto var: probData->vars_) {
        fixed = false;
        if (!isGlobal && SCIPvarGetUbLocal(var) < 0.5)
            continue;
        if(isGlobal && SCIPvarGetUbGlobal(var) < 0.5)
            continue;

        auto *varData = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));
        tourVRP &tvrp = varData->tourVrp_;

        int day = tvrp.getDay();
        /* check first/last arc and first customer */
        if (isForbidden[tvrp.tour_[tvrp.length_ - 1]][0] ||
                isForbidden[0][tvrp.tour_[0]] || !timetable[tvrp.tour_[0]][day]) {
            fixed = TRUE;
        } else {
            /* check inner arcs and rest of customers */
            for (int v = 1; v < tvrp.length_; v++) {
                if (!timetable[tvrp.tour_[v]][day] ||
                    isForbidden[tvrp.tour_[v - 1]][tvrp.tour_[v]]) {
                    fixed = TRUE;
                    break;
                }
            }
        }
        if (fixed) {
            cnt++;
            if(!SCIPisZero(scip, SCIPvarGetLPSol(var)))
            {
//                cout << "var with val: " << SCIPvarGetLPSol(var) << endl;
//                SCIPprintVar(scip, var , nullptr);
                pricerData->fixed_nonzero_ = true;
            }
//                assert(SCIPisZero(scip, SCIPvarGetLPSol(var))); // TODO check if only due to EC
            if(isGlobal)
            {
                SCIP_CALL(SCIPtightenVarUbGlobal(scip, var, 0, TRUE, &infeasible, &fixed));
            }else
            {
                SCIP_CALL(SCIPfixVar(scip, var, 0.0, &infeasible, &fixed));
            }
            assert(!infeasible); // TODO - 60_0.25
            assert(fixed);
        }
    }
//    cout << "FIXED " << cnt << " TOUR VARIABLES!!" << endl;

    return SCIP_OKAY;
}

/** execution method of propagator */
SCIP_DECL_PROPEXEC(ObjPropVarFixing::scip_exec) {
    bool success = false;
    bool success_root = false;
    auto *probData = dynamic_cast<vrp::ProbDataVRP *>(SCIPgetObjProbData(scip_));
    model_data *modelData = probData->getData();
    auto *pricerData = dynamic_cast<ObjPricerVRP *>(SCIPfindObjPricer(scip_, "VRP_Pricer"));

    *result = SCIP_DIDNOTRUN;
    /* check if new reduced costs have been calculated and if there is an optimal solution */
    if (SCIPgetLPSolstat(scip) != SCIP_LPSOLSTAT_OPTIMAL) {
        return SCIP_OKAY;
    }
//    /* skip if no new reduced costs have been calculated, and we are not at the root */
//    if(!active_redcosts_ && SCIPgetDepth(scip) != 0){
//        return SCIP_OKAY;
//    }
    /* skip if there is no new cutoff bound and reduced costs at root */
    if(!active_redcosts_ && SCIPisEQ(scip, lastCutoff_, SCIPgetCutoffbound(scip)))
        return SCIP_OKAY;

//    cout << "Start Propagator to FIX VARIABLES!!" << endl;
//    cout << "active: " << active_redcosts_ << " lastco: " << lastCutoff_ << " currco: " << SCIPgetCutoffbound(scip) << endl;
    *result = SCIP_DIDNOTFIND;

    if(SCIPgetDepth(scip) == 0)
    {
        if(active_redcosts_)
        {
//            cout << "UPDATE AT ROOT " << endl;
            SCIP_CALL(updateRootRedCosts(scip, pricerData, modelData));
        }
        if(active_redcosts_ || SCIPisLT(scip, SCIPgetCutoffbound(scip), lastCutoff_))
        {
//            cout << "FIX AT ROOT" << endl;
            SCIP_CALL(varFixingRoot(scip, pricerData, modelData, &success_root));
            lastCutoff_ = SCIPgetCutoffbound(scip);
        }
    }else{
        assert(SCIPgetDepth(scip) >= 1);
        if(SCIPisLT(scip, SCIPgetCutoffbound(scip), lastCutoff_))
        {
            assert(!active_redcosts_);
//            cout << "FIXforROOT!" << endl;
            SCIP_CALL(varFixingRoot(scip, pricerData, modelData, &success_root));
            lastCutoff_ = SCIPgetCutoffbound(scip);
        }
        if(active_redcosts_)
        {
//            cout << "FIX AT TREE" << endl;
            SCIP_CALL(varFixing(scip, pricerData, modelData, &success));
        }

    }

    if(success_root || success)
    {
        /* success XOR success_root */
        assert(success || success_root);
        assert(!success || !success_root);
        *result = SCIP_REDUCEDDOM;

        SCIP_CALL(tourVarFixing(scip, probData, pricerData, success_root));
        /* if no non-zero variable has been fixed, the next pricing iteration can be skipped  */
        pricerData->node_varsfixed_ = SCIPnodeGetNumber(SCIPgetCurrentNode(scip));
    }

    active_redcosts_ = false;
    return SCIP_OKAY;
}