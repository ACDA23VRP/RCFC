
#include "ConshdlrCPC.h"
#include "scip/cons_setppc.h"
#include "objscip/objscip.h"
#include "pricer_vrp.h"
#include "vardata.h"
#include "tourVRP.h"
#include "var_tools.h"
#include "ConshdlrCPC.h"
#include <ctime>


/** Constraint data for "CPC" constraints */
struct SCIP_ConsData
{
    SCIP_Row*           cut;            /**< Corresponding row in the LP */
    SCIP_VAR**          vars;           /**< variables of the constraint */
    int                 nvars;          /**< number of variables in the constraint */
//    vector<int>         enforcedCust;   /**< customers that have to be visited by vars of constraint */
//    int                 day;            /**< var_day, if all vars are of the same day, else -1 */
};

/** local methods */
static
SCIP_RETCODE consdataCreate(
        SCIP*           scip,
        SCIP_CONSDATA** consdata,
        vector<SCIP_VAR*>& vars
){
    SCIP_CALL(SCIPallocBlockMemory(scip, consdata));

    (*consdata)->cut = nullptr;
    (*consdata)->nvars = (int) vars.size();
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &(*consdata)->vars, (*consdata)->nvars));
    for(int i = 0; i < (*consdata)->nvars; i++)
    {
        (*consdata)->vars[i] = vars[i];
    }

    return SCIP_OKAY;
}

static
SCIP_RETCODE addVarsToCons(
    SCIP*           scip,
    SCIP_CONSDATA*  consdata
){
    assert(consdata->cut != nullptr);
    for(int i = 0; i < consdata->nvars; i++)
    {
//        SCIPprintVar(scip, consdata->vars[i], nullptr);
        SCIP_CALL(SCIPaddVarToRow(scip, consdata->cut, consdata->vars[i], 1));
    }

    return SCIP_OKAY;
}

static
SCIP_RETCODE fixVariables(
        SCIP*               scip,
        vector<SCIP_VAR*>&  fixableVars

){
    auto *probData = dynamic_cast<vrp::ProbDataVRP *>(SCIPgetObjProbData(scip));
    auto *pricerData = dynamic_cast<ObjPricerVRP *>(SCIPfindObjPricer(scip, "VRP_Pricer"));
//    cout << "FIX " << fixableVars.size() << " TOUR-VARIABLES TO 1  (" << SCIPnodeGetNumber(SCIPgetCurrentNode(scip)) << " -> d=" << SCIPnodeGetDepth(
//            SCIPgetCurrentNode(scip))<< ")" << endl;

    pricerData->tree_data_[SCIPnodeGetNumber(SCIPgetCurrentNode(scip))].gotFixed = true;

    for(auto var : fixableVars)
    {
        if(SCIPisLT(scip, SCIPvarGetLPSol(var), 1))
        {
            pricerData->tree_data_[SCIPnodeGetNumber(SCIPgetCurrentNode(scip))].gotFixed = false;
            pricerData->node_varsfixed_ = SCIPnodeGetNumber(SCIPgetCurrentNode(scip));
        }
        SCIPchgVarLb(scip, var, 1.0);
        auto* vardata = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));
        int day = vardata->tourVrp_.getDay();
        if(probData->getData()->num_v[day] == 1)
            pricerData->fixedDay_[day] = true;
        for(auto u : vardata->tourVrp_.tour_)
        {
            pricerData->eC_[u] = day;
            for(int d = 0; d < probData->getData()->nDays; d++)
            {
                if(d == day)
                    continue;
                if(SCIPnodeGetNumber(SCIPgetCurrentNode(scip)) == 1)
                    pricerData->global_timetable_[u][d] = false;
                else
                    pricerData->timetable_[u][d] = false;
            }
        }
    }
    return SCIP_OKAY;
}

static
SCIP_RETCODE addCuts(
    SCIP*                       scip,
    vector<vector<SCIP_VAR*>>&  cutVars
){
    SCIP_CONS* cons;
    for(auto& cuts : cutVars)
    {
        SCIP_CALL(SCIPcreateConsCPC(scip, &cons, "name", cuts));

        SCIPaddCons(scip, cons);

        SCIP_CALL(SCIPcreateAndAddRowCPC(scip, cons));

        SCIP_CALL(SCIPreleaseCons(scip, &cons));
    }

    return SCIP_OKAY;
}

static
SCIP_RETCODE checkFixing(
        SCIP*               scip,
        vector<SCIP_VAR*>&  checkvars,
        SCIP_RESULT*        result
){
    vector<SCIP_VAR*> fixableVars;
    vector<vector<SCIP_VAR*>> cutVars;
    double cmp_low;
    double gap_abs = SCIPgetCutoffbound(scip) - SCIPgetLPObjval(scip);
    double gap_rel = gap_abs / SCIPgetLPObjval(scip);
    double gap_percent = 100*gap_rel;
    double minvalue = 0.4;

    auto *probData = dynamic_cast<vrp::ProbDataVRP *>(SCIPgetObjProbData(scip));

    if(SCIPvarGetLPSol(checkvars[0]) >= minvalue)
    {
        probData->prop_calls_++;
    }
    for(auto* var : checkvars)
    {
        if(fixableVars.size() > 3)
           break;
        if(SCIPvarGetLbLocal(var) > 0.5)
            continue;
        if(SCIPvarGetLPSol(var) < minvalue)
            break;
//        if(SCIPvarGetLPSol(var) < 0.7 && gap_rel > 0.01) // TODO: parameter tuning
//            break;
//        if(gap / SCIPgetLocalDualbound(scip) > 0.0043)
//            break;
//        if(SCIPgetDepth(scip) < 7)
//            break;

        double varObj = SCIPvarGetObj(var);
        double varLPval = SCIPvarGetLPSol(var);
        cmp_low = SCIPgetLPObjval(scip) - ((varLPval * gap_abs) / (1 - varLPval));

        auto* vardata = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));
        auto* obj = dynamic_cast<ObjPropTourVarFixing*>(SCIPgetObjProp(scip, SCIPfindProp(scip, "tourVarFixing")));
        obj->tvrp_.copy(vardata->tourVrp_);
        obj->obj_cmp_ = SCIPgetCutoffbound(scip);

        SCIP_Bool lperror, cutoff;
        obj->up_ = true;
        obj->new_ = false;
        if(probData->use_propagator_ == 1)
        {
            SCIPstartProbing(scip);

            SCIPchgVarObjProbing(scip, var, SCIPvarGetObj(var) + gap_abs/SCIPvarGetLPSol(var));

            SCIPsolveProbingLPWithPricing(scip, false, false, -1, &lperror, &cutoff);

            assert(!lperror);
            SCIPendProbing(scip);
            assert(SCIPisEQ(scip, SCIPvarGetObj(var), varObj));
        }else
        {
            obj->new_ = true;
            SCIPstartProbing(scip);

            SCIPchgVarUb(scip, var, 0.0);
            SCIPsolveProbingLPWithPricing(scip, false, false, -1, &lperror, &cutoff);

            SCIPendProbing(scip);
        } 
        
        /* Can variable be fixed? */
        if(cutoff)
        {
            if(gap_percent > probData->highest_gap_frac_ && SCIPisLT(scip, varLPval, 1))
                probData->highest_gap_frac_ = gap_percent;
            if(gap_percent > probData->highest_gap_ && SCIPisEQ(scip, varLPval, 1))
                probData->highest_gap_ = gap_percent;
            if(probData->earliest_find_ > SCIPgetDepth(scip))
                probData->earliest_find_ = SCIPgetDepth(scip);
            if(probData->lowest_value_ > SCIPvarGetLPSol(var))
                probData->lowest_value_ = SCIPvarGetLPSol(var);
            if(SCIPisLT(scip, varLPval, 1))
                probData->num_fixed_frac_++;
            if(SCIPisEQ(scip, varLPval, 1))
                probData->num_fixed_++;

            /* Check if Node can be cut off */
            if(varLPval <= 0.6)
            {
                if(probData->use_propagator_ == 1)
                {
                    SCIPstartProbing(scip);
                    obj->obj_cmp_ = cmp_low;
                    SCIPchgVarObjProbing(scip, var, SCIPvarGetObj(var) - gap_abs/(1 - varLPval));
                    SCIPsolveProbingLPWithPricing(scip, false, false, -1, &lperror, &cutoff);
                }else{
                    SCIPstartProbing(scip);
                    SCIPchgVarLb(scip, var, 1.0);
                    SCIPsolveProbingLPWithPricing(scip, false, false, -1, &lperror, &cutoff);
                }
                if((!obj->new_ && SCIPisLT(scip, cmp_low, SCIPgetLPObjval(scip))) || (obj->new_ && cutoff))
                {
                    *result = SCIP_CUTOFF;
                    probData->num_cutoff_++;
                    if(gap_percent > probData->cutoff_gap_)
                        probData->cutoff_gap_ = gap_percent;
                    if(SCIPisGT(scip, varLPval, probData->cutoff_val_))
                        probData->cutoff_val_ = varLPval;
                    SCIPendProbing(scip);
                    return SCIP_OKAY;
                }
                assert(!lperror);
                SCIPendProbing(scip);
            }
            fixableVars.push_back(var);
        }else
        {
            if(SCIPisLT(scip, varLPval, 1.0))
                break;
        }
    }
    if(!fixableVars.empty())
    {
        probData->fixednodes_.push_back(SCIPnodeGetNumber(SCIPgetCurrentNode(scip)));
        probData->sizesubtree_.push_back(1);
        probData->fixedgap_.push_back(gap_percent);
        *result = SCIP_REDUCEDDOM;
        SCIP_CALL(fixVariables(scip, fixableVars));
        if(gap_percent > probData->highest_gap_)
            probData->highest_gap_ = gap_percent;
    }
    if(!cutVars.empty())
    {
        *result = SCIP_SEPARATED;
        SCIP_CALL(addCuts(scip, cutVars));
    }

    return SCIP_OKAY;
}



SCIP_DECL_CONSDELETE(ConshdlrCPC::scip_delete)
{
    assert(conshdlr != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), "CPC") == 0);
    assert(consdata != nullptr);
    assert(*consdata != nullptr);

    /* delete constraint data */
    SCIPreleaseRow(scip, &(*consdata)->cut);
    SCIPfreeBlockMemory(scip, consdata);
    SCIPfreeBlockMemoryArray(scip, &(*consdata)->vars, (*consdata)->nvars);

    return SCIP_OKAY;
}

SCIP_DECL_CONSENFOLP(ConshdlrCPC::scip_enfolp)
{
//    return SCIP_OKAY;

    if(stopfixing_)
        return SCIP_OKAY;
    if(SCIPinProbing(scip))
        return SCIP_OKAY;
    if(SCIPinDive(scip))
        return SCIP_OKAY;
    if(SCIPgetLPSolstat(scip) == SCIP_LPSOLSTAT_NOTSOLVED || SCIPgetLPSolstat(scip) == SCIP_LPSOLSTAT_INFEASIBLE)
        return SCIP_OKAY;

    auto *probData = dynamic_cast<vrp::ProbDataVRP *>(SCIPgetObjProbData(scip_));
    if(!probData->use_propagator_)
        return SCIP_OKAY;


    assert(SCIPgetLPSolstat(scip) == SCIP_LPSOLSTAT_OPTIMAL);

    assert(SCIPisEQ(scip, SCIPgetPrimalbound(scip), SCIPgetCutoffbound(scip)));

    *result = SCIP_FEASIBLE;

    if(probData->nonzeroVars_.empty())
    {
        SCIP_CALL(SCIPsortNonzeroVars(scip, probData));
    }
    clock_t t;
    t = clock();
    SCIP_CALL(checkFixing(scip, probData->nonzeroVars_, result));
    probData->prop_time_ += ((float)(clock()-t))/CLOCKS_PER_SEC;

    if(*result == SCIP_CUTOFF || *result == SCIP_REDUCEDDOM || *result == SCIP_SEPARATED)
    {
        probData->prop_success_++;
    }
    return SCIP_OKAY;
}

SCIP_DECL_CONSPRINT(ConshdlrCPC::scip_print)
{
    return SCIP_OKAY;
}

/** creates and adds the row of a subset row constraint to the LP */
SCIP_RETCODE SCIPcreateAndAddRowCPC(
        SCIP*                   scip,
        SCIP_Cons*              cons
){
    SCIP_CONSDATA* consdata;
    assert(scip != nullptr);
    assert(cons != nullptr);
    SCIP_Bool infeasible;
    consdata = SCIPconsGetData(cons);
    assert(consdata->cut == nullptr);

    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &(consdata->cut), cons, SCIPconsGetName(cons), 1,
                                     SCIPinfinity(scip), TRUE, FALSE, FALSE));

    assert(consdata->cut != nullptr);

    SCIP_CALL(addVarsToCons(scip, consdata));

    if(!SCIProwIsInLP(consdata->cut))
    {
        SCIP_CALL(SCIPaddRow(scip, consdata->cut, true, &infeasible));
        assert(!infeasible);
    }

    return SCIP_OKAY;
}

/** creates and captures a counterpart cut constraint */
SCIP_RETCODE SCIPcreateConsCPC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS**             cons,               /**< pointer to hold the created constraint */
        const char*             name,               /**< name of constraint */
        vector<SCIP_VAR*>&      vars
){
    SCIP_CONSDATA* consdata = nullptr;
    SCIP_CONSHDLR* conshdlr = nullptr;

    assert(scip != nullptr);

    /* find the set partitioning constraint handler */
    conshdlr = SCIPfindConshdlr(scip, "CPC");
    assert(conshdlr != nullptr);
    /* create constraint data*/
    SCIP_CALL(consdataCreate(scip, &consdata, vars));
    assert(consdata != nullptr);
    /* create constraint */
    SCIP_CALL(SCIPcreateCons(scip, cons, name, conshdlr, consdata,FALSE, TRUE, TRUE, FALSE,
                             FALSE, TRUE, FALSE, FALSE, FALSE, FALSE));

    return SCIP_OKAY;
}
