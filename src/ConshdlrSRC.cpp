
#include <iostream>

#include "scip/cons_linear.h"
#include "ConshdlrSRC.h"
#include "objscip/objscip.h"

#include "var_tools.h"
#include "probdata_vrp.h"
#include "tsptw.h"

#define CONSHDLR_NAME4   "SRC"

/** Constraint data for "SRC" constraints */
struct SCIP_ConsData
{
    vector<bool>*       customerSet;    /**< Set S of customers defining the lhs */
    vector<bool>*       lmSet;          /**< Limited Memory Set */
    double              p;              /**< parameter p */
    double              rhs;            /**< right hand side */
    SCIP_Row*           cut;            /**< Corresponding row in the LP */
    SCIP_VAR**          vars;           /**< variables of the constraint */
    SCIP_Real*          coeffs;         /**< coefficients of variables in the constraint */
    int                 nvars;          /**< number of variables in the constraint */
    int                 varssize;       /**< size of var array */
};

/** local methods */
static
SCIP_RETCODE consdataCreate(
    SCIP*           scip,
    SCIP_CONSDATA** consdata,
    vector<bool>&   setS,
    vector<bool>&   lmSet,
    double          p,
    double          rhs
){
    SCIP_CALL(SCIPallocBlockMemory(scip, consdata));

    (*consdata)->customerSet = new vector<bool>(setS);
    (*consdata)->lmSet = new vector<bool>(lmSet);
    (*consdata)->p = p;
    (*consdata)->rhs = rhs;
    (*consdata)->cut = nullptr;
    (*consdata)->nvars = 0;
    (*consdata)->varssize = 1;
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &(*consdata)->vars, 1));
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &(*consdata)->coeffs, 1));

    return SCIP_OKAY;
}

/** ensures, that the vars array can store at least num entries */
static
SCIP_RETCODE consdataEnsureVarsSize(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_CONSDATA*        consdata,           /**< setppc constraint data */
        int                   num                 /**< minimum number of entries to store */
)
{
    assert(consdata != nullptr);
    assert(consdata->nvars <= consdata->varssize);

    if( num > consdata->varssize )
    {
        int newsize;

        newsize = SCIPcalcMemGrowSize(scip, num);
        SCIP_CALL( SCIPreallocBlockMemoryArray(scip, &consdata->vars, consdata->varssize, newsize) );
        SCIP_CALL( SCIPreallocBlockMemoryArray(scip, &consdata->coeffs, consdata->varssize, newsize) );
        consdata->varssize = newsize;
    }
    assert(num <= consdata->varssize);

    return SCIP_OKAY;
}

static
SCIP_RETCODE addVarToCons(
        SCIP*               scip,
        SCIP_VAR*           var,
        SCIP_CONSDATA*      consdata,
        double              value
){
    assert(consdata != nullptr);
    assert(var != nullptr);

    SCIP_CALL( consdataEnsureVarsSize(scip, consdata, consdata->nvars+1) );
    consdata->vars[consdata->nvars] = var;
    consdata->coeffs[consdata->nvars] = value;
    consdata->nvars++;

    SCIP_CALL(SCIPaddVarToRow(scip, consdata->cut, var, value));

    return SCIP_OKAY;
}

static
SCIP_RETCODE addVarsToCons( // TODO: adjust
        SCIP*               scip,
        vector<bool>&       setS,
        double              p,
        SCIP_CONSDATA*      consdata
){
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip));
    for(auto var : probData->vars_)
    {
        double value = 0.0;
        tourVRP& tvrp = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var))->tourVrp_;
        for(auto u : tvrp.tour_)
        {
            if(setS[u])
                value += p;
        }
        value = floor(value);
        if(SCIPisPositive(scip, value))
        {
            SCIP_CALL(addVarToCons(scip, var, consdata, value));
        }
    }
    return SCIP_OKAY;
}



/**@name Callback methods
 *
 * @{
 */
/** frees specific constraint data */
SCIP_DECL_CONSDELETE(ConshdlrSRC::scip_delete)
{   /*lint --e{715}*/
    assert(conshdlr != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), "SRC") == 0);
    assert(consdata != nullptr);
    assert(*consdata != nullptr);

    /* delete constraint data */
    delete (*consdata)->customerSet;
    delete (*consdata)->lmSet;
    SCIPreleaseRow(scip, &(*consdata)->cut);
    SCIPfreeBlockMemory(scip, consdata);
    SCIPfreeBlockMemoryArray(scip, &(*consdata)->vars, (*consdata)->varssize);
    SCIPfreeBlockMemoryArray(scip, &(*consdata)->coeffs, (*consdata)->varssize);

    return SCIP_OKAY;
}

/** transforms constraint data into data belonging to the transformed problem */
SCIP_DECL_CONSTRANS(ConshdlrSRC::scip_trans)
{   /*lint --e{715}*/
    SCIP_CONSDATA* sourcedata;
    SCIP_CONSDATA* targetdata;

    assert(conshdlr != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), "SRC") == 0);
    assert(SCIPgetStage(scip) == SCIP_STAGE_TRANSFORMING);
    assert(sourcecons != nullptr);
    assert(targetcons != nullptr);

    sourcedata = SCIPconsGetData(sourcecons);
    assert(sourcedata != nullptr);

//    /* create constraint data for target constraint */
//    SCIP_CALL( consdataCreate(scip, &targetdata,
//                              sourcedata->customer, sourcedata->day, sourcedata->type, sourcedata->node) );

    /* create target constraint */
    SCIP_CALL( SCIPcreateCons(scip, targetcons, SCIPconsGetName(sourcecons), conshdlr, targetdata,
                              SCIPconsIsInitial(sourcecons), SCIPconsIsSeparated(sourcecons), SCIPconsIsEnforced(sourcecons),
                              SCIPconsIsChecked(sourcecons), SCIPconsIsPropagated(sourcecons),
                              SCIPconsIsLocal(sourcecons), SCIPconsIsModifiable(sourcecons),
                              SCIPconsIsDynamic(sourcecons), SCIPconsIsRemovable(sourcecons), SCIPconsIsStickingAtNode(sourcecons)) );

    return SCIP_OKAY;
}

SCIP_DECL_CONSCOPY(ConshdlrSRC::scip_copy){
    SCIP_VAR** sourcevars;
    SCIP_Real* coefficients;
    const char* consname;
    SCIP_Real lhs;
    SCIP_Real rhs;
    int nvars;
    SCIP_CONSDATA* consdata = SCIPconsGetData(sourcecons);
    assert(consdata != nullptr);

//    cout << "copy constraint with " << consdata->nvars << " (size: " << consdata->varssize << ")" << endl;

    /* get variables and coefficients of the source constraint */
    sourcevars = consdata->vars;
    coefficients = consdata->coeffs;
    nvars = consdata->nvars;

    lhs = -SCIPinfinity(scip);
    rhs = consdata->rhs;

    if( name != nullptr )
        consname = name;
    else
        consname = SCIPconsGetName(sourcecons);

    /* copy the logic using the linear constraint copy method */
    SCIP_CALL( SCIPcopyConsLinear(scip, cons, sourcescip, consname, nvars, sourcevars, coefficients,
                                  lhs, rhs, varmap, consmap, true, separate, enforce, check, propagate,
                                  local, modifiable, dynamic, removable, stickingatnode, global, valid) );
    assert(cons != nullptr);

    return SCIP_OKAY;
}

SCIP_DECL_CONSENFOLP(ConshdlrSRC::scip_enfolp)
{
//    cout << "SEP " << SCIPnodeGetNumber(SCIPgetFocusNode(scip)) << endl;
    return SCIP_OKAY;
}

SCIP_DECL_CONSSEPALP(ConshdlrSRC::scip_sepalp){
    SCIP_CONSDATA* consdata;
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip));
    model_data* modelData = probData->getData();

    *result = SCIP_DIDNOTFIND;
//    return SCIP_OKAY;

    for(int c = 0; c < nusefulconss; c++)
    {
        consdata = SCIPconsGetData(conss[c]);
        assert(consdata->cut != nullptr);
        if(!SCIProwIsInLP(consdata->cut))
        {
            double feasibility = SCIPgetRowSolFeasibility(scip, consdata->cut, nullptr);
            SCIP_Bool addcut = SCIPisFeasNegative(scip, feasibility);
            if(addcut)
            {
//                SCIPprintCons(scip, conss[c], nullptr);
                SCIPresetConsAge(scip, conss[c]);
                SCIP_CALL( SCIPaddRow(scip, consdata->cut, TRUE, &addcut) );
                *result = SCIP_SEPARATED;
                cout << "happens for SRC!" << endl;
            }
        }
    }

    if(probData->nonzeroVars_.empty())
    {
        SCIP_CALL(SCIPsortNonzeroVars(scip, probData));
    }

    if(*result != SCIP_SEPARATED && SCIPgetDepth(scip) == 0)
    {
        if(SCIPconshdlrGetNConss(SCIPfindConshdlr(scip, "SRC")) > 200)
            return SCIP_OKAY;
        double maxVio = 0.0;
        int numVio = 0;
        int i, j, k;
        vector<double> bestCuts(modelData->nC, 0.0);
        vector<vector<int>> bestPartners(modelData->nC, vector<int>(2));
        for(i = 1; i < modelData->nC; i++)
        {
            for(j = i + 1; j < modelData->nC; j++)
            {
                for(k = j + 1; k < modelData->nC; k++)
                {
                    double checksum = 0.0;
                    for(auto var : probData->nonzeroVars_)
                    {
                        if(!SCIPisPositive(scip, SCIPvarGetLPSol(var)))
                            continue;
                        tourVRP& tvrp = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var))->tourVrp_;
                        double val = 0.0;
                        for(auto u : tvrp.tour_)
                        {
                            if(u == i || u == j || u == k)
                                val++;
                        }
                        checksum += (floor(val / 2) * SCIPvarGetLPSol(var));
                    }

                    if(SCIPisSumPositive(scip, checksum - 1))
                    {
                        if(SCIPisSumPositive(scip, checksum - bestCuts[i]))
                        {
                            bestCuts[i] = checksum;
                            bestPartners[i][0] = j;
                            bestPartners[i][1] = k;
                        }
                        if(SCIPisSumPositive(scip, checksum - bestCuts[j]))
                        {
                            bestCuts[j] = checksum;
                            bestPartners[j][0] = i;
                            bestPartners[j][1] = k;
                        }
                        if(SCIPisSumPositive(scip, checksum - bestCuts[k]))
                        {
                            bestCuts[k] = checksum;
                            bestPartners[k][0] = i;
                            bestPartners[k][1] = j;
                        }
                        numVio++;
                        if(maxVio < checksum - 1)
                            maxVio = checksum - 1;
                    }
                }
            }
        }
//            cout << numVio << " violated cuts!" << endl;
//            cout << "MAX VIOLATION: " << maxVio << endl;
        if(SCIPisGE(scip, maxVio, 0.05))
        {
            numVio = 0;
            for(i = 1; i < modelData->nC; i++)
            {
                if(SCIPisPositive(scip, bestCuts[i]))
                {
                    numVio++;
                    j = bestPartners[i][0];
                    k = bestPartners[i][1];
                    /* create subset row cons */
                    vector<bool> custSet(modelData->nC, false);
                    custSet[i] = true;
                    custSet[bestPartners[i][0]] = true;
                    custSet[bestPartners[i][1]] = true;
                    SCIP_Cons* cons;
                    SCIPcreateConsSRC(scip, &cons, "src", custSet, custSet, 0.5, 1);

                    SCIPaddCons(scip, cons);

                    SCIPcreateAndAddRowSRC(scip, cons);
                    SCIPreleaseCons(scip, &cons);

                    if(bestPartners[j][0] == i && bestPartners[j][1] == k)
                        bestCuts[j] = 0.0;
                    if(bestPartners[k][0] == i && bestPartners[k][1] == j)
                        bestCuts[k] = 0.0;
                }
            }
            if(numVio > 0)
                *result = SCIP_SEPARATED;
//                cout << numVio << " violated ADDED cuts!" << endl;
        }
    }

    return SCIP_OKAY;
}

SCIP_DECL_CONSPRINT(ConshdlrSRC::scip_print)
{
    SCIP_CONSDATA* consdata = SCIPconsGetData(cons);
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip));
    cout << "customers: ";
    for(int i = 0; i < probData->getData()->nC; i++)
    {
        if((*consdata->customerSet)[i])
            cout << i << " ";
    }
    cout << "   parameter: " << consdata->p << endl;
    if(consdata->cut != nullptr)
    {
        SCIPprintRow(scip, consdata->cut, nullptr);
    }else
    {
        cout << "ALERT: no row included!" << endl;
    }
    return SCIP_OKAY;
}

/** global methods */

/** gets the dual solution of the subset row constraint in the current LP */
SCIP_Real SCIPgetDualsolSRC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS*              cons                /**< constraint data */
){
    SCIP_CONSDATA* consdata;

    assert(scip != nullptr);

    if( strcmp(SCIPconshdlrGetName(SCIPconsGetHdlr(cons)), CONSHDLR_NAME4) != 0 )
    {
        SCIPerrorMessage("constraint is not a subset row constraint\n");
        SCIPABORT();
        return SCIP_INVALID;  /*lint !e527*/
    }

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    if( consdata->cut != nullptr )
        return SCIProwGetDualsol(consdata->cut);
    else
        return 0.0;
}

/** gets the dual Farkas value of the subset row constraint in the current infeasible LP */
SCIP_Real SCIPgetDualfarkasSRC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS*              cons                /**< constraint data */
){
    SCIP_CONSDATA* consdata;

    assert(scip != nullptr);

    if( strcmp(SCIPconshdlrGetName(SCIPconsGetHdlr(cons)), CONSHDLR_NAME4) != 0 )
    {
        SCIPerrorMessage("constraint is not a subset row constraint\n");
        SCIPABORT();
        return SCIP_INVALID;  /*lint !e527*/
    }

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    if( consdata->cut != nullptr )
        return SCIProwGetDualfarkas(consdata->cut);
    else
        return 0.0;
}

/** adds coefficient in subset row constraint */
SCIP_RETCODE SCIPaddCoefSRC(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_CONS*            cons,               /**< constraint data */
        SCIP_VAR*             var,                /**< variable to add to the constraint */
        SCIP_Real             val                 /**< coefficient of constraint entry */
)
{
    SCIP_CONSDATA* consdata;
    assert(var != nullptr);

    if( strcmp(SCIPconshdlrGetName(SCIPconsGetHdlr(cons)), CONSHDLR_NAME4) != 0 )
    {
        SCIPerrorMessage("constraint is not a subset row constraint\n");
        return SCIP_INVALIDDATA;
    }


    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);
    assert(consdata->cut != nullptr);
    SCIP_CALL(addVarToCons(scip, var, consdata, val));

    return SCIP_OKAY;
}

/** creates and adds the row of a subset row constraint to the LP */
SCIP_RETCODE SCIPcreateAndAddRowSRC(
        SCIP*                   scip,
        SCIP_Cons*              cons
){
    SCIP_CONSDATA* consdata;
    assert(scip != nullptr);
    assert(cons != nullptr);
    SCIP_Bool infeasible;
    consdata = SCIPconsGetData(cons);
    assert(consdata->cut == nullptr);

    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &(consdata->cut), cons, SCIPconsGetName(cons), -SCIPinfinity(scip),
                                     consdata->rhs, FALSE, TRUE, FALSE));

    assert(consdata->cut != nullptr);

    SCIP_CALL(addVarsToCons(scip, *consdata->customerSet, consdata->p, consdata));

    if(!SCIProwIsInLP(consdata->cut))
    {
        SCIP_CALL(SCIPaddRow(scip, consdata->cut, true, &infeasible));
        assert(!infeasible);
    }

    return SCIP_OKAY;
}

/** creates and captures a subset row constraint */
SCIP_RETCODE SCIPcreateConsSRC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS**             cons,               /**< pointer to hold the created constraint */
        const char*             name,               /**< name of constraint */
        vector<bool>&           setS,               /**< set of customers */
        vector<bool>&           lmSet,              /**< limited memory */ // TODO: calculate here
        double                  p,                  /**< parameter p */
        double                  rhs                 /**< right hand side */
){
    SCIP_CONSDATA* consdata = nullptr;
    SCIP_CONSHDLR* conshdlr = nullptr;

    assert(scip != nullptr);

    /* find the set partitioning constraint handler */
    conshdlr = SCIPfindConshdlr(scip, CONSHDLR_NAME4);
    assert(conshdlr != nullptr);
    /* create constraint data*/
    SCIP_CALL(consdataCreate(scip, &consdata, setS, lmSet, p, rhs));
    assert(consdata != nullptr);
    /* create constraint */
    SCIP_CALL(SCIPcreateCons(scip, cons, name, conshdlr, consdata,FALSE, TRUE, TRUE, FALSE,
                             FALSE, FALSE, TRUE, FALSE, FALSE, FALSE));

    return SCIP_OKAY;
}

/** returns the Set S of a subset row constraint */
vector<bool>* SCIPgetSetOfSRC(
        SCIP_CONS*              cons
){
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->customerSet;
}

/** returns the parameter p of a subset row constraint */
double SCIPgetpOfSRC(
        SCIP_CONS*              cons
){
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->p;
}

/** returns the row of a subset row constraint */
SCIP_Row* SCIPgetRowOfSRC(
        SCIP_CONS*              cons
){
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->cut;
}