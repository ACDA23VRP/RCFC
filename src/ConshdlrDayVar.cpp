
#include "iostream"

#include "ConshdlrDayVar.h"
#include "objscip/objscip.h"
#include "branchingrule_arcflow.h"
#include "model_data.h"
#include "probdata_vrp.h"
#include "scip/scip.h"
#include "vardata.h"

/*
 * Data structures
 */

/** Constraint data for "DayVar" constraints */
struct SCIP_ConsData
{
    int                   customer;           /**< corresponding customer */
    int                   day;                /**< corresponding day */
    CONSTYPE              type;               /**< stores whether customers gets enforced or prohibited on that day */
    int                   npropagatedvars;    /**< number of variables that existed, the last time, the related node was
                                              *   propagated, used to determine whether the constraint should be
                                              *   repropagated*/
    int                   npropagations;      /**< stores the number propagations runs of this constraint */
    unsigned int          propagated:1;       /**< is constraint already propagated? */
    SCIP_NODE*            node;               /**< the node in the B&B-tree at which the cons is sticking */
};

/**@name Local methods
 *
 * @{
 */

/** create constraint data */
static
SCIP_RETCODE consdataCreate(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_CONSDATA**       consdata,           /**< pointer to store the constraint data */
        int                   customer,           /**< corresponding customer */
        int                   day,                /**< corresponding day */
        CONSTYPE              type,               /**< stores whether arc gets enforced or prohibited */
        SCIP_NODE*            node                /**< the node in the B&B-tree at which the cons is sticking */
)
{
    assert( scip != nullptr );
    assert( consdata != nullptr );
    assert( customer >= 0 );
    assert( day >= 0 );
    assert( type == ENFORCE || type == PROHIBIT );


    SCIP_CALL( SCIPallocBlockMemory(scip, consdata) );

    (*consdata)->customer = customer;
    (*consdata)->day = day;
    (*consdata)->type = type;
    (*consdata)->npropagatedvars = 0;
    (*consdata)->npropagations = 0;
    (*consdata)->propagated = FALSE;
    (*consdata)->node = node;

    return SCIP_OKAY;
}

/** Checks if (a): customer lies in the tour of the variable
 *            (b): tour takes place on the given day
 * @return  0   if (a) or  (b) is FALSE
 *          1   if (a) xor (b) is TRUE
 *          2   if (a) and (b) is TRUE
 * */
static
int customerAndDayInVar(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_VAR*             var,                /**< variable to check */
        int                   customer,           /**< customer to check */
        int                   day                 /**< day to check */
)
{
    auto* varData = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));
    assert(varData != nullptr);

    for(int u : varData->tour_)
    {
        assert(u > 0);
        if(u == customer) // case 2
        {
            if(varData->getDay() == day)
            {
                return 2;
            }
            else
            {
                return 1;
            }
        }
    }
    if(varData->getDay() == day)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/** fixes a variable to zero if the corresponding arcs are not valid for this constraint/node (due to branching) */
static
SCIP_RETCODE checkVariable(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_CONSDATA*        consdata,           /**< constraint data */
        SCIP_VAR*             var,                /**< variables to check  */
        int*                  nfixedvars,         /**< pointer to store the number of fixed variables */
        SCIP_Bool*            cutoff              /**< pointer to store if a cutoff was detected */
)
{
    int arcStatus;
    CONSTYPE type;

    SCIP_Bool fixed;
    SCIP_Bool infeasible;

    assert(scip != nullptr);
    assert(consdata != nullptr);
    assert(var != nullptr);
    assert(nfixedvars != nullptr);
    assert(cutoff != nullptr);

    /* if variables is locally fixed to zero continue */
    if( SCIPvarGetUbLocal(var) < 0.5 )
        return SCIP_OKAY;

    /* check if the tour which corresponds to the variable is feasible for this constraint */
    arcStatus = customerAndDayInVar(scip, var, consdata->customer, consdata->day);

    /** arcStatus = 0 (neither the day nor the customer fit to the variable) -> variable allowed
     *  arcStatus = 1 (either the day or the customer fit to the variable) -> variable 0-fixed if customers gets enforced on that day
     *  arcStatus = 2 (both the day and the customer fit to the variable) -> variable 0-fixed if customers gets prohibited for that day */

    if(arcStatus == 0) return SCIP_OKAY;
    type = consdata->type;
    if( (type == PROHIBIT && arcStatus == 2) || (type == ENFORCE && arcStatus == 1) )
    {
        SCIP_CALL( SCIPfixVar(scip, var, 0.0, &infeasible, &fixed) );

        if( infeasible )
        {
            assert( SCIPvarGetLbLocal(var) > 0.5 );
            SCIPdebugMsg(scip, "-> cutoff\n");
            (*cutoff) = TRUE;
        }
        else
        {
            assert(fixed);
            (*nfixedvars)++;
        }
    }

    return SCIP_OKAY;
}

/** fixes variables to zero if the corresponding tours are not valid for this constraint/node (due to branching) */
static
SCIP_RETCODE consdataFixVariables(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_CONSDATA*        consdata,           /**< constraint data */
        vector<SCIP_VAR*>&    vars,               /**< generated variables */
        int                   nvars,              /**< number of generated variables */
        SCIP_RESULT*          result              /**< pointer to store the result of the fixing */
)
{
    int nfixedvars;
    int v;
    SCIP_Bool cutoff;

    nfixedvars = 0;
    cutoff = FALSE;

    SCIPdebugMsg(scip, "check variables %d to %d\n", consdata->npropagatedvars, nvars);

    for( v = consdata->npropagatedvars; v < nvars && !cutoff; ++v )
    {
        SCIP_CALL( checkVariable(scip, consdata, vars[v], &nfixedvars, &cutoff) );
    }

    SCIPdebugMsg(scip, "fixed %d variables locally\n", nfixedvars);

    if( cutoff )
        *result = SCIP_CUTOFF;
    else if( nfixedvars > 0 )
        *result = SCIP_REDUCEDDOM;

    return SCIP_OKAY;
}

/** check if all variables are valid for the given consdata */
#ifndef NDEBUG
static
SCIP_Bool consdataCheck(
        SCIP*                 scip,               /**< SCIP data structure */
        vrp::ProbDataVRP*     probdata,           /**< problem data */
        SCIP_CONSDATA*        consdata,           /**< constraint data */
        SCIP_Bool             beforeprop          /**< is this check performed before propagation? */
)
{
    vector<SCIP_VAR*> vars = probdata->vars_;
    int nvars;
    SCIP_VAR* var;

    int arcStatus;
    CONSTYPE type;

    int v;
    nvars = (beforeprop ? consdata->npropagatedvars : probdata->nVars_);
    assert(nvars <= probdata->nVars_);

    for( v = 0; v < nvars; ++v )
    {
        var = vars[v];

        /* if variables is locally fixed to zero continue */
        if( SCIPvarGetUbLocal(var) < 0.5 )
            continue;

        arcStatus = customerAndDayInVar(scip, var, consdata->customer, consdata->day);

        if(arcStatus == 0) return SCIP_OKAY;

        type = consdata->type;
        if( (type == PROHIBIT && arcStatus == 2) || (type == ENFORCE && arcStatus == 1) )
        {
            SCIPdebug( consdataPrint(scip, consdata, NULL) );
            SCIPdebug( SCIPprintVar(scip, var, NULL) );
            cout << "Customer: " << consdata->customer << ", day: " << consdata->day << endl;
            SCIPprintVar(scip, var, nullptr);
            assert(false);
            return FALSE;
        }
    }

    return TRUE;
}
#endif

/**@} */

/**@name Callback methods
 *
 * @{
 */

/** frees specific constraint data */
SCIP_DECL_CONSDELETE(ConshdlrDayVar::scip_delete)
{   /*lint --e{715}*/
    assert(conshdlr != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME2) == 0);
    assert(consdata != nullptr);
    assert(*consdata != nullptr);

    /* delete constraint data */
    SCIPfreeBlockMemory(scip, consdata);

    return SCIP_OKAY;
}


/** transforms constraint data into data belonging to the transformed problem */
SCIP_DECL_CONSTRANS(ConshdlrDayVar::scip_trans)
{   /*lint --e{715}*/
    SCIP_CONSDATA* sourcedata;
    SCIP_CONSDATA* targetdata;

    assert(conshdlr != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME2) == 0);
    assert(SCIPgetStage(scip) == SCIP_STAGE_TRANSFORMING);
    assert(sourcecons != nullptr);
    assert(targetcons != nullptr);

    sourcedata = SCIPconsGetData(sourcecons);
    assert(sourcedata != nullptr);

    /* create constraint data for target constraint */
    SCIP_CALL( consdataCreate(scip, &targetdata,
                              sourcedata->customer, sourcedata->day, sourcedata->type, sourcedata->node) );

    /* create target constraint */
    SCIP_CALL( SCIPcreateCons(scip, targetcons, SCIPconsGetName(sourcecons), conshdlr, targetdata,
                              SCIPconsIsInitial(sourcecons), SCIPconsIsSeparated(sourcecons), SCIPconsIsEnforced(sourcecons),
                              SCIPconsIsChecked(sourcecons), SCIPconsIsPropagated(sourcecons),
                              SCIPconsIsLocal(sourcecons), SCIPconsIsModifiable(sourcecons),
                              SCIPconsIsDynamic(sourcecons), SCIPconsIsRemovable(sourcecons), SCIPconsIsStickingAtNode(sourcecons)) );

    return SCIP_OKAY;
}

/** domain propagation method of constraint handler */
SCIP_DECL_CONSPROP(ConshdlrDayVar::scip_prop)
{  /*lint --e{715}*/
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    SCIP_CONSDATA* consdata;

    int c;

    assert(scip != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME2) == 0);
    assert(result != nullptr);

    SCIPdebugMsg(scip, "propagation constraints of constraint handler <" CONSHDLR_NAME2">\n");

    assert(probData != nullptr);

    *result = SCIP_DIDNOTFIND;

    for( c = 0; c < nconss; ++c )
    {
        consdata = SCIPconsGetData(conss[c]);

        /* check if all previously generated variables are valid for this constraint */
        assert( consdataCheck(scip, probData, consdata, TRUE) );

#ifndef NDEBUG
        {
            /* check if there are no equal consdatas */
            SCIP_CONSDATA* consdata2;
            int i;

            for( i = c+1; i < nconss; ++i )
            {
                consdata2 = SCIPconsGetData(conss[i]);

                assert( !(consdata->customer == consdata2->customer && consdata->type == ENFORCE && consdata2->type == ENFORCE));
                if(consdata->customer == consdata2->customer && consdata->day == consdata2->day)
                {
                    cout << "customer "<<consdata->customer<<" day "<<consdata->day<< endl;
                    cout << "parent: " <<SCIPnodeGetNumber(SCIPnodeGetParent(SCIPgetCurrentNode(scip))) << endl;
                    cout << "type1: " << consdata->type << " vs type2: " << consdata2->type << endl;
                }
                assert( !(consdata->customer == consdata2->customer && consdata->day == consdata2->day) );
            }
        }
#endif
        if( !consdata->propagated )
        {
            SCIPdebugMsg(scip, "propagate constraint <%s> ", SCIPconsGetName(conss[c]));
            SCIPdebug( consdataPrint(scip, consdata, nullptr) );
            SCIP_CALL( consdataFixVariables(scip, consdata, probData->vars_, probData->nVars_, result) );
            consdata->npropagations++;

            if( *result != SCIP_CUTOFF )
            {
                consdata->propagated = TRUE;
                consdata->npropagatedvars = probData->nVars_;
            }
            else
                break;
        }

        /* check if constraint is completely propagated */
        assert( consdataCheck(scip, probData, consdata, FALSE) );
    }

    return SCIP_OKAY;
}

/** constraint activation notification method of constraint handler */
SCIP_DECL_CONSACTIVE(ConshdlrDayVar::scip_active)
{   /*lint --e{715}*/
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    SCIP_CONSDATA* consdata;

    assert(scip != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME2) == 0);
    assert(cons != nullptr);

    assert(probData != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);
    assert(consdata->npropagatedvars <= probData->nVars_);

    SCIPdebugMsg(scip, "activate constraint <%s> at node <%" SCIP_LONGINT_FORMAT"> in depth <%d>: ",
                 SCIPconsGetName(cons), SCIPnodeGetNumber(consdata->node), SCIPnodeGetDepth(consdata->node));
    SCIPdebug( consdataPrint(scip, consdata, nullptr) );

    if( consdata->npropagatedvars != probData->nVars_ )
    {
        SCIPdebugMsg(scip, "-> mark constraint to be repropagated\n");
        consdata->propagated = FALSE;
        SCIP_CALL( SCIPrepropagateNode(scip, consdata->node) );
    }

    return SCIP_OKAY;
}

/** constraint deactivation notification method of constraint handler */
SCIP_DECL_CONSDEACTIVE(ConshdlrDayVar::scip_deactive)
{   /*lint --e{715}*/
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    SCIP_CONSDATA* consdata;

    assert(scip != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME2) == 0);
    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);
    assert(consdata->propagated || SCIPgetNChildren(scip) == 0);

    assert(probData != nullptr);

    SCIPdebugMsg(scip, "deactivate constraint <%s> at node <%" SCIP_LONGINT_FORMAT"> in depth <%d>: ",
                 SCIPconsGetName(cons), SCIPnodeGetNumber(consdata->node), SCIPnodeGetDepth(consdata->node));
    SCIPdebug( consdataPrint(scip, consdata, nullptr) );

    /* set the number of propagated variables to current number of variables is SCIP */
    consdata->npropagatedvars = probData->nVars_;

    return SCIP_OKAY;
}

/** constraint display method of constraint handler */
SCIP_DECL_CONSPRINT(ConshdlrDayVar::scip_print)
{  /*lint --e{715}*/
    SCIP_CONSDATA*  consdata;

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    SCIPinfoMessage(scip, file, "%s(%d,%d) at node %lld\n",
                    consdata->type == PROHIBIT ? "prohibit" : "enforce",
                    consdata->customer, consdata->day, SCIPnodeGetNumber(consdata->node) );
    return SCIP_OKAY;
}

/**@} */

/**@name Interface methods
 *
 * @{
 */

/** creates day var constraint */
SCIP_RETCODE SCIPcreateConsDayVar(
        SCIP*               scip,                /**< SCIP data structure */
        SCIP_CONS**         cons,                /**< pointer to hold the created constraint */
        const char*         name,                /**< name of the constraint */
        int                 customer,            /**< corresponding customer */
        int                 day,                 /**< corresponding day */
        CONSTYPE            type,                /**< stores whether arc gets enforced or prohibited */
        SCIP_NODE*          node,                /**< the node in the B&B-tree at which the cons is sticking */
        SCIP_Bool           local                /**< is constraint only valid locally? */
){
    SCIP_CONSHDLR* conshdlr;
    SCIP_CONSDATA* consdata;

    /* find the dayVar constraint handler */
    conshdlr = SCIPfindConshdlr(scip, CONSHDLR_NAME2);
    if( conshdlr == nullptr )
    {
        SCIPerrorMessage("dayVar constraint handler not found\n");
        return SCIP_PLUGINNOTFOUND;
    }

    /* create the constraint specific data */
    SCIP_CALL( consdataCreate(scip, &consdata, customer, day, type, node) );

    /* create constraint */
    SCIP_CALL( SCIPcreateCons(scip, cons, name, conshdlr, consdata, FALSE, FALSE, FALSE, FALSE, TRUE,
                              local, FALSE, FALSE, FALSE, TRUE) );

    SCIPdebugMsg(scip, "created constraint: ");
    SCIPdebug( consdataPrint(scip, consdata, nullptr) );

    return SCIP_OKAY;
}


/** return customer of day constraint */
int SCIPgetCustomerOfDayCons(
        SCIP_CONS*            cons                /**< day constraint */
)
{
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->customer;
}

/** returns day of day constraint */
int SCIPgetDayOfDayCons(
        SCIP_CONS*            cons                /**< day constraint */
)
{
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->day;
}

/** return constraint type PROHIBIT or ENFORCE */
CONSTYPE SCIPgetTypeOfDayCons(
        SCIP_CONS*            cons                /**< day constraint */
)
{
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->type;
}

/** propagates constraints that got added during the pricing step */
SCIP_RETCODE SCIPconshdlrPerformPropagate(
        SCIP*               scip,
        SCIP_CONSHDLR*      conshdlr
){
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip));
    SCIP_CONSDATA* consdata;
    SCIP_RESULT result;
    SCIP_CONS** conss = SCIPconshdlrGetConss(conshdlr);
    int nconss = SCIPconshdlrGetNConss(conshdlr);
    int c;
    assert(probData != nullptr);
    assert(scip != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME2) == 0);

    SCIPdebugMsg(scip, "propagation additional constraints of constraint handler <" CONSHDLR_NAME2">\n");

    for( c = 0; c < nconss; ++c )
    {
        consdata = SCIPconsGetData(conss[c]);
        if(!SCIPconsIsActive(conss[c]))
            continue;
        if(!consdata->propagated)
        {
            SCIP_CALL( consdataFixVariables(scip, consdata, probData->vars_, probData->nVars_, &result) );
            consdata->npropagations++;
            consdata->propagated = TRUE;
            consdata->npropagatedvars = probData->nVars_;

            assert(result != SCIP_CUTOFF);
        }
    }
    return SCIP_OKAY;
}