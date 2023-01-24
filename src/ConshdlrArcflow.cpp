
#include "iostream"

#include "ConshdlrArcflow.h"
#include "objscip/objscip.h"
#include "branchingrule_arcflow.h"
#include "model_data.h"
#include "probdata_vrp.h"
#include "scip/scip.h"
#include "vardata.h"

/*
 * Data structures
 */

/** Constraint data for "ArcFlow" constraints */
struct SCIP_ConsData
{
    int                   tail;               /**< tail of the arc */
    int                   head;               /**< head of the arc */
    CONSTYPE              type;               /**< stores whether arc gets enforced or prohibited */
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
        int                   tail,               /**< tail of the arc */
        int                   head,               /**< head of the arc */
        CONSTYPE              type,               /**< stores whether arc gets enforced or prohibited */
        SCIP_NODE*            node                /**< the node in the B&B-tree at which the cons is sticking */
)
{
    assert( scip != nullptr );
    assert( consdata != nullptr );
    assert( tail >= 0 );
    assert( head >= 0 );
    assert( type == ENFORCE || type == PROHIBIT );

    SCIP_CALL( SCIPallocBlockMemory(scip, consdata) );

    (*consdata)->tail = tail;
    (*consdata)->head = head;
    (*consdata)->type = type;
    (*consdata)->npropagatedvars = 0;
    (*consdata)->npropagations = 0;
    (*consdata)->propagated = FALSE;
    (*consdata)->node = node;

    return SCIP_OKAY;
}

/** checks the relation between an arc and a var */
static
int arcInVar(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_VAR*             var,                /**< variable to check */
        int                   tail,               /**< tail of arc to check */
        int                   head                /**< head of arc to check */
)
{
    int j;
    bool getsUsed = false;          /* number of times the arc gets used in the tour */
    bool onlyOne = false;    /* if tail or head get visited but not successively */
    auto* varData = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));

    if(varData->getLength() <= 0)
    {
        return 0;
    }

    /** Possible cases for tail = u, head = v and w != u, w != v:
     *  1. u, v both not in tour
     *  2. tour = ... -> u -> w -> ...
     *  3. tour = ... -> w -> v -> ...
     *  4. tour = ... -> u -> v -> ...
     **/

    /* special case head/tail = depot */
    if(tail == 0)
    {
        if(varData->tour_[0] == head)
            getsUsed = true;
    }else if(varData->tour_[0] == head) // case 3
    {
        onlyOne = true;
    }

    if(head == 0)
    {
        if(varData->tour_[varData->getLength() - 1] == tail)
            getsUsed = true;
    }else if(varData->tour_[varData->getLength() - 1] == tail) // case 2
    {
        onlyOne = true;
    }

    for(j = 0; j < varData->getLength() - 1; j++)
    {
        if(j != 0 && varData->tour_[j] == head) // case 3
            onlyOne = true;
        if(varData->tour_[j] == tail)
        {
            if(varData->tour_[j+1] != head) // case 2
            {
                onlyOne = true;
            }
            else // case 4
            {
                getsUsed = true;
            }
            /* we can skip the next spot because we already checked it */
            j++;
        }
    }
    if(j == varData->getLength() - 1 && varData->tour_[varData->getLength() - 1] == head) // case 3
        onlyOne = true;

    if(onlyOne)
    {
        if(getsUsed) /* both not successively and successively */
        {
            return 3;
        }else /* visited but not successively */
        {
            return 1;
        }
    }else
    {
        if(getsUsed) /* only visited successively */
        {
            return 2;
        }else /* neither head nor tail visited */
        {
            return 0;
        }
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
    arcStatus = arcInVar(scip, var, consdata->tail, consdata->head);

    /**
     *  arcStatus = 0 (tail and head both not visited by tour) -> variable allowed
     *  arcStatus = 1 (at least on of them is visited but not the arc itself) -> variable prohibited if arc gets enforced
     *  arcStatus = 2 (arc is used) -> variable prohibited if arc gets prohibited
     *  arcStatus = 3 (arc is used) AND (at least on of them is visited but not the arc itself) -> variable prohibited
     **/
    if(arcStatus == 0) return SCIP_OKAY;
    type = consdata->type;
    if( (type == PROHIBIT && arcStatus == 2) || (type == ENFORCE && arcStatus == 1) || arcStatus == 3 )
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

        arcStatus = arcInVar(scip, var, consdata->tail, consdata->head);

        if(arcStatus == 0) return SCIP_OKAY;

        type = consdata->type;
        if( (type == PROHIBIT && arcStatus == 2) || (type == ENFORCE && arcStatus == 1) || arcStatus == 3)
        {
            SCIPdebug( consdataPrint(scip, consdata, NULL) );
            SCIPdebug( SCIPprintVar(scip, var, NULL) );
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
SCIP_DECL_CONSDELETE(ConshdlrArcflow::scip_delete)
{   /*lint --e{715}*/
    assert(conshdlr != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    assert(consdata != nullptr);
    assert(*consdata != nullptr);

    /* delete constraint data */
    SCIPfreeBlockMemory(scip, consdata);

    return SCIP_OKAY;
}

/** transforms constraint data into data belonging to the transformed problem */
SCIP_DECL_CONSTRANS(ConshdlrArcflow::scip_trans)
{   /*lint --e{715}*/
    SCIP_CONSDATA* sourcedata;
    SCIP_CONSDATA* targetdata;

    assert(conshdlr != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    assert(SCIPgetStage(scip) == SCIP_STAGE_TRANSFORMING);
    assert(sourcecons != nullptr);
    assert(targetcons != nullptr);

    sourcedata = SCIPconsGetData(sourcecons);
    assert(sourcedata != nullptr);

    /* create constraint data for target constraint */
    SCIP_CALL( consdataCreate(scip, &targetdata,
                              sourcedata->tail, sourcedata->head, sourcedata->type, sourcedata->node) );

    /* create target constraint */
    SCIP_CALL( SCIPcreateCons(scip, targetcons, SCIPconsGetName(sourcecons), conshdlr, targetdata,
                              SCIPconsIsInitial(sourcecons), SCIPconsIsSeparated(sourcecons), SCIPconsIsEnforced(sourcecons),
                              SCIPconsIsChecked(sourcecons), SCIPconsIsPropagated(sourcecons),
                              SCIPconsIsLocal(sourcecons), SCIPconsIsModifiable(sourcecons),
                              SCIPconsIsDynamic(sourcecons), SCIPconsIsRemovable(sourcecons), SCIPconsIsStickingAtNode(sourcecons)) );

    return SCIP_OKAY;
}

/** domain propagation method of constraint handler */
SCIP_DECL_CONSPROP(ConshdlrArcflow::scip_prop)
{  /*lint --e{715}*/
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    SCIP_CONSDATA* consdata;

    int c;

    assert(scip != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
    assert(result != nullptr);

    SCIPdebugMsg(scip, "propagation constraints of constraint handler <" CONSHDLR_NAME">\n");

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
                assert( !(consdata->tail != 0 && consdata->tail == consdata2->tail && consdata->type == ENFORCE && consdata2->type == ENFORCE));
                assert( !(consdata->head != 0 && consdata->head == consdata2->head && consdata->type == ENFORCE && consdata2->type == ENFORCE));
                assert( !(consdata->tail == consdata2->tail && consdata->head == consdata2->head) );
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
SCIP_DECL_CONSACTIVE(ConshdlrArcflow::scip_active)
{   /*lint --e{715}*/
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    SCIP_CONSDATA* consdata;

    assert(scip != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
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
SCIP_DECL_CONSDEACTIVE(ConshdlrArcflow::scip_deactive)
{   /*lint --e{715}*/
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    SCIP_CONSDATA* consdata;

    assert(scip != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME) == 0);
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
SCIP_DECL_CONSPRINT(ConshdlrArcflow::scip_print)
{  /*lint --e{715}*/
    SCIP_CONSDATA*  consdata;

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    SCIPinfoMessage(scip, file, "%s(%d,%d) at node %lld\n",
                    consdata->type == PROHIBIT ? "prohibit" : "enforce",
                    consdata->tail, consdata->head, SCIPnodeGetNumber(consdata->node) );
    return SCIP_OKAY;
}

/**@} */

/**@name Interface methods
 *
 * @{
 */

/** creates arc flow constraint */
SCIP_RETCODE SCIPcreateConsArcFlow(
        SCIP*               scip,                /**< SCIP data structure */
        SCIP_CONS**         cons,                /**< pointer to hold the created constraint */
        const char*         name,                /**< name of the constraint */
        int                 tail,                /**< tail of the arc */
        int                 head,                /**< head of the arc */
        CONSTYPE            type,                /**< stores whether arc gets enforced or prohibited */
        SCIP_NODE*          node,                /**< the node in the B&B-tree at which the cons is sticking */
        SCIP_Bool           local                /**< is constraint only valid locally? */
){
    SCIP_CONSHDLR* conshdlr;
    SCIP_CONSDATA* consdata;

    /* find the arcflow constraint handler */
    conshdlr = SCIPfindConshdlr(scip, CONSHDLR_NAME);
    if( conshdlr == nullptr )
    {
        SCIPerrorMessage("arcflow constraint handler not found\n");
        return SCIP_PLUGINNOTFOUND;
    }

    /* create the constraint specific data */
    SCIP_CALL( consdataCreate(scip, &consdata, tail, head, type, node) );

    /* create constraint */
    SCIP_CALL( SCIPcreateCons(scip, cons, name, conshdlr, consdata, FALSE, FALSE, FALSE, FALSE, TRUE,
                              local, FALSE, FALSE, FALSE, TRUE) );

    SCIPdebugMsg(scip, "created constraint: ");
    SCIPdebug( consdataPrint(scip, consdata, nullptr) );

    return SCIP_OKAY;
}

/** returns tail of the arc */
int SCIPgetTailArcflow(
        SCIP_CONS*            cons                /**< arcflow constraint */
)
{
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->tail;
}

/** returns head of the arc */
int SCIPgetHeadArcflow(
        SCIP_CONS*            cons                /**< arcflow constraint */
)
{
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->head;
}

/** return constraint type PROHIBIT or ENFORCE */
CONSTYPE SCIPgetTypeArcflow(
        SCIP_CONS*            cons                /**< arcflow constraint */
)
{
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->type;
}