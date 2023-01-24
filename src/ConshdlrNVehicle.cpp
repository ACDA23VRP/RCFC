
#include <iostream>

#include "ConshdlrNVehicle.h"
#include "objscip/objscip.h"

#include "probdata_vrp.h"
#include "var_tools.h"

#define CONSHDLR_NAME5   "nVehicle"

/** Constraint data for "nVehicle" constraints */
struct SCIP_ConsData
{
    double              numV;               /**< Set S of customers defining the lhs */
    CONSTYPE            type;               /**< min. Number of paths needed to serve S */
    SCIP_Row*           cut;                /**< Corresponding row in the LP */
    int                 npropagatedvars;    /**< number of variables that existed, the last time, the related node was
                                              *   propagated, used to determine whether the constraint should be
                                              *   repropagated*/
    int                 npropagations;      /**< stores the number propagations runs of this constraint */
    unsigned int        propagated:1;       /**< is constraint already propagated? */
    SCIP_NODE*          node;               /**< the node in the B&B-tree at which the cons is sticking */
};

/** local methods */
static
SCIP_RETCODE consdataCreate(
    SCIP*           scip,
    SCIP_CONSDATA** consdata,
    double          numV,
    CONSTYPE        type,
    SCIP_Node*      node
){
    SCIP_CALL(SCIPallocBlockMemory(scip, consdata));

    (*consdata)->numV = numV;
    (*consdata)->type = type;
    (*consdata)->node = node;
    (*consdata)->npropagatedvars = 0;
    (*consdata)->npropagations = 0;
    (*consdata)->propagated = FALSE;
    (*consdata)->cut = nullptr;

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

    int v;
    nvars = (beforeprop ? consdata->npropagatedvars : probdata->nVars_);
    assert(nvars <= probdata->nVars_);

    for( v = 0; v < nvars; ++v )
    {
        var = vars[v];

        /* if variables is locally fixed to zero continue */
        if( SCIPvarGetUbLocal(var) < 0.5 )
            continue;

        /* check if variable lies in row */
        if(!varInRow(var, consdata->cut))
        {
            cout << "INVALID!!" << endl;
            SCIPprintRow(scip, consdata->cut, nullptr);
            SCIPprintVar(scip, var, nullptr);
            cout << consdata->cut << endl;
            cout << var << endl;
            cout << SCIPnodeGetNumber(SCIPgetCurrentNode(scip)) << endl;
            cout << SCIPnodeGetNumber(SCIPnodeGetParent(SCIPgetCurrentNode(scip))) << endl;

            return FALSE;
        }
    }

    return TRUE;
}
#endif

/** adds variables to row of a nVehicle constraint */
static
SCIP_RETCODE addVarsToRow(
        SCIP*               scip,
        SCIP_Row*           cut
){
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip));
    for(auto var : probData->vars_)
    {
        SCIP_CALL(SCIPaddVarToRow(scip, cut, var, 1.0));
    }
    return SCIP_OKAY;
}

/** creates and adds the row of a nVehicle constraint to the LP */
static
SCIP_RETCODE SCIPcreateAndAddRowNVehicle(
        SCIP*                   scip,
        SCIP_Cons*              cons
){
    SCIP_CONSDATA* consdata;
    assert(scip != nullptr);
    assert(cons != nullptr);
    SCIP_Bool infeasible;
    consdata = SCIPconsGetData(cons);
    assert(consdata->cut == nullptr);

    if(consdata->type == PROHIBIT)
    {
        SCIP_CALL(SCIPcreateEmptyRowCons(scip, &(consdata->cut), cons, SCIPconsGetName(cons), -SCIPinfinity(scip),
                                         consdata->numV,TRUE, TRUE, FALSE));
    }else
    {
        SCIP_CALL(SCIPcreateEmptyRowCons(scip, &(consdata->cut), cons, SCIPconsGetName(cons), consdata->numV,
                                         SCIPinfinity(scip),TRUE, TRUE, FALSE));
    }

    assert(consdata->cut != nullptr);

    SCIP_CALL(addVarsToRow(scip, consdata->cut));

    if(!SCIProwIsInLP(consdata->cut))
    {
        SCIP_CALL(SCIPaddRow(scip, consdata->cut, true, &infeasible));
        assert(!infeasible);
    }

    return SCIP_OKAY;
}

/** checks if constraint is violated */
static
bool isNVehicleConsViolated(
        SCIP*               scip,
        vrp::ProbDataVRP*   probData,
        SCIP_CONSDATA*      consdata
){
    assert(consdata->cut != nullptr);
    double value = 0.0;
    for(auto* var : probData->vars_)
    {
        value += SCIPvarGetLPSol(var);
    }
    if(consdata->type == ENFORCE)
    {
        if(!SCIPisGE(scip, value, consdata->numV))
        {
            cout << "value: " << value << " vs: " << consdata->numV << endl;
            return true;
        }
    }else if(consdata->type == PROHIBIT)
    {
        if(!SCIPisLE(scip, value, consdata->numV))
        {
            cout << "value2: " << value << " vs: " << consdata->numV << endl;
            return true;
        }
    }
    return false;
}



/**@name Callback methods
 *
 * @{
 */
/** frees specific constraint data */
SCIP_DECL_CONSDELETE(ConshdlrNVehicle::scip_delete)
{   /*lint --e{715}*/
    assert(conshdlr != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), "nVehicle") == 0);
    assert(consdata != nullptr);
    assert(*consdata != nullptr);

    /* delete constraint data */
    if((*consdata)->cut != nullptr)
        SCIPreleaseRow(scip, &(*consdata)->cut);
    SCIPfreeBlockMemory(scip, consdata);

    return SCIP_OKAY;
}

/** transforms constraint data into data belonging to the transformed problem */
SCIP_DECL_CONSTRANS(ConshdlrNVehicle::scip_trans)
{   /*lint --e{715}*/
    SCIP_CONSDATA* sourcedata;
    SCIP_CONSDATA* targetdata;

    assert(conshdlr != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), "nVehicle") == 0);
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

/** separation method of constraint handler for LP solution */
SCIP_DECL_CONSSEPALP(ConshdlrNVehicle::scip_sepalp)
{
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    SCIP_CONSDATA* consdata;
    int c;
    int nrounds;
    ConshdlrNVehicle* objcons;

    assert(scip != nullptr);
    assert(conshdlr != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME5) == 0);
    assert(result != nullptr);
    assert(probData != nullptr);

    SCIPdebugMsg(scip, "separation constraints of constraint handler <" CONSHDLR_NAME5">\n");

    nrounds = SCIPgetNSepaRounds(scip);
    objcons = dynamic_cast<ConshdlrNVehicle *>(SCIPgetObjConshdlr(scip, conshdlr));

    *result = SCIP_DIDNOTFIND;
    if(nrounds >= objcons->maxseparounds_)
        return SCIP_OKAY;

    assert(nconss == nusefulconss); // TODO: does it happen?
    for( c = 0; c < nconss; c++)
    {
        consdata = SCIPconsGetData(conss[c]);

        assert(SCIPconsIsActive(conss[c]));

        if(consdata->cut == nullptr)
        {
            *result = SCIP_SEPARATED;
            SCIP_CALL(SCIPcreateAndAddRowNVehicle(scip, conss[c]));
            consdata->npropagatedvars = probData->nVars_;
        }else
        {
            if(isNVehicleConsViolated(scip, probData, consdata))
            {
                assert(false);
            }
        }
    }
    return SCIP_OKAY;
}

/** domain propagation method of constraint handler */
SCIP_DECL_CONSPROP(ConshdlrNVehicle::scip_prop)
{
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    SCIP_CONSDATA* consdata;

    int c;
    assert(scip != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME5) == 0);
    assert(result != nullptr);

    SCIPdebugMsg(scip, "propagation constraints of constraint handler <" CONSHDLR_NAME5">\n");

    assert(probData != nullptr);
    *result = SCIP_DIDNOTFIND;

    for( c = 0; c < nconss; ++c )
    {
        consdata = SCIPconsGetData(conss[c]);

        /* do not consider new constraint of leaf */
        if(consdata->cut == nullptr)
            continue;
        /* check if all previously generated variables are valid for this constraint */
//        assert( consdataCheck(scip, probData, consdata, TRUE) );

#ifndef NDEBUG
        {
            /* check if there are no equal consdatas */
            SCIP_CONSDATA* consdata2;
            int i;
            for( i = c+1; i < nconss; ++i )
            {
                consdata2 = SCIPconsGetData(conss[i]);
                assert(consdata->type != consdata2->type || consdata->numV != consdata2->numV);
                if(consdata->type == ENFORCE)
                    assert(consdata2->type == ENFORCE || consdata2->numV >= consdata->numV);
                else
                    assert(consdata2->type == PROHIBIT || consdata2->numV <= consdata->numV);
            }
        }
#endif
        if( !consdata->propagated )
        {
            SCIPdebugMsg(scip, "propagate constraint <%s> ", SCIPconsGetName(conss[c]));
            SCIPdebug( consdataPrint(scip, consdata, nullptr) );
            for(int v = consdata->npropagatedvars; v < probData->nVars_; ++v )
            {
                SCIP_VAR* var = probData->vars_[v];
                SCIP_CALL( SCIPaddVarToRow(scip, consdata->cut, var, 1.0) );
            }
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
//        assert( consdataCheck(scip, probData, consdata, FALSE) );
    }

    return SCIP_OKAY;
}

/** constraint activation notification method of constraint handler */
SCIP_DECL_CONSACTIVE(ConshdlrNVehicle::scip_active)
{   /*lint --e{715}*/
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    SCIP_CONSDATA* consdata;

    assert(scip != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME5) == 0);
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
SCIP_DECL_CONSDEACTIVE(ConshdlrNVehicle::scip_deactive)
{   /*lint --e{715}*/
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
    SCIP_CONSDATA* consdata;

    assert(scip != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), CONSHDLR_NAME5) == 0);
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

SCIP_DECL_CONSPRINT(ConshdlrNVehicle::scip_print)
{
    SCIP_CONSDATA* consdata = SCIPconsGetData(cons);
    cout << "nVehicle constraint at node " << SCIPnodeGetNumber(consdata->node) << endl;
    cout << "\tNumVehicles "<< (consdata->type == ENFORCE ? ">= " : "<= ") << consdata->numV << endl;
    if(consdata->cut != nullptr)
    {
        SCIPprintRow(scip, consdata->cut, nullptr);
    }else
    {
        cout << "ALERT: row not yet included!" << endl;
    }
    return SCIP_OKAY;
}

/** global methods */

/** gets the dual solution of the nVehicle constraint in the current LP */
SCIP_Real SCIPgetDualsolNVehicle(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS*              cons                /**< constraint data */
){
    SCIP_CONSDATA* consdata;

    assert(scip != nullptr);

    if( strcmp(SCIPconshdlrGetName(SCIPconsGetHdlr(cons)), CONSHDLR_NAME5) != 0 )
    {
        SCIPerrorMessage("constraint is not a nVehicle constraint\n");
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

/** gets the dual Farkas value of the nVehicle constraint in the current infeasible LP */
SCIP_Real SCIPgetDualfarkasNVehicle(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS*              cons                /**< constraint data */
){
    SCIP_CONSDATA* consdata;

    assert(scip != nullptr);

    if( strcmp(SCIPconshdlrGetName(SCIPconsGetHdlr(cons)), CONSHDLR_NAME5) != 0 )
    {
        SCIPerrorMessage("constraint is not a nVehicle constraint\n");
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

/** adds coefficient in nVehicle constraint */
SCIP_RETCODE SCIPaddCoefNVehicleCons(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_CONS*            cons,               /**< constraint data */
        SCIP_VAR*             var                 /**< variable to add to the constraint */
)
{
    SCIP_CONSDATA* consdata;
    assert(var != nullptr);

    if( strcmp(SCIPconshdlrGetName(SCIPconsGetHdlr(cons)), CONSHDLR_NAME5) != 0 )
    {
        SCIPerrorMessage("constraint is not a vehicle constraint\n");
        return SCIP_INVALIDDATA;
    }

    consdata = SCIPconsGetData(cons);
    consdata->npropagatedvars++;
    assert(consdata != nullptr);
    assert(consdata->cut != nullptr);
    SCIP_CALL( SCIPaddVarToRow(scip, consdata->cut, var, 1.0) );

    return SCIP_OKAY;
}

/** creates and captures a nVehicle constraint */
SCIP_RETCODE SCIPcreateConsNVehicle(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS**             cons,               /**< pointer to hold the created constraint */
        const char*             name,               /**< name of constraint */
        double                  numV,               /**< day of vehicle */
        CONSTYPE                type,               /**< stores whether number of vehicles is enforced (>=) or prohibited (<=) */
        SCIP_NODE*              node                /**< the node in the B&B-tree at which the cons is sticking */
){
    SCIP_CONSDATA* consdata = nullptr;
    SCIP_CONSHDLR* conshdlr = nullptr;

    assert(scip != nullptr);

    /* find the set partitioning constraint handler */
    conshdlr = SCIPfindConshdlr(scip, CONSHDLR_NAME5);
    assert(conshdlr != nullptr);
    /* create constraint data*/
    SCIP_CALL(consdataCreate(scip, &consdata, numV, type, node));
    assert(consdata != nullptr);
    /* create constraint */
    SCIP_CALL(SCIPcreateCons(scip, cons, name, conshdlr, consdata, FALSE, TRUE, FALSE, FALSE, TRUE,
                             TRUE, TRUE, FALSE, FALSE, TRUE));

    return SCIP_OKAY;
}

/** returns the day of a nVehicle constraint */
double SCIPgetnumVOfNVehicleCons(
        SCIP_CONS*              cons
){
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->numV;
}

/** returns the type of a nVehicle constraint */
CONSTYPE SCIPgetTypeOfNVehicleCons(
        SCIP_CONS*              cons
){
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->type;
}

/** returns the row of a nVehicle constraint */
SCIP_Row* SCIPgetRowOfNVehicleCons(
        SCIP_CONS*              cons
){
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->cut;
}