
#include <cstring>
#include "cassert"

#include "probdata_vrp.h"
#include "model_data.h"
#include "pricer_vrp.h"

#include "scip/cons_setppc.h"
#include "scip/cons_linear.h"
#include "scip/scip.h"
#include "objscip/objscip.h"
#include "initial.h"
#include "vector"

#define EVENTHDLR_NAME         "addedvar"
#define EVENTHDLR_DESC         "event handler for catching added variables"


#include "scip/def.h"
#include "scip/stat.h"
#include "scip/var.h"
#include "scip/prob.h"
#include "scip/tree.h"
#include "scip/branch.h"
#include "scip/reopt.h"
#include "scip/pub_misc.h"
#include "scip/debug.h"
#include "scip/pub_message.h"
#include "scip/scip_cons.h"
#include "scip/struct_cons.h"
#include "scip/struct_mem.h"
#include "scip/struct_scip.h"
#include "scip/struct_set.h"
#include "tourVRP.h"
#include "vardata.h"

using namespace vrp;
using namespace scip;

/** execution method of event handler */
static
SCIP_DECL_EVENTEXEC(eventExecAddedVar)
{  /*lint --e{715}*/
    assert(eventhdlr != nullptr);
    assert(strcmp(SCIPeventhdlrGetName(eventhdlr), EVENTHDLR_NAME) == 0);
    assert(event != nullptr);
    assert(SCIPeventGetType(event) == SCIP_EVENTTYPE_VARADDED);

    SCIPdebugMsg(scip, "exec method of event handler for added variable to probdata\n");

    /* add new variable to probdata */
    SCIP_CALL( SCIPprobdataAddVar(scip, dynamic_cast<ProbDataVRP*>(SCIPgetObjProbData(scip)), SCIPeventGetVar(event)) );

    return SCIP_OKAY;
}

/** destructor of user problem data to free original user data (called when original problem is freed) */
SCIP_RETCODE ProbDataVRP::scip_delorig(
        SCIP*                 scip                /**< SCIP data structure */
)
{
    for( int i = 0; i < nVars_; i++ )
    {
        SCIP_CALL( SCIPreleaseVar(scip, &vars_[i]) );
    }
    for( int i = 0; i < nCons_; i++ )
    {
        SCIP_CALL(SCIPreleaseCons(scip, &cons_[i]));
    }
    return SCIP_OKAY;
}

/** destructor of user problem data to free original user data (called when original problem is freed) */
SCIP_RETCODE ProbDataVRP::scip_deltrans(
        SCIP*                 scip                /**< SCIP data structure */
)
{
    for( int i = 0; i < nVars_; i++ )
    {
        SCIP_CALL( SCIPreleaseVar(scip, &vars_[i]) );
    }
    for( int i = 0; i < nCons_; i++ )
    {
        SCIP_CALL(SCIPreleaseCons(scip, &cons_[i]));
    }
    return SCIP_OKAY;
}

/** creates user data of transformed problem by transforming the original user problem data
 *  (called after problem was transformed) */
SCIP_RETCODE ProbDataVRP::scip_trans(
        SCIP*                 scip,               /**< SCIP data structure */
        ObjProbData**         objprobdata,        /**< pointer to store the transformed problem data object */
        SCIP_Bool*            deleteobject        /**< pointer to store whether SCIP should delete the object after solving */
)
{  /*lint --e{715}*/
    assert( objprobdata != nullptr );
    assert( deleteobject != nullptr );
    assert( modelData_ != nullptr );

//    std::vector<SCIP_VAR*> newVars(nVars_);
//    std::vector<SCIP_CONS*> newCons(nCons_);

    /* alloace transformed prob data */
    auto* transprobdatavrp = new ProbDataVRP(modelData_, nVars_, nCons_, use_propagator_);
    assert( transprobdatavrp != nullptr );

    /* transform all constraints */
    for (int i = 0; i < nCons_; i++)
    {
        assert(cons_[i] != nullptr);
        SCIP_CALL( SCIPtransformCons(scip, cons_[i], &(transprobdatavrp->cons_[i])) );
        assert(transprobdatavrp->cons_[i] != nullptr);
    }

    /* transform all variables */
    for (int i = 0; i < nVars_; i++)
    {
        SCIP_CALL( SCIPgetTransformedVar(scip, vars_[i], &(transprobdatavrp->vars_[i])) );
    }
    // save data pointer
    assert( objprobdata != nullptr );
    *objprobdata = transprobdatavrp;

    *deleteobject = TRUE;

    return SCIP_OKAY;
}

/** solving process initialization method of transformed data (called before the branch and bound process begins) */
SCIP_RETCODE ProbDataVRP::scip_initsol(
        SCIP*              scip                /**< SCIP data structure */
)
{
    SCIP_EVENTHDLR* eventhdlr;

    /* catch variable added event */
    eventhdlr = SCIPfindEventhdlr(scip, "addedvar");
    assert(eventhdlr != nullptr);

    SCIP_CALL( SCIPcatchEvent(scip, SCIP_EVENTTYPE_VARADDED, eventhdlr, nullptr, nullptr) );

    return SCIP_OKAY;
}

/** solving process deinitialization method of transformed data (called before the branch and bound data is freed) */
SCIP_RETCODE ProbDataVRP::scip_exitsol(
        SCIP*              scip,                /**< SCIP data structure */
        SCIP_Bool          restart              /**< was this exit solve call triggered by a restart? */
)
{
    SCIP_EVENTHDLR* eventhdlr;
    assert(!restart);
    /* drop variable added event */
    eventhdlr = SCIPfindEventhdlr(scip, "addedvar");
    assert(eventhdlr != nullptr);

    SCIP_CALL( SCIPdropEvent(scip, SCIP_EVENTTYPE_VARADDED, eventhdlr, nullptr, -1) );

    return SCIP_OKAY;
}

/** sets up the problem data */
SCIP_RETCODE SCIPprobdataCreate(
        SCIP*                 scip,               /**< SCIP data structure */
        model_data*           modeldata,           /**< model data */
        vector<tourVRP>&      sol_tvrps,
        int                   activate_propagator
)
{
    std::vector<SCIP_CONS*> cons(modeldata->nC - 1 + modeldata->nDays, (SCIP_CONS*) nullptr);
    std::vector<SCIP_VAR*> vars;
    char name[SCIP_MAXSTRLEN];
    int i;

    assert(scip != nullptr);
    assert(modeldata != nullptr);
    SCIPdebugMsg(scip, "number of days for this vehicle routing problem %d", modeldata->nDays);

    /* check for capacity of the largest vehicle */
    int highcap = 0;
    for(int d = 0; d < modeldata->nDays; d++)
    {
        if(modeldata->max_caps[d] > highcap)
            highcap = modeldata->max_caps[d];
    }
    SCIP_CALL( SCIPsetIntParam(scip,"conshdlr/KPC/maxcapacity",highcap) );

    /* create event handler if it does not exist yet */
    if( SCIPfindEventhdlr(scip, EVENTHDLR_NAME) == nullptr )
    {
        SCIP_CALL( SCIPincludeEventhdlrBasic(scip, nullptr, EVENTHDLR_NAME, EVENTHDLR_DESC, eventExecAddedVar, nullptr) );
    }

    /* create problem data object  */
    auto* prob = new ProbDataVRP(modeldata, 0, modeldata->nC - 1 + modeldata->nDays, activate_propagator);
    /* creates problem in SCIP with problem data */
    SCIP_CALL( SCIPcreateObjProb(scip, "VRP_Solver", prob, TRUE) );

    /* set objective sense */
    SCIP_CALL( SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE) );

//    /* tell SCIP that the objective will be always integral, this is only the case, if we set all weights to one */
    if(!modeldata->minTravel)
        SCIP_CALL( SCIPsetObjIntegral(scip) );

    /* create set partitioning constraints for each customer but the depot */
    assert(modeldata->nC > 1);
    for( i = 0; i < modeldata->nC - 1; ++i )
    {
        SCIP_CONS* con = nullptr;
        (void) SCIPsnprintf(name, SCIP_MAXSTRLEN, "customer_%d", i+1);


        //SCIP_CALL( SCIPcreateConsBasicSetcover(scip, &con, name, 0, NULL) );

//        SCIP_CALL( SCIPcreateConsBasicSetpart(scip, &con, name, 0, nullptr) );
        SCIP_CALL(SCIPcreateConsBasicLinear(scip, &con, name, 0, nullptr, nullptr, 1.0, 1.0));

        /* declare constraint modifiable for adding variables during pricing */
        SCIP_CALL( SCIPsetConsModifiable(scip, con, TRUE) );
        SCIP_CALL( SCIPaddCons(scip, con) );
//        cons[i] = con;
        prob->cons_[i] = con;
    }

    /* add constraint, such that exactly one route is choosen for every day */
    for (i = 0; i < modeldata->nDays; i++)
    {
        SCIP_CONS* con = nullptr;
        (void) SCIPsnprintf(name, SCIP_MAXSTRLEN, "day_%d", i);

        if(modeldata->num_v[i] == 1)
            SCIP_CALL( SCIPcreateConsBasicSetpack(scip, &con, name, 0, nullptr) );
        else
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &con, name, 0, nullptr, nullptr, 0, modeldata->num_v[i]));

        /* declare constraint modifiable for adding variables during pricing */
        SCIP_CALL( SCIPsetConsModifiable(scip, con, TRUE) );
        SCIP_CALL( SCIPaddCons(scip, con) );
        prob->cons_[modeldata->nC - 1 + i] = con;
    }

    /* set user problem data and pricer */
    auto* vrp_pricer_ptr = new ObjPricerVRP(scip, "VRP_Pricer");
    SCIP_CALL( SCIPincludeObjPricer(scip, vrp_pricer_ptr, true) );
    /* activate pricer */
    SCIP_CALL( SCIPactivatePricer(scip, SCIPfindPricer(scip, "VRP_Pricer")) );

    /* in case of input, add solution to the problem */
    if((int) sol_tvrps.size() > 0)
    {
        SCIP_CALL(addInitSolution(scip, prob, sol_tvrps));
    }else
    {
        /* else use initial heuristic */
        SCIP_CALL(initialDispatching(scip, prob));
    }
    /* add empty tours */
//    SCIP_CALL(addEmptyTours(scip, prob));

    return SCIP_OKAY;
}

/** adds given variable to the problem data */
SCIP_RETCODE SCIPprobdataAddVar(
        SCIP*                   scip,                   /**< SCIP data structure */
        vrp::ProbDataVRP*       objprobdata,            /**< problem data */
        SCIP_VAR*               var                     /**< variables to add */
)
{
    /* caputure variables */
    SCIP_CALL( SCIPcaptureVar(scip, var) );
    auto* vardata = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));
    if(vardata->tourVrp_.length_ == 0)
    {
        objprobdata->emptyVars_.push_back(var);
    }else
    {
        objprobdata->nVars_++;
        objprobdata->vars_.push_back(var);
    }

    SCIPdebugMsg(scip, "added variable to probdata; nvars = %d\n", objprobdata->nVars_);

    return SCIP_OKAY;
}
