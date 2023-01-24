

#include "objscip/objscip.h"
#include "branchingrule_dayvar.h"
#include "model_data.h"
#include "probdata_vrp.h"
#include "scip/scip.h"
#include "vardata.h"

#define CONSHDLR_NAME2          "dayVar"
#define CONSHDLR_DESC2          "stores the local branching decisions"

#ifndef VRP_CONSHDLRDayVar_H
#define VRP_CONSHDLRDayVar_H

using namespace scip;

class ConshdlrDayVar : public ObjConshdlr
{
public:
    /** default constructor */
    explicit ConshdlrDayVar(
            SCIP*   scip
    )
            : ObjConshdlr(scip, CONSHDLR_NAME2, CONSHDLR_DESC2, 0, 0, 1,
                          -1, 1, 1, 0, FALSE, FALSE, TRUE,
                          SCIP_PROPTIMING_BEFORELP, SCIP_PRESOLTIMING_ALWAYS)
    {
    }

    /** destructor */
    ~ConshdlrDayVar() override= default;

    /** frees specific constraint data */
    virtual SCIP_DECL_CONSDELETE(scip_delete);

    /** transforms constraint data into data belonging to the transformed problem */
    virtual SCIP_DECL_CONSTRANS(scip_trans);

    /** constraint enforcing method of constraint handler for LP solutions */
    virtual SCIP_DECL_CONSENFOLP(scip_enfolp){
        return SCIP_OKAY;
    }

    /** constraint enforcing method of constraint handler for pseudo solutions */
    virtual SCIP_DECL_CONSENFOPS(scip_enfops){
        return SCIP_OKAY;
    }

    /** feasibility check method of constraint handler for integral solutions */
    virtual SCIP_DECL_CONSCHECK(scip_check){
        return SCIP_OKAY;
    }

    /** domain propagation method of constraint handler */
    virtual SCIP_DECL_CONSPROP(scip_prop);

    /** variable rounding lock method of constraint handler */
    virtual SCIP_DECL_CONSLOCK(scip_lock){
        return SCIP_OKAY;
    }

    /** constraint activation notification method of constraint handler */
    virtual SCIP_DECL_CONSACTIVE(scip_active);

    /** constraint deactivation notification method of constraint handler */
    virtual SCIP_DECL_CONSDEACTIVE(scip_deactive);

    /** constraint display method of constraint handler */
    virtual SCIP_DECL_CONSPRINT(scip_print);
};

SCIP_RETCODE SCIPcreateConsDayVar(
        SCIP*               scip,                /**< SCIP data structure */
        SCIP_CONS**         cons,                /**< pointer to hold the created constraint */
        const char*         name,                /**< name of the constraint */
        int                 customer,            /**< corresponding customer */
        int                 day,                 /**< corresponding day */
        CONSTYPE            type,                /**< stores whether arc gets enforced or prohibited */
        SCIP_NODE*          node,                /**< the node in the B&B-tree at which the cons is sticking */
        SCIP_Bool           local                /**< is constraint only valid locally? */
);

/** returns customer of cons */
int SCIPgetCustomerOfDayCons(
        SCIP_CONS*            cons                /**< samediff constraint */
);

/** returns day of cons */
int SCIPgetDayOfDayCons(
        SCIP_CONS*            cons                /**< samediff constraint */
);

/** return constraint type PROHIBIT or ENFORCE */
CONSTYPE SCIPgetTypeOfDayCons(
        SCIP_CONS*            cons                /**< samediff constraint */
);

/** propagates constraints that got added during the pricing step */
SCIP_RETCODE SCIPconshdlrPerformPropagate(
        SCIP*               scip,
        SCIP_CONSHDLR*      conshdlr
);

#endif //VRP_CONSHDLRDayVar_H
