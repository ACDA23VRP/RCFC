
#ifndef VRP_ConshdlrSRC_H
#define VRP_ConshdlrSRC_H

#include "objscip/objscip.h"
#include "model_data.h"
#include "probdata_vrp.h"
#include "scip/scip.h"
#include "vardata.h"

using namespace scip;

class ConshdlrSRC : public ObjConshdlr
{
public:
    /** default constructor */
    explicit ConshdlrSRC(
            SCIP*   scip
    )
    : ObjConshdlr(scip, "SRC", "subset row cuts conshdlr", 5000, 400, 0,
    0, 0, 1, 0, TRUE, FALSE, FALSE,
    SCIP_PROPTIMING_BEFORELP, SCIP_PRESOLTIMING_ALWAYS)
    {
    }

    /** destructor */
    ~ConshdlrSRC() override= default;

    /** frees specific constraint data */
    virtual SCIP_DECL_CONSDELETE(scip_delete);

    /** transforms constraint data into data belonging to the transformed problem */
    virtual SCIP_DECL_CONSTRANS(scip_trans);

    /** separation method of constraint handler for LP solution */
    virtual SCIP_DECL_CONSSEPALP(scip_sepalp);

    /** constraint enforcing method of constraint handler for LP solutions */
    virtual SCIP_DECL_CONSENFOLP(scip_enfolp);

    /** constraint enforcing method of constraint handler for pseudo solutions */
    virtual SCIP_DECL_CONSENFOPS(scip_enfops){
            return SCIP_OKAY;
    }

    /** feasibility check method of constraint handler for integral solutions */
    virtual SCIP_DECL_CONSCHECK(scip_check){
            return SCIP_OKAY;
    }

    /** variable rounding lock method of constraint handler */
    virtual SCIP_DECL_CONSLOCK(scip_lock){
            return SCIP_OKAY;
    }

    virtual SCIP_DECL_CONSCOPY(scip_copy);

    /** constraint display method of constraint handler */
    virtual SCIP_DECL_CONSPRINT(scip_print);


};
/** gets the dual solution of the subset row constraint in the current LP */
SCIP_Real SCIPgetDualsolSRC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS*              cons                /**< constraint data */
);

/** gets the dual Farkas value of the subset row constraint in the current infeasible LP */
SCIP_Real SCIPgetDualfarkasSRC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS*              cons                /**< constraint data */
);

/** adds coefficient in subset row constraint */
SCIP_RETCODE SCIPaddCoefSRC(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_CONS*            cons,               /**< constraint data */
        SCIP_VAR*             var,                /**< variable to add to the constraint */
        SCIP_Real             val                 /**< coefficient of constraint entry */
);

/** creates and adds row for a subset row constraint */
SCIP_RETCODE SCIPcreateAndAddRowSRC(
        SCIP*                   scip,
        SCIP_Cons*              cons
);

/** creates and captures a subset row constraint based on Set S */
SCIP_RETCODE SCIPcreateConsSRC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS**             cons,               /**< pointer to hold the created constraint */
        const char*             name,               /**< name of constraint */
        vector<bool>&           setS,               /**< set of customers */
        vector<bool>&           lmSet,              /**< limited memory */ // TODO: calculate here
        double                  p,                  /**< parameter p */
        double                  rhs                 /**< right hand side */
);

/** returns the parameter p of a subset row constraint */
vector<bool>* SCIPgetSetOfSRC(
        SCIP_CONS*              cons
);

/** returns the parameter p of a subset row constraint */
double SCIPgetpOfSRC(
        SCIP_CONS*              cons
);

/** returns the row of a subset row constraint */
SCIP_Row* SCIPgetRowOfSRC(
        SCIP_CONS*              cons
);


#endif //VRP_ConshdlrSRC_H
