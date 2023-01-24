
#ifndef VRP_CONSHDLRKPC_H
#define VRP_CONSHDLRKPC_H

#include "objscip/objscip.h"
#include "model_data.h"
#include "probdata_vrp.h"
#include "scip/scip.h"
#include "vardata.h"

using namespace scip;

class ConshdlrKPC : public ObjConshdlr
{
public:
    int highest_capacity_{};
    /** default constructor */
    explicit ConshdlrKPC(
            SCIP*   scip
    )
    : ObjConshdlr(scip, "KPC", "k-path cuts conshdlr", 50005, 5, 0,
    1, 0, 1, 0, TRUE, TRUE, FALSE,
    SCIP_PROPTIMING_BEFORELP, SCIP_PRESOLTIMING_ALWAYS)
    {
        SCIPaddIntParam(scip, "conshdlr/KPC/maxcapacity", "capacity of the largest vehicles"
                                                          "(1: unlimited)", &highest_capacity_, FALSE, 1000, 1, INT_MAX, nullptr, nullptr);
    }

    /** destructor */
    ~ConshdlrKPC() override= default;

    /** frees specific constraint data */
    virtual SCIP_DECL_CONSDELETE(scip_delete);

    /** transforms constraint data into data belonging to the transformed problem */
    virtual SCIP_DECL_CONSTRANS(scip_trans);

    /** separation method of constraint handler for LP solution */
    virtual SCIP_DECL_CONSSEPALP(scip_sepalp);

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

    /** variable rounding lock method of constraint handler */
    virtual SCIP_DECL_CONSLOCK(scip_lock){
            return SCIP_OKAY;
    }

    virtual SCIP_DECL_CONSCOPY(scip_copy);

    /** constraint display method of constraint handler */
    virtual SCIP_DECL_CONSPRINT(scip_print);


};
/** gets the dual solution of the k-paths constraint in the current LP */
SCIP_Real SCIPgetDualsolKPC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS*              cons                /**< constraint data */
);

/** gets the dual Farkas value of the k-paths constraint in the current infeasible LP */
SCIP_Real SCIPgetDualfarkasKPC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS*              cons                /**< constraint data */
);

/** adds coefficient in k-path constraint */
SCIP_RETCODE SCIPaddCoefKPC(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_CONS*            cons,               /**< constraint data */
        SCIP_VAR*             var,                /**< variable to add to the constraint */
        SCIP_Real             val                 /**< coefficient of constraint entry */
);

/** creates and adds row for a k-paths constraint */
SCIP_RETCODE SCIPcreateAndAddRowKPC(
        SCIP*                   scip,
        SCIP_Cons*              cons
);

/** creates and captures a k-paths constraint based on Set S */
SCIP_RETCODE SCIPcreateConsKPC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS**             cons,               /**< pointer to hold the created constraint */
        const char*             name,               /**< name of constraint */
        vector<bool>&           setS,               /**< set of customers */
        SCIP_Real               k                   /**< number of paths needed (rhs) */
);

vector<bool>& SCIPgetSetOfKPC(
        SCIP_CONS*              cons
);

SCIP_Row* SCIPgetRowOfKPC(
        SCIP_CONS*              cons
);


#endif //VRP_CONSHDLRKPC_H
