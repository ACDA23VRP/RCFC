
#ifndef VRP_CONSHDLRNVEHICLE_H
#define VRP_CONSHDLRNVEHICLE_H

#include "objscip/objscip.h"
#include "model_data.h"
#include "probdata_vrp.h"
#include "scip/scip.h"
#include "vardata.h"

using namespace scip;

class ConshdlrNVehicle : public ObjConshdlr {
public:
    int maxseparounds_;

    /** default constructor */
    explicit ConshdlrNVehicle(
            SCIP *scip
    )
            : ObjConshdlr(scip, "nVehicle", "branching num vehicle cuts conshdlr", 5003, 10, 0,
                          1, 1, 1, 0, FALSE, FALSE, TRUE,
                          SCIP_PROPTIMING_BEFORELP, SCIP_PRESOLTIMING_ALWAYS) {
        maxseparounds_ = 1;
        SCIPaddIntParam(scip, "constraints/NVehicle/maxrounds", "maximal number of separation rounds per node "
                                                                "(-1: unlimited)", &maxseparounds_, FALSE, 1, -1,
                        INT_MAX, nullptr, nullptr);
    }

    /** destructor */
    ~ConshdlrNVehicle() override = default;

    /** frees specific constraint data */
    virtual SCIP_DECL_CONSDELETE(scip_delete);

    /** transforms constraint data into data belonging to the transformed problem */
    virtual SCIP_DECL_CONSTRANS(scip_trans);

    /** separation method of constraint handler for LP solution */
    virtual SCIP_DECL_CONSSEPALP(scip_sepalp);

    /** constraint enforcing method of constraint handler for LP solutions */
    virtual SCIP_DECL_CONSENFOLP(scip_enfolp) {
        return SCIP_OKAY;
    }

    /** constraint enforcing method of constraint handler for pseudo solutions */
    virtual SCIP_DECL_CONSENFOPS(scip_enfops) {
        return SCIP_OKAY;
    }

    /** feasibility check method of constraint handler for integral solutions */
    virtual SCIP_DECL_CONSCHECK(scip_check) {
        return SCIP_OKAY;
    }

    /** domain propagation method of constraint handler */
    virtual SCIP_DECL_CONSPROP(scip_prop);

    /** variable rounding lock method of constraint handler */
    virtual SCIP_DECL_CONSLOCK(scip_lock) {
        return SCIP_OKAY;
    }

    /** constraint activation notification method of constraint handler */
    virtual SCIP_DECL_CONSACTIVE(scip_active);

    /** constraint deactivation notification method of constraint handler */
    virtual SCIP_DECL_CONSDEACTIVE(scip_deactive);

    /** constraint display method of constraint handler */
    virtual SCIP_DECL_CONSPRINT(scip_print);


};
/** gets the dual solution of the nVehicle constraint in the current LP */
SCIP_Real SCIPgetDualsolNVehicle(
        SCIP *scip,                   /**< SCIP data structure */
        SCIP_CONS *cons               /**< constraint data */
);

/** gets the dual Farkas value of the nVehicle constraint in the current infeasible LP */
SCIP_Real SCIPgetDualfarkasNVehicle(
        SCIP *scip,                   /**< SCIP data structure */
        SCIP_CONS *cons               /**< constraint data */
);

/** adds coefficient in nVehicle constraint */
SCIP_RETCODE SCIPaddCoefNVehicleCons(
        SCIP *scip,                   /**< SCIP data structure */
        SCIP_CONS *cons,              /**< constraint data */
        SCIP_VAR *var                 /**< variable to add to the constraint */
);

/** creates and captures a nVehicle constraint */
SCIP_RETCODE SCIPcreateConsNVehicle(
        SCIP *scip,                   /**< SCIP data structure */
        SCIP_CONS **cons,             /**< pointer to hold the created constraint */
        const char *name,             /**< name of constraint */
        double numV,                  /**< day of vehicle */
        CONSTYPE type,                /**< stores whether number of vehicles is enforced (>=) or prohibited (<=) */
        SCIP_NODE *node               /**< the node in the B&B-tree at which the cons is sticking */
);

/** returns the numV value of a nVehicle constraint */
double SCIPgetnumVOfNVehicleCons(
        SCIP_CONS *cons
);

/** returns the type of a nVehicle constraint */
CONSTYPE SCIPgetTypeOfNVehicleCons(
        SCIP_CONS *cons
);

/** returns the row of a nVehicle constraint */
SCIP_Row *SCIPgetRowOfNVehicleCons(
        SCIP_CONS *cons
);


#endif //VRP_CONSHDLRNVEHICLE_H
