
#ifndef VRP_CONSHDLRVEHICLE_H
#define VRP_CONSHDLRVEHICLE_H

#include "objscip/objscip.h"
#include "model_data.h"
#include "probdata_vrp.h"
#include "scip/scip.h"
#include "vardata.h"

using namespace scip;

class ConshdlrVehicle : public ObjConshdlr
{
public:
    int maxseparounds_;
    /** default constructor */
    explicit ConshdlrVehicle(
            SCIP*   scip
    )
    : ObjConshdlr(scip, "Vehicle", "branching vehicle cuts conshdlr", 5002, 10, 0,
    1, 1, 1, 0, FALSE, FALSE, TRUE,
    SCIP_PROPTIMING_BEFORELP, SCIP_PRESOLTIMING_ALWAYS)
    {
        maxseparounds_ = 1;
        SCIPaddIntParam(scip, "constraints/Vehicle/maxrounds", "maximal number of separation rounds per node "
                       "(-1: unlimited)", &maxseparounds_, FALSE, 1, -1, INT_MAX, nullptr, nullptr);
    }

    /** destructor */
    ~ConshdlrVehicle() override= default;

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
/** gets the dual solution of the vehicle constraint in the current LP */
SCIP_Real SCIPgetDualsolVehicle(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS*              cons                /**< constraint data */
);

/** gets the dual Farkas value of the vehicle constraint in the current infeasible LP */
SCIP_Real SCIPgetDualfarkasVehicle(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS*              cons                /**< constraint data */
);

/** adds coefficient in vehicle constraint */
SCIP_RETCODE SCIPaddCoefVehicleCons(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_CONS*            cons,               /**< constraint data */
        SCIP_VAR*             var                 /**< variable to add to the constraint */
);

/** creates and captures a vehicle constraint */
SCIP_RETCODE SCIPcreateConsVehicle(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS**             cons,               /**< pointer to hold the created constraint */
        const char*             name,               /**< name of constraint */
        int                     day,                /**< day of vehicle */
        CONSTYPE                type,               /**< stores whether vehicle is enforced or prohibited */
        SCIP_NODE*              node                /**< the node in the B&B-tree at which the cons is sticking */
);

/** returns the day of a vehicle constraint */
int SCIPgetDayOfVehicleCons(
        SCIP_CONS*              cons
);

/** returns the type of a vehicle constraint */
CONSTYPE SCIPgetTypeOfVehicleCons(
        SCIP_CONS*              cons
);

/** returns the row of a vehicle constraint */
SCIP_Row* SCIPgetRowOfVehicleCons(
        SCIP_CONS*              cons
);


#endif //VRP_CONSHDLRVEHICLE_H
