
#ifndef VRP_CONSHDLRCPC_H
#define VRP_CONSHDLRCPC_H



#include "objscip/objscip.h"
#include "model_data.h"
#include "probdata_vrp.h"
#include "scip/scip.h"
#include "vardata.h"

using namespace scip;

class ConshdlrCPC : public ObjConshdlr
{
public:
    bool stopfixing_;
    /** default constructor */
    explicit ConshdlrCPC(
            SCIP*   scip
    )
            : ObjConshdlr(scip, "CPC", "counterpart cuts conshdlr", 0, 5000, 0,
                          0, 0, 1, 0, TRUE, FALSE, FALSE,
                          SCIP_PROPTIMING_BEFORELP, SCIP_PRESOLTIMING_ALWAYS),
                          stopfixing_(false)
    {
    }

    /** destructor */
    ~ConshdlrCPC() override= default;

    /** frees specific constraint data */
    virtual SCIP_DECL_CONSDELETE(scip_delete);

    /** transforms constraint data into data belonging to the transformed problem */
    virtual SCIP_DECL_CONSTRANS(scip_trans){
        return SCIP_OKAY;
    }

    /** separation method of constraint handler for LP solution */
    virtual SCIP_DECL_CONSSEPALP(scip_sepalp){
        return SCIP_OKAY;
    }

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

    /** constraint display method of constraint handler */
    virtual SCIP_DECL_CONSPRINT(scip_print);


};

/** creates and adds row for a subset row constraint */
SCIP_RETCODE SCIPcreateAndAddRowCPC(
        SCIP*                   scip,
        SCIP_Cons*              cons
);

/** creates and captures a subset row constraint based on Set S */
SCIP_RETCODE SCIPcreateConsCPC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS**             cons,               /**< pointer to hold the created constraint */
        const char*             name,               /**< name of constraint */
        vector<SCIP_VAR*>&      vars
);



#endif //VRP_CONSHDLRCPC_H
