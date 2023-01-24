
#ifndef VRP_PROP_VARFIXING_H
#define VRP_PROP_VARFIXING_H

#include "objscip/objscip.h"
#include "scip/scip.h"
#include "probdata_vrp.h"
#include "model_data.h"

using namespace std;
using namespace scip;

class ObjPropVarFixing : public ObjProp
{
private:
    int         arcflow_fixed_;     /** number of fixed arc flow variables */
    int         vehicleass_fixed_;  /** number of fixed vehicle assignment variables */
public:
    bool        active_redcosts_;       /** true iff red costs of the variables have been calculated beforehand */
    SCIP_Real   lastCutoff_;        /** cutoff value of the last call - only interesting at root */
    /** default constructor */
    explicit ObjPropVarFixing(
            SCIP*   scip
    )
            : ObjProp(scip, "varFixing", "reduced costs fixing for vehicle assignment and arc flow variables",
                      1000, 1, FALSE, SCIP_PROPTIMING_DURINGLPLOOP, -1, 0, SCIP_PRESOLTIMING_NONE){
        active_redcosts_ = false;
        arcflow_fixed_ = 0;
        vehicleass_fixed_ = 0;
        lastCutoff_ = SCIP_DEFAULT_INFINITY;
    }

    ~ObjPropVarFixing() override = default;

    virtual SCIP_DECL_PROPEXEC(scip_exec);
};


#endif //VRP_PROP_VARFIXING_H
