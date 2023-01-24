
#ifndef VRP_PROP_TOURVARFIXING_H
#define VRP_PROP_TOURVARFIXING_H

#include "objscip/objscip.h"
#include "scip/scip.h"
#include "probdata_vrp.h"
#include "model_data.h"

using namespace std;
using namespace scip;

class ObjPropTourVarFixing : public ObjProp
{
public:
    tourVRP         tvrp_;
    bool            fixnodedone_;
    bool            new_;
    bool            up_;
    double          obj_cmp_;
    vector<vector<SCIP_VAR*>> counterpartCuts_;
    /** default constructor */
    explicit ObjPropTourVarFixing(
            SCIP*   scip
    )
            : ObjProp(scip, "tourVarFixing", "reduced costs fixing (to 1) for tour variables",
                      1000, 1, FALSE, SCIP_PROPTIMING_AFTERLPLOOP, -1, 0, SCIP_PRESOLTIMING_NONE){
        fixnodedone_ = false;
        new_ = true;
        up_ = true;
        obj_cmp_ = -1;
    }

    ~ObjPropTourVarFixing() override = default;

    virtual SCIP_DECL_PROPEXEC(scip_exec);
};


#endif //VRP_PROP_TOURVARFIXING_H
