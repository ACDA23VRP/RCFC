
#ifndef VRP_BRANCHINGRULE_ARCFLOW_H
#define VRP_BRANCHINGRULE_ARCFLOW_H

#include "scip/scip.h"
#include "objscip/objscip.h"

using namespace scip;

class ObjBranchruleArcflow : public ObjBranchrule{
public:
    /** default constructor */
    explicit ObjBranchruleArcflow(
            SCIP*   scip,
            int     priority
    )
    : ObjBranchrule(scip, "ArcFlowBranching", "Branching rule for the arc flow variables", priority,
                    -1, 1.0)
    {
    }

    /** destructor */
    ~ObjBranchruleArcflow() override= default;

    /** branching execution method for fractional LP solutions */
    virtual SCIP_DECL_BRANCHEXECLP(scip_execlp);
};

#endif //VRP_BRANCHINGRULE_ARCFLOW_H
