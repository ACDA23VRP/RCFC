
#ifndef VRP_BRANCHINGRULE_DAYVAR_H
#define VRP_BRANCHINGRULE_DAYVAR_H

#include "scip/scip.h"
#include "objscip/objscip.h"

using namespace scip;

class ObjBranchruleDayVar : public ObjBranchrule{
public:
    /** default constructor */
    explicit ObjBranchruleDayVar(
            SCIP*   scip,
            int     priority
    )
    : ObjBranchrule(scip, "DayVarBranching", "Branching rule for the day variables", priority,
                    -1, 1.0)
    {
    }

    /** destructor */
    ~ObjBranchruleDayVar() override= default;

    /** branching execution method for fractional LP solutions */
    virtual SCIP_DECL_BRANCHEXECLP(scip_execlp);
};


#endif //VRP_BRANCHINGRULE_DAYVAR_H
