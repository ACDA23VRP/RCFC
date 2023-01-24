
#ifndef VRP_HEURDAYVARROUNDING_H
#define VRP_HEURDAYVARROUNDING_H


#include "scip/scip.h"
#include "objscip/objscip.h"
#include "pricer_vrp.h"

using namespace scip;

class HeurDayVarRounding : public ObjHeur
{
    int             ncalls_;
public:
    SCIP_Real       best_;
    vector<tourVRP> sol_tours_;
    /** default constructor */
    HeurDayVarRounding(
        SCIP*   scip
        )
        : ObjHeur(scip, "dayVarRounding", "rounding of vehicle assignement variables", 'V', 50000, 0, 0, -1,
                  SCIP_HEURTIMING_DURINGPRICINGLOOP, false),
        ncalls_(0),
        best_(SCIPinfinity(scip))
        {}

    /** destructor */
    ~HeurDayVarRounding(){}

    /** destructor of primal heuristic to free user data (called when SCIP is exiting) */
    virtual SCIP_DECL_HEURFREE(scip_free){return SCIP_OKAY;}

    /** initialization method of primal heuristic (called after problem was transformed) */
    virtual SCIP_DECL_HEURINIT(scip_init){return SCIP_OKAY;}

    /** deinitialization method of primal heuristic (called before transformed problem is freed) */
    virtual SCIP_DECL_HEUREXIT(scip_exit){return SCIP_OKAY;}

    /** solving process initialization method of primal heuristic (called when branch and bound process is about to begin) */
    virtual SCIP_DECL_HEURINITSOL(scip_initsol);//{return SCIP_OKAY;}

    /** solving process deinitialization method of primal heuristic (called before branch and bound process data is freed) */
    virtual SCIP_DECL_HEUREXITSOL(scip_exitsol){return SCIP_OKAY;}

    /** execution method of primal heuristic
     *
     *  Searches for feasible primal solutions. The method is called in the node processing loop.
     *
     *  possible return values for *result:
     *  - SCIP_FOUNDSOL   : at least one feasible primal solution was found
     *  - SCIP_DIDNOTFIND : the heuristic searched, but did not find a feasible solution
     *  - SCIP_DIDNOTRUN  : the heuristic was skipped
     *  - SCIP_DELAYED    : the heuristic was skipped, but should be called again as soon as possible, disregarding
     *                      its frequency
     */
    virtual SCIP_DECL_HEUREXEC(scip_exec);
};


#endif //VRP_HEURDAYVARROUNDING_H
