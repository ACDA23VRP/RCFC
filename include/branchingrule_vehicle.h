
#ifndef VRP_BRANCHINGRULE_VEHICLE_H
#define VRP_BRANCHINGRULE_VEHICLE_H

#include "scip/scip.h"
#include "objscip/objscip.h"

using namespace scip;

class ObjBranchruleVehicle : public ObjBranchrule{
public:
    /** default constructor */
    explicit ObjBranchruleVehicle(
            SCIP*   scip,
            int     priority
    )
    : ObjBranchrule(scip, "VehicleBranching", "Branching rules for number of and usage of vehicles", priority,
                    -1, 1.0)
    {
    }

    /** destructor */
    ~ObjBranchruleVehicle() override= default;

    /** branching execution method for fractional LP solutions */
    virtual SCIP_DECL_BRANCHEXECLP(scip_execlp);
};


#endif //VRP_BRANCHINGRULE_VEHICLE_H
