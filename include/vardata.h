
#ifndef VRP_VARDATA_H
#define VRP_VARDATA_H

#include "objscip/objscip.h"
#include "scip/pub_var.h"
#include "probdata_vrp.h"
#include "tourVRP.h"

#include <utility>
#include <vector>

using namespace std;
using namespace scip;

class ObjVarDataVRP : public ObjVardata {
private:
    int             tourLength_;
    int             day_;
    int             capacity_;
    bool            isElementary_;
public:
    tourVRP         tourVrp_;
    vector<int>     tour_;
    ObjVarDataVRP(
        tourVRP&        tourVrp,
        vector<int>     tour,
        int             tourLength,
        int             day,
        int             capacity,
        bool            isElementary
    ):
    tourLength_(tourLength),
    day_(day),
    capacity_(capacity),
    isElementary_(isElementary),
    tour_(std::move(tour))
    {
        tourVrp_.copy(tourVrp);
    }

    ~ObjVarDataVRP() override
    {
//        delete &tour_;
    };

//    virtual SCIP_RETCODE scip_copy( // TODO: delete if we do not need it
//            SCIP*              scip,               /**< SCIP data structure */
//            SCIP*              sourcescip,         /**< source SCIP main data structure */
//            SCIP_VAR*          sourcevar,          /**< variable of the source SCIP */
//            SCIP_HASHMAP*      varmap,             /**< a hashmap which stores the mapping of source variables to corresponding
//                                              *   target variables */
//            SCIP_HASHMAP*      consmap,            /**< a hashmap which stores the mapping of source contraints to corresponding
//                                              *   target constraints */
//            SCIP_VAR*          targetvar,          /**< variable of the (targert) SCIP (targetvar is the copy of sourcevar) */
//            ObjVardata**       objvardata,         /**< pointer to store the copied variable data object */
//            SCIP_RESULT*       result              /**< pointer to store the result of the call */
//    )
//    {  /*lint --e{715}*/
//        auto* oldvardata = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, sourcevar));
//        (*objvardata) = new ObjVarDataVRP(oldvardata->tourVrp_, oldvardata->tour_, oldvardata->tourLength_,
//                                          oldvardata->day_, oldvardata->capacity_, oldvardata->isElementary_);
//
//        (*result) = SCIP_SUCCESS;
//        return SCIP_OKAY;
//    }

    friend ostream &operator<<(std::ostream &os, const ObjVarDataVRP& vardata) {
        os << "Vardata: (day "<< vardata.day_<< ", length " << vardata.tourLength_ << ")\n";
        for(int u : vardata.tour_)
            os << u << " ";
        os << '\n';
        return os;
    }

    int getLength() const
    {
        return tourLength_;
    };

    int getDay() const
    {
        return day_;
    };

    int getCapacity() const
    {
        return capacity_;
    };

    bool isElementary() const
    {
        return isElementary_;
    }
//
//    tourVRP getTourVRP()
//    {
//        return tourVrp_;
//    }
};

bool SCIPcontainsTourVar(
        SCIP*                   scip,
        vrp::ProbDataVRP*       probData,
        SCIP_Var**              existingVar,
        tourVRP&                tvrp
);

SCIP_RETCODE SCIPcreateColumn(
        SCIP*                   scip,
        vrp::ProbDataVRP*       probData,
        const char*             name,
        SCIP_Bool               initial,
        tourVRP&                tvrp
);


#endif //VRP_VARDATA_H
