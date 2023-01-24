
#ifndef __SCIP_VRPPROBDATA_H__
#define __SCIP_VRPPROBDATA_H__


#include <cassert>
#include <utility>

#include "stdio.h"
#include "model_data.h"
#include "vector"
#include "scip/scip.h"
#include "objscip/objscip.h"
#include "tourVRP.h"

using namespace std;
namespace vrp
{

/** SCIP user problem data for VRP */
class ProbDataVRP : public scip::ObjProbData
{
    private:
        model_data*     modelData_;
    public:
        int                     nVars_;
        int                     nCons_;
        vector<SCIP_VAR*>       vars_;
        vector<SCIP_CONS*>      cons_;
        vector<SCIP_VAR*>       emptyVars_;
        vector<SCIP_VAR*>       nonzeroVars_;
        int                     use_propagator_;
        double                  fw_time_;
        double                  bd_time_;
        vector<vector<SCIP_Real>> prices;
        int                     prop_calls_;
        int                     prop_fixcalls_;
        int                     prop_success_;
        int                     num_fixed_;
        int                     num_fixed_frac_;
        int                     earliest_find_;
        int                     earliest_find_frac_;
        double                  highest_gap_;
        double                  highest_gap_frac_;
        double                  lowest_value_;
        double                  prop_time_;
        double                  prop_time_findvar_;
        double                  prop_time_setup_;
        double                  prop_time_solve_;
        double                  prop_time_pricing_;
        double                  prop_time_probing_;
        double                  prop_time_frac_;
        int                     num_cc_;
        int                     num_cutoff_;
        double                  cutoff_gap_;
        double                  cutoff_val_;
        int     size_subtree_;
        double  time_subtree_;
        vector<long long int> fixednodes_;
        vector<int> sizesubtree_;
        vector<double> fixedgap_;
        vector<int> nGaps_success_;
        vector<int> nGaps_calls_;
        vector<int> nVal_calls;
        vector<int> nVal_success_;

        /** default constructor */
        ProbDataVRP(
            model_data*             modelData,
            int                     nVars,
            int                     nCons,
            int                     activate_propagator
        ):
        modelData_(modelData),
        nVars_(nVars),
        nCons_(nCons),
        use_propagator_(activate_propagator),
        prop_calls_(0),
        prop_fixcalls_(0),
        prop_success_(0),
        num_fixed_(0),
        num_fixed_frac_(0),
        earliest_find_(10000),
        earliest_find_frac_(10000),
        highest_gap_(0),
        highest_gap_frac_(0),
        lowest_value_(1),
        prop_time_(0),
        prop_time_findvar_(0),
        prop_time_setup_(0),
        prop_time_solve_(0),
        prop_time_pricing_(0),
        prop_time_probing_(0),
        prop_time_frac_(0),
        num_cc_(0),
        num_cutoff_(0),
        cutoff_gap_(0),
        cutoff_val_(0),
        size_subtree_(1),
        time_subtree_(0)
        {
            cons_ = std::vector<SCIP_CONS*>(nCons);
            vars_ = std::vector<SCIP_VAR*>(nVars);
            fw_time_ = 0;
            bd_time_ = 0;
            nGaps_calls_.resize(14);
            nGaps_success_.resize(14);
            nVal_calls.resize(10);
            nVal_success_.resize(10);
        }

        /** destructor */
        ~ProbDataVRP() override
        = default;

        /** destructor of user problem data to free original user data (called when original problem is freed) */
        SCIP_RETCODE scip_delorig(
                SCIP*              scip                /**< SCIP data structure */
        ) override;

        /** destructor of user problem data to free transformed user data (called when transformed problem is freed) */
        SCIP_RETCODE scip_deltrans(
                SCIP*              scip                /**< SCIP data structure */
        ) override;

        /** creates user data of transformed problem by transforming the original user problem data
         *  (called after problem was transformed)
         */
        SCIP_RETCODE scip_trans(
                SCIP*              scip,               /**< SCIP data structure */
                ObjProbData**      objprobdata,        /**< pointer to store the transformed problem data object */
                SCIP_Bool*         deleteobject        /**< pointer to store whether SCIP should delete the object after solving */
        ) override;

        /** solving process initialization method of transformed data (called before the branch and bound process begins) */
        SCIP_RETCODE scip_initsol(
                SCIP*              scip                /**< SCIP data structure */
        ) override;

        /** solving process deinitialization method of transformed data (called before the branch and bound data is freed) */
        SCIP_RETCODE scip_exitsol(
                SCIP*              scip,                /**< SCIP data structure */
                SCIP_Bool          restart              /**< was this exit solve call triggered by a restart? */
        ) override;

        model_data* getData()
        {
            return modelData_;
        }
    };

} /* namespace vrp */

/** sets up the problem data */
SCIP_RETCODE SCIPprobdataCreate(
        SCIP*                 scip,               /**< SCIP data structure */
        model_data*           modeldata,           /**< model data */
        std::vector<tourVRP>& sol_tvrps,
        int                   activate_propagator
);

/** adds given variable to the problem data */
SCIP_RETCODE SCIPprobdataAddVar(
        SCIP*                   scip,                   /**< SCIP data structure */
        vrp::ProbDataVRP*       objprobdata,            /**< problem data */
        SCIP_VAR*               var                     /**< variables to add */
);

#endif
