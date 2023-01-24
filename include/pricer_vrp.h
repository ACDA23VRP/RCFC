
#ifndef __SCIP_PRICER_VRP_H__
#define __SCIP_PRICER_VRP_H__

#include "objscip/objscip.h"
#include "scip/pub_var.h"
#include "probdata_vrp.h"
#include "model_data.h"
#include "prop_varfixing.h"
#include "prop_tourvarfixing.h"
#include <vector>
#include <list>

using namespace std;
using namespace scip;

enum BranchType
{
    BRANCHTYPE_NOTBRANCHED = 0,
    BRANCHTYPE_DAYVAR = 1,
    BRANCHTYPE_ARCFLOW = 2,
    BRANCHTYPE_VEHICLE = 3,
    BRANCHTYPE_NVEHICLE = 4
};
typedef enum BranchType BRANCHTYPE;

/** branching tree data */
typedef struct node_data {
    bool                                visitedChild;      /**< true iff one child of node has already been visited */
    bool                                gotFixed;
    int                                 num_vars;          /**< number of variables when node got finished */
    BRANCHTYPE                          branchtype_;       /**< type of branching applied at that node */
    double                              node_varfixing_;   /**< the last gap value where variable fixing has been applied */
    vector<bool>                        node_fixedDay_;    /**< days with a 1-fixed variable in subtree */
    vector<bitset<neighborhood_size>>   node_ng_DSSR_;     /**< DSSR data for branching nodes */
    vector<vector<bool>>                node_timetable_;   /**< timetable for branching nodes */
    vector<vector<bool>>                node_isForbidden_; /**< isForbidden for branching nodes */
} node_data;

/** pricer class */
class ObjPricerVRP : public ObjPricer
{
public:
    SCIP_CONSHDLR*                          cons_arcflow_;   /**< constraint handler for arc flow branching */
    SCIP_CONSHDLR*                          cons_dayvar_;    /**< constraint handler for day branching */
    SCIP_CONSHDLR*                          cons_kpc_;       /**< constraint handler for k-path cuts */
    SCIP_CONSHDLR*                          cons_src_;       /**< constraint handler for subset row cuts */
    SCIP_CONSHDLR*                          cons_vehicle_;   /**< constraint handler for vehicle branching */
    SCIP_CONSHDLR*                          cons_nVehicle_;  /**< constraint handler for nVehicle branching */
    ObjPropVarFixing*                       prop_varfixing_; /**< propagator for variable fixing */
    ObjPropTourVarFixing*                   prop_tourfixing_;/**< propagator for variable fixing */
    long long int                           lastID_;         /**< branching node id of last iteration */
    SCIP_Real                               dual_nVehicle_;  /**< dual value of nVehicle constraint */
    vector<SCIP_Real>                       dualValues_;     /**< dual values of the current iteration */
    vector<vector<vector<int>>>             neighbors_;      /**< local neighborhood of each [customer][day] */
    vector<vector<vector<int>>>             predecessors_;   /**< local predecessors of each [customer][day] */
    vector< int >                           eC_;             /**< if customer is enforced, entry will be set to day, else -1 */
    vector< int >                           nEC_;            /**< number of enforced customers for each day */
    vector< bool >                          toDepot_;        /**< if arc to depot is active at current branching node */
    vector< bool >                          fixedDay_;       /**< indicates if a pricing for a day should not be applied */
    vector<SCIP_CONS*>                      eDays_;          /**< constraints of enforced days to receive dual variable from */
    vector<vector<bool>>                    timetable_;      /**< timetable[customer][day]=TRUE if customer visitable at day, else FALSE */
    vector<vector<bool>>                    global_timetable_;   /**< timetable[customer][day]=TRUE if customer visitable at day, else FALSE */
    vector<vector<bool>>                    isForbidden_;    /**< matrix that indicates if an arc between two customers if forbidden due to arc flow branching */
    vector<vector<bool>>                    global_isForbidden_; /**< matrix that indicates if an arc between two customers if forbidden due to arc flow branching */
    vector<vector<SCIP_Real>>               arcPrices_;      /**< the costs of traversing the arcs in the given pricing iteration */
    vector<vector<SCIP_Real>>               dayVarRedCosts_; /**< reduced costs of day-customer assignment variables */
    vector<vector<SCIP_Real>>               root_dayVarRedCosts_; /**< reduced costs of day-customer assignment variables */
    vector<vector<SCIP_Real>>               root_dayVarLPObj_; /**< reduced costs of day-customer assignment variables */
    vector<vector<SCIP_Real>>               arcRedCosts_;    /**< reduced costs of arc flow variables */
    vector<vector<SCIP_Real>>               root_arcRedCosts_;    /**< reduced costs of arc flow variables */
    vector<vector<SCIP_Real>>               root_arcLPObj_;    /**< reduced costs of arc flow variables */
    long long int                           node_varsfixed_; /**< number of node of last propargator call */
    bool                                    fixed_nonzero_;  /**< propagator fixed nonzero variable */
    vector<bitset<neighborhood_size>>       ng_DSSR_;        /**< decremental state space relaxation for ng-neighborhood */
    unordered_map<long long int, node_data> tree_data_;      /**< tree data of visited nodes with at least one unvisited child node */
    double                                  varfixing_gap_;  /**< the last gap value where variable fixing has been applied */
    int                                     nSRC_;           /**< number of robust cuts at the last iteration at node */
    int                                     nnonzSRC_;       /**< number of robust cuts with non zero dual variable in the current LP solution */
    vector<vector<bool>*>                   SRC_Set_;        /**< customer sets of robust cuts */
    vector<double>                          SRC_para_;       /**< parameters of active robust cuts */
    vector<double>                          SRC_dualv;       /**< current dual values of active robust cuts */
    bool                                    atRoot_;
    vector<bool>                                     dayisone_;
    vector<long long int>                           nodeisone_;
    vector<int>                                     depthisone_;

    /** Constructs the pricer object with the data needed */
    ObjPricerVRP(
        SCIP*                                 scip,           /**< SCIP pointer */
        const char*                           p_name         /**< name of pricer */
    );

    /** Destructs the pricer object. */
    ~ObjPricerVRP() override;

    /** initialization method of variable pricer (called after problem was transformed) */
    virtual SCIP_DECL_PRICERINIT(scip_init);

    /** reduced cost pricing method of variable pricer for feasible LPs */
    virtual SCIP_DECL_PRICERREDCOST(scip_redcost);

    /** farkas pricing method of variable pricer for infeasible LPs */
    virtual SCIP_DECL_PRICERFARKAS(scip_farkas);

    /** perform pricing */
    SCIP_RETCODE pricing(
        SCIP*              scip,               /**< SCIP data structure */
        bool               isFarkas            /**< whether we perform Farkas pricing */
    );

    /** return negative reduced cost tour (uses restricted shortest path dynamic programming algorithm) */
    SCIP_RETCODE generate_tours(
        SCIP*                   scip,
        vrp::ProbDataVRP*       probData,
        bool                    isFarkas,
        bool                    getDayVarRed,
        bool                    isHeuristic
    );

    /** return negative reduced cost tour (uses restricted shortest path dynamic programming algorithm) */
    bool heuristic_pricing(
            SCIP*                       scip,
            vrp::ProbDataVRP*           probData,
            bool                   isFarkas
    );

    SCIP_Real getTourVRPredcosts(
            model_data*     modelData,
            tourVRP&        tvrp
    );

    SCIP_RETCODE set_current_graph(
        model_data*         modelData
    );

    /** receives dual values of current LP */
    SCIP_RETCODE setDualValues(
        SCIP*               scip,
        vrp::ProbDataVRP*   probDataVrp,
        bool           isFarkas
    );

    /** sets the arc prices for current pricing iteration */
    SCIP_RETCODE setArcPrices(
        model_data*         modelData,
        bool                isFarkas
    );

    /** includes subset row cut data into pricing */
    SCIP_RETCODE includeSRCpricingData(
        SCIP*               scip,
        bool                isFarkas
    );

};

#endif
