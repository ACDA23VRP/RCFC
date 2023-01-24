#include <iostream>

#include "json.hpp"
#include "objscip/objscip.h"
#include "eventhdlr_nodeInit.hpp"
#include "vardata.h"

const auto EVENT = SCIP_EVENTTYPE_NODEFOCUSED;

EventhdlrNodeInit::EventhdlrNodeInit(SCIP *scip)
        : ObjEventhdlr(scip, "newnode", "event handler for node initialization") {
}

/** destructor of event handler to free user data (called when SCIP is exiting) */
SCIP_DECL_EVENTFREE(EventhdlrNodeInit::scip_free) {  /*lint --e{715}*/
    return SCIP_OKAY;
}


/** initialization method of event handler (called after problem was transformed) */
SCIP_DECL_EVENTINIT(EventhdlrNodeInit::scip_init) {  /*lint --e{715}*/
    return SCIP_OKAY;
}


/** deinitialization method of event handler (called before transformed problem is freed) */
SCIP_DECL_EVENTEXIT(EventhdlrNodeInit::scip_exit) {  /*lint --e{715}*/
    return SCIP_OKAY;
}


/** solving process initialization method of event handler (called when branch and bound process is about to begin)
 *
 *  This method is called when the presolving was finished and the branch and bound process is about to begin.
 *  The event handler may use this call to initialize its branch and bound specific data.
 *
 */
SCIP_DECL_EVENTINITSOL(EventhdlrNodeInit::scip_initsol) {
    SCIP_CALL(SCIPcatchEvent(scip, EVENT, eventhdlr, nullptr, nullptr));

    return SCIP_OKAY;
}


/** solving process deinitialization method of event handler (called before branch and bound process data is freed)
 *
 *  This method is called before the branch and bound process is freed.
 *  The event handler should use this call to clean up its branch and bound data.
 */
SCIP_DECL_EVENTEXITSOL(EventhdlrNodeInit::scip_exitsol) {
    return SCIP_OKAY;
}


/** frees specific constraint data */
SCIP_DECL_EVENTDELETE(EventhdlrNodeInit::scip_delete) {  /*lint --e{715}*/
    return SCIP_OKAY;
}


/** execution method of event handler
 *
 *  Processes the event. The method is called every time an event occurs, for which the event handler
 *  is responsible. Event handlers may declare themselves responsible for events by calling the
 *  corresponding SCIPcatch...() method. This method creates an event filter object to point to the
 *  given event handler and event data.
 */
SCIP_DECL_EVENTEXEC(EventhdlrNodeInit::scip_exec) {  /*lint --e{715}*/
//    if(!VARIABLE_FIXING)
//        return SCIP_OKAY;

    SCIP_Node* node = SCIPgetFocusNode(scip);
    long long int num_node = SCIPnodeGetNumber(node);

    if(num_node > 1)
    {
//        cout << "NODE " << num_node << " Parent: " << SCIPnodeGetNumber(SCIPnodeGetParent(node)) << endl;
        auto* pricerData = dynamic_cast<ObjPricerVRP*>(SCIPfindObjPricer(scip_, "VRP_Pricer"));
        auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_));
        node_data& nodeData = pricerData->tree_data_.at(SCIPnodeGetNumber(SCIPnodeGetParent(node)));

        SCIP_Bool fixed;
        SCIP_Bool infeasible;
        /* check if the (new) vars stand in conflict with the parents node data */
        for(int i = nodeData.num_vars; i < probData->nVars_; i++)
        {
            fixed = FALSE;
            if(SCIPvarGetUbLocal(probData->vars_[i]) < 0.5)
                continue;
            auto* vardata = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, probData->vars_[i]));
            tourVRP& tvrp = vardata->tourVrp_;
            int day = vardata->getDay();
//            for(auto v : vardata->tourVrp_.tour_)
//            {
//                if(!nodeData.node_timetable_[v][day])
//                {
//                    SCIP_CALL( SCIPfixVar(scip, probData->vars_[i], 0.0, &infeasible, &fixed) );
//                    assert(!infeasible);
//                    assert(fixed);
//                    break;
//                }
//            }
            if(nodeData.node_isForbidden_[tvrp.tour_[tvrp.length_-1]][0] || nodeData.node_isForbidden_[0][tvrp.tour_[0]]
            || !nodeData.node_timetable_[tvrp.tour_[0]][day])
                fixed = TRUE;
            else
            {
                /* check inner arcs and rest of customers */
                for(int v = 1; v < tvrp.length_; v++)
                {
                    if(!nodeData.node_timetable_[tvrp.tour_[v]][day] || nodeData.node_isForbidden_[tvrp.tour_[v-1]][tvrp.tour_[v]])
                    {
                        fixed = TRUE;
                        break;
                    }
                }
            }
            if(fixed)
            {
                SCIP_CALL( SCIPfixVar(scip, probData->vars_[i], 0.0, &infeasible, &fixed) );
                assert(!infeasible);
                assert(fixed);
            }
        }
//        for(auto var : probData->vars_)
//        {
//            if(SCIPvarGetUbLocal(var) < 0.5)
//                continue;
//            auto* vardata = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));
//            int day = vardata->getDay();
//            for(auto v : vardata->tourVrp_.tour_)
//            {
//                if(!nodeData.node_timetable_[v][day])
//                {
//                    assert(FALSE);
//                    SCIP_CALL( SCIPfixVar(scip, var, 0.0, &infeasible, &fixed) );
//                    assert(!infeasible);
//                    assert(fixed);
//                    break;
//                }
//            }
//        }
    }
    return SCIP_OKAY;
}


