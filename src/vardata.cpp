
#include "objscip/objscip.h"
#include "probdata_vrp.h"
#include <vector>
#include "vardata.h"
#include "ConshdlrKPC.h"
#include "ConshdlrSRC.h"
#include "ConshdlrVehicle.h"
#include "ConshdlrNVehicle.h"
#include "scip/cons_setppc.h"
#include "scip/cons_linear.h"
#include "pricer_vrp.h"

bool SCIPcontainsTourVar(
        SCIP*                   scip,
        vrp::ProbDataVRP*       probData,
        SCIP_Var**              existingVar,
        tourVRP&                tvrp
){
    for(auto var : probData->vars_)
    {
        tourVRP& tvrp_old = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var))->tourVrp_;
        if(tvrp_old.getDay() != tvrp.getDay())
            continue;
        if(tvrp_old.length_ != tvrp.length_)
            continue;
        bool same = true;
        for(int i = 0; i < tvrp.length_; i++)
        {
            if(tvrp_old.tour_[i] != tvrp.tour_[i])
            {
                same = false;
                break;
            }
        }
        if(same)
        {
//            cout << "IT HAPPENS!" << endl;
//            cout << tvrp;
//            SCIPprintVar(scip, var, nullptr);
//            cout << "Redcosts: " << SCIPgetVarRedcost(scip, var) << endl;
            *existingVar = var;
            return true;
        }
    }
    return false;
}

static
SCIP_RETCODE addVarToNVehicleCons(
        SCIP*           scip,
        SCIP_Var*       var
){
    SCIP_CONSHDLR* conshdlr = SCIPfindConshdlr(scip, "nVehicle");
    for(int c = 0; c < SCIPconshdlrGetNConss(conshdlr); c++)
    {
        SCIP_CONS *cons = SCIPconshdlrGetConss(conshdlr)[c];
        if(!SCIPconsIsActive(cons))
            continue;
        SCIP_CALL(SCIPaddCoefNVehicleCons(scip, cons, var));
    }
    return SCIP_OKAY;
}

static
SCIP_RETCODE addVarToVehicleCons(
    SCIP*           scip,
    tourVRP&        tvrp,
    SCIP_Var*       var
){
    SCIP_CONSHDLR* conshdlr = SCIPfindConshdlr(scip, "Vehicle");
    for(int c = 0; c < SCIPconshdlrGetNConss(conshdlr); c++)
    {
        SCIP_CONS *cons = SCIPconshdlrGetConss(conshdlr)[c];

        if(!SCIPconsIsActive(cons))
            continue;
        if(SCIPgetRowOfVehicleCons(cons) == nullptr)
        {
            cout << "IT HAPPENS!!!" << endl << endl << endl;
            continue;
        }
        if(SCIPgetDayOfVehicleCons(cons) == tvrp.getDay())
        {
            SCIP_CALL(SCIPaddCoefVehicleCons(scip, cons, var));
        }
    }
    return SCIP_OKAY;
}

static
SCIP_RETCODE addVarToSRC(
    SCIP*           scip,
    tourVRP&        tvrp,
    SCIP_Var*       var
){
    double p;
    SCIP_CONSHDLR* conshdlr = SCIPfindConshdlr(scip, "SRC");
    for(int c = 0; c < SCIPconshdlrGetNConss(conshdlr); c++)
    {
        SCIP_CONS* cons = SCIPconshdlrGetConss(conshdlr)[c];
        double val = 0.0;
        vector<bool>& setS = *SCIPgetSetOfSRC(cons);
        p = SCIPgetpOfSRC(cons);
        for(auto u : tvrp.tour_)
        {
            if(setS[u])
                val += p;
        }
        val = floor(val);
        if(SCIPisPositive(scip, val))
        {
            SCIP_CALL(SCIPaddCoefSRC(scip, cons, var, val));
        }
    }
    return SCIP_OKAY;
}

static
SCIP_RETCODE addVarToKPC(
    SCIP*           scip,
    const tourVRP   &tvrp,
    SCIP_Var        *var
){
    int i;
    SCIP_CONSHDLR* conshdlr = SCIPfindConshdlr(scip, "KPC");
    for(int c = 0; c < SCIPconshdlrGetNConss(conshdlr); c++)
    {
        SCIP_CONS* cons = SCIPconshdlrGetConss(conshdlr)[c];
        double val = 0.0;
        vector<bool>& setS = SCIPgetSetOfKPC(cons);
        if(setS[tvrp.tour_[0]])
            val += 1;
        for(i = 1; i < tvrp.length_; i++)
        {
            if(setS[tvrp.tour_[i]])
            {
                if(!setS[tvrp.tour_[i-1]])
                {
                    val += 1;
                }
                /* we can skip the next spot */
                i++;
            }
        }
        if(SCIPisPositive(scip, val))
        {
            SCIP_CALL(SCIPaddCoefKPC(scip, cons, var, val));
        }
    }
    return SCIP_OKAY;
}

SCIP_RETCODE SCIPcreateColumn(
        SCIP*                   scip,
        vrp::ProbDataVRP*       probData,
        const char*             name,
        SCIP_Bool               initial,
        tourVRP&                tvrp
){
    ObjVarDataVRP* varData;
    SCIP_VAR* var;
    model_data* modelData = probData->getData();
    int i;
    assert(probData != nullptr);
    assert(probData->getData() != nullptr);

    assert(tvrp.length_ > 0 || initial);
    assert(tvrp.getDay() >= 0);

//    assert(tvrp.isFeasible(modelData));
    /* generate coefficients */
    bool isElementary = true;
    vector<double> coeffs(modelData->nC, 0.0);
    for(i = 0; i < tvrp.length_; i++)
    {
        if(coeffs[tvrp.tour_[i]] > 0)
            isElementary = false;
        assert(0 < tvrp.tour_[i] && tvrp.tour_[i] < modelData->nC);
        coeffs[tvrp.tour_[i]] += 1.0;
    }

//    SCIP_Var* test;
//    if(SCIPcontainsTourVar(scip, probData, &test, tvrp))
//    {
//        cout << name << endl;
//        cout << SCIPgetLocalLowerbound(scip) << endl;
//        cout << SCIPgetLPObjval(scip) << endl;
//        cout << SCIPgetPrimalbound(scip) << endl;
////        assert(false);
//    }

    // TODO: only tvrp in vardata
    varData = new ObjVarDataVRP(tvrp, tvrp.tour_, tvrp.length_, tvrp.getDay(), tvrp.capacity_, isElementary);
    /* create variable for the vrp which contains only this customer */
    if(!modelData->minTravel)
        tvrp.obj_ = 1.0;
    SCIP_CALL(SCIPcreateObjVar(scip, &var, name, 0.0, 1.0, tvrp.obj_, SCIP_VARTYPE_BINARY, initial, TRUE, varData, TRUE));

//    SCIP_CALL(SCIPcreateObjVar(scip, &var, name, 0.0, 1.0, 1.0, SCIP_VARTYPE_BINARY, initial, TRUE, varData, TRUE));

    /* add variable to the problem */
    if (initial)
    {
        SCIP_CALL( SCIPaddVar(scip, var) );
    } else {
        SCIP_CALL( SCIPaddPricedVar(scip, var, 1.0) );
    }
//    if(tvrp.length_ == 4 && tvrp.getDay() == 16 && tvrp.tour_[0] == 37 && tvrp.tour_[1] == 42 && tvrp.tour_[2] == 44 && tvrp.tour_[3] == 13)
//        cout << "AT NODE " << SCIPnodeGetNumber(SCIPgetCurrentNode(scip)) << endl;

    /* change the upper bound of the binary variable to lazy since the upper bound is already enforced
     * due to the objective function in the set partitioning constraint;
     * The reason for doing is that, is to avoid the bound of x <= 1 in the LP relaxation since this bound
     * constraint would produce a dual variable which might have a positive reduced cost
     */

//    if(SCIPgetStage(scip) == 9 && SCIPnodeGetNumber(SCIPgetCurrentNode(scip) ) == 11)
//    {
//        cout << "!!!!!!!!!!!!!!!" << endl;
//    }else
//    {
        SCIP_CALL( SCIPchgVarUbLazy(scip, var, 1.0) );
//    }
    /* store variable in the problem data */
    if (initial)
    {
        SCIP_CALL( SCIPprobdataAddVar(scip, probData, var) );
    }
    for(auto u : tvrp.tour_)
    {
        if (coeffs[u] == 0)
            continue;
        assert(!strncmp(SCIPconshdlrGetName(SCIPconsGetHdlr(probData->cons_[u - 1])), "linear", 6));
        SCIP_CALL(SCIPaddCoefLinear(scip, probData->cons_[u - 1], var, coeffs[u]));
        coeffs[u] = 0;
    }

//    for(i = 1; i < modelData->nC; i++)
//    {
//        if (coeffs[i] == 0)
//            continue;
//        assert(!strncmp(SCIPconshdlrGetName(SCIPconsGetHdlr(probData->cons_[i - 1])), "linear", 6));
//        SCIP_CALL(SCIPaddCoefLinear(scip, probData->cons_[i - 1], var, coeffs[i]));
//    }

    /* add variable to corresponding set partitioning constraint */
    if(modelData->num_v[tvrp.getDay()] == 1)
    {
        assert( !strncmp( SCIPconshdlrGetName( SCIPconsGetHdlr(probData->cons_[modelData->nC - 1 + tvrp.getDay()]) ), "setppc", 6) );
        SCIP_CALL( SCIPaddCoefSetppc(scip, probData->cons_[modelData->nC - 1 + tvrp.getDay()], var) );
    }else
    {
        assert( !strncmp( SCIPconshdlrGetName( SCIPconsGetHdlr(probData->cons_[modelData->nC - 1 + tvrp.getDay()]) ), "linear", 6) );
        SCIP_CALL( SCIPaddCoefLinear(scip, probData->cons_[modelData->nC - 1 + tvrp.getDay()], var, 1) );
    }


    if(!initial)
    {
        /* add var to k-path cuts */
        SCIP_CALL(addVarToKPC(scip, tvrp, var));

        /* add var to subset row cuts */
        SCIP_CALL(addVarToSRC(scip, tvrp, var));

        /* add var to vehicle constraints */
        SCIP_CALL(addVarToVehicleCons(scip, tvrp, var));

        /* add var to nVehicle constraint */
        SCIP_CALL(addVarToNVehicleCons(scip, var));
    }

    /* release variable */
    SCIP_CALL( SCIPreleaseVar(scip, &var) );


    return SCIP_OKAY;
}


