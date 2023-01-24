
#include <iostream>

#include "ConshdlrKPC.h"
#include "objscip/objscip.h"

#include "probdata_vrp.h"
#include "var_tools.h"
#include "tsptw.h"
#include "scip/cons_linear.h"

#define CONSHDLR_NAME3   "KPC"

/** Constraint data for "KPC" constraints */
struct SCIP_ConsData
{
    vector<bool>*       customerSet;    /**< Set S of customers defining the lhs */
    int                 k;              /**< min. Number of paths needed to serve S */
    SCIP_Row*           cut;            /**< Corresponding row in the LP */
    SCIP_VAR**          vars;           /**< variables of the constraint */
    SCIP_Real*          coeffs;         /**< coefficients of variables in the constraint */
    int                 nvars;          /**< number of variables in the constraint */
    int                 varssize;       /**< size of var array */
};

/** local methods */
static
SCIP_RETCODE consdataCreate(
    SCIP*           scip,
    SCIP_CONSDATA** consdata,
    vector<bool>&   setS,
    int             k
){
    SCIP_CALL(SCIPallocBlockMemory(scip, consdata));

    (*consdata)->customerSet = new vector<bool>(setS);
    (*consdata)->k = k;
    (*consdata)->cut = nullptr;
    (*consdata)->nvars = 0;
    (*consdata)->varssize = 1;
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &(*consdata)->vars, 1));
    SCIP_CALL(SCIPallocBlockMemoryArray(scip, &(*consdata)->coeffs, 1));

    return SCIP_OKAY;
}

/** ensures, that the vars array can store at least num entries */
static
SCIP_RETCODE consdataEnsureVarsSize(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_CONSDATA*        consdata,           /**< setppc constraint data */
        int                   num                 /**< minimum number of entries to store */
)
{
    assert(consdata != nullptr);
    assert(consdata->nvars <= consdata->varssize);

    if( num > consdata->varssize )
    {
        int newsize;

        newsize = SCIPcalcMemGrowSize(scip, num);
        SCIP_CALL( SCIPreallocBlockMemoryArray(scip, &consdata->vars, consdata->varssize, newsize) );
        SCIP_CALL( SCIPreallocBlockMemoryArray(scip, &consdata->coeffs, consdata->varssize, newsize) );
        consdata->varssize = newsize;
    }
    assert(num <= consdata->varssize);

    return SCIP_OKAY;
}

static
SCIP_RETCODE addVarToCons(
        SCIP*               scip,
        SCIP_VAR*           var,
        SCIP_CONSDATA*      consdata,
        double              value
){
    assert(consdata != nullptr);
    assert(var != nullptr);

    SCIP_CALL( consdataEnsureVarsSize(scip, consdata, consdata->nvars+1) );
    consdata->vars[consdata->nvars] = var;
    consdata->coeffs[consdata->nvars] = value;
    consdata->nvars++;

    SCIP_CALL(SCIPaddVarToRow(scip, consdata->cut, var, value));

    return SCIP_OKAY;
}

static
SCIP_RETCODE addVarsToCons(
        SCIP*               scip,
        vector<bool>*       setS,
        SCIP_CONSDATA*      consdata
){
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip));
    for(auto var : probData->vars_)
    {
        auto* vardata = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));
        tourVRP& tvrp = vardata->tourVrp_;
        if(tvrp.length_ == 0)
            continue;
        SCIP_Real value = 0.0;
        assert(!(*setS)[0]);
        if((*setS)[tvrp.tour_[0]])
            value += 1.0;
        for(int i = 1; i < tvrp.length_; i++)
        {
            if((*setS)[tvrp.tour_[i-1]])
                continue;
            if((*setS)[tvrp.tour_[i]])
                value += 1.0;
        }
        if(SCIPisPositive(scip, value))
        {
            SCIP_CALL(addVarToCons(scip, var, consdata, value));
        }
    }
    return SCIP_OKAY;
}

static
bool needTwoRoutes(
        model_data*         modelData,
        vector<bool>&       setS,
        vector<int>&        nodeList
){
    int count;
    int setSize = (int) nodeList.size();
    for(int day = 0; day < modelData->nDays; day++)
    {
        /* first check if every customer is available on that day */
        count = 0;
        for(auto v : modelData->neighbors[0][day])
        {
            if(setS[v])
                count++;
        }
        if(count == (int) nodeList.size())
        {
            /* check if each pair of customers in nodeset has at least on arc */
            bool conflict = false;
            for(int i = 0; i < setSize; i++)
            {
                for(int j = i + 1; j < setSize; j++)
                {
                    if(!modelData->adjacency_k[day][nodeList[i]][nodeList[j]] && !modelData->adjacency_k[day][nodeList[j]][nodeList[i]])
                    {
                        conflict = true;
                        break;
                    }
                }
                if(conflict)
                    break;
            }
            if(conflict)
                continue;

            if((int) nodeList.size() >= 3)
            {
                // TODO: inefficient
//                if(!feasibleTSP(modelData, nodeList, day))
//                {
//                    assert(nodeList[nodeList.size()-1] == 0);
//                    nodeList.pop_back();
//                    continue;
//                }
            }
            return false;
        }
    }

    return true;
}

static
SCIP_RETCODE greedy2PC(
        SCIP*                       scip,
        model_data*                 modelData,
        vector<vector<SCIP_Real>>&  arcflowvalues,
        vector<vector<bool>>&       cutSets,
        vector<SCIP_Real>&          cutValues,
        vector<vector<bool>>&       checkedSets,
        vector<bool>                setS,
        vector<int>                 nodeList,
        SCIP_Real                   xval,
        int                         nextC
){
    if(nextC > 0)
    {
        setS[nextC] = true;
        nodeList.push_back(nextC);
    }
    bool maximal = true;
    for(int i = 1; i < modelData->nC; i++)
    {
        if(setS[i])
            continue;
        SCIP_Real newval = xval;
        for(int j = 0; j < modelData->nC; j++)
        {
            if(setS[j])
            {
                newval -= arcflowvalues[i][j];
            }else
            {
                newval += arcflowvalues[j][i];
            }
        }
        if(SCIPisNegative(scip, newval - 2))
        {
            maximal = false;
            SCIP_CALL(greedy2PC(scip, modelData, arcflowvalues, cutSets, cutValues, checkedSets, setS, nodeList, newval, i));
        }
    }
    if(maximal)// && nodeList.size() > 4)
    {
        assert(SCIPisLT(scip, xval, 2));
        for(auto val : cutValues)
            if(SCIPisEQ(scip, val, xval))
                return SCIP_OKAY;
        for(auto& set : checkedSets)
            if(set == setS)
                return SCIP_OKAY;

        checkedSets.emplace_back(setS);
        if(needTwoRoutes(modelData, setS, nodeList))
        {
            /* new cut found */
            cutValues.push_back(xval);
            cutSets.emplace_back(setS);
        }
    }
    return SCIP_OKAY;
}

static
SCIP_RETCODE find2PathCuts(
        SCIP*                       scip,
        vrp::ProbDataVRP*           probData,
        vector<vector<SCIP_Real>>&  arcflowvalues,
        vector<vector<bool>>&        cutSets
){
//    cout << "Find 2PCs!"<< endl;
    model_data* modelData = probData->getData();
    vector<bool> emptySet(modelData->nC, false);
    vector<int> nodeList;
    vector<SCIP_Real> cutValues;
    vector<vector<bool>> checkedSets;

    SCIP_CALL(greedy2PC(scip, modelData, arcflowvalues, cutSets, cutValues, checkedSets, emptySet, nodeList, 0.0, 0));
//    cout << "finished" << endl;
    return SCIP_OKAY;
}

static
SCIP_RETCODE separateNewCuts(
    SCIP*           scip,
    SCIP_CONS**     conss,
    int             nconss,
    SCIP_RESULT*    result
){
    int narcs;
    vector<int> nonzeroes;

    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip));
    model_data* modelData = probData->getData();
    vector<vector<SCIP_Real>> arcflowvalues(modelData->nC, vector<SCIP_Real>(modelData->nC, 0.0));

    SCIP_CALL(getArcFlowValues(scip, arcflowvalues, &narcs));


    vector<vector<bool>> cutSets;
    SCIP_CALL(find2PathCuts(scip, probData, arcflowvalues, cutSets));

    if(!cutSets.empty() )
    {
        *result = SCIP_SEPARATED;
        /* Add cuts to the LP/Pool */
        for(auto set : cutSets)
        {
            bool isnew = true;
            for(int h = 0; h < nconss; h++)
            {
                vector<bool>& cutSets2 = SCIPgetSetOfKPC(conss[h]);
                if(cutSets2 == set)
                {
                    isnew = false;
                    break;
                }
            }
            if(isnew)
            {
                /* create kpc */
                SCIP_Cons* cons;
                SCIPcreateConsKPC(scip, &cons, "2pc", set, 2);

                SCIPaddCons(scip, cons);

                SCIPcreateAndAddRowKPC(scip, cons);
                SCIPreleaseCons(scip, &cons);

            }
        }
    }

    return SCIP_OKAY;
}

/**@name Callback methods
 *
 * @{
 */
/** frees specific constraint data */
SCIP_DECL_CONSDELETE(ConshdlrKPC::scip_delete)
{   /*lint --e{715}*/
    assert(conshdlr != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), "KPC") == 0);
    assert(consdata != nullptr);
    assert(*consdata != nullptr);

    /* delete constraint data */
    delete (*consdata)->customerSet;
    SCIPreleaseRow(scip, &(*consdata)->cut);
    SCIPfreeBlockMemory(scip, consdata);

    return SCIP_OKAY;
}

/** transforms constraint data into data belonging to the transformed problem */
SCIP_DECL_CONSTRANS(ConshdlrKPC::scip_trans)
{   /*lint --e{715}*/
    SCIP_CONSDATA* sourcedata;
    SCIP_CONSDATA* targetdata;

    assert(conshdlr != nullptr);
    assert(strcmp(SCIPconshdlrGetName(conshdlr), "KPC") == 0);
    assert(SCIPgetStage(scip) == SCIP_STAGE_TRANSFORMING);
    assert(sourcecons != nullptr);
    assert(targetcons != nullptr);

    sourcedata = SCIPconsGetData(sourcecons);
    assert(sourcedata != nullptr);

//    /* create constraint data for target constraint */
//    SCIP_CALL( consdataCreate(scip, &targetdata,
//                              sourcedata->customer, sourcedata->day, sourcedata->type, sourcedata->node) );

    /* create target constraint */
    SCIP_CALL( SCIPcreateCons(scip, targetcons, SCIPconsGetName(sourcecons), conshdlr, targetdata,
                              SCIPconsIsInitial(sourcecons), SCIPconsIsSeparated(sourcecons), SCIPconsIsEnforced(sourcecons),
                              SCIPconsIsChecked(sourcecons), SCIPconsIsPropagated(sourcecons),
                              SCIPconsIsLocal(sourcecons), SCIPconsIsModifiable(sourcecons),
                              SCIPconsIsDynamic(sourcecons), SCIPconsIsRemovable(sourcecons), SCIPconsIsStickingAtNode(sourcecons)) );

    return SCIP_OKAY;
}


SCIP_DECL_CONSCOPY(ConshdlrKPC::scip_copy){
    SCIP_VAR** sourcevars;
    SCIP_Real* coefficients;
    const char* consname;
    SCIP_Real lhs;
    SCIP_Real rhs;
    int nvars;
    SCIP_CONSDATA* consdata = SCIPconsGetData(sourcecons);
    assert(consdata != nullptr);

//    cout << "copy constraint with " << consdata->nvars << " (size: " << consdata->varssize << ")" << endl;

    /* get variables and coefficients of the source constraint */
    sourcevars = consdata->vars;
    coefficients = consdata->coeffs;
    nvars = consdata->nvars;

    lhs = consdata->k;
    rhs = SCIPinfinity(scip);

    if( name != nullptr )
        consname = name;
    else
        consname = SCIPconsGetName(sourcecons);

    /* copy the logic using the linear constraint copy method */
    SCIP_CALL( SCIPcopyConsLinear(scip, cons, sourcescip, consname, nvars, sourcevars, coefficients,
                                  lhs, rhs, varmap, consmap, true, separate, enforce, check, propagate,
                                  local, modifiable, dynamic, removable, stickingatnode, global, valid) );
    assert(cons != nullptr);

    return SCIP_OKAY;
}

SCIP_DECL_CONSSEPALP(ConshdlrKPC::scip_sepalp){ // TODO
//    return SCIP_OKAY;
    *result = SCIP_DIDNOTRUN;
    if(SCIPgetDepth(scip) > 2)
        return SCIP_OKAY;
    SCIP_CONSDATA* consdata;
//    cout << "KPC SEPA" << endl;
    *result = SCIP_DIDNOTFIND;
    for(int c = 0; c < nusefulconss; c++)
    {
        consdata = SCIPconsGetData(conss[c]);
        assert(consdata->cut != nullptr);
        if(!SCIProwIsInLP(consdata->cut))
        {
            double feasibility = SCIPgetRowSolFeasibility(scip, consdata->cut, nullptr);
            SCIP_Bool addcut = SCIPisFeasNegative(scip, feasibility);
            if(addcut)
            {
//                SCIPprintCons(scip, conss[c], nullptr);
                SCIPresetConsAge(scip, conss[c]);
                SCIP_CALL( SCIPaddRow(scip, consdata->cut, TRUE, &addcut) );
                *result = SCIP_SEPARATED;
            }
        }
    }
    if(*result != SCIP_SEPARATED)
    {
        SCIP_CALL(separateNewCuts(scip, conss, nconss, result));
    }

    return SCIP_OKAY;
}

SCIP_DECL_CONSPRINT(ConshdlrKPC::scip_print)
{
    SCIP_CONSDATA* consdata = SCIPconsGetData(cons);
    cout << "Set S: ";
    for(int i = 0; i < (int) (*consdata->customerSet).size(); i++)
    {
        if((*consdata->customerSet)[i])
        {
            cout << i << " ";
        }
    }
    cout << endl;
    cout << "min. num of path needed: " << consdata->k << endl;
    if(consdata->cut != nullptr)
    {
        SCIPprintRow(scip, consdata->cut, nullptr);
    }else
    {
        cout << "ALERT: no row included!" << endl;
    }
    return SCIP_OKAY;
}

/** global methods */

/** gets the dual solution of the k-paths constraint in the current LP */
SCIP_Real SCIPgetDualsolKPC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS*              cons                /**< constraint data */
){
    SCIP_CONSDATA* consdata;

    assert(scip != nullptr);

    if( strcmp(SCIPconshdlrGetName(SCIPconsGetHdlr(cons)), CONSHDLR_NAME3) != 0 )
    {
        SCIPerrorMessage("constraint is not a k-path constraint\n");
        SCIPABORT();
        return SCIP_INVALID;  /*lint !e527*/
    }

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    if( consdata->cut != nullptr )
        return SCIProwGetDualsol(consdata->cut);
    else
        return 0.0;
}

/** gets the dual Farkas value of the k-paths constraint in the current infeasible LP */
SCIP_Real SCIPgetDualfarkasKPC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS*              cons                /**< constraint data */
){
    SCIP_CONSDATA* consdata;

    assert(scip != nullptr);

    if( strcmp(SCIPconshdlrGetName(SCIPconsGetHdlr(cons)), CONSHDLR_NAME3) != 0 )
    {
        SCIPerrorMessage("constraint is not a k-path constraint\n");
        SCIPABORT();
        return SCIP_INVALID;  /*lint !e527*/
    }

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    if( consdata->cut != nullptr )
        return SCIProwGetDualfarkas(consdata->cut);
    else
        return 0.0;
}

/** adds coefficient in k-path constraint */
SCIP_RETCODE SCIPaddCoefKPC(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_CONS*            cons,               /**< constraint data */
        SCIP_VAR*             var,                /**< variable to add to the constraint */
        SCIP_Real             val                 /**< coefficient of constraint entry */
)
{
    SCIP_CONSDATA* consdata;
    assert(var != nullptr);

    if( strcmp(SCIPconshdlrGetName(SCIPconsGetHdlr(cons)), CONSHDLR_NAME3) != 0 )
    {
        SCIPerrorMessage("constraint is not a k-path constraint\n");
        return SCIP_INVALIDDATA;
    }

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);
    assert(consdata->cut != nullptr);
    SCIP_CALL( addVarToCons(scip, var, consdata, val) );

    return SCIP_OKAY;
}

/** creates and adds the row of a k-path constraint to the LP */
SCIP_RETCODE SCIPcreateAndAddRowKPC(
        SCIP*                   scip,
        SCIP_Cons*              cons
){
    // TODO: try out: go from x(S-) (ingoing arcs) to x(S:S) (in between arcs)
    SCIP_CONSDATA* consdata;
    assert(scip != nullptr);
    assert(cons != nullptr);
    SCIP_Bool infeasible;
    consdata = SCIPconsGetData(cons);
    assert(consdata->cut == nullptr);

    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &(consdata->cut), cons, SCIPconsGetName(cons), consdata->k,
                                     SCIPinfinity(scip),FALSE, TRUE, FALSE));

    assert(consdata->cut != nullptr);

    SCIP_CALL(addVarsToCons(scip, consdata->customerSet, consdata));

    if(!SCIProwIsInLP(consdata->cut))
    {
        SCIP_CALL(SCIPaddRow(scip, consdata->cut, true, &infeasible));
        assert(!infeasible);
    }
    
    return SCIP_OKAY;
}

/** creates and captures a k-path constraint */
SCIP_RETCODE SCIPcreateConsKPC(
        SCIP*                   scip,               /**< SCIP data structure */
        SCIP_CONS**             cons,               /**< pointer to hold the created constraint */
        const char*             name,               /**< name of constraint */
        vector<bool>&           setS,               /**< set of customers */
        SCIP_Real               k                   /**< number of paths needed to serve S */
){
    SCIP_CONSDATA* consdata = nullptr;
    SCIP_CONSHDLR* conshdlr = nullptr;

    assert(scip != nullptr);

    /* find the set partitioning constraint handler */
    conshdlr = SCIPfindConshdlr(scip, CONSHDLR_NAME3);
    assert(conshdlr != nullptr);
    /* create constraint data*/
    SCIP_CALL(consdataCreate(scip, &consdata, setS, k));
    assert(consdata != nullptr);
    /* create constraint */
    SCIP_CALL(SCIPcreateCons(scip, cons, name, conshdlr, consdata,FALSE, TRUE, TRUE, FALSE,
                             FALSE, FALSE, TRUE, FALSE, FALSE, FALSE));

    return SCIP_OKAY;
}

/** returns the Set S of a k-path constraint */
vector<bool>& SCIPgetSetOfKPC(
        SCIP_CONS*              cons
){
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return *consdata->customerSet;
}

/** returns the row of a k-path constraint */
SCIP_Row* SCIPgetRowOfKPC(
        SCIP_CONS*              cons
){
    SCIP_CONSDATA* consdata;

    assert(cons != nullptr);

    consdata = SCIPconsGetData(cons);
    assert(consdata != nullptr);

    return consdata->cut;
}