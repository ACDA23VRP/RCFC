
#include "var_tools.h"

static
SCIP_RETCODE increaseValue(
    SCIP*                       scip,
    vector<vector<SCIP_Real>>&  values,
    SCIP_Real                   val,
    int*                        narcs,
    int                         tail,
    int                         head
){
    if(SCIPisSumEQ(scip, values[tail][head] + values[head][tail], 0.0))
        (*narcs)++;
    values[tail][head] += val;
//    if(SCIPisSumPositive(scip, values[tail][head] - 1))
//        cout << setprecision(12) << fixed << "VALUE " << values[tail][head] << endl;
//    assert(!SCIPisSumPositive(scip, values[tail][head] - 1));
    return SCIP_OKAY;
}

SCIP_RETCODE getArcFlowValues(
    SCIP*                       scip,
    vector<vector<SCIP_Real>>&  values,
    int*                        narcs
){
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip));
    *narcs = 0;
    SCIP_Real lpval;

    for(auto var : probData->vars_)
    {
        lpval = SCIPvarGetLPSol(var);

        if(SCIPisSumPositive(scip, lpval))
        {
            auto* vardata = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));

            SCIP_CALL(increaseValue(scip, values, lpval, narcs, 0, vardata->tour_[0]));
            SCIP_CALL(increaseValue(scip, values, lpval, narcs, vardata->tour_[vardata->getLength() - 1], 0));
//            values[0][vardata->tour_[0]] += lpval;
//            values[vardata->tour_[vardata->getLength() - 1]][0] += lpval;
            for(int j = 1; j < vardata->getLength(); j++)
            {
//                values[vardata->tour_[j - 1]][vardata->tour_[j]] += lpval;
                SCIP_CALL(increaseValue(scip, values, lpval, narcs, vardata->tour_[j - 1], vardata->tour_[j]));
            }
        }
    }
    assert(*narcs > 0);

    return SCIP_OKAY;
}

SCIP_RETCODE getVehiAssValues(
    SCIP*                       scip,
    vector<vector<SCIP_Real>>&  values,
    vector<int>&                numofDays
){
    int j, day;
    SCIP_Real lpval;
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip));

    for(auto var : probData->vars_)
    {
        lpval = SCIPvarGetLPSol(var);

        if(SCIPisSumPositive(scip, lpval))
        {
            auto* vardata = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));
            if(vardata->getLength() == 0) continue;
            day = vardata->getDay();
            for(j = 0; j < vardata->getLength(); j++)
            {
                if(values[vardata->tour_[j]][day] == 0)
                    numofDays[vardata->tour_[j]]++;
                values[vardata->tour_[j]][day] += lpval;
            }
        }
    }
    return SCIP_OKAY;
}

bool varInRow(
        SCIP_VAR*                   var,
        SCIP_ROW*                   row
){
//    SCIP_COL* col = SCIPvarGetCol(var);
//    for(int i = 0; i < SCIPcolGetNNonz(col); i++)
//    {
//        SCIP_ROW* comp_row = SCIPcolGetRows(col)[i];
//        if(comp_row == row)
//            return true;
//    }
    SCIP_COL* col = SCIPvarGetCol(var);
    for(int i = 0; i < SCIProwGetNNonz(row); i++)
    {
        if(SCIProwGetCols(row)[i] == col)
            return true;
    }

    return false;
}

SCIP_RETCODE SCIPsortNonzeroVars(
        SCIP*                       scip,
        vrp::ProbDataVRP*           probData
){
    assert(probData->nonzeroVars_.empty());
    for(auto* var : probData->vars_)
    {
        if(SCIPisPositive(scip, SCIPvarGetLPSol(var)))
        {
            probData->nonzeroVars_.push_back(var);
        }
    }
    sort(probData->nonzeroVars_.begin(), probData->nonzeroVars_.end(), [](auto &left, auto &right){
        return SCIPvarGetLPSol(left) > SCIPvarGetLPSol(right);
    });

//    for(auto* var : probData->nonzeroVars_)
//        cout << SCIPvarGetLPSol(var) << " " << endl;
//    assert(false);

    return SCIP_OKAY;
}
