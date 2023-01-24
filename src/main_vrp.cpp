
#include <stdio.h>
#include <time.h>
#include <string.h>

#include "scip/scip.h"
#include "scip/scipshell.h"
#include "scip/scipdefplugins.h"
#include "objscip/objscip.h"

#include "model_data.h"
#include "probdata_vrp.h"
#include "branchingrule_arcflow.h"
#include "branchingrule_dayvar.h"
#include "branchingrule_vehicle.h"
#include "ConshdlrArcflow.h"
#include "ConshdlrDayVar.h"
#include "ConshdlrKPC.h"
#include "ConshdlrSRC.h"
#include "ConshdlrCPC.h"
#include "ConshdlrVehicle.h"
#include "ConshdlrNVehicle.h"
#include "tourVRP.h"
#include "tools_vrp.h"
#include "eventhdlr_nodeInit.hpp"
#include "printer.h"
#include "heurDayVarRounding.h"
#include "prop_varfixing.h"
#include "prop_tourvarfixing.h"


#include "iostream"
#include "fstream"
#include "json.hpp"

using json = nlohmann::json;

static
SCIP_RETCODE readArguments(
    int                 argc,               /**< number of shell parameters */
    char**              argv,               /**< array with shell parameters */
    char**              input_file,
    int*                dayVarBranching,
    int*                activate_propagator,
    int*                seed
)
{
    char usage[SCIP_MAXSTRLEN];
    int status;
    char* locstr;

    assert( argc >= 1 );
    assert( argv != nullptr );
    assert( input_file != nullptr );

    /* init usage text */
    status = snprintf(usage, SCIP_MAXSTRLEN - 1, "usage: %s <path of inputfile> ", argv[0]);
    assert( 0 <= status && status < SCIP_MAXSTRLEN );

    /* init arguments */
    *input_file = nullptr;

    /* mandatory argument: inputfile */
    *input_file = argv[1];
    if ( *input_file == nullptr )
    {
        fprintf(stderr, "No path of data supplied.\n");
        fprintf(stderr, "%s\n", usage);
        return SCIP_ERROR;
    }
    /* check for branching strategy */
    for (int i = 2; i < argc; i++)
    {
        if ( ! strcmp(argv[i], "-b"))
        {
            if( i == argc - 1 || (! strncmp(argv[i+1], "-",1)))
            {
                fprintf(stderr, "Missing objective function weigth. ");
                SCIPerrorMessage("%s\n", usage);
                return SCIP_ERROR;
            }
            i++;
            locstr = (char *) malloc ( (int)strlen(argv[i]) * sizeof(char) +1 );
            strcpy(locstr, argv[i]);
            *dayVarBranching = atoi(locstr);
            if(*dayVarBranching != 0 && *dayVarBranching != 1)
            {
                fprintf(stderr, "Invalid branching strategy -> Choose 1 for day var and 0 for arc flow. ");
                SCIPerrorMessage("%s\n", usage);
                return SCIP_ERROR;
            }
            free(locstr);
        }else if(! strcmp(argv[i], "-p"))
        {
            if( i == argc - 1 || (! strncmp(argv[i+1], "-",1)))
            {
                fprintf(stderr, "Missing propagation info. ");
                SCIPerrorMessage("%s\n", usage);
                return SCIP_ERROR;
            }
            i++;
            locstr = (char *) malloc ( (int)strlen(argv[i]) * sizeof(char) +1 );
            strcpy(locstr, argv[i]);
            *activate_propagator = atoi(locstr);
            if(*activate_propagator < 0 || *activate_propagator > 2)
            {
                fprintf(stderr, "Invalid prop! -> Choose propagation setting in [0 (noprop), 1 (prop), 2 (exact)]!");
                SCIPerrorMessage("%s\n", usage);
                return SCIP_ERROR;
            }
            free(locstr);
            if(*activate_propagator == 1)
                cout << "USE (RCFC)-based PROPAGATOR!" << endl;
            else if(*activate_propagator == 2)
                cout << "USE EXACT PROPAGATOR!" << endl;
        }else if ( ! strcmp(argv[i], "-s"))
        {
            if( i == argc - 1 || (! strncmp(argv[i+1], "-",1)))
            {
                fprintf(stderr, "Missing seed number. ");
                SCIPerrorMessage("%s\n", usage);
                return SCIP_ERROR;
            }
            i++;
            locstr = (char *) malloc ( (int)strlen(argv[i]) * sizeof(char) +1 );
            strcpy(locstr, argv[i]);
            *seed = atoi(locstr);
            if(*seed < 0)
            {
                fprintf(stderr, "Invalid seed! -> Choose seed >= 0!");
                SCIPerrorMessage("%s\n", usage);
                return SCIP_ERROR;
            }
            free(locstr);
        }
    }

    return SCIP_OKAY;
}

static
SCIP_RETCODE setUpScip(
        SCIP**              scip,
        SCIP_Bool           minTravel,
        int                 dayVarBranching
) {
    /* initialize SCIP */
    SCIP_CALL(SCIPcreate(scip));

    /* include default SCIP plugins */
    SCIP_CALL( SCIPincludeDefaultPlugins(*scip) );

    /* include branching rules */
    SCIP_CALL(SCIPincludeObjBranchrule(*scip, new ObjBranchruleVehicle(*scip, 5000), TRUE));
    if(!minTravel)
    {
        SCIP_CALL(SCIPincludeObjBranchrule(*scip, new ObjBranchruleArcflow(*scip, 50000), TRUE));
        SCIP_CALL(SCIPincludeObjBranchrule(*scip, new ObjBranchruleDayVar(*scip, 50001), TRUE));
    }else
    {
        if(dayVarBranching)
        {
            SCIP_CALL(SCIPincludeObjBranchrule(*scip, new ObjBranchruleArcflow(*scip, 50000), TRUE));
            SCIP_CALL(SCIPincludeObjBranchrule(*scip, new ObjBranchruleDayVar(*scip, 50001), TRUE));
        }else{
            SCIP_CALL(SCIPincludeObjBranchrule(*scip, new ObjBranchruleArcflow(*scip, 50001), TRUE));
            SCIP_CALL(SCIPincludeObjBranchrule(*scip, new ObjBranchruleDayVar(*scip, 50000), TRUE));
        }
    }
    /* turn off all separation algorithms */
    SCIP_CALL( SCIPsetSeparating(*scip, SCIP_PARAMSETTING_OFF, TRUE) );
    /* include robust cuts separator */
//    SCIP_CALL(SCIPincludeObjSepa(*scip, new ObjCvrpSep(*scip), TRUE));

    /* include propagator */
    SCIP_CALL(SCIPincludeObjProp(*scip, new ObjPropVarFixing(*scip), TRUE));
    SCIP_CALL(SCIPincludeObjProp(*scip, new ObjPropTourVarFixing(*scip), TRUE));

    /* include primal heuristics */
    SCIP_CALL(SCIPincludeObjHeur(*scip, new HeurDayVarRounding(*scip), TRUE));

    /* include event handler */
    SCIP_CALL(SCIPincludeObjEventhdlr(*scip, new EventhdlrNodeInit(*scip), TRUE));

    /* include constraint handler */
    SCIP_CALL( SCIPincludeObjConshdlr(*scip, new ConshdlrArcflow(*scip), TRUE) );
    SCIP_CALL( SCIPincludeObjConshdlr(*scip, new ConshdlrDayVar(*scip), TRUE) );
    SCIP_CALL( SCIPincludeObjConshdlr(*scip, new ConshdlrKPC(*scip), TRUE) );
    SCIP_CALL( SCIPincludeObjConshdlr(*scip, new ConshdlrSRC(*scip), TRUE) );
    SCIP_CALL( SCIPincludeObjConshdlr(*scip, new ConshdlrCPC(*scip), TRUE) );
    SCIP_CALL( SCIPincludeObjConshdlr(*scip, new ConshdlrVehicle(*scip), TRUE) );
    SCIP_CALL( SCIPincludeObjConshdlr(*scip, new ConshdlrNVehicle(*scip), TRUE) );

    /** change parameters */

    /* Constraint handler */
    SCIP_CALL( SCIPsetIntParam(*scip,"constraints/Vehicle/maxrounds",2) );
    SCIP_CALL( SCIPsetIntParam(*scip,"constraints/NVehicle/maxrounds",2) );

    /* change display columns */
    SCIP_CALL( SCIPsetIntParam(*scip,"display/nfrac/active",2) );
    SCIP_CALL( SCIPsetIntParam(*scip,"display/curcols/active",2) );
    SCIP_CALL( SCIPsetIntParam(*scip,"display/cuts/active",2) );
    SCIP_CALL( SCIPsetIntParam(*scip,"display/lpobj/active",2) );
    SCIP_CALL( SCIPsetIntParam(*scip,"display/primalgap/active",0) );
    SCIP_CALL( SCIPsetIntParam(*scip,"display/gap/active",2) );
    SCIP_CALL( SCIPsetIntParam(*scip,"display/lpavgiterations/active",0) );
    SCIP_CALL( SCIPsetIntParam(*scip,"display/lpiterations/active",0) );
    SCIP_CALL( SCIPsetIntParam(*scip,"display/vars/active",2) );
    SCIP_CALL( SCIPsetIntParam(*scip,"display/conflicts/active",0) );
    SCIP_CALL( SCIPsetIntParam(*scip,"display/strongbranchs/active",0) );
    /* Branching */
    SCIP_CALL( SCIPsetIntParam(*scip,"display/nnodes/active",2) );
    SCIP_CALL( SCIPsetIntParam(*scip,"display/nodesleft/active",2) );
    SCIP_CALL( SCIPsetIntParam(*scip,"display/maxdepth/active",2) );

    /* for column generation instances, disable restarts */
    SCIP_CALL( SCIPsetIntParam(*scip,"presolving/maxrestarts",0) );

    /* activate reduced costs propagator */
//    SCIP_CALL(SCIPsetBoolParam(*scip, "propagating/redcost/force", true));

    /* set the limit for the relative gap after which the solving process is stopped */
    SCIP_CALL (SCIPsetRealParam(*scip, "limits/gap", 0.0));

    /* set a time limit */
    if(minTravel)
    {
        SCIP_CALL( SCIPsetIntParam(*scip, "display/freq", 200));
        SCIP_CALL( SCIPsetRealParam(*scip, "limits/time", 3600));
    }else
    {
        SCIP_CALL( SCIPsetIntParam(*scip, "display/freq", 200));
        SCIP_CALL( SCIPsetRealParam(*scip, "limits/time", 60));
    }

    return SCIP_OKAY;
}

static
SCIP_RETCODE addInitTours(
    SCIP*           scip,
    vector<int>&    hash_day,
    SCIP*           scip_new
)
{
    char algoName[] = "initTour";
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip));
    for(auto var : probData->vars_)
    {
        if(SCIPgetSolVal(scip, SCIPgetBestSol(scip), var) < 0.5)
            continue;
        tourVRP tvrp;
        auto* vardata = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var));
        tvrp.copy(vardata->tourVrp_);
        tvrp.setDay(hash_day[vardata->getDay()]);
        SCIP_CALL(add_tour_variable(scip_new, dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip_new)),
                                    FALSE, TRUE, algoName, tvrp));
    }
    return SCIP_OKAY;
}

static
SCIP_RETCODE readSolution(
    string&             sol_file,
    vector<tourVRP>&    tvrps
){
    int day, length, cap;
    double obj;
    int count = 0;
    string line;
    ifstream solstream;
    solstream.open(sol_file);
    if(solstream.is_open())
    {
        while (getline(solstream, line))
        {
            day = stoi(line.substr(0, line.find(' ')));
            line = line.substr(line.find(' ') + 1);
            obj = stod(line.substr(0, line.find(' ')));
            line = line.substr(line.find(' ') + 1);
            cap = stoi(line.substr(0, line.find(' ')));
            line = line.substr(line.find(' ') + 1);
            length = stoi(line.substr(0, line.find(' ')));
            line = line.substr(line.find(' ') + 1);
            tvrps.emplace_back(length, day);
            tvrps[count].obj_ = obj;
            tvrps[count].capacity_ = cap;
            for(int i = 0; i < length; i++)
            {
                tvrps[count].tour_[i] = stoi(line.substr(0, line.find(' ')));
                line = line.substr(line.find(' ') + 1);
            }

            count++;
        }
        solstream.close();
    }
    return SCIP_OKAY;
}

/** output solution */
static
SCIP_RETCODE outputSolution(
    SCIP*       scip,
    char*       input
)
{
    auto* probData = dynamic_cast<vrp::ProbDataVRP*>(SCIPgetObjProbData(scip));
    string s(input);
    string s1 = s.substr(s.find('_')-2, -1 );
    s1 = s1.substr(0, s1.find(".j"));

    string outfile = "cmptest_sol/" + s1 + ".sol";
    ofstream solfile;
    solfile.open(outfile, std::ios_base::app);
    cout << "PRINT SOLUTION: " << outfile << endl;
    SCIP_Sol* sol = SCIPgetBestSol(scip);
    for(auto var : probData->vars_)
    {
        if(SCIPgetSolVal(scip, sol, var) > 0.5)
        {
            tourVRP tvrp = dynamic_cast<ObjVarDataVRP*>(SCIPgetObjVardata(scip, var))->tourVrp_;
            solfile << tvrp.getDay() << " " << tvrp.obj_ << " " << tvrp.capacity_ << " " << tvrp.length_;
            for(auto v : tvrp.tour_)
                solfile << " " << v;
            solfile << endl;
        }
    }

    return SCIP_OKAY;
}


static
SCIP_RETCODE findInfeasiblityReason(
    SCIP*   scip
){
    auto *pricerData = dynamic_cast<ObjPricerVRP *>(SCIPfindObjPricer(scip, "VRP_Pricer"));
    auto *probData = dynamic_cast<vrp::ProbDataVRP *>(SCIPgetObjProbData(scip));
    model_data* modelData = probData->getData();
    cout << "Infeasibility might come from the following days / customers" << endl;
    for(int i = 0; i < modelData->nDays; i++)
    {
        if(!SCIPisZero(scip, pricerData->dualValues_[modelData->nC + i]))
            cout << "day " << i << endl;
    }
    for(int i = 0; i < modelData->nC; i++)
    {
        if(!SCIPisZero(scip, pricerData->dualValues_[i]))
        {
            assert(SCIPisEQ(scip, pricerData->dualValues_[i], SCIPgetDualfarkasLinear(scip, probData->cons_[i-1])));
            cout << "cust " << i << " available on days: ";
            for(auto day : modelData->availableDays[i])
                cout << day << " (" << modelData->timeWindows[i][day].start << "-"
                << modelData->timeWindows[i][day].end << ")  ";
            cout << endl;
        }
    }
    return SCIP_OKAY;
}

/** creates a SCIP instance with default plugins, evaluates command line parameters, runs SCIP appropriately,
 *  and frees the SCIP instance
 */
static
SCIP_RETCODE runColumnGenerationModel(
    int                   argc,               /**< number of shell parameters */
    char**                argv                /**< array with shell parameters */
)
{
    SCIP* scip = nullptr;
    char* input_file = nullptr;
//    string sol_file = "../instances/solutions/r1_n40_nv25_p1_l10.sol";
    auto* modelData = new model_data;
    int dayVarBranching = 1;
    int ng_parameter = 8;
    int activate_propagator = 0;
    int seed = 0;

    SCIP_CALL(readArguments(argc, argv, &input_file, &dayVarBranching, &activate_propagator, &seed));

    string s(input_file);
    string s1 = s.substr(s.find('_')-2, -1 );
    s1 = s1.substr(0, s1.find(".j"));

    string sol_file = "../instances/compare_sol/" + s1 + ".sol";
    cout << "SOLFILE " << sol_file << endl;
    SCIP_CALL(setUpScip(&scip, true, dayVarBranching));

    SCIP_CALL(getModelDataFromJson(modelData, input_file, ng_parameter));
    modelData->minTravel = true;

    vector<tourVRP> sol_tvrps;

    /** deactivate for no warm start */
    SCIP_CALL(readSolution(sol_file, sol_tvrps));

    SCIP_CALL(SCIPprobdataCreate(scip, modelData, sol_tvrps, activate_propagator));

    cout << "SET SEED " << seed << endl;
    SCIPsetIntParam(scip, "randomization/randomseedshift", seed);

    SCIP_CALL(SCIPsolve(scip));
    /********************
    * Print Solution *
    ********************/
//    SCIPprintBestSol(scip, nullptr, false);

    if(SCIPgetBestSol(scip) == nullptr)
    {
        SCIP_CALL(findInfeasiblityReason(scip));
    }else
    {
        auto *probData = dynamic_cast<vrp::ProbDataVRP *>(SCIPgetObjProbData(scip));
        cout << setprecision(4) << fixed << s1 << "\ttotal time: "<< SCIPgetSolvingTime(scip) << "\tnnodes: " << SCIPgetNNodes(scip)
            << "\tprop_time: " << SCIPconshdlrGetEnfoLPTime(SCIPfindConshdlr(scip, "CPC")) << "\tnsuccess: " << probData->prop_success_
            << "\tnfixed int: " << probData->num_fixed_ << "\tnfixed frac: " << probData->num_fixed_frac_ << "\tncutoffs: " << probData->num_cutoff_ << endl;

//        }
    }

    /** Output */
//    SCIP_CALL(outputSolution(scip, input_file));

    /********************
    * Deinitialization *
    ********************/
    delete modelData;
    SCIP_CALL( SCIPfree(&scip) );

    return SCIP_OKAY;
}


int
main(
    int                        argc,
    char**                     argv
)
{
    SCIP_RETCODE retcode;

    retcode = runColumnGenerationModel(argc, argv);
    if( retcode != SCIP_OKAY )
    {
      SCIPprintError(retcode);
      return -1;
    }

    return 0;
}

