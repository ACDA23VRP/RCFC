
#define PRINT_BRANCHING_INFORMATION false
#define USE_DSSR false
#define ENFORCED_PRICE_COLLECTING 10000
#define PARALLEL_LABELING TRUE
#define MAX_SRC 100
#define ROUNDING_FACTOR 1000

#ifndef BRANCHANDPRICE_MODEL_DATA_H
#define BRANCHANDPRICE_MODEL_DATA_H

#include "scip/scip.h"
#include "json.hpp"
#include "bitset"

using namespace std;

constexpr unsigned neighborhood_size = 128;

enum ConsType
{
    PROHIBIT = 0,
    ENFORCE = 1
};
typedef enum ConsType CONSTYPE;

typedef struct timeWindow {
    int                 day;
    int                 start;
    int                 end;
} timeWindow;

typedef struct model_data {
        int                         nC;             /**< number of customers + depot */
        int                         nDays;          /**< number of days */
        int                         nVehicles;      /**< number of vehicles */
//        int                         max_cap;        /**< capacity of the vehicles */
        SCIP_Bool                   minTravel;      /**< if True min. travel time, else min. num of vehicles used */
        vector<int>                 max_caps;       /**< capacities of the vehicles */
        vector<int>                 num_v;          /**< number of each vehicle */
        vector<int>                 dayofVehicle;   /**< day of each vehicle */
        vector<int>                 firstVehicleofday; /**< */
        vector<int>                 demand;         /**< demand of the customers */
        vector<int>                 service;        /**< service time of the customers */
        vector<vector<double>>      travel;         /**< travel time matrix */
        vector<vector<timeWindow>>  timeWindows;    /**< time windows for each customer */
        vector<vector<int>>         availableDays;  /**< list of days with time window for each customer */
        vector<vector<int>>         availableVehicles;  /**< list of vehicles with time window for each customer */
        vector<vector<vector<int>>> neighbors;      /**< neighbors for each customer and each day */
        vector<vector<vector<int>>> predecessors;   /**< predecessors for each customer and each day */
        vector<vector<vector<bool>>>adjacency_k;    /**< adjacency for each day k */
        vector<bitset<neighborhood_size>> ng_set;   /**< ng-neighborhood */
} model_data;

SCIP_RETCODE getModelDataFromJson(model_data* modelData, char* input_file, int ng_parameter);

SCIP_RETCODE transformModelData
(
            SCIP*               scip,
            model_data*         modelData_old,
            model_data*         modelData_new,
            std::vector<int>&        hash_day
);

int getNumOfArcs(
        model_data* modelData
);

int getNumOfTW(
        model_data* modelData
);

#endif //BRANCHANDPRICE_MODEL_DATA_H
