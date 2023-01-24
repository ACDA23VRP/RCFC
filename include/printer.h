
#ifndef PRINTER_H
#define PRINTER_H

#include <utility>
#include "iostream"
#include "scip/scip.h"
#include "vector"
#include "pricer_vrp.h"
#include "model_data.h"
#include "tourVRP.h"

using namespace std;

SCIP_RETCODE printTimetable(
        model_data*             modelData,
        vector<vector<bool>>&   timetable
);

SCIP_RETCODE printIsForbidden(
        model_data*             modelData,
        vector<vector<bool>>&   isForbidden
);

#endif //PRINTER_H
