
#include "printer.h"

SCIP_RETCODE printTimetable(
        model_data*             modelData,
        vector<vector<bool>>&   timetable
){
    cout << "days: ";
    for(int d = 0; d < modelData->nDays; d++)
    {
        cout << "\t" << d;
    }cout << endl;
    for(int c = 1; c < modelData->nC; c++)
    {
        cout << "c " << c << " ";
        for(int d = 0; d < modelData->nDays; d++)
        {
            cout << "\t" << timetable[c][d];
        }cout << endl;
    }
    return SCIP_OKAY;
}
