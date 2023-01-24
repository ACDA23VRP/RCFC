cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 17)

project(vrp)

if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fopenmp")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")


### This Project #####
add_subdirectory(src)

set(SOURCES
        src/main_vrp.cpp
        src/model_data.cpp
        src/probdata_vrp.cpp
        src/pricer_vrp.cpp
        src/label2.cpp
        src/labellist.cpp
        src/labeling_algorithm_vrp.cpp
        src/vardata.cpp
        src/initial.cpp
        src/tools_vrp.cpp
        src/branchingrule_arcflow.cpp
        src/ConshdlrArcflow.cpp
        src/branchingrule_dayvar.cpp
        src/ConshdlrDayVar.cpp
        src/branchingrule_vehicle.cpp
        src/ConshdlrVehicle.cpp
        src/ConshdlrNVehicle.cpp
        src/local_search_pricing.cpp
        src/tourVRP.cpp
        src/eventhdlr_nodeInit.cpp
        src/printer.cpp
        src/var_tools.cpp
        src/tsptw.cpp
        src/ConshdlrKPC.cpp
        src/ConshdlrSRC.cpp
        src/heurDayVarRounding.cpp
        src/prop_varfixing.cpp
        src/prop_tourvarfixing.cpp
        src/ConshdlrCPC.cpp
        )


add_executable(vrp ${SOURCES})

target_include_directories(vrp
        PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        )

# for testing
#enable_testing()
#add_test(NAME vrpbuild/small-test.txt
#        COMMAND vrp ../instances/r101/r1_n20_nv25_p0.25_l10.json)
#add_test(vrpbuild/small-test.txt ${EXE})

# SCIP
# Note: SCIP_DIR as a path is only a recommendation for cmake
message(STATUS "SCIP_DIR from ENV I see is: $ENV{SCIP8_PATH}")
find_package(SCIP 8.0 PATHS $ENV{SCIP8_PATH} REQUIRED)
message(STATUS "Using SCIP library from here ${SCIP_DIR}, was considering SCIP_DIR from env: $ENV{SCIP8_PATH}")
message(STATUS "Using SCIP library from here ${SCIP_LIBRARIES}")

## This project links

target_link_libraries(vrp
        ${SCIP_LIBRARIES}
        )


# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()



