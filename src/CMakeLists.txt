add_executable(arcis main.cpp 
                     best.cpp
                     cli.cpp
                     ArcRoute.cpp
                     Instance.cpp 
                     Result.cpp 
                     Heuristic.cpp
        Model_RT.cpp
                     RouteSolver.cpp
                     Preprocessing.cpp
                     local_search.cpp
        ../libs/vidal/Genetic.cpp
                ../libs/vidal/Population.cpp
                ../libs/vidal/LocalSearch.cpp
                ../libs/vidal/Noeud.cpp
                ../libs/vidal/Route.cpp
                ../libs/vidal/SeqData.cpp
                ../libs/vidal/Client.cpp
                ../libs/vidal/Params.cpp
                ../libs/vidal/Vehicle.cpp
                ../libs/vidal/Individu.cpp
)

target_include_directories(arcis PUBLIC ${GUROBI_INCLUDE_DIRS})
target_link_libraries(arcis PUBLIC ${GUROBI_CPP_LIBRARY} ${GUROBI_LIBRARY} fmt::fmt -pthread)