add_executable(arcis main.cpp 
                     best.cpp
                     cli.cpp
                     ArcRoute.cpp
                     Instance.cpp 
                     Result.cpp 
                     Heuristic.cpp 
                     ModelAux.cpp 
                     ModelRT.cpp 
                     RouteSolver.cpp
                     Preprocessing.cpp
                     LocalSearch.cpp
                     bounds.cpp
                ../libs/vidal/Genetic.cpp
                ../libs/vidal/Population.cpp
                ../libs/vidal/LocalSearch.cpp
                ../libs/vidal/Noeud.cpp
                ../libs/vidal/Route.cpp
                ../libs/vidal/SeqData.cpp
                ../libs/vidal/Client.cpp
                ../libs/vidal/Params.cpp
                ../libs/vidal/Vehicle.cpp
                ../libs/vidal/Individu.cpp)

target_include_directories(arcis PUBLIC ${GUROBI_INCLUDE_DIRS})
target_link_libraries(arcis PUBLIC ${GUROBI_CPP_LIBRARY} ${GUROBI_LIBRARY} fmt::fmt -pthread)


add_executable(test_route main2.cpp 
                    solver_test.cpp
                     best.cpp
                     cli.cpp
                     ArcRoute.cpp
                     Instance.cpp 
                     Result.cpp 
                     Heuristic.cpp 
                     ModelAux.cpp 
                     ModelRT.cpp 
                     RouteSolver.cpp
                     Preprocessing.cpp
                     LocalSearch.cpp
                     bounds.cpp
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

target_include_directories(test_route PUBLIC ${GUROBI_INCLUDE_DIRS})
target_link_libraries(test_route PUBLIC ${GUROBI_CPP_LIBRARY} ${GUROBI_LIBRARY} fmt::fmt -pthread)