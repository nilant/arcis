cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGUROBI_DIR=/opt/gurobi/linux64
cmake --build build -j 8

mkdir -p bin
find build/src -executable -type f -exec cp {} bin/ \;
