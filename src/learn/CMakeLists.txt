add_executable(mprog1 mprog1.cc)
target_link_libraries(mprog1 drake::drake)

add_executable(xarm_sim xarm_simulation.cc)
target_link_libraries(xarm_sim drake::drake gflags)
