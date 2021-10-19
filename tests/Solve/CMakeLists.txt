cmake_minimum_required (VERSION 3.19)

add_test_with_conf(SimulatorSelfTest Solve/simulator_self.conf)
add_test_with_conf(Staic5mTest Solve/static5m.conf)
