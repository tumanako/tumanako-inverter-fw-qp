g++ filter.hpp
g++ $(gtest-config --cppflags --cxxflags) -o unit_test.o -c unit_test.cpp
g++ $(gtest-config --ldflags --libs) -o unit_test unit_test.o

