# Unit Tests
This contains unit tests for the arduino utilities.

## Framework
We are using a lightweight testing framework that is compatible with our project sturcture. To write a unit test, follow this procedure:
- Include this line to import the library: `#include "../../libs/unit_test_framework.hpp"`
- Include this line to generate a main function `TEST_MAIN();`
- Write a test using this format `TEST(<name>) {};`

For a simple example look at `Vbase.cpp`

## Running Tests
You can either run tests individually with `sh run-test.sh <test1> <test2> ...`, or you can run `sh run-all.sh` to compile and run all tests.

## Naming Convention
All tests are named the same as the unit they are testing.

## Dependencies
The sh scripts require
- git bash or a linux terminal
- g++ compiler