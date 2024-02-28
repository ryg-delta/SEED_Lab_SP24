# Unit Tests
This contains unit tests for the arduino utilities.

## Framework
We are using a lightweight testing framework that is compatible with our project sturcture. To write a unit test, follow this procedure:
- Include this line to import the library: `#include "../../libs/unit_test_framework.hpp"`
- Include this line to generate a main function `TEST_MAIN();`
- Write a test using this format `TEST(<name>) {};`

For a simple example look at `Vbase.cpp`

## Running Tests
You can either compile and run tests individually, or you can run `sh run.sh` to compile and run all tests.

## Naming Convention
All tests are named the same as the unit they are testing.