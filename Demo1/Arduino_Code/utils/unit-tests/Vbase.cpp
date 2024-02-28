#include "../../libs/unit_test_framework.hpp"
#include "../Vbase.h"

TEST_MAIN();


TEST(constructor) {
    Vbase voltages(1.5, 0.2);
    ASSERT_ALMOST_EQUAL(voltages.getVright(), 0.85, 0.01);
    ASSERT_ALMOST_EQUAL(voltages.getVleft(), 0.65, 0.01);
}

TEST(setter) {
    Vbase voltages;
    voltages.setVoltages(1.5, 0.2);
    ASSERT_ALMOST_EQUAL(voltages.getVright(), 0.85, 0.01);
    ASSERT_ALMOST_EQUAL(voltages.getVleft(), 0.65, 0.01);
}

TEST(static_method) {
    ASSERT_ALMOST_EQUAL(Vbase::calcVright(1.5, 0.2), 0.85, 0.01);
    ASSERT_ALMOST_EQUAL(Vbase::calcVleft(1.5, 0.2), 0.65, 0.01);
}

TEST(negative_numbers) {
    Vbase voltages(-1.5, 0.2);
    ASSERT_ALMOST_EQUAL(voltages.getVright(), -0.65, 0.01);
    ASSERT_ALMOST_EQUAL(voltages.getVleft(), -0.85, 0.01);

    voltages = Vbase(1.5, -0.2);
    ASSERT_ALMOST_EQUAL(voltages.getVright(), 0.65, 0.01);
    ASSERT_ALMOST_EQUAL(voltages.getVleft(), 0.85, 0.01);

    voltages = Vbase(-1.5, -0.2);
    ASSERT_ALMOST_EQUAL(voltages.getVright(), -0.85, 0.01);
    ASSERT_ALMOST_EQUAL(voltages.getVleft(), -0.65, 0.01);
}

