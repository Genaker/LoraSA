#include <stdio.h>
#define LOG(args...) printf(args)
#include <scan.cpp>
#include <unity.h>

void setUp(void) {}

void tearDown(void) {}

struct TestScan : Scan
{
    TestScan(float *ctx, int sz) : ctx(ctx), sz(sz), idx(0) {}

    float getRSSI() override;

    float *ctx;
    int sz;
    int idx;
};

float TestScan::getRSSI()
{
    if (idx >= sz)
    {
        return -1000000;
    }

    return ctx[idx++];
}

constexpr int test_sz = 13;
constexpr int inputs_sz = 13;

void test_rssi(void)
{
    uint16_t samples[test_sz];
    float inputs[inputs_sz] = {-40.0, -100.0, -200.0, -50.0, -400.0, -60.0, -20,
                               -75.5, -70,    -80,    -90,   -55.9,  -110};

    TestScan t = TestScan(inputs, inputs_sz);

    uint16_t r = t.rssiMethod(inputs_sz, samples, test_sz);

    uint16_t expect[test_sz] = {20, 50, 55, 60, 0, 70, 75, 80, 0, 90, 0, 100, 110};

    TEST_ASSERT_EQUAL_INT16(20, r);
    TEST_ASSERT_EQUAL_INT16_ARRAY(expect, samples, test_sz);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_rssi);

    UNITY_END();
}
