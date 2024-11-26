#include <unity.h>

void test_rssi();
void test_detect();
void test_push();

void setUp(void) {}

void tearDown(void) {}

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_rssi);
    RUN_TEST(test_detect);

    RUN_TEST(test_push);

    UNITY_END();
}
