#define TO_STRING
#include "../lib/models/WaterfallModel.cpp"
#include <stdio.h>
#include <unity.h>

void test_push()
{
    size_t *ms = new size_t[6]{5, 3, 4, 15, 4, 3};
    WaterfallModel m(1, 1, 6, ms);
    delete ms;

    char *r = m.toString();
    TEST_ASSERT_EQUAL_STRING("w:1 b:34 "
                             "[dt:1 t:0 [ c:0 e:0 ]"
                             "dt:1 t:0 [ c:0 e:0 ]"
                             "dt:1 t:0 [ c:0 e:0 ]"
                             "dt:1 t:0 [ c:0 e:0 ]"
                             "dt:1 t:0 [ c:0 e:0 ]"
                             "dt:5 t:0 [ c:0 e:0 ]"
                             "dt:5 t:0 [ c:0 e:0 ]"
                             "dt:5 t:0 [ c:0 e:0 ]"
                             "dt:15 t:0 [ c:0 e:0 ]"
                             "dt:15 t:0 [ c:0 e:0 ]"
                             "dt:15 t:0 [ c:0 e:0 ]"
                             "dt:15 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:60 t:0 [ c:0 e:0 ]"
                             "dt:900 t:0 [ c:0 e:0 ]"
                             "dt:900 t:0 [ c:0 e:0 ]"
                             "dt:900 t:0 [ c:0 e:0 ]"
                             "dt:900 t:0 [ c:0 e:0 ]"
                             "dt:3600 t:0 [ c:0 e:0 ]"
                             "dt:3600 t:0 [ c:0 e:0 ]"
                             "dt:3600 t:0 [ c:0 e:0 ] ]",
                             r);
    delete r;

    m.reset(0, 1);
    uint64_t i = 0;
    for (; i < 10; i++)
        m.updateModel(i, 0, 1);

    r = m.toString();
    TEST_ASSERT_EQUAL_STRING("w:1 b:34 "
                             "[dt:1 t:9 [ c:1 e:1 ]"
                             "dt:1 t:8 [ c:1 e:1 ]"
                             "dt:1 t:7 [ c:1 e:1 ]"
                             "dt:1 t:6 [ c:1 e:1 ]"
                             "dt:1 t:5 [ c:1 e:1 ]"
                             "dt:5 t:5 [ c:5 e:5 ]"
                             "dt:5 t:5 [ c:0 e:0 ]"
                             "dt:5 t:5 [ c:0 e:0 ]"
                             "dt:15 t:15 [ c:0 e:0 ]"
                             "dt:15 t:15 [ c:0 e:0 ]"
                             "dt:15 t:15 [ c:0 e:0 ]"
                             "dt:15 t:15 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:900 t:900 [ c:0 e:0 ]"
                             "dt:900 t:900 [ c:0 e:0 ]"
                             "dt:900 t:900 [ c:0 e:0 ]"
                             "dt:900 t:900 [ c:0 e:0 ]"
                             "dt:3600 t:3600 [ c:0 e:0 ]"
                             "dt:3600 t:3600 [ c:0 e:0 ]"
                             "dt:3600 t:3600 [ c:0 e:0 ] ]",
                             r);
    delete r;

    for (; i < 100; i += 10)
        m.updateModel(i, 0, 1);

    r = m.toString();
    TEST_ASSERT_EQUAL_STRING("w:1 b:34 "
                             "[dt:1 t:90 [ c:1 e:1 ]"
                             "dt:1 t:89 [ c:0 e:0 ]"
                             "dt:1 t:88 [ c:0 e:0 ]"
                             "dt:1 t:87 [ c:0 e:0 ]"
                             "dt:1 t:86 [ c:0 e:0 ]"
                             "dt:5 t:85 [ c:0 e:0 ]"
                             "dt:5 t:80 [ c:1 e:1 ]"
                             "dt:5 t:75 [ c:0 e:0 ]"
                             "dt:15 t:75 [ c:1 e:1 ]"
                             "dt:15 t:60 [ c:2 e:2 ]"
                             "dt:15 t:45 [ c:1 e:1 ]"
                             "dt:15 t:30 [ c:2 e:2 ]"
                             "dt:60 t:60 [ c:11 e:11 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:60 t:60 [ c:0 e:0 ]"
                             "dt:900 t:900 [ c:0 e:0 ]"
                             "dt:900 t:900 [ c:0 e:0 ]"
                             "dt:900 t:900 [ c:0 e:0 ]"
                             "dt:900 t:900 [ c:0 e:0 ]"
                             "dt:3600 t:3600 [ c:0 e:0 ]"
                             "dt:3600 t:3600 [ c:0 e:0 ]"
                             "dt:3600 t:3600 [ c:0 e:0 ] ]",
                             r);
    delete r;

    for (; i < 10000; i++)
        m.updateModel(i, 0, 1);

    r = m.toString();
    TEST_ASSERT_EQUAL_STRING("w:1 b:34 "
                             "[dt:1 t:9999 [ c:1 e:1 ]"
                             "dt:1 t:9998 [ c:1 e:1 ]"
                             "dt:1 t:9997 [ c:1 e:1 ]"
                             "dt:1 t:9996 [ c:1 e:1 ]"
                             "dt:1 t:9995 [ c:1 e:1 ]"
                             "dt:5 t:9995 [ c:4 e:4 ]"
                             "dt:5 t:9990 [ c:5 e:5 ]"
                             "dt:5 t:9985 [ c:5 e:5 ]"
                             "dt:15 t:9990 [ c:5 e:5 ]"
                             "dt:15 t:9975 [ c:15 e:15 ]"
                             "dt:15 t:9960 [ c:15 e:15 ]"
                             "dt:15 t:9945 [ c:15 e:15 ]"
                             "dt:60 t:9960 [ c:30 e:30 ]"
                             "dt:60 t:9900 [ c:60 e:60 ]"
                             "dt:60 t:9840 [ c:60 e:60 ]"
                             "dt:60 t:9780 [ c:60 e:60 ]"
                             "dt:60 t:9720 [ c:60 e:60 ]"
                             "dt:60 t:9660 [ c:60 e:60 ]"
                             "dt:60 t:9600 [ c:60 e:60 ]"
                             "dt:60 t:9540 [ c:60 e:60 ]"
                             "dt:60 t:9480 [ c:60 e:60 ]"
                             "dt:60 t:9420 [ c:60 e:60 ]"
                             "dt:60 t:9360 [ c:60 e:60 ]"
                             "dt:60 t:9300 [ c:60 e:60 ]"
                             "dt:60 t:9240 [ c:60 e:60 ]"
                             "dt:60 t:9180 [ c:60 e:60 ]"
                             "dt:60 t:9120 [ c:60 e:60 ]"
                             "dt:900 t:9900 [ c:60 e:60 ]"
                             "dt:900 t:9000 [ c:900 e:900 ]"
                             "dt:900 t:8100 [ c:900 e:900 ]"
                             "dt:900 t:7200 [ c:900 e:900 ]"
                             "dt:3600 t:7200 [ c:2700 e:2700 ]"
                             "dt:3600 t:3600 [ c:3520 e:3520 ]"
                             "dt:3600 t:3600 [ c:0 e:0 ] ]",
                             r);
    delete r;

    for (; i < 5000; i++)
        m.updateModel(i, 0, 1);

    r = m.toString();
    TEST_ASSERT_EQUAL_STRING("w:1 b:34 "
                             "[dt:1 t:9999 [ c:1 e:1 ]"
                             "dt:1 t:9998 [ c:1 e:1 ]"
                             "dt:1 t:9997 [ c:1 e:1 ]"
                             "dt:1 t:9996 [ c:1 e:1 ]"
                             "dt:1 t:9995 [ c:1 e:1 ]"
                             "dt:5 t:9995 [ c:4 e:4 ]"
                             "dt:5 t:9990 [ c:5 e:5 ]"
                             "dt:5 t:9985 [ c:5 e:5 ]"
                             "dt:15 t:9990 [ c:5 e:5 ]"
                             "dt:15 t:9975 [ c:15 e:15 ]"
                             "dt:15 t:9960 [ c:15 e:15 ]"
                             "dt:15 t:9945 [ c:15 e:15 ]"
                             "dt:60 t:9960 [ c:30 e:30 ]"
                             "dt:60 t:9900 [ c:60 e:60 ]"
                             "dt:60 t:9840 [ c:60 e:60 ]"
                             "dt:60 t:9780 [ c:60 e:60 ]"
                             "dt:60 t:9720 [ c:60 e:60 ]"
                             "dt:60 t:9660 [ c:60 e:60 ]"
                             "dt:60 t:9600 [ c:60 e:60 ]"
                             "dt:60 t:9540 [ c:60 e:60 ]"
                             "dt:60 t:9480 [ c:60 e:60 ]"
                             "dt:60 t:9420 [ c:60 e:60 ]"
                             "dt:60 t:9360 [ c:60 e:60 ]"
                             "dt:60 t:9300 [ c:60 e:60 ]"
                             "dt:60 t:9240 [ c:60 e:60 ]"
                             "dt:60 t:9180 [ c:60 e:60 ]"
                             "dt:60 t:9120 [ c:60 e:60 ]"
                             "dt:900 t:9900 [ c:60 e:60 ]"
                             "dt:900 t:9000 [ c:900 e:900 ]"
                             "dt:900 t:8100 [ c:900 e:900 ]"
                             "dt:900 t:7200 [ c:900 e:900 ]"
                             "dt:3600 t:7200 [ c:2700 e:2700 ]"
                             "dt:3600 t:3600 [ c:3520 e:3520 ]"
                             "dt:3600 t:3600 [ c:0 e:0 ] ]",
                             r);
    delete r;
}
