#include <Arduino.h>

String toBinary(int num, int bitSize)
{
    if (num == 0)
        return "0";

    String binary = "";
    while (num > 0)
    {
        binary = String(num % 2) + binary;
        num /= 2;
    }

    // Pad with leading zeros to match `bitSize`
    while (binary.length() < bitSize)
    {
        binary = "0" + binary;
    }

    return binary;
}
