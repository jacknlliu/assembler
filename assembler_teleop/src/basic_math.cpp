#include "assembler_teleop/basic_math.h"

bool is_equal(double a, double b)
{
    if ((a-b < 0.00001 && a-b >= 0.0) || (a-b > -0.00001 && a-b <= 0.0) ){
        return true;
    } else {
        return false;
    }
}


int sign_of_float(double x)
{
    if (x >= 0) {
        return  1;
    } else {
        return -1;
    }
}