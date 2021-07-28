
#include "float_buffer.h"
#include <CircularBuffer.h>

typedef CircularBuffer<float, 10> bufType;
typedef CircularBuffer<float, 50> bigBufType;

void *get_buffer()
{
    return nullptr;
}

void push(void *buf, float val)
{
}

float get(void *buf, int idx)
{
    return 1.;
}

float get_delta(void *buf)
{
    return 1.;
}

float conv(void *buf, float *coefs, int num)
{
    return 1.;
}

void *big_get_buffer()
{
    return nullptr;
}

void big_push(void *buf, float val)
{
}

float big_get(void *buf, int idx)
{
    return 1.;
}

float big_get_delta(void *buf)
{
    return 1.;
}

float big_conv(void *buf, float *coefs, int num)
{

    return 1.;
}
