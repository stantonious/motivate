
#include "float_buffer.h"
#include "CircularBuffer.h"


typedef CircularBuffer<float, BUFSIZE> bufType;
typedef CircularBuffer<float, BIG_BUFSIZE> bigBufType;

void *get_buffer()
{
    bufType *buf = new bufType();
    return (void*)buf;
}

void *big_get_buffer()
{
    bigBufType *buf = new bigBufType();
    return (void*)buf;
}

void push(void *buf, float f)
{
    ((bufType*)buf)->push(f);
}

void big_push(void *buf, float f)
{
    ((bigBufType*)buf)->push(f);
}

float conv(void *buf, float *f, int coefsize)
{
    float res = 0.;

    for (int i = 0; i < coefsize; i++)
    {
        res += (*(bufType*)buf)[i] * f[i];
    }
    return res;
}

float get(void *buf,int idx)
{
    return (*(bufType*)buf)[idx];
}

float big_get(void *buf,int idx)
{
    return (*(bigBufType*)buf)[idx];
}

float get_delta(void *buf)
{
    float min = 1e10;
    float max = -1e10;

    for (int i=0;i<BUFSIZE;i++)
    {
        float v = get(buf,i);
        if (v < min) min = v;
        if (v > max) max = v;
    }

    return max - min;
}