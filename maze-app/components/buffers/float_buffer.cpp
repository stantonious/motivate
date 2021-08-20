
#include "float_buffer.h"
#include "CircularBuffer.h"
#include "math.h"

#include "esp_log.h"


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

float stdev(void *buf){
    float sum = 0.0, mean, SD = 0.0;
    int i;
    for (int i = 0; i < BUFSIZE; i++)
    {
        sum += (*(bufType*)buf)[i];
    }
    mean = sum / BUFSIZE;
    for (int i = 0; i < BUFSIZE; i++)
    {
        SD += pow((*(bufType*)buf)[i] - mean, 2);
    }
    return sqrt(SD / BUFSIZE);
}
float sum_abs(void *buf)
{
    float res = 0.;

    for (int i = 0; i < BUFSIZE; i++)
    {
        res += fabs((*(bufType*)buf)[i]);
    }
    return res;
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

void big_mk_copy(void *buf,float* out_buf,int size)
{
    bigBufType* cBuf =  (bigBufType*)buf;

    for (int i=0;i<cBuf->size() && i < size;i++)
    {
        out_buf[i]=(*cBuf)[i];
    }
}

void mk_copy(void *buf,float out_buf[BUFSIZE],int size)
{
    bufType* cBuf =  (bufType*)buf;

    for (int i=0;i<cBuf->size() && i < size;i++)
    {
        out_buf[i]=(*cBuf)[i];
    }
}

void dump(float **buf,int m )
{
    for (int i = 0;i<m;i++)
    {
        for (int j=0;j<BUFSIZE;j++)
        {
            ESP_LOGI("FLOATBUF","i:%d,j:%d,v:%f\n",i,j,buf[i][j]);
        }

    }

}