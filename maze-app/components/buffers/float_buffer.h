
#ifndef FLOAT_BUFFER_H
#define FLOAT_BUFFER_H

#ifdef __cplusplus
extern "C"
{
#endif

    void *get_buffer();
    void *big_get_buffer();

    void push(void *buf, float val);
    void big_push(void *buf, float val);

    float get(void *buf, int idx);
    float big_get(void *buf, int idx);

    float get_delta(void *buf);
    float big_get_delta(void *buf);

    float conv(void *buf,float* coefs,int num);
    float big_conv(void *buf,float* coefs,int num);

#ifdef __cplusplus
}
#endif

#endif