
#ifndef FLOAT_BUFFER_H_
#define FLOAT_BUFFER_H_

#define BUFSIZE 10
#define BIG_BUFSIZE 50

#ifdef __cplusplus
extern "C"
{
#endif
  void *get_buffer();
  void *big_get_buffer();

  void push(void *buf, float f);
  void big_push(void *buf, float f);

  float conv(void *buf, float *f, int coefsize);
  float sum_abs(void *buf);
  float stdev(void *buf);

  float get(void *buf,int idx);
  float big_get(void *buf,int idx);

  float get_delta(void *buf);

  void mk_copy(void* buf,float out_buf[BUFSIZE],int size);
  void big_mk_copy(void* buf,float* out_buf,int size);

  void dump(float **buf,int m);
#ifdef __cplusplus
}
#endif

#endif
