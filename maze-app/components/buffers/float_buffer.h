
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

  float get(void *buf,int idx);
  float big_get(void *buf,int idx);

  float get_delta(void *buf);
#ifdef __cplusplus
}
#endif

#endif
