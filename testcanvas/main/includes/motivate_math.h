

#include "math.h"
#include "esp_dsp.h"


void unit_vect(const float* src1,float* dest,int len)
{
    //dest = malloc(sizeof(float) * len);
    float rss = .0;

    for (int i=0;i<len;i++){
      rss += pow(src1[i],2.);
    }
    rss = sqrt(rss);
    for (int i=0;i<len;i++){
        dest[i] = src1[i]/rss;
    }
}