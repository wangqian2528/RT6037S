

#include "NetCloud.h"





unsigned int GetnMaxActionStep(unsigned int nAddress)
{
  unsigned int *p;
  p=(unsigned int *)(nAddress+4);
  return(*p/16);
} 

unsigned int IsNetCloud(unsigned int nAddress)
{
  unsigned int *p;
  p=(unsigned int *)(nAddress);
  if(*p==0||*p==0xffffffff)
    return(0);
  else
    return(1);
}

