#include <stdio.h>

double c_fibonacci(double n)
{
  int i;
  double a=0.0, b=1.0, tmp;
  printf("This is C version.\n");
  for (i=0; i<n; i++) {
    tmp = a; a=a+b; b=tmp;
  }
  return a;
}
