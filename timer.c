#include <stdio.h>
#include <sys/types.h>
#include <time.h>

main()
{ int i;
  time_t t1,t2;

  (void) time(&t1);
  
   for (i=1;i<=100;++i) printf("%d %d\n",i, i*i);
   
   
   (void) time(&t2);
   
   printf("\nTime to do 100 squares = %d seconds\n", (int) t2-t1);
} 

