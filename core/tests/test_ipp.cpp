#include <stdio.h>

#include "/opt/intel/composer_xe_2013.3.171/ipp/include/ipp.h"

int main(int argc, char* argv[])
{
	const int SIZE = 256;
	Ipp8u pSrc[SIZE], pDst[SIZE];

	int i;
	for (i=0; i<SIZE; i++)
		pSrc[i] = (Ipp8u)i;

    ippsCopy_8u(pSrc, pDst, SIZE);

	printf("pDst[%d] = %d\n", SIZE-1, pDst[SIZE-1]);
}