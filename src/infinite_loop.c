#include <unistd.h>
#include <stdio.h>

int main()
{
	int i;

	for (i=0; i<20000; i++) {
		printf("%d\n", i);
		sleep(1);
	}

	return 0;
}
