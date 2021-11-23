/*
 * Compile:
 * gcc -lm -o brightness smooth_led_brightness.c
 *
 * Example:
 * [username@localhost ~]$ ./brightness 10 255
 * Output:
 * int brightness[10] = {   0,    0,    2,    4,    8,   14,   26,   47,   83,  145};
 *
 *
 * argv[1] - количество шагов ШИМ (по умолчанию 100)
 * argv[2] - до скольки считает ШИМ (по умолчанию 255 как у Arduino)
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(int argc, char *argv[]) {
	int pwmSteps;
	int pwmCounts;
	int brightness;
	float R;

	if (argc < 2) {
		pwmSteps = 100;
		pwmCounts = 255;
	} else {
		pwmSteps = strtol(argv[1], NULL, 10);
		pwmCounts = strtol(argv[2], NULL, 10);
	}

	R = (pwmSteps * log10(2))/(log10(pwmCounts));

	printf("\n");
	printf("int brightness[%d] = {", pwmSteps*2);

	for (int interval = 0; interval < pwmSteps; interval++) {
		brightness = pow(2, (interval / R)) - 1;
		printf("%3d, ", brightness);
	}
	for (int interval = pwmSteps-1; interval > 0; interval--) {
                brightness = pow(2, (interval / R)) - 1;
                printf("%3d, ", brightness);
        }
	
	printf("\b\b};\n\n");

	return 0;
}
