#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#define TABLE_SIZE 65

#if F_CPU == 14745600UL
	#define TIMER1_TOP 288
#elif F_CPU == 18432000UL
	#define TIMER1_TOP 360
#else
	#error define this value
#endif

double tempcalc(int adc_val);

int main(int argc, char *argv[])
{
	int endpoint = 0;
	int i;
	double result;
	FILE *fp;
	double fan_profile[TABLE_SIZE] = {
		0.00,0.00,0.00,0.00,0.00, // 0  - 4
		0.00,0.00,0.00,0.00,0.00, // 5  - 9
		0.00,0.00,0.00,0.00,0.00, // 10 - 14
		0.00,0.00,0.00,0.00,0.00, // 15 - 19
		0.00,0.00,0.00,0.00,0.00, // 20 - 24
		0.00,0.00,0.00,0.00,0.00, // 24 - 29 
		0.00,0.00,0.00,0.00,0.00, // 39 - 34
		0.10,0.20,0.25,0.30,0.35, // 35 - 39
		0.40,0.45,0.50,0.55,0.70, // 40 - 44
		0.80,0.90,1.00,1.00,1.00, // 45 - 49
		1.00,1.00,1.00,1.00,1.00, // 50 - 54
		1.00,1.00,1.00,1.00,1.00, // 55 - 59
		1.00,1.00,1.00,1.00,1.00  // 60 - 64
	};

	fp = fopen("eetable.c","w");
	assert(fp);

	fprintf(fp, "#include <avr/io.h>\n");
	fprintf(fp, "#include <avr/eeprom.h>\n");
	fprintf(fp, "#include \"globals.h\"\n");
	fprintf(fp, "uint16_t fan_table[FAN_TABLE_SIZE] EEMEM = {\n");

	for(i = 0; i < TABLE_SIZE; i++){
		const int epval = endpoint * 16;
		const double eptemp = tempcalc(epval);
		result = round(fan_profile[i] * TIMER1_TOP);
		fprintf(fp, "%d,  /* ep %d - %d (%03.1fc)*/\n",(int)result, i, epval, eptemp);

		endpoint++;
	}

	fprintf(fp,"};\n");

	fclose(fp);

	return 0;
}

double tempcalc(int adc_val)
{
	const int ref_resistor = 10000;
	double temp = 0;

	temp = log((1024.0 * ref_resistor / adc_val) - ref_resistor);
	temp = 1 / (0.001129148 + (0.000234125 * temp) + (0.0000000876741 * temp * temp * temp));
	temp -= 273.15;

	return temp;
}
