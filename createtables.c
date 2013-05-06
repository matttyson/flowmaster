#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <assert.h>
#include <unistd.h>

#define FILE_NAME "tables.c"
#define FILE_NAME_H "tables.h"
#define TIMER1_TOP 288

static int
get_sign(double x)
{
	return x != fabs(x);
}

static void
write_temp_table()
{
	const int ref_resistor = 10000;
	int i;
	double temp = 0;
	double integral;
	double fractional;
	unsigned int mini = 0xFFFFFFFF, maxi = 0;

	FILE *fp;

	fp = fopen(FILE_NAME,"r+");
	assert(fp);

	fseek(fp,0,SEEK_END);

	fprintf(fp,"const struct temp16 temp_table[] PROGMEM = {\n");

	for(i = 1; i < 1024; i++){
		const int adc_val = i;

		temp = log((1024 * ref_resistor / adc_val) - ref_resistor);
		temp = 1 / (0.001129148 + (0.000234125 * temp) + (0.0000000876741 * temp * temp * temp));
		temp -= 273.15;

		if(temp < -5.0){
			continue;
		}
		if(temp > 100.0){
			continue;
		}

		if(i < mini){
			mini = i;
		}
		if(i > maxi){
			maxi = i;
		}


		fractional = modf(temp, &integral);
		fractional *= 10;

		int remainder = abs(round(fractional));
		int whole = integral;

		if(remainder == 10){
			remainder = 0;

			if(whole > 0){
				whole++;
			}
			else {
				whole--;
			}
		}

		if(i % 10 == 0){
			fputc('\n',fp);
		}

		fprintf(fp, "{%2d,%d,%d},",
			(int)abs(whole),
			(int)get_sign(temp),
			(int)remainder
		);
	}

	fflush(fp);

	/* go back over the last comma*/
	fseek(fp,-1,SEEK_CUR);

	fprintf(fp,"\n};\n\n");
	
	fclose(fp);

	fp = fopen(FILE_NAME_H,"a+");
	
	fprintf(fp,"struct temp16 {\n");
	fprintf(fp,"\tuint8_t whole : 8;\n");
	fprintf(fp,"\tuint8_t sign  : 1;\n");
	fprintf(fp,"\tuint8_t remainder : 7;\n");
	fprintf(fp,"};\n");

	fprintf(fp,"extern const struct temp16 temp_table[] PROGMEM;\n");

	fprintf(fp,"#define ADC_MIN %d\n",mini);
	fprintf(fp,"#define ADC_MAX %d\n",maxi);

	fprintf(fp,"#define TEMP_ADDR(x) ((x) - ADC_MIN)\n");

	fclose(fp);
}

static void
write_pc_table()
{
	FILE *fp;

	fp = fopen(FILE_NAME,"r+");

	fseek(fp,0,SEEK_END);

	fprintf(fp,"const uint8_t dcycle_to_pc[] PROGMEM = {\n");

	int i;
	for(i = 0; i < TIMER1_TOP; i++){
		const int num = round(((double)i / TIMER1_TOP) * 100.0);
		if((i % 15) == 0){
			fputc('\n',fp);
		}
		fprintf(fp,"%2d,",num);
	}
	/* Due to some code being shit somewhere, add another 100 */
	fprintf(fp,"100,");
	
	fseek(fp,-1,SEEK_CUR);

	fprintf(fp,"\n};\n");

	fclose(fp);

	fp = fopen(FILE_NAME_H,"a");

	fprintf(fp,"extern const uint8_t dcycle_to_pc[] PROGMEM;\n");

	fclose(fp);
}


static void
create_tables()
{
	FILE *fp;
	
	fp = fopen(FILE_NAME,"w");
	fprintf(fp,"#include \"tables.h\"\n\n");
	fclose(fp);


	fp = fopen(FILE_NAME_H,"w");
	fprintf(fp,"#ifndef TABLES_H\n");
	fprintf(fp,"#define TABLES_H\n");
	fprintf(fp,"#include <avr/io.h>\n");
	fprintf(fp,"#include <avr/pgmspace.h>\n");

	fclose(fp);
}

static void
close_header()
{
	FILE *fp = fopen(FILE_NAME_H,"a");

	fprintf(fp,"#endif\n");

	fclose(fp);
}

int
main(int argc, char *argv[])
{
	create_tables();
	write_temp_table();
	write_pc_table();


	close_header();

	return 0;
}
