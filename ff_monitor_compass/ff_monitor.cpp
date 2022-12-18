#include "ff_monitor.h"
#include "char7segment2.h"

#define COUNT_PIXEL 8
#define COUNT_LINE 6

#define COUNT_PIXELCMPS 3
#define COUNT_LINECMPS 1

#define YCHAR1 50
#define CHAR_WIDTH 5

// compass labels
#define YCMPSLABEL 730
#define CMPSLABEL_MARGINY 10

// compass scale
#define YCMPS1 770
#define CMPS_MARGINY 8



#define BLOCK_SEPARATION 200
#define DIGIT_SEPARATION 15
#define DISPLAY_MARGINY 10

#define XCHAR0 + 800
#define XCHAR1 XCHAR0 + COUNT_PIXEL * CHAR_WIDTH + DIGIT_SEPARATION
#define XCHAR2 XCHAR1 + COUNT_PIXEL * CHAR_WIDTH + DIGIT_SEPARATION
#define XCHAR3 XCHAR2 + COUNT_PIXEL * CHAR_WIDTH + DIGIT_SEPARATION
#define XCHAR4 XCHAR3 + COUNT_PIXEL * CHAR_WIDTH + DIGIT_SEPARATION + 500
#define XCHAR5 XCHAR4 + COUNT_PIXEL * CHAR_WIDTH + DIGIT_SEPARATION
#define XCHAR6 XCHAR5 + COUNT_PIXEL * CHAR_WIDTH + DIGIT_SEPARATION + 100
#define XCHAR7 XCHAR6 + COUNT_PIXEL * CHAR_WIDTH + DIGIT_SEPARATION

extern const unsigned int display_0[11][5];
extern const unsigned int display_1[11][5];
extern const unsigned int display_2[11][5];
extern const unsigned int display_3[11][5];
extern const unsigned int display_4[11][5];
extern const unsigned int display_5[11][5];
extern const unsigned int display_6[11][5];
extern const unsigned int display_7[11][5];
extern const unsigned int display_8[11][5];
extern const unsigned int display_9[11][5];
extern const unsigned int display_n[11][5];
extern const unsigned int display_e[11][5];
extern const unsigned int display_s[11][5];
extern const unsigned int display_w[11][5];

#define MAX_DIGITS 8


void drawDigitPixel(video_stream &hud_int, int line, int pixel, int digit);
void updateCounters(int &pixel, int &count_pixel);
void updateCountersCmps(int &pixel, int &count_pixel);


#define NORTH 10
#define EAST 11
#define SOUTH 12
#define WEST 13
#define COMPASSXMIN 240
#define COMPASSXMAX COMPASSXMIN + 4 * 360

// 90 * 4
#define DEGREES_90_1_4 360
// 180 * 4
#define DEGREES_180_1_4 720
// 270 * 4
#define DEGREES_270_1_4 1080

/**
 * envData BCD representation of the 8 digits to display on the top
 * angle compass orientation angle in 1/4 degrees units
 */
void ff_monitor_gen(axis& op, int row, int column, int envData, int angle) {
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE s_axilite port=envData
#pragma HLS INTERFACE s_axilite port=angle
#pragma HLS INTERFACE s_axilite port=column
#pragma HLS INTERFACE s_axilite port=row
#pragma HLS INTERFACE axis register both port=op


	int y = 0;
	int x = 0;
    int displacement = angle  ;
	int compassX = 0;
	if (angle >= DEGREES_270_1_4) {
		displacement = (angle - DEGREES_270_1_4) ;
	} else if (angle >= DEGREES_180_1_4 ){
		displacement = (angle - DEGREES_180_1_4) ;
	} else if (angle >= DEGREES_90_1_4){
		displacement = (angle - DEGREES_90_1_4) ;
	} else {
		displacement = angle ;
	}
	int xcardinal0 = COMPASSXMIN +  359 - displacement - (COUNT_PIXELCMPS + 1 )* CHAR_WIDTH/2;
	int xcardinal1 = COMPASSXMIN +  719 - displacement - (COUNT_PIXELCMPS + 1 ) * CHAR_WIDTH/2;
	int xcardinal2 = COMPASSXMIN +  1079 - displacement - (COUNT_PIXELCMPS + 1 ) * CHAR_WIDTH/2;
	int xcardinal3 = COMPASSXMIN +  1439 - displacement - (COUNT_PIXELCMPS + 1 ) * CHAR_WIDTH/2;
    int compassdisplacement = angle;
	unsigned char digits[MAX_DIGITS] = {0};
	digits[0] =	(unsigned char)((envData >> 28) & 0x0F);
	digits[1] =	(unsigned char)((envData >> 24) & 0x0F);
	digits[2] = (unsigned char)((envData >> 20) & 0x0F);
	digits[3] = (unsigned char)((envData >> 16) & 0x0F);
	digits[4] = (unsigned char)((envData >> 12) & 0x0F);
	digits[5] = (unsigned char)((envData >>  8) & 0x0F);
	digits[6] = (unsigned char)((envData >>  4) & 0x0F);
	digits[7] = (unsigned char)((envData >>  0) & 0x0F);

	unsigned char cardinals[4] = {SOUTH, WEST, NORTH, EAST};
	if (angle >= 0 & angle <DEGREES_90_1_4) {
		cardinals[0] = WEST;
		cardinals[1] = NORTH;
		cardinals[2] = EAST;
		cardinals[3] = SOUTH;
	} else 	if (angle >= DEGREES_90_1_4 & angle <DEGREES_180_1_4) {
		cardinals[0] = NORTH;
		cardinals[1] = EAST;
		cardinals[2] = SOUTH;
		cardinals[3] = WEST;
	} else if (angle >= DEGREES_180_1_4 & angle <DEGREES_270_1_4) {
		cardinals[0] = EAST;
		cardinals[1] = SOUTH;
		cardinals[2] = WEST;
		cardinals[3] = NORTH;
	}

	int line = 0;
	int pixel = 0;
	int count_line = 0;
	int count_pixel = 0;
	int lineCmps = 0;
	int pixelCmps = 0;
	int count_lineCmps = 0;
	int count_pixelCmps = 0;



	video_stream hud_int;

	row_loop:for (y =0; y<row; y++){

		column_loop:for (x =0; x <  column; x++) {
			if (y == 0 && x == 0 ){
				hud_int.user = 1;
			}
			else{
				if (x == (column-1) ){
					hud_int.last = 1;
				}
				else{
					hud_int.last = 0;
					hud_int.user = 0;
					compassX = x + compassdisplacement;
					if((y>YCHAR1-DISPLAY_MARGINY ) & (y<(YCHAR1+ 11 * (COUNT_LINE + 1) +DISPLAY_MARGINY )) ){
						if((y>YCHAR1) & (y<(YCHAR1+ 11 * (COUNT_LINE + 1)))){
							if(x>XCHAR0 & x < (COUNT_PIXEL + 1) * CHAR_WIDTH + XCHAR0 + 1){
								drawDigitPixel(hud_int, line, pixel, digits[0]);
								updateCounters(pixel, count_pixel);
							} else	if(x>XCHAR1 & x < (COUNT_PIXEL + 1) * CHAR_WIDTH + XCHAR1 + 1){
								drawDigitPixel(hud_int, line, pixel, digits[1]);
								updateCounters(pixel, count_pixel);
							} else	if(x>XCHAR2 & x < (COUNT_PIXEL + 1) * CHAR_WIDTH + XCHAR2 + 1){
								drawDigitPixel(hud_int, line, pixel, digits[2]);
								updateCounters(pixel, count_pixel);
							} else	if(x>XCHAR3 & x < (COUNT_PIXEL + 1) * CHAR_WIDTH + XCHAR3 + 1){
								drawDigitPixel(hud_int, line, pixel, digits[3]);
								updateCounters(pixel, count_pixel);
							}else	if(x>XCHAR4 & x < (COUNT_PIXEL + 1) * CHAR_WIDTH + XCHAR4 + 1){
								drawDigitPixel(hud_int, line, pixel, digits[4]);
								updateCounters(pixel, count_pixel);
							} else	if(x>XCHAR5 & x < (COUNT_PIXEL + 1) * CHAR_WIDTH + XCHAR5 + 1){
								drawDigitPixel(hud_int, line, pixel, digits[5]);
								updateCounters(pixel, count_pixel);
							} else	if(x>XCHAR6 & x < (COUNT_PIXEL + 1) * CHAR_WIDTH + XCHAR6 + 1){
								drawDigitPixel(hud_int, line, pixel, digits[6]);
								updateCounters(pixel, count_pixel);
							} else	if(x>XCHAR7 & x < (COUNT_PIXEL + 1) * CHAR_WIDTH + XCHAR7 + 1){
								drawDigitPixel(hud_int, line, pixel, digits[7]);
								if (count_pixel  == COUNT_PIXEL) {
									if ((pixel  == (CHAR_WIDTH - 1)) & (count_line  == COUNT_LINE)) {
										count_line  = 0;
										pixel  = 0;
										count_pixel  = 0;
										line ++;
									} else 	if ((pixel  == (CHAR_WIDTH - 1)) & (count_line  < COUNT_LINE)) {
										count_line ++;
										pixel  = 0;
										count_pixel  = 0;
									} else {
										count_pixel  = 0;
										pixel ++;
									}
								} else {
									count_pixel ++;
								}
							}

							else {  //////
								hud_int.data = BGN;
							}
						} else {
							hud_int.data = hud_int.data = BGN;
						}

					}  //// COMPASS LABELS
					else if((y>YCMPSLABEL-CMPSLABEL_MARGINY )
							& (y<(YCMPSLABEL + CMPSLABEL_MARGINY + 11 * (COUNT_LINECMPS + 1) ))	){
						if((y>YCMPSLABEL ) & (y<(YCMPSLABEL + 11 * (COUNT_LINECMPS + 1)  )) ){
							if((x>xcardinal0) & (x < xcardinal0 + (COUNT_PIXELCMPS + 1) * CHAR_WIDTH + 1)){
								drawDigitPixel(hud_int, lineCmps, pixelCmps, cardinals[0]);
								updateCountersCmps(pixelCmps, count_pixelCmps);
							} else if((x>xcardinal1) & (x < xcardinal1  + (COUNT_PIXELCMPS + 1) * CHAR_WIDTH + 1)){
								drawDigitPixel(hud_int, lineCmps, pixelCmps, cardinals[1]);
								updateCountersCmps(pixelCmps, count_pixelCmps);
							} else 	if((x>xcardinal2) & (x < xcardinal2 + (COUNT_PIXELCMPS + 1) * CHAR_WIDTH + 1)){
								drawDigitPixel(hud_int, lineCmps, pixelCmps, cardinals[2]);
								updateCountersCmps(pixelCmps, count_pixelCmps);
							} else if((x>xcardinal3) & (x < xcardinal3 + (COUNT_PIXELCMPS + 1) * CHAR_WIDTH + 1)){
								drawDigitPixel(hud_int, lineCmps, pixelCmps, cardinals[3]);
								if (count_pixelCmps  == COUNT_PIXELCMPS) {
									if ((pixelCmps  == (CHAR_WIDTH - 1)) & (count_lineCmps  == COUNT_LINECMPS)) {
										count_lineCmps  = 0;
										lineCmps ++;
									} else 	if ((pixelCmps  == (CHAR_WIDTH - 1)) & (count_lineCmps  < COUNT_LINECMPS)) {
										count_lineCmps ++;
									} else {
  										count_pixelCmps  = 0;
										pixelCmps ++;
									}
								} else {
									count_pixelCmps ++;
								}
							} else if ((x >= 959) & (x<=961)) {
								hud_int.data = 0xFFFF0000;
							} else {
								hud_int.data = BGN;
							}

						} else if ((x >= 959) & (x<=961)) {
							hud_int.data = 0xFFFF0000;
						} else {
							hud_int.data = BGN;
						}

					}  //// COMPASS AXIS
					else if((y>YCMPS1-CMPS_MARGINY ) & (y<(YCMPS1 + 10 )) & (x > COMPASSXMIN) & (x<COMPASSXMAX)  ){
						if ((x >= 959) & (x<=961)) {
							hud_int.data = 0xFFFF0000;
						} else if((y>YCMPS1 ) & (y<(YCMPS1 + 10  )) ){
							if(!(compassX & 0x00000007)) {
								hud_int.data = CPX;
							} else {
								hud_int.data = BGN;
							}
						} else	if(!(compassX & 0x0000003F)) {
							hud_int.data = CPX;
						} else {
							hud_int.data = BGN;
						}
					}else {
						hud_int.data = 0;
					}
				}
			}
			op.write(hud_int);
		}
// row
		pixelCmps  = 0;
		count_pixelCmps  = 0;
	}

}


void drawDigitPixel(video_stream &hud_int, int line, int pixel, int digit) {
#pragma HLS inline
	if (digit == 0) {
		hud_int.data = display_0[line][pixel];
	} else if (digit == 1) {
		hud_int.data = display_1[line][pixel];
	} else if (digit == 2) {
		hud_int.data = display_2[line][pixel];
	} else if (digit == 3) {
		hud_int.data = display_3[line][pixel];
	} else if (digit == 4) {
		hud_int.data = display_4[line][pixel];
	} else if (digit == 5) {
		hud_int.data = display_5[line][pixel];
	} else if (digit == 6) {
		hud_int.data = display_6[line][pixel];
	} else if (digit == 7) {
		hud_int.data = display_7[line][pixel];
	} else if (digit == 8) {
		hud_int.data = display_8[line][pixel];
	} else if (digit == 9) {
		hud_int.data = display_9[line][pixel];
	} else if (digit == 10) {
		hud_int.data = display_n[line][pixel];
	}  else if (digit == 11) {
		hud_int.data = display_e[line][pixel];
	}  else if (digit == 12) {
		hud_int.data = display_s[line][pixel];
	}  else if (digit == 13) {
		hud_int.data = display_w[line][pixel];
	}  else {
		hud_int.data = BGN;
	}
}

void updateCounters(int &pixel, int &count_pixel) {
#pragma HLS inline off
	if (count_pixel == COUNT_PIXEL) {
		count_pixel = 0;
		if (pixel == (CHAR_WIDTH - 1)) {
			pixel = 0;
		} else {
			pixel++;
		}
	} else {
		count_pixel++;
	}
}

void updateCountersCmps(int &pixel, int &count_pixel) {
#pragma HLS inline off
	if (count_pixel == COUNT_PIXELCMPS) {
		count_pixel = 0;
		if (pixel == (CHAR_WIDTH - 1)) {
			pixel = 0;
		} else {
			pixel++;
		}
	} else {
		count_pixel++;
	}
}


