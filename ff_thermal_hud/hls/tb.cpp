#include "ff_monitor.h"
#include <iostream>
using namespace std;

#include <stdio.h>

const int BYTES_PER_PIXEL = 3; /// red, green, & blue
const int FILE_HEADER_SIZE = 14;
const int INFO_HEADER_SIZE = 40;

void generateBitmapImage(unsigned char *image, int height, int width,
		char *imageFileName);
unsigned char* createBitmapFileHeader(int height, int stride);
unsigned char* createBitmapInfoHeader(int height, int width);
void AXIvideo2Bmp(axis &AXI_video_strm, int h, int w, char *filename);

int main(int argc, char **argv) {

	char *filename = (char*) malloc(13 * sizeof(char));

	axis dst_axi;
	cout << "Starting simulation..!\r\n";
	for (int i = 267; i < 274; ++i) {
		ff_monitor_gen(dst_axi, 1080, 1920, 0x12345678, i * 4);
		sprintf(filename, "img%03d.bmp", i);
		AXIvideo2Bmp(dst_axi, 1080, 1920, filename);
	}
	cout << "Finished!\r\n";
	return 0;
}

void AXIvideo2Bmp(axis &AXI_video_strm, int h, int w, char *filename) {
	int i, j;
	ap_axiu<WIDTH, 1, 1, 1> axi;
	bool sof = 0;

	int y, x;
	FILE *f;
	unsigned char *img = NULL;
	int filesize = 54 + 3 * w * h; //w is your image width, h is image height, both int

	img = (unsigned char*) malloc(3 * w * h);
	memset(img, 0, 3 * w * h);
	for (i = 0; i < h; i++) {
		for (j = 0; j < (w); j++) {
			AXI_video_strm >> axi;
			if ((i == 0) && (j == 0)) {
				if (axi.user.to_int() == 1) {
					sof = 1;
				} else {
					j--;
				}
			}
			if (sof) {
				x = j;
				y = i;
				if (axi.data == 0) {
					img[(x + y * w) * 3 + 2] = 0;
					img[(x + y * w) * 3 + 1] = 0;
					img[(x + y * w) * 3 + 0] = 0;
				} else {
					// RGBA to RGB Bitmap
					img[(x + y * w) * 3 + 2] = (unsigned char) (axi.data >> 16)
							& 0xFF; /// red
					img[(x + y * w) * 3 + 1] = (unsigned char) (axi.data)
							& 0xFF; /// green
					img[(x + y * w) * 3 + 0] = (unsigned char) (axi.data >> 8)
							& 0xFF; /// blue
				}
			}
		}

	}
	// BM Windows 3.1x, 95, NT, ... etc.
	// 54 The offset of the byte where the bitmap image data (pixel array) can be found.
	unsigned char bmpfileheader[14] = { 'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0,
			0, 0 };
	//BITMAPINFOHEADER Windows NT, 3.1x or later[2]
	// 40 the size of this header, in bytes
	// 1 color plane
	// bits per pixel 24
	unsigned char bmpinfoheader[40] = { 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
			0, 24, 0 };
	unsigned char bmppad[3] = { 0, 0, 0 };
	// The size of the BMP file in bytes
	bmpfileheader[2] = (unsigned char) (filesize);
	bmpfileheader[3] = (unsigned char) (filesize >> 8);
	bmpfileheader[4] = (unsigned char) (filesize >> 16);
	bmpfileheader[5] = (unsigned char) (filesize >> 24);
	// the bitmap width in pixels (signed integer)
	bmpinfoheader[4] = (unsigned char) (w);
	bmpinfoheader[5] = (unsigned char) (w >> 8);
	bmpinfoheader[6] = (unsigned char) (w >> 16);
	bmpinfoheader[7] = (unsigned char) (w >> 24);
	// the bitmap height in pixels (signed integer)
	bmpinfoheader[8] = (unsigned char) (h);
	bmpinfoheader[9] = (unsigned char) (h >> 8);
	bmpinfoheader[10] = (unsigned char) (h >> 16);
	bmpinfoheader[11] = (unsigned char) (h >> 24);

	f = fopen(filename, "wb");
	fwrite(bmpfileheader, 1, 14, f);
	fwrite(bmpinfoheader, 1, 40, f);
	for (int i = 0; i < h; i++) {
		fwrite(img + (w * (h - i - 1) * 3), 3, w, f);
		fwrite(bmppad, 1, (4 - (w * 3) % 4) % 4, f);
	}

	free(img);
	fclose(f);
}
