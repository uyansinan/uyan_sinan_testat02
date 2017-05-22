/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT
#define RGB TRUE
const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;

bool ManualThreshold;

/* skip pixel at border */
const int Border = 2;

/* minimum size of objects (sum of all pixels) */
const int MinArea = 800;

/* size of centroid marker */
const int SizeCross = 10;

struct OSC_VIS_REGIONS ImgRegions;/* these contain the foreground objects */

unsigned char OtsuThreshold(int InIndex);
void Binarize(unsigned char threshold);
void Erode_3x3(int InIndex, int OutIndex);
void Dilate_3x3(int InIndex, int OutIndex);
void DetectRegions();
void DrawBoundingBoxes();
void ConvertToYCbCr();
void ChangeDetection();
void ChangeDetectionYCbCr();
enum ObjColor selectColor(int index);
void ResetProcess() {
	//called when "reset" button is pressed
	if (ManualThreshold == false)
		ManualThreshold = true;
	else
		ManualThreshold = false;
}

void ProcessFrame() {
	//initialize counters

	if (data.ipc.state.nStepCounter == 1) {
		ManualThreshold = false;
	} else {
		ConvertToYCbCr();
		memcpy(data.u8TempImage[THRESHOLD], data.u8TempImage[INDEX0], IMG_SIZE);
		memset(data.u8TempImage[INDEX1], 0, IMG_SIZE);

		if (RGB == TRUE) {
			ChangeDetection();
		} else {
			ChangeDetectionYCbCr();
		}

		memcpy(data.u8TempImage[THRESHOLD], data.u8TempImage[INDEX1], IMG_SIZE);
		DetectRegions();
		DrawBoundingBoxes();

		if (ManualThreshold) {
			char Text[] = "manual threshold";
			DrawString(20, 20, strlen(Text), SMALL, CYAN, Text);
		} else {
			char Text[] = " Otsu's threshold";
			DrawString(20, 20, strlen(Text), SMALL, CYAN, Text);
		}
	}

}

void Binarize(unsigned char threshold) {
	int r, c;
	//set result buffer to zero
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);

	//loop over the rows
	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		//loop over the columns
		for (c = Border; c < (nc - Border); c++) {
			//manual threshold?
			if (ManualThreshold) {
				if (data.u8TempImage[SENSORIMG][r + c]
						< data.ipc.state.nThreshold) {
					data.u8TempImage[THRESHOLD][r + c] = 255;
				}
			} else {
				if (data.u8TempImage[SENSORIMG][r + c] < threshold) {
					data.u8TempImage[THRESHOLD][r + c] = 255;
				}
			}
		}
	}
}

unsigned char OtsuThreshold(int InIndex) {
	//first part: extract gray value histogram
	unsigned int i1, best_i, K;
	float Hist[256];
	unsigned char* p = data.u8TempImage[InIndex];
	float best;
	memset(Hist, 0, sizeof(Hist));

	for (i1 = 0; i1 < nr * nc; i1++) {
		Hist[p[i1]] += 1;
	}
	//second part: determine threshold according to Otsu's method
	best = 0;
	best_i = 0;
	for (K = 0; K < 255; K++) {
		//the class accumulators
		float w0 = 0, mu0 = 0, w1 = 0, mu1 = 0;
		float bestloc;

		//class 0 and 1 probabilities and averages
		for (i1 = 0; i1 <= K; i1++) {
			w0 += Hist[i1];
			mu0 += (Hist[i1] * i1);
		}
		for (; i1 <= 255; i1++) {
			w1 += Hist[i1];
			mu1 += (Hist[i1] * i1);
		}
		//do normalization of average values
		mu0 /= w0;
		mu1 /= w1;

		bestloc = w0 * w1 * (mu0 - mu1) * (mu0 - mu1);
		if (bestloc > best) {
			best = bestloc;
			best_i = K;
			//OscLog(INFO, "%d %d %d %d %d\n", i1, w0, w1, mu0, mu1);
		}
	}
	//OscLog(INFO, "%d %f\n", best_i, best);
	return (unsigned char) best_i;
}

void Erode_3x3(int InIndex, int OutIndex) {
	int c, r;

	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r + c];
			data.u8TempImage[OutIndex][r + c] = *(p - nc - 1) & *(p - nc)
					& *(p - nc + 1) & *(p - 1) & *p & *(p + 1) & *(p + nc - 1)
					& *(p + nc) & *(p + nc + 1);
		}
	}
}

void Dilate_3x3(int InIndex, int OutIndex) {
	int c, r;

	for (r = Border * nc; r < (nr - Border) * nc; r += nc) {
		for (c = Border; c < (nc - Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r + c];
			data.u8TempImage[OutIndex][r + c] = *(p - nc - 1) | *(p - nc)
					| *(p - nc + 1) | *(p - 1) | *p | *(p + 1) | *(p + nc - 1)
					| *(p + nc) | *(p + nc + 1);
		}
	}
}

void DetectRegions() {
	struct OSC_PICTURE Pic;
	int i;

	//set pixel value to 1 in INDEX0 because the image MUST be binary (i.e. values of 0 and 1)
	for (i = 0; i < IMG_SIZE; i++) {
		data.u8TempImage[INDEX1][i] = data.u8TempImage[THRESHOLD][i] ? 1 : 0;
	}

	//wrap image INDEX0 in picture struct
	Pic.data = data.u8TempImage[INDEX1];
	Pic.width = nc;
	Pic.height = nr;
	Pic.type = OSC_PICTURE_BINARY;

	//now do region labeling and feature extraction
	OscVisLabelBinary(&Pic, &ImgRegions);
	OscVisGetRegionProperties(&ImgRegions);
}

void DrawBoundingBoxes() {
	uint16 o;
	for (o = 0; o < ImgRegions.noOfObjects; o++) {
		if (ImgRegions.objects[o].area > MinArea) {
			enum ObjColor color = selectColor(o);
			DrawBoundingBox(ImgRegions.objects[o].bboxLeft,
					ImgRegions.objects[o].bboxTop,
					ImgRegions.objects[o].bboxRight,
					ImgRegions.objects[o].bboxBottom, false, color);

			DrawLine(ImgRegions.objects[o].centroidX - SizeCross,
					ImgRegions.objects[o].centroidY,
					ImgRegions.objects[o].centroidX + SizeCross,
					ImgRegions.objects[o].centroidY, color);
			DrawLine(ImgRegions.objects[o].centroidX,
					ImgRegions.objects[o].centroidY - SizeCross,
					ImgRegions.objects[o].centroidX,
					ImgRegions.objects[o].centroidY + SizeCross, color);

		}
	}
}

void ChangeDetection() {
	//{{217,71, 56}, {4, 64, 150}}
	const int NumFgrCol = 2;
	uint8 FrgCol[2][3] = { { 217, 71, 56 }, { 4, 64, 150 } };
	int r, c, frg, p;

	memset(data.u8TempImage[INDEX0], 0, IMG_SIZE);
	memset(data.u8TempImage[BACKGROUND], 0, IMG_SIZE);

	for (r = 0; r < nr * nc; r += nc) {
		for (c = 0; c < nc; c++) {
			int MinDif = 1 << 30;
			int MinInd = 0;
			for (frg = 0; frg < NumFgrCol; frg++) {
				int Dif = 0;
				for (p = 1; p < 3; p++) {
					Dif += abs(
							(int) data.u8TempImage[INDEX0][(r + c) * NUM_COLORS
									+ p] - (int) FrgCol[frg][p]);
				}
				if (Dif < MinDif) {
					MinDif = Dif;
					MinInd = frg;
				}
			}
			if (MinDif < data.ipc.state.nThreshold) {
				data.u8TempImage[THRESHOLD][(r + c)] = 255;
				for (p = 0; p < 3; p++) {
					data.u8TempImage[BACKGROUND][(r + c) * NUM_COLORS + p] =
							FrgCol[MinInd][p];
				}
			}
		}
	}
}

void ConvertToYCbCr() {
	for (int r = 0; r < nr * nc; r += nc) {
		for (int c = 0; c < nc; c++) {
			float B_ = data.u8TempImage[SENSORIMG][(r + c) * NUM_COLORS + 0];
			float G_ = data.u8TempImage[SENSORIMG][(r + c) * NUM_COLORS + 1];
			float R_ = data.u8TempImage[SENSORIMG][(r + c) * NUM_COLORS + 2];

			uint8 Y_ = (uint8) (0 + 0.299 * R_ + 0.587 * G_ + 0.144 * B_);
			uint8 Cb_ = (uint8) (128 - 0.169 * R_ - 0.331 * G_ + 0.500 * B_);
			uint8 Cr_ = (uint8) (128 + 0.500 * R_ - 0.419 * G_ + 0.081 * B_);

			data.u8TempImage[INDEX0][(r + c) * NUM_COLORS + 0] = Y_;
			data.u8TempImage[INDEX0][(r + c) * NUM_COLORS + 1] = Cb_;
			data.u8TempImage[INDEX0][(r + c) * NUM_COLORS + 2] = Cr_;

		}
	}
}

enum ObjColor selectColor(int index) {
	struct OSC_VIS_REGIONS_RUN* regionObject = ImgRegions.objects[index].root;

	int red = 0;
	int blue = 0;

	do {
		for (int c = regionObject->startColumn; c <= regionObject->endColumn;
				c++) {
			int row = regionObject->row;
			red += data.u8TempImage[SENSORIMG][(row * nc + c) * NUM_COLORS + 2];
			blue += data.u8TempImage[SENSORIMG][(row * nc + c) * NUM_COLORS];

		}
	} while (regionObject != NULL);
	if (red > blue) {
		return RED;
	}

	return BLUE;
}

void ChangeDetectionYCbCr() {
	//{{217,71, 56}, {4, 64, 150}}
	const int NumFgrCol = 2;
	uint8 FrgCol[2][2] = { { 128 - 12, 128 + 38 }, { 128 + 24, 128 - 17 } };
	int r, c, frg, p;

	memset(data.u8TempImage[INDEX0], 0, IMG_SIZE);
	memset(data.u8TempImage[BACKGROUND], 0, IMG_SIZE);

	for (r = 0; r < nr * nc; r += nc) {
		for (c = 0; c < nc; c++) {
			int MinDif = 1 << 30;
			int MinInd = 0;
			for (frg = 0; frg < NumFgrCol; frg++) {
				int Dif = 0;
				for (p = 0; p < 2; p++) {
					Dif += abs(
							(int) data.u8TempImage[INDEX0][(r + c) * NUM_COLORS
									+ p + 1] - (int) FrgCol[frg][p]);
				}
				if (Dif < MinDif) {
					MinDif = Dif;
					MinInd = frg;
				}
			}
			if (MinDif < data.ipc.state.nThreshold) {
				data.u8TempImage[THRESHOLD][(r + c)] = 255;
				for (p = 0; p < 2; p++) {
					data.u8TempImage[BACKGROUND][(r + c) * NUM_COLORS + p + 1] =
							FrgCol[MinInd][p];
				}
			}
		}
	}
}
