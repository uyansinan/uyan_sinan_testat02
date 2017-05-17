/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file draw.c
 * @brief Contains drawing routines; work is done in cgi.c only in case an image is
 * requested by browser; the data is transmitted by pasting it at the end of the image
 * buffer (data.ipc.req.pAddr).
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT

const int sizRect = sizeof(struct IMG_RECT);
const int sizLine = sizeof(struct IMG_LINE);
const int sizString = sizeof(struct IMG_STRING);


void DrawBoundingBox(uint16 left, uint16 bottom, uint16 right, uint16 top, bool recFill, uint8 color)
{
	uint8* pData = (uint8*) data.u8TempImage[ADDINFO];
	int dataSiz = data.AddBufSize;
	struct IMG_RECT imgRect;

	if(dataSiz+2+sizRect < IMG_SIZE) {//2byte for type
		uint16 oType = OBJ_RECT;
		memcpy(pData+dataSiz, &oType, sizeof(uint16));
		dataSiz += sizeof(uint16);
		imgRect.left = left;
		imgRect.bottom = bottom;
		imgRect.right = right;
		imgRect.top = top;
		imgRect.recFill = recFill;
		imgRect.color = color;
		memcpy(pData+dataSiz, &imgRect, sizRect);
		dataSiz += sizRect;
	}
	data.AddBufSize = dataSiz;
}


void DrawLine(uint16 x1, uint16 y1, uint16 x2, uint16 y2, uint8 color)
{
	uint8* pData = (uint8*) data.u8TempImage[ADDINFO];
	int dataSiz = data.AddBufSize;
	struct IMG_LINE imgLine;

	if(dataSiz+2+sizLine < IMG_SIZE) {//2byte for type
		uint16 oType = OBJ_LINE;
		memcpy(pData+dataSiz, &oType, sizeof(uint16));
		dataSiz += sizeof(uint16);
		imgLine.x1 = x1;
		imgLine.y1 = y1;
		imgLine.x2 = x2;
		imgLine.y2 = y2;
		imgLine.color = color;
		memcpy(pData+dataSiz, &imgLine, sizLine);
		dataSiz += sizLine;
	}
	data.AddBufSize = dataSiz;
}


void DrawString(uint16 xPos, uint16 yPos, uint16 len, uint16 font, uint8 color, char* str)
{
	uint8* pData = (uint8*) data.u8TempImage[ADDINFO];
	int dataSiz = data.AddBufSize;
	struct IMG_STRING imgString;

	if(dataSiz+2+sizString+len < IMG_SIZE) {//2byte for type
		uint16 oType = OBJ_STRING;
		memcpy(pData+dataSiz, &oType, sizeof(uint16));
		dataSiz += sizeof(uint16);
		imgString.xPos = xPos;
		imgString.yPos = yPos;
		imgString.len = len+1;//0 termination
		imgString.font = font;
		imgString.color = color;
		memcpy(pData+dataSiz, &imgString, sizString);
		dataSiz += sizLine;
		memcpy(pData+dataSiz, str, len);
		dataSiz += len;
		*(pData+dataSiz) = 0;//be sure that string is null terminated
		dataSiz++;
	}
	data.AddBufSize = dataSiz;
}
