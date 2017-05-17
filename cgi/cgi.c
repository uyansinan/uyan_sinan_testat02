/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file cgi_template.c
 * @brief CGI used for the webinterface of the SHT application.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "cgi.h"
#include "gd.h"
#include "gdfontg.h"
#include "gdfontl.h"
#include "gdfontmb.h"
#include "gdfonts.h"
#include "gdfontt.h"

#include <time.h>

const int sizRect = sizeof(struct IMG_RECT);
const int sizLine = sizeof(struct IMG_LINE);
const int sizString = sizeof(struct IMG_STRING);
const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;
const int siz = OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT;

const int colorLUT[MAX_NUM_COLORS][3] = {{255, 255, 255}, {0, 0, 0}, {255, 0, 0}, {0, 255, 0}, {0, 0, 255},
										 {255, 255, 0}, {255, 0, 255}, {0, 255, 255}};

/*! @brief Main object structure of the CGI. Contains all 'global'
 * variables. */
struct CGI_TEMPLATE cgi;

/*! @brief All potential arguments supplied to this CGI. */
struct ARGUMENT args[] =
{
	{ "exposureTime", INT_ARG, &cgi.args.nExposureTime, &cgi.args.bExposureTime_supplied },
	{ "Threshold", INT_ARG, &cgi.args.nThreshold, &cgi.args.bThreshold_supplied },
	{ "ImageType", INT_ARG, &cgi.args.nImageType, &cgi.args.bImageType_supplied },
	{ "AddInfo", INT_ARG, &cgi.args.nAddInfo, &cgi.args.bAddInfo_supplied }
};


/*! @brief look up function for color palette in order to allow switching form gray to true color */
int colorLoolUp(int color)
{
#if NUM_COLORS == 1
	return (255-2*color);
#else
	return gdTrueColor(colorLUT[color][0], colorLUT[color][1], colorLUT[color][2]);
#endif
}

/*! @brief Strips whiltespace from the beginning and the end of a string and returns the new beginning of the string. Be advised, that the original string gets mangled! */
char * strtrim(char * str) {
	char * end = strchr(str, 0) - 1;

	while (*str != 0 && strchr(" \t\n", *str) != NULL)
		str += 1;

	while (end > str && strchr(" \t\n", *end) != NULL)
		end -= 1;

	*(end + 1) = 0;

	return str;
}

/*********************************************************************//*!
 * @brief Split the supplied URI string into arguments and parse them.
 *
 * Matches the argument string with the arguments list (args) and fills in
 * their values. Unknown arguments provoke an error, but missing
 * arguments are just ignored.
 *
 * @param strSrc The argument string.
 * @param srcLen The length of the argument string.
 * @return SUCCESS or an appropriate error code otherwise
 *//*********************************************************************/
static OSC_ERR CGIParseArguments()
{
	char buffer[1024];

	/* Intialize all arguments as 'not supplied' */
	for (int i = 0; i < sizeof args / sizeof (struct ARGUMENT); i += 1)
	{
		*args[i].pbSupplied = false;
	}

	while (fgets (buffer, sizeof buffer, stdin)) {
		struct ARGUMENT *pArg = NULL;
		char * key, * value = strchr(buffer, ':');

		if (value == NULL) {
			OscLog(ERROR, "%s: Invalid line: \"%s\"\n", __func__, buffer);
			return -EINVALID_PARAMETER;
		}

		*value = 0;
		value += 1;

		key = strtrim(buffer);
		value = strtrim(value);

		OscLog(INFO, "obtained key: %s, and Value: %s\n", key, value);

		for (int i = 0; i < sizeof(args)/sizeof(struct ARGUMENT); i += 1) {
			if (strcmp(args[i].strName, key) == 0) {
				pArg = args + i;
				break;
			}
		}

		if (pArg == NULL) {
			OscLog(ERROR, "%s: Unknown argument encountered: \"%s\"\n", __func__, key);
			return -EINVALID_PARAMETER;
		} else {
			if (pArg->enType == STRING_ARG) {
				// FIXME: Could someone fix this buffer overflow?
				strcpy((char *) pArg->pData, value);
			} else if (pArg->enType == INT_ARG) {
				if (sscanf(value, "%d", (int *) pArg->pData) != 1) {
					OscLog(ERROR, "%s: Unable to parse int value of variable \"%s\" (%s)!\n", __func__, pArg->strName, value);
					return -EINVALID_PARAMETER;
				}
			} else if (pArg->enType == SHORT_ARG) {
				if (sscanf(value, "%hd", (short *) pArg->pData) != 1) {
					OscLog(ERROR, "%s: Unable to parse short value of variable \"%s\" (%s)!\n", __func__, pArg->strName, value);
					return -EINVALID_PARAMETER;
				}
			} else if (pArg->enType == BOOL_ARG) {
				if (strcmp(value, "true") == 0) {
					*((bool *) pArg->pData) = true;
				} else if (strcmp(value, "false") == 0) {
					*((bool *) pArg->pData) = false;
				} else {
					OscLog(ERROR, "CGI %s: Unable to parse boolean value of variable \"%s\" (%s)!\n", __func__, pArg->strName, value);
					return -EINVALID_PARAMETER;
				}
			}

			if (pArg->pbSupplied != NULL)
				*pArg->pbSupplied = true;
		}
	}

	return SUCCESS;
}

/*********************************************************************//*!
 * @brief Query the current state of the application and see what else
 * we need to get from it
 *
 * Depending on the current state of the application, other additional
 * parameters may be queried.
 *
 * @return SUCCESS or an appropriate error code otherwise
 *//*********************************************************************/
static OSC_ERR QueryApp()
{
	OSC_ERR err;

	/* First, get the current state of the algorithm. */
	err = OscIpcGetParam(cgi.ipcChan, &cgi.appState, GET_APP_STATE, sizeof(struct APPLICATION_STATE));
	if (err != SUCCESS)
	{
		/* This request is defined in all states, and thus must succeed. */
		OscLog(ERROR, "CGI: Error querying application! (%d)\n", err);
		return err;
	}

	switch(cgi.appState.enAppMode)
	{
	case APP_OFF:
		/* Algorithm is off, nothing else to do. */
		break;
	case APP_CAPTURE_ON:
		if (cgi.appState.bNewImageReady)
		{
			FILE* F;
			uint16 r,c;
			uint8* pData;
			uint32 dataSiz, i;
			uint16 oType;

			/* If there is a new image ready, request it from the application. */
			/* We get TWICE the size of an image because metadata might be available */
			err = OscIpcGetParam(cgi.ipcChan, cgi.imgBuf, GET_NEW_IMG, NUM_COLORS*nc*nr*2);
			if (err != SUCCESS)
			{
				OscLog(DEBUG, "CGI: Getting new image failed! (%d)\n", err);
				return err;
			}
//we have to take care of the different ways gdlib treats gray and color data
#if NUM_COLORS == 1
			//create gd image and ...
			gdImagePtr im_out =  gdImageCreate(nc, nr);
			//initialize with sensor image
			for(r = 0; r < nr; r++)
			{
				//in case the original image should not be modified replace the following loop by the memcpy statement
				//memcpy(im_out->pixels[r], cgi.imgBuf+r*nc, nc*sizeof(uint8));
				for(c = 0; c < nc; c++)
				{
					im_out->pixels[r][c] = (*(cgi.imgBuf+r*nc+c) & 0xfe);//mask out first bit -> only even gray values
				}
			}
			//allocate color palette (255 is red -> we did not change the sensor image!! should rather use a LUT)
			for(c = 0; c < 256; c++)
			{
				if((c%2) && c > 255-2*MAX_NUM_COLORS){
					i = (255-c)/2;
					gdImageColorAllocate (im_out, colorLUT[i][0], colorLUT[i][1], colorLUT[i][2]);
				} else {
					gdImageColorAllocate (im_out, c, c, c);
				}
			}
#else
			//create gd image and ...
			gdImagePtr im_out =  gdImageCreateTrueColor(nc, nr);
			//initialize with sensor image
			for(r = 0; r < nr; r++)
			{
				for(c = 0; c < nc; c++)
				{
					uint8* p = (cgi.imgBuf+r*3*nc+3*c);
					im_out->tpixels[r][c] = gdTrueColor(p[2], p[1], p[0]);
				}
			}


#endif
			//there might be additional data to be written to image
			pData = (uint8*) (cgi.imgBuf+NUM_COLORS*nc*nr);
			memcpy(&dataSiz, pData, sizeof(uint32));
			//OscLog(DEBUG, "received %d number of bytes\n", dataSiz);
			pData += sizeof(uint32);//skip dataSiz
			if(dataSiz)
			{
				i = 0;
				while(i < dataSiz)
				{
					memcpy(&oType, pData+i, sizeof(uint16));
					i += sizeof(uint16);
					switch(oType) {
						case OBJ_LINE:
						{
							struct IMG_LINE imgLine;
							memcpy(&imgLine, pData+i, sizLine);
							i += sizLine;
							//OscLog(DEBUG, "received line (%d,%d)-(%d,%d), color(%d)\n", imgLine.x1, imgLine.y1, imgLine.x2, imgLine.y2, (int) imgLine.color);
							gdImageLine(im_out, imgLine.x1, imgLine.y1, imgLine.x2, imgLine.y2, colorLoolUp(imgLine.color));
							break;
						}
						case OBJ_RECT:
						{
							struct IMG_RECT imgRect;
							memcpy(&imgRect, pData+i, sizRect);
							i += sizRect;
							//OscLog(DEBUG, "received rect (%d,%d)-(%d,%d), %s, color(%d)\n", imgRect.left, imgRect.bottom, imgRect.right, imgRect.top, imgRect.recFill ? "fill" : "not fill", (int) imgRect.color);
							if(imgRect.recFill) {
								gdImageFilledRectangle(im_out, imgRect.left, imgRect.bottom, imgRect.right, imgRect.top, colorLoolUp(imgRect.color));
							} else {
								gdImageRectangle(im_out, imgRect.left, imgRect.bottom, imgRect.right, imgRect.top, colorLoolUp(imgRect.color));
							}
							break;
						}
						case OBJ_STRING:
						{
							gdFontPtr font = gdFontSmall;
							struct IMG_STRING imgString;
							memcpy(&imgString, pData+i, sizRect);
							i += sizString;
							//OscLog(DEBUG, "received string (%d,%d), font %d, %s, color(%d)\n", imgString.xPos, imgString.yPos, imgString.font, pData+i, imgString.color);
							switch(imgString.font)
							{
								case GIANT:
									font = gdFontGiant;
									break;
								case LARGE:
									font = gdFontLarge;
									break;
								case MEDIUMBOLD:
									font = gdFontMediumBold;
									break;
								case SMALL:
									font = gdFontSmall;
									break;
								case TINY:
									font = gdFontTiny;
									break;
								default:
									break;//set in definition of font
							}
							gdImageString(im_out, font, imgString.xPos, imgString.yPos, pData+i, colorLoolUp(imgString.color));
						}
					}
				}
			}

			F = fopen(IMG_FN, "wb");
			//gdImageGif(im_out, F);
			gdImageJpeg(im_out, F, 100);
			fclose(F);
			gdImageDestroy(im_out);

			return SUCCESS;
		}
		break;
	default:
		OscLog(ERROR, "%s: Invalid application mode (%d)!\n", __func__, cgi.appState.enAppMode);
		break;
	}
	return SUCCESS;
}

/*********************************************************************//*!
 * @brief Set the parameters for the application supplied by the web
 * interface.
 *
 * @return SUCCESS or an appropriate error code otherwise
 *//*********************************************************************/
static OSC_ERR SetOptions()
{
	OSC_ERR err;
	struct ARGUMENT_DATA *pArgs = &cgi.args;

	if (pArgs->bImageType_supplied)
	{
		err = OscIpcSetParam(cgi.ipcChan, &pArgs->nImageType, SET_IMAGE_TYPE, sizeof(pArgs->nImageType));
		if (err != SUCCESS)
		{
			OscLog(DEBUG, "CGI: Error setting option! (%d)\n", err);
			return err;
		}
	}

	if (pArgs->bThreshold_supplied)
	{
		err = OscIpcSetParam(cgi.ipcChan, &pArgs->nThreshold, SET_THRESHOLD, sizeof(pArgs->nThreshold));
		if (err != SUCCESS)
		{
			OscLog(DEBUG, "CGI: Error setting option! (%d)\n", err);
			return err;
		}
	}

	if (pArgs->bExposureTime_supplied)
	{
		err = OscIpcSetParam(cgi.ipcChan, &pArgs->nExposureTime, SET_EXPOSURE_TIME, sizeof(pArgs->nExposureTime));
		if (err != SUCCESS)
		{
			OscLog(DEBUG, "CGI: Error setting option! (%d)\n", err);
			return err;
		}
	}

	if (pArgs->bAddInfo_supplied)
	{
		err = OscIpcSetParam(cgi.ipcChan, &pArgs->nAddInfo, SET_ADDINFO, sizeof(pArgs->nAddInfo));
		if (err != SUCCESS)
		{
			OscLog(DEBUG, "CGI: Error setting option! (%d)\n", err);
			return err;
		}
	}

	return SUCCESS;
}

/*********************************************************************//*!
 * @brief Take all the gathered info and formulate a valid AJAX response
 * that can be parsed by the Javascript in the browser.
 *//*********************************************************************/
static void FormCGIResponse()
{
	struct APPLICATION_STATE  *pAppState = &cgi.appState;

	/* Header */
	printf("Content-type: text/plain\n\n" );

	printf("imgTS: %u\n", (unsigned int)pAppState->imageTimeStamp);
	printf("exposureTime: %d\n", pAppState->nExposureTime);
	printf("Threshold: %d\n", pAppState->nThreshold);
	printf("Stepcounter: %d\n", pAppState->nStepCounter);
	printf("width: %d\n", OSC_CAM_MAX_IMAGE_WIDTH);
	printf("height: %d\n", OSC_CAM_MAX_IMAGE_HEIGHT);
	printf("ImageType: %u\n", pAppState->nImageType);
	printf("AddInfo: %d\n", pAppState->nAddInfo);

	fflush(stdout);
}

OscFunction(mainFunction)
	OSC_ERR err;
	struct stat socketStat;

	/* Initialize */
	memset(&cgi, 0, sizeof(struct CGI_TEMPLATE));

	/* First, check if the algorithm is even running and ready for IPC
	 * by looking if its socket exists.*/
	if(stat(USER_INTERFACE_SOCKET_PATH, &socketStat) != 0)
	{
		/* Socket does not exist => Algorithm is off. */
		/* Form a short reply with that info and shut down. */
		cgi.appState.enAppMode = APP_OFF;
		OscFail_m("Algorithm is off!");
		return -1;
	}

	/******* Create the framework **********/
	OscCall(OscCreate,
		&OscModule_log,
		&OscModule_ipc);

	OscLogSetConsoleLogLevel(CRITICAL);
	OscLogSetFileLogLevel(DEBUG);

	OscCall( OscIpcRegisterChannel, &cgi.ipcChan, USER_INTERFACE_SOCKET_PATH, 0);

	OscCall( CGIParseArguments);

	/* The algorithm negative acknowledges if it cannot supply
	 * the requested data, i.e. it changed state during the
	 * process of getting the data.
	 * Try again until we succeed. */
	do
	{
		do
		{
			err = QueryApp();
		} while (err == -ENEGATIVE_ACKNOWLEDGE);

		OscAssert_m( err == SUCCESS, "Error querying algorithm!");
		err = SetOptions();
	} while (err == -ENEGATIVE_ACKNOWLEDGE);
	FormCGIResponse();

	OscDestroy();

OscFunctionCatch()
	OscDestroy();
	OscLog(INFO, "Quit application abnormally!\n");
OscFunctionEnd()

	/*********************************************************************//*!
	 * @brief Execution starting point
	 *
	 * Handles initialization, control and unloading.
	 * @return 0 on success, -1 otherwise
	 *//*********************************************************************/
int main(void) {
	if (mainFunction() == SUCCESS)
		return 0;
	else
		return 1;
}

