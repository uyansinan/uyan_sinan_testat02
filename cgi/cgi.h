/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file cgi_template.h
 * @brief Header file of the CGI used for the webinterface of the
 * template application.
 */

#ifndef CGI_TEMPLATE_H_
#define CGI_TEMPLATE_H_

#include "oscar.h"
#include "../template_ipc.h"

/*! @brief The maximum length of the POST argument string supplied
 * to this CGI.*/
#define MAX_ARGUMENT_STRING_LEN 1024
/*! @brief The maximum length of the name string of a GET/POST
 * argument. */
#define MAX_ARG_NAME_LEN 32

/*! @brief The file name of the live image. */
#define IMG_FN "../image.gif"

/* @brief The different data types of the argument string. */
enum EnArgumentType
{
	STRING_ARG,
	INT_ARG,
	SHORT_ARG,
	BOOL_ARG
};

/*! @brief A single POST/GET argument. */
struct ARGUMENT
{
	/*! @brief The name of the argument. */
	char strName[MAX_ARG_NAME_LEN];
	/*! brief The data type of the argument. */
	enum EnArgumentType enType;
	/*! @brief Pointer to the variable this argument should be parsed
	 * to.*/
	void *pData;
	/*! @brief Pointer to a variable storing whether this argument has
	 * been supplied or not.*/
	bool *pbSupplied;
};

/*! @brief Holds the values parsed from the argument string. */
struct ARGUMENT_DATA
{
	/*! @Shutter time in micro seconds.*/
	int nExposureTime;
	/*! @brief Says whether the argument ExposureTime has been
	 * supplied or not. */
	bool bExposureTime_supplied;
	/*! @brief cut off value for change detection.*/
	int nThreshold;
	/*! @brief Says whether the argument threshold has been
	 * supplied or not. */
	bool bThreshold_supplied;
	/*! @brief index of image to be sent via cgi to webserver.*/
	int nImageType;
	/*! @brief Says whether the argument ImageType has been
	 * supplied or not. */
	bool bImageType_supplied;
	/*! @brief additionl info to be sent via cgi to webserver.*/
	int nAddInfo;
	/*! @brief Says whether the argument nAddInfo has been
	 * supplied or not. */
	bool bAddInfo_supplied;
};

/*! @brief Main object structure of the CGI. Contains all 'global'
 * variables. */
struct CGI_TEMPLATE
{
	/*! @brief IPC channel ID*/
	OSC_IPC_CHAN_ID ipcChan;

	/*! @brief The raw argument string as supplied by the web server. */
	char strArgumentsRaw[MAX_ARGUMENT_STRING_LEN];
	/*! @brief Temporary variable for argument extraction. */
	char strArgumentsTemp[MAX_ARGUMENT_STRING_LEN];

	/*! @brief The state queried from the application. */
	struct APPLICATION_STATE appState;
	/*! @brief The GET/POST arguments of the CGI. */
	struct ARGUMENT_DATA    args;
	/*! @brief Temporary data buffer for the images to be saved. */
	/* we reserve space for ADD_INFO buffer for drawing objects */
	uint8 imgBuf[2*NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT+4];
};
#endif /*CGI_TEMPLATE_H_*/
