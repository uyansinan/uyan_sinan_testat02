/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file template_ipc.h
 * @brief Shared header file between application and its
 * CGI. Contains all information relevant to IPC between these two.
 */

#ifndef TEMPLATE_IPC_H_
#define TEMPLATE_IPC_H_

/*! @brief set to three to work with color images */
/* IMPORTANT!!: do a clean after changing this value to ensure rebuild of cgi-script */
#define NUM_COLORS 3

/* The parameter IDs to identify the different requests/responses. */
enum EnIpcParamIds
{
	GET_APP_STATE,
	GET_NEW_IMG,
	SET_IMAGE_TYPE,
	SET_EXPOSURE_TIME,
	SET_ADDINFO,
	SET_THRESHOLD
};

/*! @brief The path of the unix domain socket used for IPC between the application and its user interface. */
#define USER_INTERFACE_SOCKET_PATH "/tmp/IPCSocket.sock"

enum ObjType {OBJ_RECT, OBJ_LINE, OBJ_STRING};

enum ObjColor {WHITE, BLACK, RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, MAX_NUM_COLORS};

enum FontType {GIANT, LARGE, MEDIUMBOLD, SMALL, TINY};

/*! @brief Describes a rectangular sub-area of an image. */
struct IMG_RECT
{
	/*! @brief Rectangle left. */
	uint16 left;
	/*! @brief Rectangle right. */
	uint16 right;
	/*! @brief Rectangle top. */
	uint16 top;
	/*! @brief Rectangle bottom. */
	uint16 bottom;
	/*! @brief whether to fill the rectangle (same color).*/
	bool recFill;
	/*! @brief color enum value.*/
	uint8 color;
};


/*! @brief Describes a line in an image. */
struct IMG_LINE
{
	/*! @brief line x1 coordinate. */
	uint16 x1;
	/*! @brief line y1 coordinate. */
	uint16 y1;
	/*! @brief line x2 coordinate. */
	uint16 x2;
	/*! @brief line y2 coordinate. */
	uint16 y2;
	/*! @brief color enum value.*/
	uint8 color;
};

/*! @brief Describes a string in an image. */
struct IMG_STRING
{
	/*! @brief string x coordinate. */
	uint16 xPos;
	/*! @brief string y coordinate. */
	uint16 yPos;
	/*! @brief string length. */
	uint16 len;
	/*! @brief string font enum value. */
	uint16 font;
	/*! @brief color enum value.*/
	uint8 color;
};


/*! @brief The different modes the application can be in. */
enum EnAppMode
{
	APP_OFF,
	APP_CAPTURE_ON
};

/*! @brief Object describing all the state information the web interface needs to know about the application. */
struct APPLICATION_STATE
{
	/*! @brief Whether a new image is ready to display by the web interface. */
	bool bNewImageReady;
	/*! @brief The time stamp when the last live image was taken. */
	uint32 imageTimeStamp;
	/*! @brief The mode the application is running in. Depending on the mode different information may have to be displayed on the web interface.*/
	enum EnAppMode enAppMode;
	/*! @brief the image type index */
	unsigned int nImageType;
	/*! @brief Shutter time in micro seconds.*/
	int nExposureTime;
	/*! @brief cut off value for change detection.*/
	int nThreshold;
	/*! @brief  the step counter */
	unsigned int nStepCounter;
	/*! @brief  additional info set from browser*/
	int nAddInfo;
};

#endif /*TEMPLATE_IPC_H_*/
