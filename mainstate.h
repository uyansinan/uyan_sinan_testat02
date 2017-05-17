/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file mainstate.h
 * @brief Definitions for main state machine
	************************************************************************/
	
#ifndef MAINSTATE_H
#define MAINSTATE_H

#include "template.h"

enum MainStateEvents {
	FRAMESEQ_EVT,       /* frame ready to process (before setting up next frame capture) */
	FRAMEPAR_EVT,       /* frame ready to process (parallel to next capture) */
	IPC_GET_APP_STATE_EVT, /* Webinterface asks for the current application state. */
	IPC_GET_NEW_IMG_EVT, /* Webinterface asks for a new image. */
	IPC_SET_IMAGE_TYPE_EVT /* Webinterface wants to set the image type. */
};


/*typedef struct MainState MainState;*/
typedef struct MainState {
	Hsm super;
	State showGray, showThreshold, showBackground;
} MainState;


void MainStateConstruct(MainState *me);


#endif /*MAINSTATE_H*/
