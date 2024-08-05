/* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Robotiq, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Robotiq, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Copyright (c) 2014, Robotiq, Inc
*/

/*
 *  \file rq_thread.c
 *  \date July 3, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotic.com>
 *  \maintener Nicolas Lauzier <nicolas@robotiq.com>
 */

#include "rq_thread.h"

#ifdef __unix__ //For Unix

#include <pthread.h>
#include <signal.h>

static pthread_t threads[5];
static int libre[5] = {0,0,0,0,0};

int create_Thread(void *(*start_rtn)(void *))
{
	int i = 0;
	long t = 0;

	for(i = 0;i < 5; i++){
		if (libre[i] == 0){
			libre[i] = 1;
			break;
		}
	}
	int rc = pthread_create(&threads[i], NULL, start_rtn, (void *)t);
	if (rc != 0){
		//printf("ERROR; return code from pthread_create() is %d\n", rc);
		return -1;
	}
	return i;
}

int close_Thread(int iId){
	if(iId >= 0 || iId < 5){
		pthread_kill(threads[iId], SIGKILL);
		libre[iId] = 0;
		threads[iId] = 0;
		return 0;
	}
	else{
		return 1;
	}
}

void wait_thread(int iId){
	if(iId >= 0 || iId < 5){
		pthread_join(threads[iId], NULL);
		libre[iId] = 0;
		threads[iId] = 0;
	}
}

#elif defined(_WIN32)||defined(WIN32) //For Windows

#include <process.h>
#include <windows.h>

HANDLE pID[5];
int libre[5] = {0,0,0,0,0};

//int create_Thread(int iId,unsigned (__stdcall *start_address)( void * ))
int create_Thread(void( __cdecl *start_address )( void * ))
{
	int i;
	for(i = 0;i < 5; i++){
		if (libre[i] == 0){
			libre[i] = 1;
			break;
		}
	}
	pID[i] = (HANDLE)_beginthread(start_address,0,NULL);
	if(pID[i] == NULL){
		return -1;
	}
	return i;
}

int close_Thread(int iId){
	if(iId >= 0 || iId < 5){
		CloseHandle(pID[iId]);
		libre[iId] == 0;
		pID[iId] = NULL;
	}
}


void wait_thread(int iID){
	if(iID >= 0 || iID < 5){
		WaitForSingleObject(pID[iID], INFINITE);
	}
}

#endif
