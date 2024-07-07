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
 *  \file socket.c
 *  \date May 20, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotic.com>
 *  \maintener Nicolas Lauzier <nicolas@robotiq.com>
 */

#include "rq_sensor_socket.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mutex.h"

//pour les mutex
#ifdef __unix__  //For Unix
#include <pthread.h>
#define MUTEX pthread_mutex_t
#elif defined(_WIN32) || defined(WIN32)  //For Windows
#include <windows.h>
#include <process.h>
#define MUTEX HANDLE
#endif

#define CRLF "\r\n"
#define PORT 63350
#define PORT_STREAM 63351
#define MAX_CLIENTS 1
#define BUF_SIZE 512

#define CAPTEUR_STREAM 0
#define CAPTEUR_ACCESSEUR 1

static uint8_t message[BUF_SIZE];   //message recu par TCP
static SOCKET s_socket = 0;         //Socket du serveur
static SOCKET s_socket_stream = 0;  //Socket du serveurStream
static SOCKET c_socket_acc;         //Socket du client accesseur
static SOCKET c_socket_capt = 0;    //Socket du client capteur
static MUTEX mtx;                   // Mutex
static MUTEX mtxWrite;              // Mutex pour la fonction write

static int read_client(SOCKET sock, uint8_t* buffer);
static void start_socket(int iSock);

/**
 * \fn void init_connection(void)
 * \brief Fonction qui ouvre le socket serveur et du stream et attend un client stream.
 */
int init_connection() {
  while (s_socket_stream == 0) {
    usleep(500000);
    start_socket(CAPTEUR_STREAM);
  }
  while (s_socket == 0) {
    usleep(500000);
    start_socket(CAPTEUR_ACCESSEUR);
  }

  //Attente de connection au serveur
  SOCKADDR_IN csin;
  int sinsize = sizeof(csin);
  c_socket_capt = accept(s_socket_stream, (SOCKADDR*)&csin,
                         &sinsize);  //accept attend la connexion d'un client

  return s_socket;
}

/**
 * \fn void end_connection(int sock)
 * \brief Fonction qui ferme le socket passer en parametre.
 * \param sock, integer du socket a fermer.
 */
void end_connection(int sock) {
  closesocket(sock);
}

/**
 * \fn static int read_client(SOCKET sock, char *buffer)
 * \brief Fonction qui lis ce que le client a envoyer.
 * \param sock, SOCKET client
 * \param *buffer, adresse memoire d'un buffer de char pour le message
 * \return n, nombre de caractere dans le buffer
 */
static int read_client(SOCKET sock, uint8_t* buffer) {
  int n = 0;
  if ((n = recv(sock, buffer, BUF_SIZE - 1, 0)) < 0) {
    return 0;  //signifie une erreur
  } else {
    MUTEX_LOCK(&mtx);
    strcpy(message, buffer);
    message[n] = 0;
    MUTEX_UNLOCK(&mtx);
    buffer[n] = 0;

    return n;
  }
}

/**
 * \fn void write_client(SOCKET sock, const char *buffer)
 * \brief Fonction qui envoie un message au client.
 * \param sock, SOCKET client
 * \param *buffer, adresse memoire d'un buffer de char pour le message a envoyer
 */
void write_client(SOCKET sock, uint8_t* buffer) {
  MUTEX_LOCK(&mtxWrite);
  int ret = send(sock, buffer, strlen(buffer), MSG_DONTWAIT);
  if (ret == -1 && errno == EAGAIN) {
    ret = send(sock, buffer, strlen(buffer), MSG_DONTWAIT);
  } else if (ret == -1 && errno == ECONNRESET) {
    SOCKADDR_IN csin;
    int sinsize = sizeof(csin);
    if (sock == c_socket_capt) {
      close(c_socket_capt);
      close(s_socket_stream);
      start_socket(CAPTEUR_STREAM);
      c_socket_capt = 0;
      while (c_socket_capt == 0) {
        c_socket_capt =
            accept(s_socket_stream, (SOCKADDR*)&csin,
                   &sinsize);  //accept attend la connexion d'un client
      }
    } else if (sock == c_socket_acc) {
      close(c_socket_acc);
      close(s_socket);
      start_socket(CAPTEUR_ACCESSEUR);
      c_socket_acc = 0;
      while (c_socket_acc == 0) {
        c_socket_acc =
            accept(s_socket, (SOCKADDR*)&csin,
                   &sinsize);  //accept attend la connexion d'un client
      }
    }
  }
  MUTEX_UNLOCK(&mtxWrite);
}

/**
 * \fn void *Read_socket(void *t)
 * \brief Fonction utiliser par le thread pour faire une boucle
 *     infini de lecture de message recu par le client et si client en erreur,
 *     on attend le prochain client qui se connecte et continu la boucle.
 * \param *t, paramettre bidon pour que le thread fonctionne.
 */
#ifdef __unix__  //For Unix
void* Read_socket(void* t)
#elif defined(_WIN32) || defined(WIN32)  //For Windows
void __cdecl Read_socket(void* t)
#endif
{
  SOCKADDR_IN csin;
  int sinsize = sizeof(csin);
  char testConnexion[BUF_SIZE];
  c_socket_acc = accept(s_socket, (SOCKADDR*)&csin, &sinsize);
  while (1) {
    if (read_client(c_socket_acc, testConnexion) == 0) {
      close(c_socket_acc);
      close(s_socket);
      start_socket(CAPTEUR_ACCESSEUR);
      //Si on perd la connexion on attend la prochaine.
      c_socket_acc = accept(s_socket, (SOCKADDR*)&csin, &sinsize);
    }
  }
}

/**
 * \fn void set_socket_message_to_null(char * chr_return)
 * \brief Fonction qui vide la variable globale message.
 */
void set_socket_message_to_null(void) {
  MUTEX_LOCK(&mtx);
  message[0] = 0;
  MUTEX_UNLOCK(&mtx);
}

/**
 * \fn void socket_message(char * chr_return)
 * \brief Fonction qui retourne le message via le paramettre.
 * \param * chr_return, adresse memoire pour evoie du message via le parammetre
 * \param *buffer, adresse memoire d'un buffer de char pour le message a envoyer
 */
void socket_message(uint8_t* chr_return) {
  MUTEX_LOCK(&mtx);
  strcpy(chr_return, message);
  MUTEX_UNLOCK(&mtx);
}

/**
 * \fn SOCKET get_client_sock_accessor(void)
 * \brief Fonction qui retourne socket du client presentement connecter.
 */
SOCKET get_client_sock_accessor(void) {
  return c_socket_acc;
}

/**
 * \fn SOCKET get_client_sock_stream(void)
 * \brief Fonction qui retourne socket du capteur presentement connecter.
 */
SOCKET get_client_sock_stream(void) {
  return c_socket_capt;
}

/**
 * \fn SOCKET get_socket_stream(void)
 * \brief Fonction qui retourne socket serveur pour le stream.
 */
SOCKET get_socket_stream() {
  return s_socket_stream;
}

static void start_socket(int iSock) {
  SOCKET sock = socket(AF_INET, SOCK_STREAM, 0);
  SOCKADDR_IN sin = {0};

  if (sock == INVALID_SOCKET) {
    return;
  }

  //Pour le socket des accesseur
  sin.sin_addr.s_addr = htonl(INADDR_ANY);
  if (iSock == CAPTEUR_ACCESSEUR)
    sin.sin_port = htons(PORT);
  if (iSock == CAPTEUR_STREAM)
    sin.sin_port = htons(PORT_STREAM);
  sin.sin_family = AF_INET;

  //Permet d'ouvrir les sockets sur les bons ports
  if (bind(sock, (SOCKADDR*)&sin, sizeof sin) == SOCKET_ERROR) {
    return;
  }

  //Permet l'ecoute des client
  if (listen(sock, MAX_CLIENTS) == SOCKET_ERROR) {
    return;
  }

  if (iSock == CAPTEUR_ACCESSEUR)
    s_socket = sock;
  if (iSock == CAPTEUR_STREAM)
    s_socket_stream = sock;
}
