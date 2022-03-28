/*
Copyright (c) 2013, IIT Madras
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

*  Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
*  Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
*  Neither the name of IIT Madras  nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Author : Paul George , Shiv Nadar University , Summer 2017
Email : command.paul@gmail.com
*/

#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#define port 10000
  
#ifdef __cplusplus
extern "C" {
#endif

  
  int init_rbb_jtag(unsigned char dummy){
    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    int client_fd = -1;

    if (socket_fd == -1) {
      fprintf(stderr, "remote_bitbang failed to make socket: %s (%d)\n",
          strerror(errno), errno);
      return -1;
    }

    fcntl(socket_fd, F_SETFL, O_NONBLOCK);
    int reuseaddr = 1;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuseaddr,
          sizeof(int)) == -1) {
      fprintf(stderr, "remote_bitbang failed setsockopt: %s (%d)\n",
          strerror(errno), errno);
      return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(socket_fd, (struct sockaddr *) &addr, sizeof(addr)) == -1) {
      fprintf(stderr, "remote_bitbang failed to bind socket: %s (%d)\n",
          strerror(errno), errno);
      return -1;
    }

    if (listen(socket_fd, 1) == -1) {
      fprintf(stderr, "remote_bitbang failed to listen on socket: %s (%d)\n",
          strerror(errno), errno);
      return -1;
    }

    socklen_t addrlen = sizeof(addr);
    if (getsockname(socket_fd, (struct sockaddr *) &addr, &addrlen) == -1) {
      fprintf(stderr, "remote_bitbang getsockname failed: %s (%d)\n",
          strerror(errno), errno);
      return -1;
    }

    printf("Listening for remote bitbang connection on port %d.\n",
        ntohs(addr.sin_port));
    fflush(stdout);
    printf("Waiting for OpenOCD .... \n");
    while (client_fd == -1){
      client_fd = accept(socket_fd, NULL, NULL);
      if (client_fd == -1) {
        if (errno != EAGAIN) {
          fprintf(stderr, "failed to accept on socket: %s (%d)\n", strerror(errno),
              errno);
        }
      } else {
        fcntl(client_fd, F_SETFL, O_NONBLOCK);
      }
    }
    return client_fd;
  }


  char decode_frame(char command){
    char frame = 0; // frame = {NA,NA,NA,reset,request_tdo,tck,tms,tdi}
    switch (command) {
      case 'B': /* fprintf(stderr, "*BLINK*\n"); */ break; // not supported in spike
      case 'b': /* fprintf(stderr, "_______\n"); */ break; // not supported in spike
      case 'r': frame &= ~((char)24); frame |= 16 ; break;
      case '0': frame = 0; break;
      case '1': frame = 1; break;
      case '2': frame = 2; break;
      case '3': frame = 3; break;
      case '4': frame = 4; break;
      case '5': frame = 5; break;
      case '6': frame = 6; break;
      case '7': frame = 7; break;
      case 'R': frame &= ~((char)24); frame |= 8 ; break;  // push out a word with the previous state held with the read bit enabled maintain previous state and just push enable the read bit
      case 'Q': frame = 32; break;
      default:
              frame &= ~((char)24);   //fprintf(stderr, "remote_bitbang got unsupported command '%d'\n",
  //                  command); // essentially de assert the read bit if it was ever up;
    }
    return frame;
  }

  // frame = {NA,NA,NA,reset,request_tdo,tck,tms,tdi}
  unsigned char get_frame(int client_fd){
    char packet;
    read(client_fd,&packet, 1);
    char msg_bits = decode_frame(packet);
    //if(packet != 0 ) printf("%x,%x,%x,%x,%x,%c\n",(msg_bits & 0x10),(msg_bits & 0x8),(msg_bits & 0x4),(msg_bits & 0x2),(msg_bits & 0x1),packet);
    return decode_frame(packet);
  }

  void send_tdo(bool tdo,int client_fd){
    char a = (tdo)? '1' :'0' ;
    write(client_fd,&a,1);
  };


#ifdef __cplusplus
}
#endif
