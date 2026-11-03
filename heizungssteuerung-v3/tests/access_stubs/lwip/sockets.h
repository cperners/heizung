#pragma once
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define getpeername test_getpeername
int test_getpeername(int,struct sockaddr*,socklen_t*);
