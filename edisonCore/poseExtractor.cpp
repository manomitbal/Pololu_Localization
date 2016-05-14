#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "mraa.h"
#include <time.h>
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>


using namespace std;

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int main()
{
    
    FILE *in1;
    char buff[1024];
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    portno = 1989;
    //sockfd = socket(AF_INET, SOCK_STREAM, 0);  
       
    if(!(in1 = popen("./imuData", "r")))
    {
        return -1;
    }

    while(fgets(buff, sizeof(buff), in1)!=NULL)
    {
        
        string full(buff);
        if(full.size() > 5)
        {
           // portno = 1989;
            sockfd = socket(AF_INET, SOCK_STREAM, 0);
            if (sockfd < 0)
                error("ERROR opening socket");
            server = gethostbyname("192.168.2.48");
            if (server == NULL)
            {
                fprintf(stderr,"ERROR, no such host\n");
                exit(0);
            }
            bzero((char *) &serv_addr, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            bcopy((char *)server->h_addr,
                    (char *)&serv_addr.sin_addr.s_addr,
                    server->h_length);
            serv_addr.sin_port = htons(portno);
            if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
                error("ERROR connecting");
           //printf("Please enter the message: ");
           // bzero(buffer,256);
           // fgets(buffer,255,stdin);
            n = write(sockfd,buff,strlen(buff));
            //cout<<full<<endl;
            close(sockfd);
        }
    }
    close(sockfd);
    pclose(in1);
    return 0;
}
