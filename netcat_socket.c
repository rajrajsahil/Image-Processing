#include <stdio.h>
int main(int argc, char* argv[])
{
        WSADATA wsaData = {0};
	WORD wVer = MAKEWORD(2,2);
         int nRet = WSAStartup( wVer, &wsaData );
 
        WORD WSAEvent = 0;
	WORD WSAErr = 0;
	SOCKET hServer = {0};
 
    hServer = socket( AF_INET, SOCK_STREAM, IPPROTO_IP );
    sockaddr_in saServer = {0};
    saServer.sin_family = PF_INET;
    saServer.sin_port = htons( 5001 );
    saServer.sin_addr.s_addr = inet_addr( "172.16.26.92" ); //address of raspberry pi
 
    nRet = connect( hServer, (sockaddr*)&saServer, sizeof( sockaddr ) );
    if( nRet == SOCKET_ERROR ) 
	{
		cout << "Connection to server failed" << endl;
         }	
	else
               do
	       {
		        stat = recv(socket, (char*)&imsize, sizeof(imsize),0);
	       }while(stat<2040); //the reason i used a sat value as finite is because i am reading a slice of data from a live video stream.
return 0;
}