#include "NetMsgProcess.h"
#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"



void Net_Msg_Handler(void)
{
	struct netconn *conn;
	struct netbuf *buf;
	err_t err;
	char buffer[1500] = {0};
	
	conn = netconn_new(NETCONN_UDP);
	netconn_bind(conn, IP_ADDR_ANY, 8000);

	/* Infinite loop */
	for(;;)
	{
		//osDelay(1);
		err = netconn_recv(conn, &buf);
		if(err == ERR_OK)
		{
			printf("recv msg: %s\r\n", (char*)buf);

		}else{

			printf("netcon_recv error\r\n");
		}
	}


}
