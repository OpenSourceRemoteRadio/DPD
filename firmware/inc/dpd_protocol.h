/*
Copyright (c) 2017, BigCat Wireless Pvt Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.



THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*//**
 * @file dpd_protocol.h
 * @brief Protocol Header
 * */

#ifndef HEPTA_A10_2X4_PROTOCOL_H_
#define HEPTA_A10_2X4_PROTOCOL_H_
#ifdef __cplusplus
extern "C"
{
#endif
/******************************************************************************
 * Include public/global Header files
******************************************************************************/

#include <stdio.h>	
#define _GNU_SOURCE 
#define __USE_GNU	 
#include <stdlib.h>      
#include <string.h>      
#include <unistd.h>      
#include <sys/types.h>   
#include <sys/socket.h>  
#include <sys/select.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>   
#include <netdb.h>       
#include <signal.h>      
#include <unistd.h>      
#include <stdio.h>       
#include <stdlib.h>      
#include <string.h>      
#include <unistd.h>      
#include <sys/types.h>   
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>   
#include <sys/mman.h>    
#include <netdb.h>       
#include <sys/stat.h>    
#include <fcntl.h>       
#include <pthread.h>     
#include <sys/ioctl.h>   
#include <stdbool.h>

/******************************************************************************
 * Include private Header files
******************************************************************************/


#define SERVER_PORT  "4545"
#define MAX_SERVER_COUNT 2
#define MAX_QUEUE_NODES 30  //0 and 1 are not acceptable
#define CONNECTION_TIME_OUT_SEC 5
#define SEND_TIME_OUT_SEC 5
#define RECV_TIME_OUT_SEC 5

#define TCP_KEEPALIVE_ENABLE 1
#define TCP_KEEPCNT_VALUE 5
#define TCP_KEEPIDLE_VALUE 5
#define TCP_KEEPINTVL_VALUE 1

typedef volatile unsigned int * bus_address;
void * transport_read_thread(void *ptr);
void * transport_write_thread(void *ptr);
void * command_thread(void *ptr);
void tcpClient_socket_close(void * handles_ptr);

/** 
 * @ingroup CommandManager
 * @brief Type of the command to be transfered.
 */

typedef enum
{
	TRANSFER_TYPE_POLL =  0, /**< Command will be polled from the server and the client will respond for the command whenever it receives poll command. */ 
	TRANSFER_TYPE_SERVICE, /**< Command will be sent from the server and the client will register the operation in the async database and responds periodically which trigers callback registered in the server. */ 
}Transfer_Type_Enum_Def;


/** 
 * @ingroup CommandManager
 * @brief Actions to manage the running application.
 */

typedef enum
{
	COMMAND_MANAGE_CLOSE, 			/**< Command to close the application .*/
	COMMAND_MANAGE_VERSION,		/**< Command to read version numbers for applications and drivers.*/
}mannage_command_action;


/** 
 * @ingroup SocketManager
 * @brief Socket descpriptors.
 */

typedef struct
{
	int 	connectedfd 	; 					/**< Descriptor for the connected socket for read and write. */
	char * 	pserver_ip_address ; 				/**< Server IPv4 address */
	char * 	port_no      	;
	struct addrinfo	* addrinfo_ptr;
	bool	connected		;
	bool	connecting		;
	bool	reading			;
	unsigned int server_index;
}socket_handles;


/** 
 * @ingroup CommandManager
 * @brief Link list to maintain the commands in the scheduler.
 */


typedef struct
{
	struct command_queue *Next;	/**< Pointer to the next element in queue.*/
	void * pQueueData;			/**< Command of the current queue element.*/
	unsigned int id;				/**< Id for the current command.*/
}command_queue,*pcommand_queue;

/**
 * @ingroup CommandManager
 * @brief Link list Heads to maintain the commands in the scheduler.
 */

typedef struct
{
	pthread_mutex_t queue_lock;		/**< Mutex lock to synchronise queueing and dequeuing .*/
	struct command_queue *Head;		/**< Pointer to the first command in the queue.*/
	char *queue_name;			/**< Name of the command queue.*/
	unsigned int num_of_nodes;			/**< Number of command elements in the queue.*/
}command_queue_head, *pcommand_queue_head;


/**
 * @ingroup CommandManager
 * @ingroup SocketManager
 * @brief Scheduler to maintain the firmware details.
 */

typedef struct
{
	command_queue_head        	WriteQueueHead;									/**< Queue head for commands to be sent to server*/
	command_queue_head        	CommandQueueHead;								/**< Queue head for valid commands to be executed*/
	pthread_t 		          	pServerConnectThreadHandle[MAX_SERVER_COUNT]; 	/**< Thread to perform write in the hardware modules*/
	pthread_t 		          	pTransportReadThreadHandle[MAX_SERVER_COUNT]; 	/**< Thread to perform write in the hardware modules*/
	pthread_t 		          	pTransportWriteThreadHandle; 					/**< Thread to perform read from the hardware modules*/
	pthread_cond_t	          	WriteThreadCondition;							/**< Condition signal to signal write thread*/
	pthread_t 		          	pCommandThreadHandle; 							/**< Thread to manage hardware modules*/
	pthread_cond_t            	CommandThreadCondition;							/**< Condition signal to signal command thread*/
	pthread_cond_t	          	condition_signal; 								/**< Condition to signal the read/write threads*/
	pthread_mutex_t           	lock; 											/**< Lock to synchronise the access to the fields in the scheduler header*/
	bool			          	is_thread		;								/**< Condition to proceed to next iteration in threads*/
	socket_handles	  		  	ServerClientDetails[MAX_SERVER_COUNT]; 			/**< Heptaconnection handles to get server client details*/
	unsigned int				number_of_server;								/**< Number of servers initiated*/
	bus_address        		  	ddrmemmapped;									/**< Virtual address for DDR3 memory*/
}socket_manager_header,*psocket_manager_header;



extern char server_ip_address[15]; /**< Server IP address. */


#ifdef __cplusplus
}/* close the extern "C" { */
#endif
#endif
