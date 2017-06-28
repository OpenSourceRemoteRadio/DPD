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
*/
/**
 * @file hepta_a10_2x4_dpd_reg_map.c
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Application file for dpd_reg_map module
 * */

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

#include "dpd_reg_map.h"
#include "dpd_protocol.h"
#include "dpd_firmware.h"
#include "dpd_command_format.h"
char tcp_pkt_headr[3]="alt";
extern socket_manager_header hepta_a10_2x4_socket_manager;
/******************************************************************************
 * Device handler Function definition
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Driver handler function. Interacts with dpd_reg_map driver and gets address map information
 * @param instance_num Instance number of the dpd_reg_map module for which handler is to be returned
 * @retval Handler Pointer Requested Instance of dpd_reg_map module exits
 * @retval NULL Requested Instance of dpd_reg_map module doesnot exit
 * */


pdevice_handler hepta_a10_2x4_dpd_reg_map_get_handler(unsigned int instance_num)
{
    int fd;
    unsigned char ret;
    char number[4];
    char device_node[256];
    static pdevice_handler device_handler_ptr=NULL;
    static unsigned int device_num_of_instance=-1;
    
    unsigned int i=0;
    int pid;
    memory_map_resource *memory_map_resource_ptr;
    unsigned int num_of_resource=0;
    unsigned int current_resource=0;
    bus_address mmap_address;
    int service_index = 0;
    
    
    if(device_handler_ptr==NULL)
    {
    	//memset(device_node,0,256);	//erase device_node
    	for(i=0;i<256;i++)
    	    	{
    	    		device_node[i]=0;
    	    	}
    	i = 0;
    	snprintf(device_node,sizeof(device_node),"/proc/%s/num_of_instance","dpd_reg_map");
    	DEBUG_PRINT(DEBUG_LEVEL_INFO,("open file path %s \n",device_node));
    	fd=open(device_node,O_RDWR); //open corresponding proc file
    	if(fd <=0)
    	{
    		DEBUG_PRINT(DEBUG_LEVEL_ERROR,("unable to open %s \n",device_node));
    		return NULL;
    	}
    	ret=read((int)fd,number,4);  
    	close(fd);
    	number[ret-1]='\0';
    	device_num_of_instance=atoi(number);
    	
    	
    	device_handler_ptr = malloc(sizeof(device_handler)*device_num_of_instance);
    	if(device_handler_ptr == NULL)
    	{
    		DEBUG_PRINT(DEBUG_LEVEL_ERROR,("Unable allocate memory \n"));
    		return NULL;			
    	}
    	
    	
    	for(i=0;i<device_num_of_instance;i++)
    	{
    		//memset(device_node,0,256);	//erase device_node content
    		for(i=0;i<256;i++)
    		    	{
    		    		device_node[i]=0;
    		    	}
    		i = 0;
    		/**< assembling device file name from module type*/
    		snprintf(device_node,sizeof(device_node),"%s%s%d","/dev/","dpd_reg_map",i);
    		//DEBUG_PRINT(DEBUG_LEVEL_INFO,("%s %d %s \n",__FUNCTION__,__LINE__,device_node));
    		device_handler_ptr[i].dev_name = strdup(device_node);
    		device_handler_ptr[i].dev_file_handler = open(device_node,O_RDWR);
    		device_handler_ptr[i].device_instance_number = i;
    
    		if(device_handler_ptr[i].dev_file_handler < 1)
    		{
    			DEBUG_PRINT(DEBUG_LEVEL_ERROR,("Resource is not available \n"));
    			return NULL;
    		}
    		
    		pid = getpid();
    		ioctl(device_handler_ptr[i].dev_file_handler,IOCTL_SET_PID,(unsigned long)&pid);
    
    		ioctl(device_handler_ptr[i].dev_file_handler,IOCTL_GET_NUM_RESOURCE,(unsigned long)&num_of_resource);
    		if(num_of_resource > 0)
    		{
    			memory_map_resource_ptr=(memory_map_resource *)malloc(sizeof(memory_map_resource)*num_of_resource);
    			ioctl(device_handler_ptr[i].dev_file_handler,IOCTL_GET_RESOURCES,(unsigned long)memory_map_resource_ptr);
    
    
    			device_handler_ptr[i].reg_phy_addr = (bus_address *)malloc(sizeof(bus_address)*(num_of_resource));
    			device_handler_ptr[i].reg_remap_addr = (bus_address *)malloc(sizeof(bus_address)*(num_of_resource));
    
    			for(current_resource=0;current_resource<num_of_resource;current_resource++)
    			{
    				mmap_address = mmap(NULL,memory_map_resource_ptr[current_resource].size_in_bytes, PROT_READ|PROT_WRITE, MAP_SHARED, \
    						device_handler_ptr[i].dev_file_handler,(off_t)memory_map_resource_ptr[current_resource].start_address);
    				if (mmap_address == MAP_FAILED)
    				{
    					perror("mmap");
    					DEBUG_PRINT(DEBUG_LEVEL_ERROR,("Memory Map Failed  \n"));
    					return NULL;
    				}
    
    				device_handler_ptr[i].reg_phy_addr[current_resource]=memory_map_resource_ptr[current_resource].start_address;
    				device_handler_ptr[i].reg_remap_addr[current_resource]=mmap_address;
    			}
    			free(memory_map_resource_ptr);
    		}
    
    		ioctl(device_handler_ptr[i].dev_file_handler,IOCTL_GET_UIO_MINOR_NUMBER,(unsigned long)& (device_handler_ptr[i].uio_irq_device_number));
    
    		if(Hepta_A10_2x4_dpd_reg_map_SERVICE_COUNT > 0)
    		{
    			device_handler_ptr[i].service_id_array =  malloc(sizeof(unsigned int)* Hepta_A10_2x4_dpd_reg_map_SERVICE_COUNT );
    			for(service_index=0;service_index< Hepta_A10_2x4_dpd_reg_map_SERVICE_COUNT ; service_index++)
    			{
    				device_handler_ptr[i].service_id_array[service_index] = (unsigned int) (-1);
    			}
    		}
    	}
    }
    
    if(instance_num >= device_num_of_instance)
    {
    	return NULL;
    }
    else
    {
    	return (device_handler_ptr+instance_num);
    }	
}
/******************************************************************************/
bool command_queue_append(pcommand_queue_head pHead,void *data, int id)
{
	pcommand_queue Queue=malloc(sizeof(command_queue));
	pcommand_queue ptr;
	if(Queue==NULL)
		return false;
	Queue->Next=NULL;
	Queue->pQueueData=data;
	Queue->id=id;
	pthread_mutex_lock(&pHead->queue_lock);
	if(pHead->Head==NULL)
	{
		pHead->Head=(struct command_queue *)Queue;
	}
	else
	{
		ptr=(pcommand_queue)pHead->Head;
		while(ptr->Next)
			ptr=(pcommand_queue)ptr->Next;
		ptr->Next=(struct command_queue *)Queue;
	}
	/*release condition sleep whoever waiting for this queue*/
	pHead->num_of_nodes++;

	//to maintain maximum queue entry count
	if(pHead->num_of_nodes >= MAX_QUEUE_NODES)
	{
		//printf("Number of nodes in %s exceeds limit %d\n",pHead->queue_name,pHead->num_of_nodes);
		ptr = (pcommand_queue)pHead->Head;
		if(ptr != NULL)
		{
			pHead->Head = ptr->Next;
			free(ptr->pQueueData);
			free(ptr);
			pHead->num_of_nodes--;
		}
	}

	//printf("Number of nodes in %s queue %d\n",pHead->queue_name,pHead->num_of_nodes);

	pthread_mutex_unlock(&pHead->queue_lock);
	return true;
}

/******************************************************************************/
/**
 * @ingroup SocketManager
 * @brief Sends service disable message to the server with Id as id. This function queues the command in the write queue.
 * @param Id Command id for the service message to be disabled.
 * @param module_index Module type that should be specified in the service message
 * @return Status 0 on queueing the service message in the write queue.
 * @return Status -1 on fail .
 */


int send_service_ack_disable(psocket_manager_header socket_handler,unsigned int Id,Module_Index_Enum_Def module_index)
{
	pPacket_Header pTcpPacket=0;
	pCommand_Header pCmdHeader;
unsigned int j = 0;
	if(socket_handler==NULL)
	{
		return -1;
	}

	pTcpPacket=(pPacket_Header)malloc(TCP_PACKET_HEADER_SIZE+sizeof(Command_Header));

	//strncpy((char*)pTcpPacket->Header_String,TCP_PACKET_HEADER_STRING,TCP_PACKET_HEADER_STRING_SIZE);

	for(j=0;j<TCP_PACKET_HEADER_STRING_SIZE;j++)
	    	{
		pTcpPacket->Header_String[j]=tcp_pkt_headr[j];
	    	}
	j = 0;
	pTcpPacket->Id = Id;
	pTcpPacket->Length = sizeof(Command_Header);
	//pTcpPacket->server_index = server_index;

	pCmdHeader=(pCommand_Header)pTcpPacket->pData;

	pCmdHeader->Transfer_Type = TRANSFER_TYPE_SERVICE;
	pCmdHeader->Operation_Type = OPERATION_TYPE_ACK;
	pCmdHeader->Address = 0;
	pCmdHeader->Instance = 0;
	pCmdHeader->Length = 0;
	//pCmdHeader->server_index = server_index;
	pCmdHeader->Module_Index = module_index;


	command_queue_append(&socket_handler->WriteQueueHead,pTcpPacket,pTcpPacket->Id);
	pthread_mutex_lock(&socket_handler->lock);
	pthread_cond_signal(&socket_handler->WriteThreadCondition);
	pthread_mutex_unlock(&socket_handler->lock);

	return 0;
}
/******************************************************************************
 * ISR Function definition
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Interrupt Service Routine. Interacts with dpd_reg_map UIO driver and gets interrupt status
 * @param dpd_reg_map_handler_ptr Device handler of the dpd_reg_map module
 * */


void * Hepta_A10_2x4_dpd_reg_map_isr_thread(void * dpd_reg_map_handler_ptr)
{
    pdevice_handler pdpd_reg_map_handler = (pdevice_handler)dpd_reg_map_handler_ptr;
    
    int uio_fd;
    char number[4];
    char device_node[256];
    //unsigned int no_of_interrupts_triggered = 0;
    unsigned char ret;
    unsigned int i = 0;
    
   // memset(device_node,0,256);	//erase device_node
    for(i=0;i<256;i++)
        	{
        		device_node[i]=0;
        	}
    i = 0;
    snprintf(device_node,sizeof(device_node),"/dev/uio%d",pdpd_reg_map_handler->uio_irq_device_number);
    
    uio_fd = open(device_node,O_RDONLY);
    
    if(uio_fd <= 0)
    {
    	return NULL;
    }
    
    while(1)
    {
    	ret = read(uio_fd,number,4);
    	DEBUG_PRINT(DEBUG_LEVEL_INFO,("dpd_reg_map %s Interrupt received\n",pdpd_reg_map_handler->device_instance_number));
    	number[ret-1]='\0';
    	//no_of_interrupts_triggered=atoi(number);
    	
    	if(pdpd_reg_map_handler->ISR_callback_function_ptr != NULL)
    	{
    		pdpd_reg_map_handler->ISR_callback_function_ptr(pdpd_reg_map_handler,pdpd_reg_map_handler->ISR_callback_argument);
    	}
    	else
    	{	
    		break;
    	}
    }
    
    close(uio_fd);
    
    return NULL;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Interrupt Service Register API. Register callback function for dpd_reg_map interrupt
 * @param dpd_reg_map_handler_ptr Device handler of the dpd_reg_map module
 * @param callback_function Callback function for dpd_reg_map IRQ
 * @param callback_argument Argument to be passed to dpd_reg_map ISR callback function
 * */


void * Hepta_A10_2x4_dpd_reg_map_register_isr(void * dpd_reg_map_handler_ptr,isr_callback callback_function,void * callback_argument)
{
    pdevice_handler pdpd_reg_map_handler = (pdevice_handler)dpd_reg_map_handler_ptr;
    if(pdpd_reg_map_handler->uio_irq_device_number >=0 )
    {
    	pdpd_reg_map_handler->ISR_callback_function_ptr = callback_function;
    	pdpd_reg_map_handler->ISR_callback_argument = callback_argument;
    	pthread_create(&(pdpd_reg_map_handler->ISR_thread_handler),NULL, Hepta_A10_2x4_dpd_reg_map_isr_thread,pdpd_reg_map_handler);

    }
    return 0; //added to clear the warning: control reaches end of non-void function C
}

/******************************************************************************
 * User defined Function definition
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Start Correlation
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_control_corr_start_config_Start_Correlation(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->CONTROL.reg.Corr_start = DPD_REG_MAP_Corr_start_Start_Correlation;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Corr_start based on input parameter
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @param Corr_start_configuration Input configuration of Corr_start
 * */


void Hepta_A10_2x4_dpd_reg_map_control_config_Corr_start(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_CONTROL_Corr_start_Enum_Def Corr_start_configuration)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map)dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->CONTROL.reg.Corr_start = Corr_start_configuration;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Read Corr_start
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval DPD_REG_MAP_Corr_start_Start_Correlation Start Correlation
 * */


DPD_REG_MAP_CONTROL_Corr_start_Enum_Def Hepta_A10_2x4_dpd_reg_map_control_read_Corr_start(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return resource_ptr->CONTROL.reg.Corr_start;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Corr_start status is Start Correlation
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Corr_start status is Start Correlation
 * @retval false Corr_start status is not Start Correlation
 * */


bool Hepta_A10_2x4_dpd_reg_map_control_isCorr_start_Start_Correlation(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->CONTROL.reg.Corr_start == DPD_REG_MAP_Corr_start_Start_Correlation);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Read RP_Capture Performed
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval DPD_REG_MAP_RP_Capture_Performed_Reverse_path_capture_done Reverse path capture done
 * */


DPD_REG_MAP_FPGA_EVENTS_RP_Capture_Performed_Enum_Def Hepta_A10_2x4_dpd_reg_map_fpga_events_read_RP_Capture_Performed(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return resource_ptr->FPGA_EVENTS.reg.RP_Capture_Performed;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check RP_Capture Performed status is Reverse path capture done
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true RP_Capture Performed status is Reverse path capture done
 * @retval false RP_Capture Performed status is not Reverse path capture done
 * */


bool Hepta_A10_2x4_dpd_reg_map_fpga_events_isRP_Capture_Performed_Reverse_path_capture_done(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->FPGA_EVENTS.reg.RP_Capture_Performed == DPD_REG_MAP_RP_Capture_Performed_Reverse_path_capture_done);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Read Lag Value Computed
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval DPD_REG_MAP_Lag_Value_Computed_Lag_computation_done Lag computation done
 * */


DPD_REG_MAP_FPGA_EVENTS_Lag_Value_Computed_Enum_Def Hepta_A10_2x4_dpd_reg_map_fpga_events_read_Lag_Value_Computed(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return resource_ptr->FPGA_EVENTS.reg.Lag_Value_Computed;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Lag Value Computed status is Lag computation done
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Lag Value Computed status is Lag computation done
 * @retval false Lag Value Computed status is not Lag computation done
 * */


bool Hepta_A10_2x4_dpd_reg_map_fpga_events_isLag_Value_Computed_Lag_computation_done(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->FPGA_EVENTS.reg.Lag_Value_Computed == DPD_REG_MAP_Lag_Value_Computed_Lag_computation_done);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Ctx Start Capture
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_captx_buf_cntrl_start_config_Ctx_Start_Capture(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->CAPTX_BUF_CNTRL.reg.Start = DPD_REG_MAP_Start_Ctx_Start_Capture;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Ctx Stop Capture
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_captx_buf_cntrl_start_config_Ctx_Stop_Capture(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->CAPTX_BUF_CNTRL.reg.Start = DPD_REG_MAP_Start_Ctx_Stop_Capture;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Start based on input parameter
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @param Start_configuration Input configuration of Start
 * */


void Hepta_A10_2x4_dpd_reg_map_captx_buf_cntrl_config_Start(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_CAPTX_BUF_CNTRL_Start_Enum_Def Start_configuration)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map)dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->CAPTX_BUF_CNTRL.reg.Start = Start_configuration;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Read Start
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval DPD_REG_MAP_Start_Ctx_Start_Capture Ctx Start Capture
 * @retval DPD_REG_MAP_Start_Ctx_Stop_Capture Ctx Stop Capture
 * */


DPD_REG_MAP_CAPTX_BUF_CNTRL_Start_Enum_Def Hepta_A10_2x4_dpd_reg_map_captx_buf_cntrl_read_Start(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return resource_ptr->CAPTX_BUF_CNTRL.reg.Start;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Start status is Ctx Start Capture
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Start status is Ctx Start Capture
 * @retval false Start status is not Ctx Start Capture
 * */


bool Hepta_A10_2x4_dpd_reg_map_captx_buf_cntrl_isStart_Ctx_Start_Capture(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->CAPTX_BUF_CNTRL.reg.Start == DPD_REG_MAP_Start_Ctx_Start_Capture);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Start status is Ctx Stop Capture
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Start status is Ctx Stop Capture
 * @retval false Start status is not Ctx Stop Capture
 * */


bool Hepta_A10_2x4_dpd_reg_map_captx_buf_cntrl_isStart_Ctx_Stop_Capture(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->CAPTX_BUF_CNTRL.reg.Start == DPD_REG_MAP_Start_Ctx_Stop_Capture);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Cobsrx Start Capture
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_capobsrx_buf_cntrl_start_config_Cobsrx_Start_Capture(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->CAPOBSRX_BUF_CNTRL.reg.Start = DPD_REG_MAP_Start_Cobsrx_Start_Capture;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Cobsrx Stop Capture
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_capobsrx_buf_cntrl_start_config_Cobsrx_Stop_Capture(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->CAPOBSRX_BUF_CNTRL.reg.Start = DPD_REG_MAP_Start_Cobsrx_Stop_Capture;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Start based on input parameter
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @param Start_configuration Input configuration of Start
 * */


void Hepta_A10_2x4_dpd_reg_map_capobsrx_buf_cntrl_config_Start(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_CAPOBSRX_BUF_CNTRL_Start_Enum_Def Start_configuration)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map)dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->CAPOBSRX_BUF_CNTRL.reg.Start = Start_configuration;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Read Start
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval DPD_REG_MAP_Start_Cobsrx_Start_Capture Cobsrx Start Capture
 * @retval DPD_REG_MAP_Start_Cobsrx_Stop_Capture Cobsrx Stop Capture
 * */


DPD_REG_MAP_CAPOBSRX_BUF_CNTRL_Start_Enum_Def Hepta_A10_2x4_dpd_reg_map_capobsrx_buf_cntrl_read_Start(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return resource_ptr->CAPOBSRX_BUF_CNTRL.reg.Start;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Start status is Cobsrx Start Capture
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Start status is Cobsrx Start Capture
 * @retval false Start status is not Cobsrx Start Capture
 * */


bool Hepta_A10_2x4_dpd_reg_map_capobsrx_buf_cntrl_isStart_Cobsrx_Start_Capture(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->CAPOBSRX_BUF_CNTRL.reg.Start == DPD_REG_MAP_Start_Cobsrx_Start_Capture);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Start status is Cobsrx Stop Capture
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Start status is Cobsrx Stop Capture
 * @retval false Start status is not Cobsrx Stop Capture
 * */


bool Hepta_A10_2x4_dpd_reg_map_capobsrx_buf_cntrl_isStart_Cobsrx_Stop_Capture(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->CAPOBSRX_BUF_CNTRL.reg.Start == DPD_REG_MAP_Start_Cobsrx_Stop_Capture);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Cobs Pattern data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_cobs_capture_select_config_Cobs_Pattern_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Cobs_Capture_Select = DPD_REG_MAP_Cobs_Capture_Select_Cobs_Pattern_data;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Cobs DPD input data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_cobs_capture_select_config_Cobs_DPD_input_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Cobs_Capture_Select = DPD_REG_MAP_Cobs_Capture_Select_Cobs_DPD_input_data;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Cobs DPD output data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_cobs_capture_select_config_Cobs_DPD_output_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Cobs_Capture_Select = DPD_REG_MAP_Cobs_Capture_Select_Cobs_DPD_output_data;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Cobs ADC data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_cobs_capture_select_config_Cobs_ADC_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Cobs_Capture_Select = DPD_REG_MAP_Cobs_Capture_Select_Cobs_ADC_data;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Cobs_Capture_Select based on input parameter
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @param Cobs_Capture_Select_configuration Input configuration of Cobs_Capture_Select
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_config_Cobs_Capture_Select(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_Rev_Capture_Config_Cobs_Capture_Select_Enum_Def Cobs_Capture_Select_configuration)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map)dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Cobs_Capture_Select = Cobs_Capture_Select_configuration;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Read Cobs_Capture_Select
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval DPD_REG_MAP_Cobs_Capture_Select_Cobs_Pattern_data Cobs Pattern data
 * @retval DPD_REG_MAP_Cobs_Capture_Select_Cobs_DPD_input_data Cobs DPD input data
 * @retval DPD_REG_MAP_Cobs_Capture_Select_Cobs_DPD_output_data Cobs DPD output data
 * @retval DPD_REG_MAP_Cobs_Capture_Select_Cobs_ADC_data Cobs ADC data
 * */


DPD_REG_MAP_Rev_Capture_Config_Cobs_Capture_Select_Enum_Def Hepta_A10_2x4_dpd_reg_map_rev_capture_config_read_Cobs_Capture_Select(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return resource_ptr->Rev_Capture_Config.reg.Cobs_Capture_Select;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Cobs_Capture_Select status is Cobs Pattern data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Cobs_Capture_Select status is Cobs Pattern data
 * @retval false Cobs_Capture_Select status is not Cobs Pattern data
 * */


bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCobs_Capture_Select_Cobs_Pattern_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->Rev_Capture_Config.reg.Cobs_Capture_Select == DPD_REG_MAP_Cobs_Capture_Select_Cobs_Pattern_data);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Cobs_Capture_Select status is Cobs DPD input data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Cobs_Capture_Select status is Cobs DPD input data
 * @retval false Cobs_Capture_Select status is not Cobs DPD input data
 * */


bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCobs_Capture_Select_Cobs_DPD_input_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->Rev_Capture_Config.reg.Cobs_Capture_Select == DPD_REG_MAP_Cobs_Capture_Select_Cobs_DPD_input_data);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Cobs_Capture_Select status is Cobs DPD output data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Cobs_Capture_Select status is Cobs DPD output data
 * @retval false Cobs_Capture_Select status is not Cobs DPD output data
 * */


bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCobs_Capture_Select_Cobs_DPD_output_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->Rev_Capture_Config.reg.Cobs_Capture_Select == DPD_REG_MAP_Cobs_Capture_Select_Cobs_DPD_output_data);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Cobs_Capture_Select status is Cobs ADC data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Cobs_Capture_Select status is Cobs ADC data
 * @retval false Cobs_Capture_Select status is not Cobs ADC data
 * */


bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCobs_Capture_Select_Cobs_ADC_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->Rev_Capture_Config.reg.Cobs_Capture_Select == DPD_REG_MAP_Cobs_Capture_Select_Cobs_ADC_data);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Ctx Pattern data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_ctx_capture_select_config_Ctx_Pattern_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Ctx_Capture_Select = DPD_REG_MAP_Ctx_Capture_Select_Ctx_Pattern_data;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Ctx DPD input data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_ctx_capture_select_config_Ctx_DPD_input_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Ctx_Capture_Select = DPD_REG_MAP_Ctx_Capture_Select_Ctx_DPD_input_data;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Ctx DPD output data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_ctx_capture_select_config_Ctx_DPD_output_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Ctx_Capture_Select = DPD_REG_MAP_Ctx_Capture_Select_Ctx_DPD_output_data;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Ctx ADC data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_ctx_capture_select_config_Ctx_ADC_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Ctx_Capture_Select = DPD_REG_MAP_Ctx_Capture_Select_Ctx_ADC_data;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Ctx_Capture_Select based on input parameter
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @param Ctx_Capture_Select_configuration Input configuration of Ctx_Capture_Select
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_config_Ctx_Capture_Select(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_Rev_Capture_Config_Ctx_Capture_Select_Enum_Def Ctx_Capture_Select_configuration)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map)dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Ctx_Capture_Select = Ctx_Capture_Select_configuration;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Read Ctx_Capture_Select
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval DPD_REG_MAP_Ctx_Capture_Select_Ctx_Pattern_data Ctx Pattern data
 * @retval DPD_REG_MAP_Ctx_Capture_Select_Ctx_DPD_input_data Ctx DPD input data
 * @retval DPD_REG_MAP_Ctx_Capture_Select_Ctx_DPD_output_data Ctx DPD output data
 * @retval DPD_REG_MAP_Ctx_Capture_Select_Ctx_ADC_data Ctx ADC data
 * */


DPD_REG_MAP_Rev_Capture_Config_Ctx_Capture_Select_Enum_Def Hepta_A10_2x4_dpd_reg_map_rev_capture_config_read_Ctx_Capture_Select(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return resource_ptr->Rev_Capture_Config.reg.Ctx_Capture_Select;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Ctx_Capture_Select status is Ctx Pattern data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Ctx_Capture_Select status is Ctx Pattern data
 * @retval false Ctx_Capture_Select status is not Ctx Pattern data
 * */


bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCtx_Capture_Select_Ctx_Pattern_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->Rev_Capture_Config.reg.Ctx_Capture_Select == DPD_REG_MAP_Ctx_Capture_Select_Ctx_Pattern_data);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Ctx_Capture_Select status is Ctx DPD input data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Ctx_Capture_Select status is Ctx DPD input data
 * @retval false Ctx_Capture_Select status is not Ctx DPD input data
 * */


bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCtx_Capture_Select_Ctx_DPD_input_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->Rev_Capture_Config.reg.Ctx_Capture_Select == DPD_REG_MAP_Ctx_Capture_Select_Ctx_DPD_input_data);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Ctx_Capture_Select status is Ctx DPD output data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Ctx_Capture_Select status is Ctx DPD output data
 * @retval false Ctx_Capture_Select status is not Ctx DPD output data
 * */


bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCtx_Capture_Select_Ctx_DPD_output_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->Rev_Capture_Config.reg.Ctx_Capture_Select == DPD_REG_MAP_Ctx_Capture_Select_Ctx_DPD_output_data);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Ctx_Capture_Select status is Ctx ADC data
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Ctx_Capture_Select status is Ctx ADC data
 * @retval false Ctx_Capture_Select status is not Ctx ADC data
 * */


bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCtx_Capture_Select_Ctx_ADC_data(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->Rev_Capture_Config.reg.Ctx_Capture_Select == DPD_REG_MAP_Ctx_Capture_Select_Ctx_ADC_data);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Reverse path capture start
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_capture_start_config_Reverse_path_capture_start(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Capture_Start = DPD_REG_MAP_Capture_Start_Reverse_path_capture_start;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Reverse path capture stop
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_capture_start_config_Reverse_path_capture_stop(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Capture_Start = DPD_REG_MAP_Capture_Start_Reverse_path_capture_stop;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Capture Start based on input parameter
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @param Capture_Start_configuration Input configuration of Capture Start
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_config_Capture_Start(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_Rev_Capture_Config_Capture_Start_Enum_Def Capture_Start_configuration)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map)dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Capture_Start = Capture_Start_configuration;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Read Capture Start
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval DPD_REG_MAP_Capture_Start_Reverse_path_capture_start Reverse path capture start
 * @retval DPD_REG_MAP_Capture_Start_Reverse_path_capture_stop Reverse path capture stop
 * */


DPD_REG_MAP_Rev_Capture_Config_Capture_Start_Enum_Def Hepta_A10_2x4_dpd_reg_map_rev_capture_config_read_Capture_Start(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return resource_ptr->Rev_Capture_Config.reg.Capture_Start;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Capture Start status is Reverse path capture start
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Capture Start status is Reverse path capture start
 * @retval false Capture Start status is not Reverse path capture start
 * */


bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCapture_Start_Reverse_path_capture_start(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->Rev_Capture_Config.reg.Capture_Start == DPD_REG_MAP_Capture_Start_Reverse_path_capture_start);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Capture Start status is Reverse path capture stop
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Capture Start status is Reverse path capture stop
 * @retval false Capture Start status is not Reverse path capture stop
 * */


bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isCapture_Start_Reverse_path_capture_stop(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->Rev_Capture_Config.reg.Capture_Start == DPD_REG_MAP_Capture_Start_Reverse_path_capture_stop);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Reverse path sync capture disable
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_sync_capture_bypass_config_Reverse_path_sync_capture_disable(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Sync_Capture_bypass = DPD_REG_MAP_Sync_Capture_bypass_Reverse_path_sync_capture_disable;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Reverse path sync capture enable
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_sync_capture_bypass_config_Reverse_path_sync_capture_enable(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Sync_Capture_bypass = DPD_REG_MAP_Sync_Capture_bypass_Reverse_path_sync_capture_enable;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Configure Sync Capture bypass based on input parameter
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @param Sync_Capture_bypass_configuration Input configuration of Sync Capture bypass
 * */


void Hepta_A10_2x4_dpd_reg_map_rev_capture_config_config_Sync_Capture_bypass(pdevice_handler dpd_reg_map_device_handler,DPD_REG_MAP_Rev_Capture_Config_Sync_Capture_bypass_Enum_Def Sync_Capture_bypass_configuration)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map)dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->Rev_Capture_Config.reg.Sync_Capture_bypass = Sync_Capture_bypass_configuration;
    return;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Read Sync Capture bypass
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval DPD_REG_MAP_Sync_Capture_bypass_Reverse_path_sync_capture_disable Reverse path sync capture disable
 * @retval DPD_REG_MAP_Sync_Capture_bypass_Reverse_path_sync_capture_enable Reverse path sync capture enable
 * */


DPD_REG_MAP_Rev_Capture_Config_Sync_Capture_bypass_Enum_Def Hepta_A10_2x4_dpd_reg_map_rev_capture_config_read_Sync_Capture_bypass(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return resource_ptr->Rev_Capture_Config.reg.Sync_Capture_bypass;
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Sync Capture bypass status is Reverse path sync capture disable
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Sync Capture bypass status is Reverse path sync capture disable
 * @retval false Sync Capture bypass status is not Reverse path sync capture disable
 * */


bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isSync_Capture_bypass_Reverse_path_sync_capture_disable(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->Rev_Capture_Config.reg.Sync_Capture_bypass == DPD_REG_MAP_Sync_Capture_bypass_Reverse_path_sync_capture_disable);
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Check Sync Capture bypass status is Reverse path sync capture enable
 * @param dpd_reg_map_device_handler Device handler dpd_reg_map required instance 
 * @retval true Sync Capture bypass status is Reverse path sync capture enable
 * @retval false Sync Capture bypass status is not Reverse path sync capture enable
 * */


bool Hepta_A10_2x4_dpd_reg_map_rev_capture_config_isSync_Capture_bypass_Reverse_path_sync_capture_enable(pdevice_handler dpd_reg_map_device_handler)
{
    phepta_a10_2x4_dpd_reg_map_Register_Map resource_ptr = (phepta_a10_2x4_dpd_reg_map_Register_Map) dpd_reg_map_device_handler->reg_remap_addr[Hepta_A10_2x4_dpd_reg_map_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->Rev_Capture_Config.reg.Sync_Capture_bypass == DPD_REG_MAP_Sync_Capture_bypass_Reverse_path_sync_capture_enable);
}

/******************************************************************************
 * Custom Function definition
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Register read command handler
 * @param pCmdHeader Command packet
 * @param packet_id Command packet id
 * */


void Hepta_A10_2x4_dpd_reg_map_reg_read(pCommand_Header pCmdHeader,unsigned int packet_id)
{
    int read_count;
    pdevice_handler pdev = hepta_a10_2x4_dpd_reg_map_get_handler(pCmdHeader->Instance);
    //unsigned int Length = pCmdHeader->Length;
    unsigned int *pData = (unsigned int *)pCmdHeader->pData;
    unsigned int offset = pCmdHeader->Address;
    unsigned int resource_index = pCmdHeader->Resource;
    bus_address resource = pdev->reg_remap_addr[resource_index];
    if(pdev!=NULL)
    {
        for(read_count=0;read_count<pCmdHeader->Length;read_count++)
       {
        	pData[read_count] = (resource[offset+read_count] ) & (pCmdHeader->Bit_Mask) ;
       }
        return;
    }
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Register write command handler
 * @param pCmdHeader Command packet
 * @param packet_id Command packet id
 * */


void Hepta_A10_2x4_dpd_reg_map_reg_write(pCommand_Header pCmdHeader,unsigned int packet_id)
{
    int write_count;
    pdevice_handler pdev = hepta_a10_2x4_dpd_reg_map_get_handler(pCmdHeader->Instance);
    unsigned int Length = pCmdHeader->Length;
    unsigned int *pData = (unsigned int *)pCmdHeader->pData;
    unsigned int offset = pCmdHeader->Address;
    unsigned int read_data=0;
    unsigned int resource_index = pCmdHeader->Resource;
    bus_address resource = pdev->reg_remap_addr[resource_index];
    if(pdev!=NULL)
    {
        for(write_count=0;write_count<Length;write_count++)
       {
            read_data = 0;
            if (pCmdHeader->Bit_Mask != 0)
           {
                read_data = resource[offset+write_count];
                read_data = read_data & (~(pCmdHeader->Bit_Mask));
           }
            resource[offset+write_count] = (read_data) | pData[write_count];
       }
        return;
    }
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Config command handler
 * @param pCmdHeader Command packet
 * @param packet_id Command packet id
 * */


void Hepta_A10_2x4_dpd_reg_map_config(pCommand_Header pCmdHeader,unsigned int packet_id)
{
    pdevice_handler pdev = hepta_a10_2x4_dpd_reg_map_get_handler(pCmdHeader->Instance);
    //unsigned int Length = pCmdHeader->Length;
    //unsigned int *pData = (unsigned int *)pCmdHeader->pData;
    unsigned int config_id = pCmdHeader->Address;
    if(pdev!=NULL)
    {
        switch(config_id) //Check for config ID
       {
       }
    }
}

/**
 * @ingroup hepta_a10_2x4_dpd_reg_map
 * @brief Config command handler
 * @param pCmdHeader Command packet
 * @param packet_id Command packet id
 * */


void Hepta_A10_2x4_dpd_reg_map_service(pCommand_Header pCmdHeader,unsigned int packet_id)
{
    pdevice_handler pdev = hepta_a10_2x4_dpd_reg_map_get_handler(pCmdHeader->Instance);
    //unsigned int Enable = pCmdHeader->Length;
    //unsigned int *pData = (unsigned int *)pCmdHeader->pData;
    unsigned int service_id = pCmdHeader->Address;
    if(pdev!=NULL)
    {
        if(service_id >= Hepta_A10_2x4_dpd_reg_map_SERVICE_COUNT)
       {
            pCmdHeader->Length = 0;
            return;
       }
        else
       {
            if(pCmdHeader->Length == 1)
           {
                if(pdev->service_id_array[service_id] != ((unsigned int)-1))
               {
                    pdev->service_id_array[service_id] = packet_id;
               }
                else
               {
                    pCmdHeader->Length = 0;
                    return;
               }
           }
            else
           {
                if(pdev->service_id_array[service_id] != ((unsigned int)-1))
               {
                    send_service_ack_disable(&hepta_a10_2x4_socket_manager,pdev->service_id_array[service_id],pCmdHeader->Module_Index);
               }
           }
       }
        switch(service_id) //Check for config ID
       {
       }
    }
}

