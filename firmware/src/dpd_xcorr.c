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
 * @file hepta_a10_2x4_hepta_dpd_xcorr.c
 * @ingroup hepta_a10_2x4_hepta_dpd_xcorr
 * @brief Application file for hepta_dpd_xcorr module
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

#include "dpd_xcorr.h"
#include "dpd_protocol.h"
#include "dpd_firmware.h"
#include "dpd_command_format.h"
extern socket_manager_header hepta_a10_2x4_socket_manager;
/******************************************************************************
 * Device handler Function definition
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_hepta_dpd_xcorr
 * @brief Driver handler function. Interacts with hepta_dpd_xcorr driver and gets address map information
 * @param instance_num Instance number of the hepta_dpd_xcorr module for which handler is to be returned
 * @retval Handler Pointer Requested Instance of hepta_dpd_xcorr module exits
 * @retval NULL Requested Instance of hepta_dpd_xcorr module doesnot exit
 * */


pdevice_handler hepta_a10_2x4_hepta_dpd_xcorr_get_handler(unsigned int instance_num)
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
    	snprintf(device_node,sizeof(device_node),"/proc/%s/num_of_instance","hepta_dpd_xcorr");
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
    		snprintf(device_node,sizeof(device_node),"%s%s%d","/dev/","hepta_dpd_xcorr",i);
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
    
    		if(Hepta_A10_2x4_hepta_dpd_xcorr_SERVICE_COUNT > 0)
    		{
    			device_handler_ptr[i].service_id_array =  malloc(sizeof(unsigned int)* Hepta_A10_2x4_hepta_dpd_xcorr_SERVICE_COUNT );
    			for(service_index=0;service_index< Hepta_A10_2x4_hepta_dpd_xcorr_SERVICE_COUNT ; service_index++)
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

/******************************************************************************
 * ISR Function definition
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_hepta_dpd_xcorr
 * @brief Interrupt Service Routine. Interacts with hepta_dpd_xcorr UIO driver and gets interrupt status
 * @param hepta_dpd_xcorr_handler_ptr Device handler of the hepta_dpd_xcorr module
 * */


void * Hepta_A10_2x4_hepta_dpd_xcorr_isr_thread(void * hepta_dpd_xcorr_handler_ptr)
{
    pdevice_handler phepta_dpd_xcorr_handler = (pdevice_handler)hepta_dpd_xcorr_handler_ptr;
    
    int uio_fd;
    char number[4];
    char device_node[256];
    //unsigned int no_of_interrupts_triggered = 0;
    unsigned char ret;
    unsigned int i = 0;
    
    //memset(device_node,0,256);	//erase device_node
    for(i=0;i<256;i++)
        	{
        		device_node[i]=0;
        	}
    i = 0;
    snprintf(device_node,sizeof(device_node),"/dev/uio%d",phepta_dpd_xcorr_handler->uio_irq_device_number);
    
    uio_fd = open(device_node,O_RDONLY);
    
    if(uio_fd <= 0)
    {
    	return NULL;
    }
    
    while(1)
    {
    	ret = read(uio_fd,number,4);
    	DEBUG_PRINT(DEBUG_LEVEL_INFO,("hepta_dpd_xcorr %s Interrupt received\n",phepta_dpd_xcorr_handler->device_instance_number));
    	number[ret-1]='\0';
    	//no_of_interrupts_triggered=atoi(number);
    	
    	if(phepta_dpd_xcorr_handler->ISR_callback_function_ptr != NULL)
    	{
    		phepta_dpd_xcorr_handler->ISR_callback_function_ptr(phepta_dpd_xcorr_handler,phepta_dpd_xcorr_handler->ISR_callback_argument);
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
 * @ingroup hepta_a10_2x4_hepta_dpd_xcorr
 * @brief Interrupt Service Register API. Register callback function for hepta_dpd_xcorr interrupt
 * @param hepta_dpd_xcorr_handler_ptr Device handler of the hepta_dpd_xcorr module
 * @param callback_function Callback function for hepta_dpd_xcorr IRQ
 * @param callback_argument Argument to be passed to hepta_dpd_xcorr ISR callback function
 * */


void * Hepta_A10_2x4_hepta_dpd_xcorr_register_isr( void * hepta_dpd_xcorr_handler_ptr,isr_callback callback_function,void * callback_argument)
{
    pdevice_handler phepta_dpd_xcorr_handler = (pdevice_handler)hepta_dpd_xcorr_handler_ptr;
    if(phepta_dpd_xcorr_handler->uio_irq_device_number >=0 )
    {
    	phepta_dpd_xcorr_handler->ISR_callback_function_ptr = callback_function;
    	phepta_dpd_xcorr_handler->ISR_callback_argument = callback_argument;
    	pthread_create(&(phepta_dpd_xcorr_handler->ISR_thread_handler),NULL, Hepta_A10_2x4_hepta_dpd_xcorr_isr_thread,phepta_dpd_xcorr_handler);
    }
return NULL;
}

/******************************************************************************
 * User defined Function definition
******************************************************************************/

/******************************************************************************
 * Custom Function definition
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_hepta_dpd_xcorr
 * @brief Register read command handler
 * @param pCmdHeader Command packet
 * @param packet_id Command packet id
 * */


void Hepta_A10_2x4_hepta_dpd_xcorr_reg_read(pCommand_Header pCmdHeader,unsigned int packet_id)
{
    int read_count;
    pdevice_handler pdev = hepta_a10_2x4_hepta_dpd_xcorr_get_handler(pCmdHeader->Instance);
    unsigned int Length = pCmdHeader->Length;
    unsigned int *pData = (unsigned int *)pCmdHeader->pData;
    unsigned int offset = pCmdHeader->Address;
    unsigned int resource_index = pCmdHeader->Resource;
    bus_address resource = pdev->reg_remap_addr[resource_index];
    if(pdev!=NULL)
    {
        for(read_count=0;read_count<Length;read_count++)
       {
            pData[read_count] = (resource[offset+read_count] ) & (pCmdHeader->Bit_Mask) ;
       }
        return;
    }
}

/**
 * @ingroup hepta_a10_2x4_hepta_dpd_xcorr
 * @brief Register write command handler
 * @param pCmdHeader Command packet
 * @param packet_id Command packet id
 * */


void Hepta_A10_2x4_hepta_dpd_xcorr_reg_write(pCommand_Header pCmdHeader,unsigned int packet_id)
{
    int write_count;
    pdevice_handler pdev = hepta_a10_2x4_hepta_dpd_xcorr_get_handler(pCmdHeader->Instance);
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
 * @ingroup hepta_a10_2x4_hepta_dpd_xcorr
 * @brief Config command handler
 * @param pCmdHeader Command packet
 * @param packet_id Command packet id
 * */


void Hepta_A10_2x4_hepta_dpd_xcorr_config(pCommand_Header pCmdHeader,unsigned int packet_id)
{
    pdevice_handler pdev = hepta_a10_2x4_hepta_dpd_xcorr_get_handler(pCmdHeader->Instance);
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
 * @ingroup hepta_a10_2x4_hepta_dpd_xcorr
 * @brief Config command handler
 * @param pCmdHeader Command packet
 * @param packet_id Command packet id
 * */


void Hepta_A10_2x4_hepta_dpd_xcorr_service(pCommand_Header pCmdHeader,unsigned int packet_id)
{
    pdevice_handler pdev = hepta_a10_2x4_hepta_dpd_xcorr_get_handler(pCmdHeader->Instance);
    //unsigned int Enable = pCmdHeader->Length;
    //unsigned int *pData = (unsigned int *)pCmdHeader->pData;
    unsigned int service_id = pCmdHeader->Address;
    if(pdev!=NULL)
    {
        if(service_id >= Hepta_A10_2x4_hepta_dpd_xcorr_SERVICE_COUNT)
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

