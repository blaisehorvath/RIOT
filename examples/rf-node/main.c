/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Example application for demonstrating the RIOT network stack
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "shell.h"
#include "msg.h"

//TEMP INCLUDES, TO BE CLEARED UP LATER!!! \\TODO
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "thread.h"
#include "net/ipv6/addr.h"
#include "net/gnrc/ipv6/netif.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netapi.h"
#include "net/netopt.h"
#include "net/gnrc/pkt.h"
#include "net/gnrc/pktbuf.h"
#include "net/gnrc/netif/hdr.h"
#include "net/gnrc/sixlowpan/netif.h"
#include "net/gnrc/rpl.h"
#include "net/gnrc/rpl/structs.h"
#include "net/gnrc/rpl/dodag.h"
#define ENABLE_DEBUG (1)
#include "debug.h"

//From pktdump
#include <errno.h>
#include "byteorder.h"
#include "thread.h"
#include "msg.h"
#include "kernel.h"
#include "net/gnrc/pktdump.h"
#include "net/gnrc.h"
#include "net/ipv6/addr.h"
#include "net/ipv6/hdr.h"
#include "net/udp.h"
#include "net/sixlowpan.h"
#include "od.h"
//From udp.c

#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "timex.h"
#include "xtimer.h"
//END OF TEMP INCLUDES
#include "phydat.h"
#include "rfnode.h"
#include "saul_reg.h"
static gnrc_netreg_entry_t server = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, KERNEL_PID_UNDEF };
#define MAIN_QUEUE_SIZE     (8)
// 					OUR DEFINES
#define RFNODE_UDPTHREAD_MSG_QUEUE_SIZE 8
#define RFNODE_UDPTHREAD_STACKSIZE (THREAD_STACKSIZE_MAIN)
#define RFNODE_UDPTHREAD_PRIO 6
//					END OF OUR DEFINES
void snd_rfnode_pkt_to_root(rfnode_pkt* );
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];


#define OURDEV_TEMP 7
//*********************************OUR OWN EVENTLOOP****************************************//
static char rfnode_udpthread_stack[RFNODE_UDPTHREAD_STACKSIZE];
kernel_pid_t _rfnode_udpthread_pid = KERNEL_PID_UNDEF;
typedef enum {
	LISTEN /**< Initial state, wait for incoming packet*/
} rfnode_state;

//static rfnode_state rfstate = LISTEN;
int rfnode_statemachine(rfnode_pkt* pkt,rfnode_pkt* pkt_back)
{
	/**
	 * @brief   Our application level communication protocol
	 * Flow:
	 * The flow is directed be the msg in the rfnode_pkt type.
	 * The server first sends a GET_SENSACT_LIST command where cnt is 0
	 * The node then sends a SENSACT_LIST_ACK back, where the cnt contains the count of the
	 * available sensors/actuators
	 * After that the server sends a SENSACT_LIST_ITEM command with the number of the item in the cnt
	 * The response is a SENSACT_LIST_ITEM with the number of the sensor/actuator in the cnt
	 * and the name of the device in the name field of the rfnode_pkt, and the possible
	 * binary values of dimensions in the data field, and the type of the sensor from saul in the new_device field
	 *
	 * Setting/getting the actuator/sensor is possible with the SET/GET_SENSACT type, the answer from the
	 * node is a SENSACT_ACK type with the new values. The name and the value in the cnt must be the same.
	 */
	///Clear pkt_back
	int i = 0;
	saul_reg_t *dev;
	//saul_driver_t temp;
	pkt_back->cnt = 0;
	pkt_back->data.scale = 0;
	pkt_back->data.unit = 0;
	pkt_back->data.val[0] = 0;
	pkt_back->data.val[1] = 0;
	pkt_back->data.val[2] = 0;
	pkt_back->msg = 0;
	for(i = 0; i < 20; i++)
	pkt_back->name[i] = 0;
	pkt_back->new_device = 0;
	i = 0;
	switch(pkt->msg){
	case GET_SENSACT_LIST:
		//TODO: fill pkt_back with the the number of saul devices, SENSACT_LIST_ACK type
		dev = saul_reg_get();
		while(dev)
		{
			dev = dev->next;
			i++;//i is the number of devices
		}
		pkt_back->cnt = i;
		pkt_back->msg = SENSACT_LIST_ACK;
		break;

	case SENSACT_LIST_ACK:
		puts("Node catched SENSACT_LIST_ACK type, not available at nodes!");
		return 0;
		break;

	case SENSACT_LIST_ITEM:
		//TODO: fill pkt_back with the corresponting cnt, name and dimension in data, SENSACT_LIST_ITEM type
		dev = saul_reg_get();
		for(i = 0; i <pkt->cnt; i++) // i is the number of the device
		{
			dev = dev->next;
		}
		pkt_back->cnt = pkt->cnt;
		strcpy(pkt_back->name, dev->name);
		pkt_back->new_device = dev->driver->type;
		switch(dev->driver->type)// Possibly not needed, could be handled on the gateway side // fill up dimensions from type
		{
			case SAUL_ACT_SWITCH:
				pkt_back->data.val[0] = 1;
				break;
			default:
				break;
		}
		pkt_back->msg = SENSACT_LIST_ITEM;
		break;

	case GET_SENSACT:
		//TODO: get the values of the sensor/actuator, SENSACT_ACK type
		dev = saul_reg_get();
		for(i = 0; i <pkt->cnt; i++) // Is is the number of the device
		{
			dev = dev->next;
		}
		if (strcmp(pkt->name,dev->name))
		{
			DEBUG("Bad name for device number!!!");// Possibly better to send back error msg in the name field
			return 0;
		}
		pkt_back->cnt = pkt->cnt;
		strcpy(pkt_back->name, dev->name);
		saul_reg_read(dev,&pkt_back->data);
		pkt_back->msg = SENSACT_ACK;
		break;

	case SET_SENSACT:
		//TODO: get the values of the sensor/actuator, SENSACT_ACK type
		dev = saul_reg_get();
		for(i = 0; i <pkt->cnt; i++) // Is is the number of the device
		{
			dev = dev->next;
		}
		if (strcmp(pkt->name,dev->name))
		{
			DEBUG("Bad name for device number!!!");// Possibly better to send back error msg in the name field
			return 0;
		}
		pkt_back->cnt = pkt->cnt;
		strcpy(pkt_back->name, dev->name);
		phydat_dump(&pkt->data, 3);
		saul_reg_write(dev,&pkt->data);
		saul_reg_read(dev,&pkt_back->data);
		pkt_back->msg = SENSACT_ACK;
		break;
	case SENSACT_ACK:
		puts("Node catched SENSACT_ACK type, not available at nodes!");
		return 0;
		break;
	}
	return 1; // If answer else 0
}
static void rfnode_udp_get(gnrc_pktsnip_t *pkt)
{
	int answer = 0;
    //int snips = 0;
    //int size = 0;
	rfnode_pkt pkt_back;
    gnrc_pktsnip_t *snip = pkt;
    while (snip != NULL) {
        if (snip->type == GNRC_NETTYPE_UNDEF ) {
        		answer = rfnode_statemachine((rfnode_pkt*)snip->data, &pkt_back);
                printf("data arrived!"); // Handle the state machine here.
        }
        //++snips;
        //size += snip->size;
        snip = snip->next;
    }
    gnrc_pktbuf_release(pkt);
    if (answer)
    {
    	snd_rfnode_pkt_to_root(&pkt_back);
    }
}
static void* rfnode_udp_eventloop(void* arg)
{
    (void)arg;
    msg_t msg, reply;
    msg_t msg_queue[RFNODE_UDPTHREAD_MSG_QUEUE_SIZE];
    msg_init_queue(msg_queue, RFNODE_UDPTHREAD_MSG_QUEUE_SIZE);
    reply.content.value = (uint32_t)(-ENOTSUP);
    reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
    while (1) {
        msg_receive(&msg);
        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                puts("RFNODE_UDP_EVENTLOOP: data received:");
                rfnode_udp_get((gnrc_pktsnip_t *)msg.content.ptr);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                puts("RFNODE_UDP_EVENTLOOP: data to send:");
                //_dump((gnrc_pktsnip_t *)msg.content.ptr);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
            case GNRC_NETAPI_MSG_TYPE_SET:
                msg_reply(&msg, &reply);
                break;
            default:
                puts("RFNODE_UDP_EVENTLOOP: received something unexpected");
                break;
        }
    }
    return NULL; // Never reached
}
kernel_pid_t rfnode_udpthread_init(void)
{
	if(_rfnode_udpthread_pid ==  KERNEL_PID_UNDEF){
		_rfnode_udpthread_pid = thread_create(rfnode_udpthread_stack, sizeof(rfnode_udpthread_stack),RFNODE_UDPTHREAD_PRIO,
				THREAD_CREATE_STACKTEST, rfnode_udp_eventloop, NULL, "rfnode_udp_rec");
	}
	return _rfnode_udpthread_pid;
}
kernel_pid_t rfnode_udpthread_getpid(void)
{
	return _rfnode_udpthread_pid;
}
void rfnode_udpserver_start(uint32_t port)
{
    if (server.pid != KERNEL_PID_UNDEF) {
        printf("Error: server already running on port %d\n",
              (int) server.demux_ctx);
        return;
    }
    if (port == 0) {
        puts("Error: invalid port specified");
        return;
    }
    server.pid = rfnode_udpthread_init();
    server.demux_ctx = (uint32_t)port;
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);
    printf("Success: started UDP server on port %d\n",(int) port);
}
void rfnode_udpserver_stop(void)
{
    /* check if server is running at all */
    if (server.pid == KERNEL_PID_UNDEF) {
        printf("Error: server was not running\n");
        return;
    }
    /* stop server */
    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &server);
    server.pid = KERNEL_PID_UNDEF;
    puts("Success: stopped UDP server");
}
void rfnode_udpsend(ipv6_addr_t addr, uint16_t portin, char *data, unsigned int num,
                 unsigned int delay)
{
	uint8_t port[2];
    port[0] = (uint8_t)portin;
    port[1] = portin >> 8;
	//char tempstr[IPV6_ADDR_MAX_STR_LEN];
    for (unsigned int i = 0; i < num; i++) {
        gnrc_pktsnip_t *payload, *udp, *ip;
        /* allocate payload */
        payload = gnrc_pktbuf_add(NULL, data, /*strlen(data)*/sizeof(rfnode_pkt), GNRC_NETTYPE_UNDEF);
        if (payload == NULL) {
            puts("Error: unable to copy data to packet buffer");
            return;
        }
        /* allocate UDP header, set source port := destination port */
        udp = gnrc_udp_hdr_build(payload, port, 2, port, 2);
        if (udp == NULL) {
            puts("Error: unable to allocate UDP header");
            gnrc_pktbuf_release(payload);
            return;
        }
        /* allocate IPv6 header */
        ip = gnrc_ipv6_hdr_build(udp, NULL, 0, (uint8_t *)&addr, sizeof(addr));
        if (ip == NULL) {
            puts("Error: unable to allocate IPv6 header");
            gnrc_pktbuf_release(udp);
            return;
        }
        /* send packet */
        if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
            puts("Error: unable to locate UDP thread");
            gnrc_pktbuf_release(ip);
            return;
        }
       // ipv6_addr_to_str(tempstr,addr,IPV6_ADDR_MAX_STR_LEN);
        printf(/*"Success: send %u byte to [%s]:%u\n"*/"Success: send %u byte \n", (unsigned)payload->size/*,
        		addr, port*/);

        xtimer_usleep(delay);
    }
}
void snd_rfnode_pkt_to_root(rfnode_pkt* pkt)
{
	puts("sensing pkt do dodag root");
    ipv6_addr_t addr;
    if (ipv6_addr_from_str(&addr, "fe80::01") == NULL) {
        puts("Error: unable to parse destination address");
        return;
    }
	//rfnode_udpsend(1000000,)
    rfnode_udpsend(addr, (uint16_t) 12345,(char*) pkt, 1,
                     (unsigned int) 1000000);
    return;
}
int sndpkt_dodagroot(int argc, char **argv)
{
	puts("sensing pkt do dodag root");
	rfnode_pkt pkttemp;
	rfnode_pkt* pkt = &pkttemp;
	strcpy(pkt->name, "faszfasz");
    ipv6_addr_t addr;
    if (ipv6_addr_from_str(&addr, "fe80::01") == NULL) {
        puts("Error: unable to parse destination address");
        return -1;
    }
	//rfnode_udpsend(1000000,)
    rfnode_udpsend(addr, (uint16_t) 12345,(char*) pkt, 1,
                     (unsigned int) 1000000);
    return 0;
}
//*********************************END OF OUR OWN EVENTLOOP****************************************//
extern int udp_cmd(int argc, char **argv);
static const shell_command_t shell_commands[] = {
	{"pkt_to_root","send packet to RPL dodag root",sndpkt_dodagroot},
	{ "udp", "send data over UDP and listen on UDP ports", udp_cmd },
	{ NULL, NULL, NULL },

};
int main(void)
{
	//OUR STUFF
	//#ifdef MODULE_GNRC_IPV6_NETIF
	DEBUG("**************IN OUR STUFF**************\n");
	gnrc_pktdump_init();
		gnrc_ipv6_netif_t *entry = gnrc_ipv6_netif_get((kernel_pid_t)OURDEV_TEMP);
		char ipv6_addr[IPV6_ADDR_MAX_STR_LEN];
		for (int i = 0; i < GNRC_IPV6_NETIF_ADDR_NUMOF; i++) {
		        if (!ipv6_addr_is_unspecified(&entry->addrs[i].addr)) {
		        	if((entry->addrs[i].addr.u64[0].u64) == 0x80fe) // If our address in in the 0x80fe field, then we set the hwaddr to the last bit
		        	{
		        	    size_t addr_len = 2;
		        	    if (gnrc_netapi_set(OURDEV_TEMP, NETOPT_ADDRESS, 0, //Set the adress
		        	    		&entry->addrs[i].addr.u8[15], addr_len) < 0) {
		        	            printf("error: unable to set ");
		        	            puts("");
		        	        }
		        		//gnrc_netapi_set(OURDEV_TEMP, NETOPT_ADDRESS, 0, &entry->addrs[i].addr.u8[15], addr_len);
		        		gnrc_rpl_init(OURDEV_TEMP);// TODO ERROR HANDLING
		        		rfnode_udpserver_start(12345);
		        	}
		            if (ipv6_addr_to_str(ipv6_addr, &entry->addrs[i].addr,
		                                 IPV6_ADDR_MAX_STR_LEN)) {
		                printf("dev:%d: %s\n",i, ipv6_addr);
		            }
		        }
		    }
	//#endif
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    /**/
	msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT network stack example application");
    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}
