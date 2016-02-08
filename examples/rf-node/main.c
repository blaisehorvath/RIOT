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
//END OF TEMP INCLODER
static gnrc_netreg_entry_t server = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, KERNEL_PID_UNDEF };
#define MAIN_QUEUE_SIZE     (8)
// 					OUR DEFINES
#define RFNODE_UDPTHREAD_MSG_QUEUE_SIZE 8
#define RFNODE_UDPTHREAD_STACKSIZE (THREAD_STACKSIZE_MAIN)
#define RFNODE_UDPTHREAD_PRIO 6
//					END OF OUR DEFINES

static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];


#define OURDEV_TEMP 7
//*********************************OUR OWN EVENTLOOP****************************************//
static char rfnode_udpthread_stack[RFNODE_UDPTHREAD_STACKSIZE];
kernel_pid_t _rfnode_udpthread_pid = KERNEL_PID_UNDEF;
static void rfnode_udp_get(gnrc_pktsnip_t *pkt)
{
    //int snips = 0;
    //int size = 0;
    gnrc_pktsnip_t *snip = pkt;
    while (snip != NULL) {
        if (snip->type == GNRC_NETTYPE_UNDEF ) {
                printf("snip->data: %s\n",(char*)snip->data); // Handle the state machine here.
        }
        //++snips;
        //size += snip->size;
        snip = snip->next;
    }
    gnrc_pktbuf_release(pkt);
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
//*********************************END OF OUR OWN EVENTLOOP****************************************//
//***************************STOLEN UDP*****************************************************//


static void send(char *addr_str, char *port_str, char *data, unsigned int num,
                 unsigned int delay)
{
    uint8_t port[2];
    uint16_t tmp;
    ipv6_addr_t addr;

    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        return;
    }
    /* parse port */
    tmp = (uint16_t)atoi(port_str);
    if (tmp == 0) {
        puts("Error: unable to parse destination port");
        return;
    }
    port[0] = (uint8_t)tmp;
    port[1] = tmp >> 8;

    for (unsigned int i = 0; i < num; i++) {
        gnrc_pktsnip_t *payload, *udp, *ip;
        /* allocate payload */
        payload = gnrc_pktbuf_add(NULL, data, strlen(data), GNRC_NETTYPE_UNDEF);
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
        printf("Success: send %u byte to [%s]:%u\n", (unsigned)payload->size,
               addr_str, tmp);
        xtimer_usleep(delay);
    }
}

static void start_server(char *port_str)
{
    uint16_t port;

    /* check if server is already running */
    if (server.pid != KERNEL_PID_UNDEF) {
        printf("Error: server already running on port %" PRIu32 "\n",
               server.demux_ctx);
        return;
    }
    /* parse port */
    port = (uint16_t)atoi(port_str);
    if (port == 0) {
        puts("Error: invalid port specified");
        return;
    }
    /* start server (which means registering pktdump for the chosen port) */
    server.pid = rfnode_udpthread_init();// gnrc_pktdump_getpid();
    server.demux_ctx = (uint32_t)port;
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);
    printf("Success: started UDP server on port %" PRIu16 "\n", port);
}

static void stop_server(void)
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

int udp_cmd(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s [send|server]\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "send") == 0) {
        uint32_t num = 1;
        uint32_t delay = 1000000;
        if (argc < 5) {
            printf("usage: %s send <addr> <port> <data> [<num> [<delay in us>]]\n",
                   argv[0]);
            return 1;
        }
        if (argc > 5) {
            num = (uint32_t)atoi(argv[5]);
        }
        if (argc > 6) {
            delay = (uint32_t)atoi(argv[6]);
        }
        send(argv[2], argv[3], argv[4], num, delay);
    }
    else if (strcmp(argv[1], "server") == 0) {
        if (argc < 3) {
            printf("usage: %s server [start|stop]\n", argv[0]);
            return 1;
        }
        if (strcmp(argv[2], "start") == 0) {
            if (argc < 4) {
                printf("usage %s server start <port>\n", argv[0]);
                return 1;
            }
            start_server(argv[3]);
        }
        else if (strcmp(argv[2], "stop") == 0) {
            stop_server();
        }
        else {
            puts("error: invalid command");
        }
    }
    else {
        puts("error: invalid command");
    }
    return 0;
}
//********************************END OF STOLEN UDP****************************************//
static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
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
		        		gnrc_netapi_set(OURDEV_TEMP, NETOPT_ADDRESS, 0, &entry->addrs[i].addr.u8[15], addr_len);
		        		gnrc_rpl_init(OURDEV_TEMP);
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
