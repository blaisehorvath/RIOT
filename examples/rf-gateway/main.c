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
#include "../rf-stuff/rfnode.h"
//#include "rfnode.h"
//FROM ipv6_hdr_print.c
#include "net/ipv6/hdr.h"
static gnrc_netreg_entry_t server = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, KERNEL_PID_UNDEF };
#define MAIN_QUEUE_SIZE     (8)
// 					OUR DEFINES
#define RFNODE_UDPTHREAD_MSG_QUEUE_SIZE 8
#define RFNODE_UDPTHREAD_STACKSIZE (THREAD_STACKSIZE_MAIN)
#define RFNODE_UDPTHREAD_PRIO 6
//#define GNRC_NETTYPE_RFPKT GNRC_NETTYPE_NUMOF+2
//					END OF OUR DEFINES

static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];


#define OURDEV_TEMP 7
//*********************************OUR OWN EVENTLOOP****************************************//
static char rfnode_udpthread_stack[RFNODE_UDPTHREAD_STACKSIZE];
kernel_pid_t _rfnode_udpthread_pid = KERNEL_PID_UNDEF;
/*static*/ void rfnode_udp_get(gnrc_pktsnip_t *pkt)
{
    //int snips = 0;
    //int size = 0;
    gnrc_pktsnip_t *snip = pkt;
    char addr_str[IPV6_ADDR_MAX_STR_LEN];
    while (snip != NULL) {
        switch (snip->type){
		case GNRC_NETTYPE_UNDEF:// Handle the state machine here.
			printf("msg         : %d\n",((rfnode_pkt*)snip->data)->msg);
			printf("cnt         : %u\n",((rfnode_pkt*)snip->data)->cnt);
			printf("data.val[0] : %d\n",((rfnode_pkt*)snip->data)->data.val[0]);
			printf("data.val[1] : %d\n",((rfnode_pkt*)snip->data)->data.val[1]);
			printf("data.val[2] : %d\n",((rfnode_pkt*)snip->data)->data.val[2]);
			printf("data.unit   : %u\n",((rfnode_pkt*)snip->data)->data.unit);
			printf("data.scale : %d\n",((rfnode_pkt*)snip->data)->data.scale);
			printf("name        : %s\n",((rfnode_pkt*)snip->data)->name);
			printf("new_device  : %u\n",((rfnode_pkt*)snip->data)->new_device);
			printf("pkt_cnt  : %u\n",(unsigned int)((rfnode_pkt*)snip->data)->pkt_cnt);
			break;
		case GNRC_NETTYPE_IPV6://FROM ipv6_hdr_print.c
		    printf("Got pkt!source address: %s\n", ipv6_addr_to_str(addr_str, &((ipv6_hdr_t *)snip->data)->src,
		            sizeof(addr_str)));
			break;
		default:
			break;
        }
        //++snips;
        //size += snip->size;
        snip = snip->next;
    }
    gnrc_pktbuf_release(pkt);
}
/*static*/ void* rfnode_udp_eventloop(void* arg)
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
                //puts("RFNODE_UDP_EVENTLOOP: data received:");
                rfnode_udp_get((gnrc_pktsnip_t *)msg.content.ptr);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                //puts("RFNODE_UDP_EVENTLOOP: data to send:");
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
        printf(/*"Success: send %u byte to [%s]:%u\n"*/"Success: send %u byte \n", (unsigned)payload->size/*,
        		addr, port*/);

        xtimer_usleep(delay);
    }
}
int sndpkt(int argc, char **argv)
{
	if (argc != 12) printf(
			"Not enough arguments!Usage:\n1: addr\n2: msg\n3: cnt\n4: data->val[0]\n5: data -> val[1]\n6: data ->val[2]\n7: data->unit\n8: data->scale\n9: name\n10: new_device\n");
	//printf("sending pkt to %s!\n", argv[1]);
	rfnode_pkt pkttemp;
	rfnode_pkt* pkt = &pkttemp;
	/**<Fill out pkt values*/
	pkt->msg = (pkt_msg)atoi(argv[2]);
	pkt->cnt = (uint16_t)atoi(argv[3]);
	pkt->data.val[0] = (int16_t)atoi(argv[4]);
	pkt->data.val[1] = (int16_t)atoi(argv[5]);
	pkt->data.val[2] = (int16_t)atoi(argv[6]);
	pkt->data.unit = (uint8_t)atoi(argv[7]);
	pkt->data.scale = (int8_t)atoi(argv[8]);
	strcpy(pkt->name, argv[9]);
	pkt->new_device = (uint8_t)atoi(argv[10]);
	pkt->pkt_cnt = (uint32_t)atoi(argv[11]);
    ipv6_addr_t addr;
    if (ipv6_addr_from_str(&addr, argv[1]) == NULL) {
        puts("Error: unable to parse destination address\n");
        return -1;
    }
    rfnode_udpsend(addr, (uint16_t) 12345,(char*) pkt, 1,
                     (unsigned int) 1000000);
    return 0;
}
//*********************************END OF OUR OWN EVENTLOOP****************************************//
extern int udp_cmd(int argc, char **argv);
static const shell_command_t shell_commands[] = {
	{"sndpkt","send packets to one of the nodes",sndpkt},
	{ "udp", "send data over UDP and listen on UDP ports", udp_cmd },
	{ NULL, NULL, NULL },

};
int main(void)
{
	rfnode_udpserver_start(12345); //TODO: hardcoded port
	msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT network stack example application");
    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}
