/**
 * @ingroup     rfnode
 * @{
 *
 * @file
 * @brief       rfnode functions implementation
 *
 * @author      Viktor Vaczi
 *
 * @}
 */
#include "../rf-stuff/rfnode.h"
#include <stdio.h>

#include "shell.h"
#include "msg.h"
#include "thread.h"
#include "kernel.h"

#include "net/gnrc/rpl/dodag.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"

#include "timex.h"
#include "phydat.h"
#include "saul_reg.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define RFNODE_UDPTHREAD_MSG_QUEUE_SIZE 8
#define RFNODE_UDPTHREAD_STACKSIZE (THREAD_STACKSIZE_MAIN)
#define RFNODE_UDPTHREAD_PRIO 6
#define OURDEV_TEMP 7
static gnrc_netreg_entry_t server = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, KERNEL_PID_UNDEF };
static gnrc_netreg_entry_t gatewayserver = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, KERNEL_PID_UNDEF };

static char rfgateway_udpthread_stack[RFNODE_UDPTHREAD_STACKSIZE];
kernel_pid_t _rfgateway_udpthread_pid = KERNEL_PID_UNDEF;
static char rfnode_udpthread_stack[RFNODE_UDPTHREAD_STACKSIZE];
kernel_pid_t _rfnode_udpthread_pid = KERNEL_PID_UNDEF;
/**
	@par The following function sets up our node doing the following things:
	- Setting the hw adress of the node to the last byte of its link local ipv6 adress ( needed for cc110x )
	- Starts the rpl to connect to the mesh network
	- Initializes an udp server on port 12345
*/
void setup_gnrc_node(void)
{
		gnrc_ipv6_netif_t *entry = gnrc_ipv6_netif_get((kernel_pid_t)OURDEV_TEMP);
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
		        		gnrc_rpl_init(OURDEV_TEMP);// TODO ERROR HANDLING
		        		rfnode_udpserver_start(12345);
		        	}
		        }
		    }
}
/**
	This function function sends an application level packet to the root of our DODAG
	which is currently hard coded to fe80::01 IP adress
	@param[in]	pkt a pointer to the pkt that the function will send to the root
*/
void snd_rfnode_pkt_to_root(rfnode_pkt* pkt)
{
	puts("sensing pkt do dodag root");
    ipv6_addr_t addr;
    if (ipv6_addr_from_str(&addr, "fe80::01") == NULL) {
        puts("Error: unable to parse destination address");
        return;
    }
    rfnode_udpsend(addr, (uint16_t) 12345,(char*) pkt, 1,
                     (unsigned int) 1000000);
    return;
}
/**
This function sends an rfnode_pkt type packtet to the given adress
@param[in] addr This is the destination adress
@param[in] portin The UDP port to send the packet
@param[in] data The pointer of the rfnode_pkt
@param[in] num How many times the pkt should be sent to the destination
@param[in] delay How much time to wait after the packet has been sent
*/
void rfnode_udpsend(ipv6_addr_t addr, uint16_t portin, char *data, unsigned int num,
                 unsigned int delay)
{
	uint8_t port[2];
    port[0] = (uint8_t)portin;
    port[1] = portin >> 8;
    for (unsigned int i = 0; i < num; i++) {
        gnrc_pktsnip_t *payload, *udp, *ip;
        /* allocate payload */
        payload = gnrc_pktbuf_add(NULL, data, sizeof(rfnode_pkt), GNRC_NETTYPE_UNDEF);
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
	/**
	 * @brief This function handles the packet replying on the application level
	 * If needed, then the function also request SAUL devices and writes/reads them
	 * @param[in] pkt the incoming packet
	 * @param[out] pkt_back the reply packet to the pkt
	 * @param[out] return The function returns 1 if we should send pkt_back to the root and 0 otherwise
	 * @par   Our application level communication protocol flow:
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
int rfnode_statemachine(rfnode_pkt* pkt,rfnode_pkt* pkt_back)
{


	int i = 0;
	saul_reg_t *dev;
	pkt_back->cnt = 0; 	///Clear pkt_back
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
	return 1; // If we should answer then the flow arrives here, it escapes with 0 on error
}
/**
	This function handles the packet snippets, and also calls the rfnode_statemachine, which fills the reply packet
	and decides whether it should send it
	@param[in] pkt the pointer to the incoming packet
*/
void rfnode_udp_get(gnrc_pktsnip_t *pkt)
{
	int answer = 0;
	rfnode_pkt pkt_back;
    gnrc_pktsnip_t *snip = pkt;
    while (snip != NULL) {
        if (snip->type == GNRC_NETTYPE_UNDEF ) {
        		answer = rfnode_statemachine((rfnode_pkt*)snip->data, &pkt_back);/// Handle the state machine here.
                printf("data arrived!"); 
        }
        snip = snip->next;
    }
    gnrc_pktbuf_release(pkt);
    if (answer)
    {
    	snd_rfnode_pkt_to_root(&pkt_back);
    }
}
/**
This function is the eventloop of our UDP server, the received messages are forwarded to rfnode_udp_get
*/
void* rfnode_udp_eventloop(void* arg)
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
/**
This function creates a stack and initializes it with our UDP server eventloop rfnode_udp_eventloop
@param[out] return The function returns the pid of the thread. If the thread is already initialized then we could use this 
		function instead of rfnode_udpthread_getpid
*/
kernel_pid_t rfnode_udpthread_init(void)
{
	if(_rfnode_udpthread_pid ==  KERNEL_PID_UNDEF){
		_rfnode_udpthread_pid = thread_create(rfnode_udpthread_stack, sizeof(rfnode_udpthread_stack),RFNODE_UDPTHREAD_PRIO,
				THREAD_CREATE_STACKTEST, rfnode_udp_eventloop, NULL, "rfnode_udp_rec");
	}
	return _rfnode_udpthread_pid;
}
/**
This function returns the pid of our UDP server
*/
kernel_pid_t rfnode_udpthread_getpid(void)
{
	return _rfnode_udpthread_pid;
}
/**
This function starts a UDP server on the input port. It starts an event thread by calling 
rfnode_udpthread_init, and it registers the event loop to the GNRC with GNRC_NETTYPE_UDP
@param[in] port the port where the server will start
*/
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
/**
This function stops the udp server
*/
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
/**
This function sends a packtet to the root, only for debugging purposes
*/
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
    rfnode_udpsend(addr, (uint16_t) 12345,(char*) pkt, 1,
                     (unsigned int) 1000000);
    return 0;
}
///////////////////************************** GATEWAY CODE ************************/////////////////
void rfgateway_udp_get(gnrc_pktsnip_t *pkt)
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
			printf("data->scale : %d\n",((rfnode_pkt*)snip->data)->data.scale);
			printf("name        : %s\n",((rfnode_pkt*)snip->data)->name);
			printf("new_device  : %u\n",((rfnode_pkt*)snip->data)->new_device);
			break;
		case GNRC_NETTYPE_IPV6://FROM ipv6_hdr_print.c
		    printf("source address: %s\n", ipv6_addr_to_str(addr_str, &((ipv6_hdr_t *)snip->data)->src,
		            sizeof(addr_str)));
			break;
		default:
			break;
        }
        snip = snip->next;
    }
    gnrc_pktbuf_release(pkt);
}
void* rfgateway_udp_eventloop(void* arg)
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
                rfgateway_udp_get((gnrc_pktsnip_t *)msg.content.ptr);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                puts("RFNODE_UDP_EVENTLOOP: data to send:");
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
kernel_pid_t rfgateway_udpthread_init(void)
{
	if(_rfgateway_udpthread_pid ==  KERNEL_PID_UNDEF){
		_rfgateway_udpthread_pid = thread_create(rfgateway_udpthread_stack, sizeof(rfgateway_udpthread_stack),RFNODE_UDPTHREAD_PRIO,
				THREAD_CREATE_STACKTEST, rfgateway_udp_eventloop, NULL, "rfnode_udp_rec");
	}
	return _rfgateway_udpthread_pid;
}
kernel_pid_t rfgateway_udpthread_getpid(void)
{
	return _rfgateway_udpthread_pid;
}
void rfgateway_udpserver_start(uint32_t port)
{
    if (gatewayserver.pid != KERNEL_PID_UNDEF) {
        printf("Error: server already running on port %d\n",
              (int) gatewayserver.demux_ctx);
        return;
    }
    if (port == 0) {
        puts("Error: invalid port specified");
        return;
    }
    gatewayserver.pid = rfgateway_udpthread_init();
    gatewayserver.demux_ctx = (uint32_t)port;
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &gatewayserver);
    printf("Success: started UDP server on port %d\n",(int) port);
}
void rfgateway_udpserver_stop(void)
{
    /* check if server is running at all */
    if (gatewayserver.pid == KERNEL_PID_UNDEF) {
        printf("Error: server was not running\n");
        return;
    }
    /* stop server */
    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &gatewayserver);
    gatewayserver.pid = KERNEL_PID_UNDEF;
    puts("Success: stopped UDP server");
}
int sndpkt(int argc, char **argv)
{
	if (argc != 12) printf(
			"Not enough arguments!Usage:\n1: addr\n2: msg\n3: cnt\n4: data->val[0]\n5: data -> val[1]\n6: data ->val[2]\n7: data->unit\n8: data->scale\n9: name\n10: new_device");
	printf("sending pkt to %s!\n", argv[1]);
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

    ipv6_addr_t addr;
    if (ipv6_addr_from_str(&addr, argv[1]) == NULL) {
        puts("Error: unable to parse destination address");
        return -1;
    }
    rfnode_udpsend(addr, (uint16_t) 12345,(char*) pkt, 1,
                     (unsigned int) 1000000);
    return 0;
}