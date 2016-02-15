/*
 * rfnode.h
 *
 *  Created on: Feb 9, 2016
 *      Author: v
 */

#ifndef EXAMPLES_RF_NODE_RFNODE_H_
#define EXAMPLES_RF_NODE_RFNODE_H_
#include "kernel_types.h"
#include "phydat.h"
#include "net/gnrc/pkt.h"
#include "net/ipv6/addr.h"

#ifdef __cplusplus
extern "C" {
#endif

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
 * binary values of dimensions in the data field
 *
 * Setting/getting the actuator/sensor is possible with the SET/GET_SENSACT type, the answer from the
 * node is a SENSACT_ACK type with the new values. The name and the value in the cnt must be the same.
 */
typedef enum {
	GET_SENSACT_LIST, /**< The gateway requests a list of sensors with this command				(server->node)*/
	SENSACT_LIST_ACK, /**< Reply command for a list request: count of sensors in cnt			(node->server)*/
	SENSACT_LIST_ITEM,/**< Item from the sensor list, with number in cnt				(node->server)*/
	GET_SENSACT,      /**< Get value and properties of sensor, number in cnt						(server->node)*/
	SET_SENSACT,      /**< Set value of sensor, number in cnt									(server->node)*/
	SENSACT_ACK		  /**< Return the value of sensor for set/get, number in cnt				(node->server)*/
} pkt_msg;

typedef struct {
	pkt_msg msg;        /**< Message type */
	uint16_t cnt;       /**< Count: for ex count of devs / number of current dev */
	phydat_t data;      /**< Measured / current data */
	char name[20];
	uint8_t new_device; /**< New sensor on the node side if true, request of a GET_SENSACT_LIST */
} rfnode_pkt;

int rfnode_statemachine(rfnode_pkt* ,rfnode_pkt* );
/*static */void rfnode_udp_get(gnrc_pktsnip_t *);
/*static */void* rfnode_udp_eventloop(void* );
kernel_pid_t rfnode_udpthread_init(void);
kernel_pid_t rfnode_udpthread_getpid(void);
void rfnode_udpserver_start(uint32_t);
void rfnode_udpserver_stop(void);
void rfnode_udpsend(ipv6_addr_t, uint16_t, char *, unsigned int, unsigned int);
void snd_rfnode_pkt_to_root(rfnode_pkt*);
int sndpkt_dodagroot(int , char **);
void setup_gnrc_node(void);

int sndpkt(int argc, char **argv);
void rfgateway_udpserver_stop(void);
void rfgateway_udpserver_start(uint32_t port);
kernel_pid_t rfgateway_udpthread_getpid(void);
kernel_pid_t rfgateway_udpthread_init(void);
void* rfgateway_udp_eventloop(void* arg);
void rfgateway_udp_get(gnrc_pktsnip_t *pkt);

#ifdef __cplusplus
}
#endif


#endif /* EXAMPLES_RF_NODE_RFNODE_H_ */
