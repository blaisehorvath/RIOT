/*
 * rfnode.h
 *
 *  Created on: Feb 9, 2016
 *      Author: v
 */

#ifndef EXAMPLES_RF_NODE_RFNODE_H_
#define EXAMPLES_RF_NODE_RFNODE_H_

#include "phydat.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Direct mapped GPIO configuration values
 */
typedef enum {
	GET_SENSACT_LIST, /**< The gateway requests a list of sensors with this command		(server->node)*/
	SENSACT_LIST_ACK, /**< Reply command for a list request: count of sensors in cnt	(node->server)*/
	SENSACT_LIST_ITEM,/**< Item from the sensor list, with number in cnt				(node->server)*/
	GET_SENSACT,      /**< Get value of sensor, number in cnt							(server->node)*/
	SET_SENSACT,      /**< Set value of sensor, number in cnt							(server->node)*/
	SENSACT_ACK		  /**< Return the value of sensor for set/get, number in cnt		(node->server)*/
} pkt_msg;

typedef struct {
	pkt_msg msg;        /**< Message type */
	uint16_t cnt;       /**< Count: for ex count of devs / number of current dev */
	phydat_t data;      /**< Measured / current data */
	char name[20];
	uint8_t new_device; /**< New sensor on the node side if true, request of a GET_SENSACT_LIST */
} rfnode_pkt;

#ifdef __cplusplus
}
#endif


#endif /* EXAMPLES_RF_NODE_RFNODE_H_ */
