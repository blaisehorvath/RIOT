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
#include "../rf-stuff/rfnode.h"
#include <stdio.h>

#include "shell.h"
#include "msg.h"

//TEMP INCLUDES, TO BE CLEARED UP LATER!!! \\TODO

#define OURDEV_TEMP 7
#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];
extern int udp_cmd(int argc, char **argv);
extern int sndpkt_dodagroot(int argc, char **argv);
extern void rfnode_udpserver_start(uint32_t port);
extern void setup_gnrc_node(void);
static const shell_command_t shell_commands[] = {
	{"pkt_to_root","send packet to RPL dodag root",sndpkt_dodagroot},
	{ "udp", "send data over UDP and listen on UDP ports", udp_cmd },
	{ NULL, NULL, NULL },

};
int main(void)
{
	setup_gnrc_node();
	msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT network stack example application");
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
