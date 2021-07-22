/*
 * Copyright (c) 2020 InnBlue
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(tftp_client, CONFIG_TFTP_LOG_LEVEL);

#include <stddef.h>
#include <net/tftp.h>
#include <stdio.h>

#include "tftp_client.h"

#define ADDRLEN(sock) \
	(((struct sockaddr *)sock)->sa_family == AF_INET ? \
		sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6))

/* TFTP Global Buffer. */
static uint8_t   tftpc_buffer[TFTPC_MAX_BUF_SIZE];

/* Global mutex to protect critical resources. */
K_MUTEX_DEFINE(tftpc_lock);


// /*
//  * Converts block number to length-2 string.
//  */
// void s_to_i(char *f, int n)
// {
// 	if (n == 0) 
// 	{
// 		f[0] = '0', f[1] = '0', f[2] = '\0';
// 	} 
// 	else if (n % 10 > 0 && n / 10 == 0)
// 	{
// 		char c = n + '0';
// 		f[0] = '0', f[1] = c, f[2] = '\0';
// 	} else if(n % 100 > 0 && n / 100 == 0){
// 		char c2 = (n % 10) + '0';
// 		char c1 = (n / 10) + '0';
// 		f[0] = c1, f[1] = c2, f[2] = '\0';
// 	} else {
// 		f[0] = '9', f[1] = '9', f[2] = '\0';
// 	}
// }

/*
 * Prepare the read request as required by RFC1350. This packet can be sent
 * out directly to the TFTP server.
 */
static size_t make_request(uint8_t *buf, int request,
			   const char *remote_file, const char *mode)
{
	char *ptr = (char *)buf;
	const char def_mode[] = "octet";

	/* Fill in the Request Type. */
	sys_put_be16(request, ptr);
	ptr += 2;

	/* Copy the name of the remote file. */
	strncpy(ptr, remote_file, TFTP_MAX_FILENAME_SIZE);
	ptr += strlen(remote_file);
	*ptr++ = '\0';

	/* Default to "Octet" if mode not specified. */
	if (mode == NULL) {
		mode = def_mode;
	}

	/* Copy the mode of operation. */
	strncpy(ptr, mode, TFTP_MAX_MODE_SIZE);
	ptr += strlen(mode);
	*ptr++ = '\0';

	return ptr - (char *)buf;
}

/*
 * Send an Error Message to the TFTP Server.
 */
static inline int send_err(int sock, int err_code, char *err_msg)
{
	uint32_t req_size;

	LOG_DBG("Client sending error code: %d", err_code);

	/* Fill in the "Err" Opcode and the actual error code. */
	sys_put_be16(ERROR_OPCODE, tftpc_buffer);
	sys_put_be16(err_code, tftpc_buffer + 2);
	req_size = 4;

	/* Copy the Error String. */
	if (err_msg != NULL) {
		strcpy(tftpc_buffer + req_size, err_msg);
		req_size += strlen(err_msg);
	}

	/* Send Error to server. */
	return send(sock, tftpc_buffer, req_size, 0);
}

/*
 * Send an Ack Message to the TFTP Server.
 */
static inline int send_ack(int sock, struct tftphdr_ack *ackhdr)
{
	LOG_DBG("Client acking block number: %d", ntohs(ackhdr->block));

	send(sock, ackhdr, sizeof(struct tftphdr_ack), 0);

	return 0;
}

/*
 * Send a data message to the TFTP Server.
 */
static inline int send_data(int sock, int block, char *msg, int msg_size)
{
	uint32_t req_size = 0;

	sys_put_be16(DATA_OPCODE, tftpc_buffer);
	sys_put_be16(block, tftpc_buffer + 2);
	req_size = TFTP_HEADER_SIZE;

	strncpy(tftpc_buffer + req_size, msg, msg_size);
	req_size += msg_size;

	return send(sock, tftpc_buffer, req_size, 0);
}


static int tftp_send_request(int sock, struct sockaddr *server_addr,
		int request, const char *remote_file, const char *mode)
{
	int tx_count = 0;
	size_t req_size;
	int ret;

	/* Create TFTP Request. */
	req_size = make_request(tftpc_buffer, request, remote_file, mode);

	do {
		tx_count++;

		LOG_DBG("Sending TFTP request %d file %s", request,
			log_strdup(remote_file));

		/* Send the request to the server */
		ret = sendto(sock, tftpc_buffer, req_size, 0, server_addr,
			     ADDRLEN(server_addr));
		if (ret < 0) {
			break;
		}

		/* Poll for the response */
		struct pollfd fds = {
			.fd     = sock,
			.events = ZSOCK_POLLIN,
		};

		ret = poll(&fds, 1, CONFIG_TFTPC_REQUEST_TIMEOUT);
		if (ret <= 0) {
			LOG_DBG("Failed to get data from the TFTP Server"
				", req. no. %d", tx_count);
			continue;
		}

		/* Receive data from the TFTP Server. */
		struct sockaddr from_addr;
		socklen_t from_addr_len = sizeof(from_addr);

		ret = recvfrom(sock, tftpc_buffer, TFTPC_MAX_BUF_SIZE, 0,
				&from_addr, &from_addr_len);
		if (ret < TFTP_HEADER_SIZE) {
			req_size = make_request(tftpc_buffer, request,
						remote_file, mode);
			continue;
		}

		/* Limit communication to the specific address:port */
		connect(sock, &from_addr, from_addr_len);

		break;

	} while (tx_count <= TFTP_REQ_RETX);

	return ret;
}

int tftp_get(struct sockaddr *server_addr, struct tftpc *client,
	     const char *remote_file, const char *mode)
{
	int sock;
	uint32_t tftpc_block_no = 1;
	uint32_t tftpc_index = 0;
	int tx_count = 0;
	struct tftphdr_ack ackhdr = {
		.opcode = htons(ACK_OPCODE),
		.block = htons(1)
	};
	int rcv_size;
	int ret;

	sock = socket(server_addr->sa_family, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		printf("Failed to create UDP socket (IPv4): %d\n", errno);
		return -errno;
	}

	/* Obtain Global Lock before accessing critical resources. */
	k_mutex_lock(&tftpc_lock, K_FOREVER);

	/* Send out the request to the TFTP Server. */
	ret = tftp_send_request(sock, server_addr, READ_REQUEST, remote_file, mode);
	rcv_size = ret;

	while (rcv_size >= TFTP_HEADER_SIZE && rcv_size <= TFTPC_MAX_BUF_SIZE) {
		/* Process server response. */

		uint16_t opcode = sys_get_be16(tftpc_buffer);
		uint16_t block_no = sys_get_be16(tftpc_buffer + 2);

		LOG_DBG("Received data: opcode %u, block no %u, size %d",
			opcode, block_no, rcv_size);

		if (opcode != DATA_OPCODE) {
			printf("Server responded with invalid opcode.\n");
			ret = TFTPC_REMOTE_ERROR;
			break;
		}

		if (block_no == tftpc_block_no) {
			uint32_t data_size = rcv_size - TFTP_HEADER_SIZE;

			tftpc_block_no++;
			ackhdr.block = htons(block_no);
			tx_count = 0;

			// printf("data_sz=%d user_buf=%d index=%d\n", data_size, client->user_buf_size, tftpc_index);
			/* Only copy block if user buffer has enough space */
			if (data_size > (client->user_buf_size - tftpc_index)) {
				printf("User buffer is full.\n");
				send_err(sock, TFTP_ERROR_DISK_FULL, NULL);
				ret = TFTPC_BUFFER_OVERFLOW;
				break;
			}

			/* Perform the actual copy and update the index. */
			memcpy(client->user_buf + tftpc_index,
				tftpc_buffer + TFTP_HEADER_SIZE, data_size);
			tftpc_index += data_size;

			/* Per RFC1350, the end of a transfer is marked
			 * by datagram size < TFTPC_MAX_BUF_SIZE.
			 */
			if (rcv_size < TFTPC_MAX_BUF_SIZE) {
				ret = send_ack(sock, &ackhdr);
				client->user_buf_size = tftpc_index;
				break;
			}
		}

		/* Poll for the response */
		struct pollfd fds = {
			.fd     = sock,
			.events = ZSOCK_POLLIN,
		};

		do {
			if (tx_count > TFTP_REQ_RETX) {
				printf("No more retransmits. Exiting\n");
				ret = TFTPC_RETRIES_EXHAUSTED;
				goto req_abort;
			}

			/* Send ACK to the TFTP Server */
			send_ack(sock, &ackhdr);
			tx_count++;

		} while (poll(&fds, 1, CONFIG_TFTPC_REQUEST_TIMEOUT) <= 0);

		/* Receive data from the TFTP Server. */
		ret = recv(sock, tftpc_buffer, TFTPC_MAX_BUF_SIZE, 0);
		rcv_size = ret;
	}

	if (!(rcv_size >= TFTP_HEADER_SIZE && rcv_size <= TFTPC_MAX_BUF_SIZE)) {
		ret = TFTPC_REMOTE_ERROR;
	}

req_abort:
	k_mutex_unlock(&tftpc_lock);
	close(sock);
	return ret;
}

int tftp_put(struct sockaddr *server_addr, struct tftpc *client,
	     const char *remote_file, const char *mode)
{
	int sock;
	int send_size = client->user_buf_size;
	int ret;

	sock = socket(server_addr->sa_family, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		printf("Failed to create UDP socket (IPv4): %d\n", errno);
		return -errno;
	}

	/* Obtain Global Lock before accessing critical resources. */
	k_mutex_lock(&tftpc_lock, K_FOREVER);

	/* Send out the request to the TFTP Server. */
	ret = tftp_send_request(sock, server_addr, WRITE_REQUEST, remote_file, mode);
	
	/* Wait for an ACK from the server. */
	struct pollfd fds = {
		.fd     = sock,
		.events = ZSOCK_POLLIN,
	};

	if (poll(&fds, 1, CONFIG_TFTPC_REQUEST_TIMEOUT) <= 0)
	{
		printf("Poll error waiting for WRITE ACK\n");
		return -1; // TODO
	}

	/* Receive an ACK from the TFTP Server. */
	ret = recv(sock, tftpc_buffer, TFTPC_MAX_BUF_SIZE, 0);
	if (tftpc_buffer[0] != '0' && tftpc_buffer[1] != '4')
	{
		printf("Invalid ACK for WRITE request\n");
		return -1; // TODO
	}

	int remaining = send_size;
	int block = 1;
	int max_packet_sz = TFTPC_MAX_BUF_SIZE - TFTP_HEADER_SIZE;
	uint8_t *p_data = tftpc_buffer;

	while (remaining > 0)
	{
		// Next packet size.
		int next_block_sz = (remaining < max_packet_sz) ? remaining : max_packet_sz;

		// Send the current block.
		printf("Sending block %d\n", block);
		if (send_data(sock, block, p_data, next_block_sz) < 0)
		{
			printf("failed to send block %d\n", block);
			return -1; // TODO
		}

		/* Wait for an ACK from the server. */
		struct pollfd fds = {
			.fd     = sock,
			.events = ZSOCK_POLLIN,
		};

		if (poll(&fds, 1, CONFIG_TFTPC_REQUEST_TIMEOUT) <= 0)
		{
			printf("Poll error waiting for WRITE ACK\n");
			return -1; // TODO
		}

		/* Receive an ACK from the TFTP Server. */
		ret = recv(sock, tftpc_buffer, TFTPC_MAX_BUF_SIZE, 0);
		if (tftpc_buffer[0] != '0' && tftpc_buffer[1] != '4')
		{
			printf("Invalid ACK for WRITE request\n");
			return -1; // TODO
		}

		/* Update the data pointer to the next block */
		p_data += next_block_sz;

		remaining -= next_block_sz;
		if (remaining == 0 && next_block_sz == max_packet_sz)
		{
			printf("TODO: Send dummy here to signal end of file.\n");
			return -1; // TODO
		}
	}

	return ret;
}
