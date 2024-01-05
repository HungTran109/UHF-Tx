#ifndef TCP_CONSOLE_H
#define TCP_CONSOLE_H

#include "stdint.h"

/**
 * @brief           Start TCP console
 */
void tcp_console_start(void);

/**
 * @brief           Put data to tcp console
 * @param[in]       TCP console data
 */
int32_t tcp_console_puts(char *msg);

/**
 * @brief           Close tcp console sock
 */
void tcp_console_close(void);

/**
 * @brief           Set default tcp console password
 */
void tcp_console_set_password(char *password);

/**
 * @brief           Set default tcp console username
 */
void tcp_console_set_username(char *username);

#endif /* TCP_CONSOLE_H */
