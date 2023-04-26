#pragma once

#include <stdint.h>
#include "lwip/ip_addr.h"

err_t bcmp_send_ping(uint64_t node_id);
err_t bcmp_process_ping_request(bcmp_echo_request_t *echo_req, const ip_addr_t *src, const ip_addr_t *dst);
err_t bcmp_process_ping_reply(bcmp_echo_reply_t *echo_reply);
