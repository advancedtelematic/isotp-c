#ifndef __ISOTP_PROTOCOL_H__
#define __ISOTP_PROTOCOL_H__

#include <stdint.h>

/* Functions related to session-level protocol, should be implemented by user */

uint32_t protocol_swap_sa_ta (uint32_t arbitration);

#endif // __ISOTP_PROTOCOL_H_
