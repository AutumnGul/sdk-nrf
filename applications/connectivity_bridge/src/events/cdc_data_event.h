/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _CDC_DATA_EVENT_H_
#define _CDC_DATA_EVENT_H_

/**
 * @brief CDC Data Event
 * @defgroup cdc_data_event CDC Data Event
 * @{
 */

#include <string.h>
#include <toolchain/common.h>

#include <event_manager.h>
#include <event_manager_profiler.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Peer connection event. */
struct cdc_data_event {
	struct event_header header;

	uint8_t dev_idx;
	uint8_t *buf;
	size_t len;
};

EVENT_TYPE_DECLARE(cdc_data_event);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* _CDC_DATA_EVENT_H_ */
