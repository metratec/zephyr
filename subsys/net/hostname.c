/** @file
 * @brief Hostname configuration
 */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_hostname, CONFIG_NET_HOSTNAME_LOG_LEVEL);

#include <zephyr/kernel.h>

#include <zephyr/net/hostname.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_mgmt.h>

static char _hostname[NET_HOSTNAME_SIZE];

static void trigger_net_event(void)
{
	if (IS_ENABLED(CONFIG_NET_MGMT_EVENT_INFO)) {
		struct net_event_l4_hostname info;

		memcpy(info.hostname, _hostname, sizeof(_hostname));
		net_mgmt_event_notify_with_info(NET_EVENT_HOSTNAME_CHANGED, NULL,
						&info, sizeof(info));
	} else {
		net_mgmt_event_notify(NET_EVENT_HOSTNAME_CHANGED, NULL);
	}
}

const char *net_hostname_get(void)
{
	return _hostname;
}

#if defined(CONFIG_NET_HOSTNAME_DYNAMIC)
int net_hostname_set(char *hostname, size_t len)
{
	if (len > NET_HOSTNAME_MAX_LEN) {
		return -ENOMEM;
	}

	memcpy(_hostname, hostname, len);
	_hostname[len] = 0;

	NET_DBG("New hostname %s", _hostname);
	trigger_net_event();

	return 0;
}
#endif

#if defined(CONFIG_NET_HOSTNAME_UNIQUE)
int net_hostname_set_postfix(const uint8_t *hostname_postfix,
			     int postfix_len)
{
#if !defined(CONFIG_NET_HOSTNAME_UNIQUE_UPDATE)
	static bool postfix_set;
#endif
	int pos = 0;
	int i;

#if !defined(CONFIG_NET_HOSTNAME_UNIQUE_UPDATE)
	if (postfix_set) {
		return -EALREADY;
	}
#endif

	NET_ASSERT(postfix_len > 0);

	/* Note that we convert the postfix to hex (2 chars / byte) */
	if ((postfix_len * 2) >
	    (NET_HOSTNAME_MAX_LEN - (sizeof(CONFIG_NET_HOSTNAME) - 1))) {
		return -EMSGSIZE;
	}

	for (i = 0; i < postfix_len; i++, pos += 2) {
		snprintk(&_hostname[sizeof(CONFIG_NET_HOSTNAME) - 1 + pos], 2 + 1, "%02x",
			 hostname_postfix[i]);
	}

	NET_DBG("New hostname %s", _hostname);

#if !defined(CONFIG_NET_HOSTNAME_UNIQUE_UPDATE)
	postfix_set = true;
#endif
	trigger_net_event();

	return 0;
}
#endif /* CONFIG_NET_HOSTNAME_UNIQUE */

void net_hostname_init(void)
{
	memcpy(_hostname, CONFIG_NET_HOSTNAME, sizeof(CONFIG_NET_HOSTNAME) - 1);

	NET_DBG("Hostname set to %s", CONFIG_NET_HOSTNAME);
	trigger_net_event();
}
