/*
 * shell-wpan.h - WPAN transceiver configuration
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

#ifndef SHELL_WPAN_H
#define	SHELL_WPAN_H

/*
 * @@@ hack - just noop it. It's currently used by platform/freertos but
 * shouldn't be anywhere near platform code.
 */

#define shell_wpan_init()

#endif /* !SHELL_WPAN_H */
