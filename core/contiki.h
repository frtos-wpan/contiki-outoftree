/*
 * core/contiki.h - Detour to force inclusion of modified rtimer.h
 *
 * Developed by Werner Almesberger for Actility S.A., and
 * licensed under LGPLv2 by Actility S.A.
 */

/*
 * Hack: if we let contiki/cote/contiki.h to be found via the include path,
 * the search for sys/rtimer.h will begin at contiki/core/ and thus find
 * contiki/core/sys/rtimer.h
 *
 * Since we're overriding core/sys/rtimer.h we need to divert the include.
 * Note that this will still confuse anything under contiki/core/ that
 * includes sys/rtimer.h directly, which is unfortunately quite a lot of code.
 */

#include "sys/rtimer.h"
#include "contiki/core/contiki.h"
