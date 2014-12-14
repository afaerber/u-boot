#include <config.h>

#if defined(__ASSEMBLY__)
	.syntax unified
#endif

#ifdef CONFIG_SYS_THUMB_BUILD
#define ARM(x...)
#define THUMB(x...) x
#else
#define ARM(x...) x
#define THUMB(x...)
#endif
