#include <config.h>
#include <linux/string.h>

void _start(void);

extern char _bss_start;
extern char _bss_end;

void _start(void)
{
	asm volatile ("cpsid i");

	memset(&_bss_start, 0, &_bss_end - &_bss_start);
}
