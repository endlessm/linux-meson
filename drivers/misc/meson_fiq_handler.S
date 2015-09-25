#include <linux/linkage.h>
#include <asm/assembler.h>

	.text
	.global meson_fiq_handler_end

ENTRY(meson_fiq_handler)
	ldr	r9, [r8]
        tst	r9, #1 << 27
        orreq	r9, r9, #1 << 27
	streq	r9, [r8]
        beq	out
        bic	r9, r9, #1 << 27
	str	r9, [r8]
out:
	subs	pc, lr, #4
meson_fiq_handler_end:
