/*
 * Minimal replacement for the Snapmaker bootloader - ONLY if the original one
 * is lost (e.g. after a readout-protection unlock wiped the chip).
 * Lives at 0x08000000, jumps straight to the application at 0x08009800.
 * No USB-stick update support - flash the firmware with ST-Link instead.
 */
	.syntax unified
	.cpu cortex-m3
	.thumb

	.equ APP_BASE, 0x08009800
	.equ SCB_VTOR, 0xE000ED08

	.section .text
	.word 0x20010000		/* initial SP: top of 64K RAM */
	.word stub_reset		/* reset vector */

	.thumb_func
	.global stub_reset
stub_reset:
	ldr r0, =APP_BASE
	ldr r1, =SCB_VTOR
	str r0, [r1]			/* vector table -> application */
	ldr r1, [r0]
	msr msp, r1			/* application's stack pointer */
	ldr r0, [r0, #4]
	bx  r0				/* application's Reset_Handler */
	.ltorg
