/* ULP Example: pulse counting

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This file contains assembly code which runs on the ULP.

   ULP wakes up to run this code at a certain period, determined by the values
   in SENS_ULP_CP_SLEEP_CYCx_REG registers. On each wake up, the program checks
   the input on GPIO0. If the value is different from the previous one, the
   program "debounces" the input: on the next debounce_max_count wake ups,
   it expects to see the same value of input.
   If this condition holds true, the program increments signal_count and starts
   waiting for input signal polarity to change again.
   When the edge counter reaches certain value (set by the main program),
   this program running triggers a wake up from deep sleep.
*/

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files
 */
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss
	/* Next input signal edge expected: 0 (negative) or 1 (positive) */
	.global previous_count_l
previous_count_l:
	.long 0

	/* Next input signal edge expected: 0 (negative) or 1 (positive) */
	.global previous_count_h
previous_count_h:
	.long 0

	/* Next input signal edge expected: 0 (negative) or 1 (positive) */
	.global time1
time1:
	.long 0

	/* Next input signal edge expected: 0 (negative) or 1 (positive) */
	.global time2
time2:
	.long 0

	/* Next input signal edge expected: 0 (negative) or 1 (positive) */
	.global time3
time3:
	.long 0

	/* Next input signal edge expected: 0 (negative) or 1 (positive) */
	.global time4
time4:
	.long 0

	/* Next input signal edge expected: 0 (negative) or 1 (positive) */
	.global next_edge
next_edge:
	.long 0

	/* Total number of signal edges acquired - low 16 bits*/
	.global signal_count_l
signal_count_l:
	.long 0

	/* Total number of signal edges acquired - high 16 bits*/
	.global signal_count_h
signal_count_h:
	.long 0

	/* RTC IO number used to sample the input signal.
	   Set by main program. */
	.global io_number
io_number:
	.long 0

	/* RTC IO number with red LED */
	.global io_number_rl
io_number_rl:
	.long 0

	/* RTC IO number with green LED. */
	.global io_number_gl
io_number_gl:
	.long 0

	/* Code goes into .text section */
	.text
	.global entry
entry:
	/* Load io_number */
	move r1, io_number
	ld r1, r1, 0

    .global read_IO
read_IO:
	/* Read the value of lower 16 RTC IOs into R0 */
	READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)
	rsh r0, r0, r1

    .global read_done
read_done:
	and r0, r0, 1
	/* State of input changed? */
	move r3, next_edge
	ld r3, r3, 0
	add r3, r0, r3
	and r3, r3, 1
	jump edge_detected, eq
	/* Not changed */
	/* End program */
	jump read_IO

	.global edge_detected
edge_detected:
	/* Flip next_edge */
	move r3, next_edge
	ld r2, r3, 0
	add r2, r2, 1
	and r2, r2, 1
	st r2, r3, 0
    /* Will we wait next log1 */
    jump increase_count, eq
    /* Not full cycle yet 0->1->0 */
	/* End program */
	jump read_IO

    .global increase_count
increase_count:
    /* Increment signal_count */
	move r3, signal_count_l
	ld r2, r3, 0
	add r2, r2, 1
	st r2, r3, 0
    jump overflow, ov
    /* Finish */
    jump read_IO

    .global overflow
overflow:
    /* increase by 1 higher 16 bit of counter */
    move r3, signal_count_h
    ld r2, r3, 0
    add r2, r2, 1
    st r2, r3, 0
    /* Finish */
    jump read_IO
