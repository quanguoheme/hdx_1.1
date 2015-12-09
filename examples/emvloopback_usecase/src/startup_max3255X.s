;/*****************************************************************************
; * @file:    startup_max3255X.s
; * @purpose: Cobra Cortex-M3 Core Device Startup File 
; *           for the Maxim Integrated MAX3255X Microcontrollers 
; * @version: V1.00
; * @date:    19. Jan. 2015
; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------
; *
; * Copyright (C) 2008-2009 ARM Limited. All rights reserved.
; * ARM Limited (ARM) is supplying this software for use with Cortex-M3 
; * processor based microcontrollers.  This file can be freely distributed 
; * within development tools that are supporting such ARM based processors. 
; *
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; *****************************************************************************/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00004000

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00010000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB

; System Reset
                AREA    RESET, CODE, READONLY

                LDR     R0, =System_Reset
                BX      R0

; Vector Table in RAM

                AREA    |.data|, DATA, READWRITE, ALIGN=8

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     System_Reset              ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; Extra Interrupts
                DCD     handler_default_undefined_                    ; Interrupt #16
                DCD     handler_default_undefined_                    ; Interrupt #17
                DCD     handler_default_undefined_                    ; Interrupt #18
                DCD     handler_default_undefined_                    ; Interrupt #19
                DCD     handler_default_undefined_                    ; Interrupt #20
                DCD     handler_default_undefined_                    ; Interrupt #21
                DCD     handler_default_undefined_                    ; Interrupt #22
                DCD     handler_default_undefined_                    ; Interrupt #23
                DCD     handler_default_undefined_                    ; Interrupt #24
                DCD     handler_default_undefined_                    ; Interrupt #25
                DCD     handler_default_undefined_                    ; Interrupt #26
                DCD     handler_default_undefined_                    ; Interrupt #27
                DCD     handler_default_undefined_                    ; Interrupt #28
                DCD     handler_default_undefined_                    ; Interrupt #29
                DCD     handler_default_undefined_                    ; Interrupt #30
                DCD     handler_default_undefined_                    ; Interrupt #31
                DCD     handler_default_undefined_                    ; Interrupt #32
                DCD     handler_default_undefined_                    ; Interrupt #33
                DCD     handler_default_undefined_                    ; Interrupt #34
                DCD     handler_default_undefined_                    ; Interrupt #35
                DCD     handler_default_undefined_                    ; Interrupt #36
                DCD     handler_default_undefined_                    ; Interrupt #37
                DCD     handler_default_undefined_                    ; Interrupt #38
                DCD     handler_default_undefined_                    ; Interrupt #39
                DCD     handler_default_undefined_                    ; Interrupt #40
                DCD     handler_default_undefined_                    ; Interrupt #41
                DCD     handler_default_undefined_                    ; Interrupt #42
                DCD     handler_default_undefined_                    ; Interrupt #43
                DCD     handler_default_undefined_                    ; Interrupt #44
                DCD     handler_default_undefined_                    ; Interrupt #45
                DCD     handler_default_undefined_                    ; Interrupt #46
                DCD     handler_default_undefined_                    ; Interrupt #47
                DCD     handler_default_undefined_                    ; Interrupt #48
                DCD     handler_default_undefined_                    ; Interrupt #49
                DCD     handler_default_undefined_                    ; Interrupt #50
                DCD     handler_default_undefined_                    ; Interrupt #51
                DCD     handler_default_undefined_                    ; Interrupt #52
                DCD     handler_default_undefined_                    ; Interrupt #53
                DCD     handler_default_undefined_                    ; Interrupt #54
                DCD     handler_default_undefined_                    ; Interrupt #55
                DCD     handler_default_undefined_                    ; Interrupt #56
                DCD     handler_default_undefined_                    ; Interrupt #57
                DCD     handler_default_undefined_                    ; Interrupt #58
                DCD     handler_default_undefined_                    ; Interrupt #59
                DCD     handler_default_undefined_                    ; Interrupt #60
                DCD     handler_default_undefined_                    ; Interrupt #61
                DCD     handler_default_undefined_                    ; Interrupt #62
                DCD     handler_default_undefined_                    ; Interrupt #63

                EXPORT  __Vectors


                AREA    |.constdata|, DATA, READONLY
                ALIGN   4
Part_Number     DCD     __MAXIM_DEVICE__                                    ; Maxim Part Number
Clock_Speed     DCD     __CLOCK_SPEED__                                     ; CPU Speed in Hz

                EXPORT  Part_Number
                EXPORT  Clock_Speed


                AREA    |.text|, CODE, READONLY

; Disable interrupts, set initial SP and initialize vector table address
VTOR_ADDR       EQU     0xE000ED08  
System_Reset    PROC
                EXPORT  System_Reset
                CPSID   I
                LDR     SP, =__initial_sp
                LDR     R3, =VTOR_ADDR
                LDR     R0, =__Vectors
                STR     R0, [R3]
                LDR     R0, =Reset_Handler
                BX      R0
                ENDP

; Reset Handler (can be modified)
Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  __main
                LDR     R0, =__main
                BX      R0
                ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

handler_default_undefined_ PROC
                export  handler_default_undefined_ [WEAK]
                B       .
                ENDP

                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB
                
                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit
                
                ELSE
                
                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF


                END
