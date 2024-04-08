######################
#     our TESTS      #
######################

.globl __start

.text

__start:
    # fill memory with data
    li t4, 0x12
    sw t4, 0(x0)
    li t5, 0x0a
    sw t5, 4(x0)
    li t6, 0xc2
    sw t6, 8(x0)
    addi t4, t4, 3
    sw t4, 12(x0)
    addi t5, t5, 8
    sw t5, 16(x0)
    addi t6, t6, 7
    sw t6, 20(x0)
    li t0, 0 # base address of array to read
    li t1, 4 # incrementer
    li t2, 0 # counter
    li t3, 10 # number of iterations

lw_loop:
    lw t4, 0(t0) # loading word from base
    addi t2, t2, 1
    add t0, t0, t1
    blt t2, t3, lw_loop
    nop
    nop
    ecall