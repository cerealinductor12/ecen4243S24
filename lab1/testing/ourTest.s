######################
#     our TESTS      #
######################

.globl __start

.text

__start:
    li t0, 0 # base address of array to read
    li t1, 4 # incrementer
    li t2, 0 # counter
    li t3, 10 # number of iterations
    li t5, 0 # value to store

loop:
    sw t5, 0(t0)
    addi t2, t2, 1
    add t0, t0, t1
    addi t5, 7
    lw t4, 0(t0) # loading word from base
    addi t2, t2, 1
    add t0, t0, t1
    nop
    nop
    bne t4, t5, failure
    blt t2, t3, loop
    nop
    nop
    ecall

failure:
    ecall