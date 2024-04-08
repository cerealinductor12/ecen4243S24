######################
#     our TESTS      #
######################

.globl __start

.text

__start:

test01:
    li t0, 0 # base address of array to read
    li t1, 4 # incrementer
    li t2, 0 # counter
    li t3, 10 # number of iterations

lw_loop:
    lw t4, 0(t0) # loading word from base
    addi t2, t2, 1
    add t0, t0, t1
    blt t2, t3, lw_loop
    ecall