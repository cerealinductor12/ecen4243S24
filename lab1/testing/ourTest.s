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
    li t4, 0 # value to load
    li t5, 0 # value to store 

loop:
    sw t5, 0(t0)
    lw t4, 0(t0)
    bne t4, t5, failure # if stored value != loaded value, failed
    addi t2, t2, 1 # increment counter
    add t0, t0, t1 # increment address
    addi t5, t5, 7 # increment value to store
    blt t2, t3, loop
    ecall

failure:
    ecall