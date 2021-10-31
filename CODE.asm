
include "minimal.inc"

    ldi A, 0FFH
    sta R0, A
    sta R1, A
    inc A
    inc A
    out A
    shfl A
    out A
    shfl A
    out A
    shfl A
    halt
    lda A, R0
_loop:
    out A
    jmp _loop
