
element accumulator?
element register?

element A?: accumulator
repeat 8H i:0H
    element R#i?: (register + i)
end repeat

macro _ensure_accumulator? target?*
    local _target
    _target = target
    assert ((_target eq (_target element 1H)) & ((_target metadata 1) relativeto accumulator))
end macro

struc (result?) _ensure_register? target?*
    local _target, _metadata
    _target = target
    _metadata = (_target metadata 1H)
    assert ((_target eq (_target element 1H)) & (_metadata relativeto register))
    result = (_metadata scale 0H)
end struc

macro _ensure_range? value?*
    assert (((value) >= 0H) & ((value) <= 0FFH))
end macro

struc (result?) _ensure_address? memory?*
    result = (not 0H)
    match =[ address =], memory
        result = address
    end match
    _ensure_range result
end struc

_lda?   := 00010B
_sta?   := 00100B
_jmp?   := 001100000B
_jz?    := 001110000B
_jnz?   := 010000000B
_jp?    := 010010000B
_and?   := 10100B
_or?    := 10110B
_add?   := 11000B
_sub?   := 11010B
_not?   := 0H
_inc?   := 1H
_dec?   := 2H
_shfl?  := 3H
_shfr?  := 4H
_rotr?  := 5H
_in?    := 0H
_out?   := 1H

macro nop?
    dd 0H
end macro

iterate _kind?*, lda,and,or,add,sub
    macro _kind? destination?*, register?*
        _ensure_accumulator destination
        idx _ensure_register register
        dd ((_#_kind shl 3H) or idx)
    end macro
end iterate

macro sta? register?*, destination?*
    idx _ensure_register register
    _ensure_accumulator destination
    dd ((_sta shl 3H) or idx)
end macro

macro ldm? destination?*, memory?*
    _ensure_accumulator destination
    adr _ensure_address memory
    dd (000110000B), adr
end macro

macro stm? memory?*, destination?*
    adr _ensure_address memory
    _ensure_accumulator destination
    dd (001000000B), adr
end macro

macro ldi? destination?*, immediate?*
    _ensure_accumulator destination
    _ensure_range immediate
    dd (001010000B), immediate
end macro

iterate _kind?*, jmp,jz,jnz,jp
    macro _kind? absolute?*
        _absolute = ((absolute) shr 2H)
        _ensure_range _absolute
        dd (_#_kind), _absolute
    end macro
end iterate

iterate _kind?*, not,inc,dec,shfl,shfr,rotr
    macro _kind? destination?*
        _ensure_accumulator destination
        dd ((01110B shl 4H) or (_#_kind))
    end macro
end iterate

iterate _kind?*, in,out
    macro _kind? destination?*
        _ensure_accumulator destination
        dd ((01111B shl 4H) or (_#_kind))
    end macro
end iterate

macro halt?
    dd (011110010B)
end macro

org 0H

