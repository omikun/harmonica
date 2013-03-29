.perm rwx
.entry
entry:    ldi    %r0, #0
          ldi    %r1, #1
          ld     %r3, %r0, #0
loop:     shli   %r3, %r0, #2
          st     %r1, %r3, #0
          subi   %r2, %r1, #10
          jali   %r3, subroutine
          iszero @p1, %r2
    @p1 ? jmpi   sumthem
          jmpi   loop
sumthem:  ldi    %r0, #0
          ldi    %r1, #0
theloop:  ld     %r3, %r0, #0
          add    %r1, %r1, %r3
          addi   %r0, %r0, #4
          subi   %r3, %r0, #40
          rtop   @p0, %r3
    @p0 ? jmpi   theloop
finished: divi   %r1, %r1, #10
          jmpi   finished

subroutine:
          addi   %r0, %r0, #1
          addi   %r1, %r1, #1
          jmpr   %r3
