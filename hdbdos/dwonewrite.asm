*******************************************************
*
* DWWrite - One ROM DriveWire bridge Edition
*    Send a packet to the DriveWire server via One ROM's drivewire plugin,
*    over the ROM address bus, instead of a real or emulated serial port.
*    See plugins/user/drivewire in the One ROM repository for the device
*    side of this protocol, and the comment block below for why this code
*    runs from the stack rather than from ROM.
*
* Entry:
*    X  = starting address of data to send
*    Y  = number of bytes to send (1-256; 256 is a normal 16-bit 256, not
*         a special case on entry - only the wire's count byte, which is
*         8 bits, needs a 256->0 mapping)
*
* Exit:
*    X  = address of last byte sent + 1
*    Y  = 0
*    All others preserved
*
* Why this runs from the stack, not from ROM:
*    One ROM's drivewire plugin recognises the knock and count/data bytes by
*    watching every address the CPU reads from the ROM socket while it is
*    CS-active - it cannot tell a deliberate signalling read from an
*    ordinary instruction fetch.  If DWWrite executed in place from the ROM
*    image, its own opcode fetches (from wherever DWWrite itself sits in
*    ROM) would be interleaved with the deliberate address-bus reads below,
*    breaking the knock's exact-sequence match.  DWWrite therefore copies
*    the position-independent DWWCODE block onto the stack and calls it
*    there, so every read the CPU performs during that call is either from
*    RAM (invisible to the plugin's ROM address monitor) or one of the
*    deliberate reads below.  DWWCODE relies on nothing but PC-relative
*    branches and immediate operands, so a byte-for-byte copy runs
*    correctly from any address - no fixups are needed.
*
* Wire protocol (must exactly match plugins/user/drivewire):
*    "!DWSEND!" knock (8 bytes, low byte of the address IS the value - see
*    plugins/user/drivewire's file header comment on why that is safe), then
*    a bounded poll-with-retry against $C200 (a byte address in a separate
*    page from the $C100-$C1FF count/data range below - see DWWACKTIMEOUT's
*    own comment) for the plugin's ack, resending the whole knock if it
*    doesn't arrive in time.  Then one count byte (0 means 256), then N
*    further reads whose address low byte is the data byte.  The value
*    actually read back from the ROM at each of these count/data addresses
*    is discarded - only the address matters; the ack poll is the one read
*    in this whole routine where the value read back is what's checked.

* Poll-with-retry budget for the knock's ack (see DWWCODE below).  The
* plugin's ROM-bus capture can occasionally miss the knock's own address
* sequence under load (root cause not fully pinned down); this makes the
* write session self-healing against it regardless, at the cost of retrying
* rather than proceeding immediately.  DWWACKRETRIES is bounded so a system
* with no drivewire plugin present (the ack byte never changes) falls
* through and proceeds exactly as before this existed, rather than hanging
* forever - roughly DWWACKTIMEOUT*DWWACKRETRIES poll iterations worst case.
DWWACKTIMEOUT equ      $4000
DWWACKRETRIES equ      8

DWWrite     pshs      u,d,cc
            orcc      #$50               ; mask interrupts - an interleaved
                                          ; ISR could desync the sequence
            ldu       #DWWCODEEND
cpydww@     lda       ,-u
            pshs      a
            cmpu      #DWWCODE
            bne       cpydww@
            jsr       ,s
            leas      DWWCODELEN,s
            puls      cc,d,u,pc

* Position-independent body - copied onto the stack and run from there.
* Entered with X/Y as DWWrite's own entry parameters (untouched by the
* copy loop above), U free to use, and interrupts already masked.
DWWCODE     ldu       #$C100             ; signalling page - see
                                          ; plugins/user/drivewire: low byte
                                          ; of the effective address of
                                          ; "LDA b,u" IS the value b,
                                          ; regardless of which 256-byte
                                          ; page that lands in

            IFDEF     ONEROM_TEST_PATTERN
            ldb       #'!
            lda       b,u
            ldb       #'D
            lda       b,u
            ldb       #'W
            lda       b,u
            ldb       #'S
            lda       b,u
            ldb       #'E
            lda       b,u
            ldb       #'N
            lda       b,u
            ldb       #'D
            lda       b,u
            ldb       #'!
            lda       b,u
* Bring-up test: after the knock, loop forever sending an incrementing
* 0x00-0xFF byte pattern - see plugins/user/drivewire's
* DRIVEWIRE_TEST_PATTERN build, which lights the status LED on the knock and
* then just relays every subsequent address-encoded byte to UART1 TX, with
* no count framing.  Does not return: DWWrite is only ever "called" once.
            clrb
dwwtest@    lda       b,u
            incb
            bra       dwwtest@
            ELSE
* Entered once with U already at $C100 above; re-loaded on every knock
* (re)send below since the ack poll further down repoints U at $C200 to
* check it - X/Y (the caller's data pointer/count) are never touched here,
* only D and the stack, since both are still needed for the count/data
* bytes after this.
            ldd       #DWWACKRETRIES
            pshs      d                  ; outer retry budget, held on the
                                          ; stack across every knock (re)send
* No blank lines from here through dwwgiveup@ below - lwasm starts a new
* local-label scope at every blank line (comments don't break it), and
* dwwknock@ in particular needs to stay reachable across this whole block.
dwwknock@   ldu       #$C100
            ldb       #'!
            lda       b,u
            ldb       #'D
            lda       b,u
            ldb       #'W
            lda       b,u
            ldb       #'S
            lda       b,u
            ldb       #'E
            lda       b,u
            ldb       #'N
            lda       b,u
            ldb       #'D
            lda       b,u
            ldb       #'!
            lda       b,u
            ldu       #$C200             ; DW_KNOCK_ACK_ADDR's page in
                                          ; plugins/user/drivewire - deliberately
                                          ; not $C100-$C1FF, which the count/data
                                          ; bytes below can land anywhere in
            ldd       #DWWACKTIMEOUT
dwwpoll@    lda       ,u                 ; poll the ack byte's actual value -
                                          ; the one read in this routine where
                                          ; what comes back is checked, not
                                          ; just the address itself
            cmpa      #$A5
            beq       dwwacked@
            subd      #1
            bne       dwwpoll@
            puls      d                  ; this attempt timed out - consume
            subd      #1                 ; one retry
            beq       dwwgiveup@         ; none left: stack already balanced,
                                          ; proceed anyway
            pshs      d                  ; save the decremented budget back
            bra       dwwknock@          ; resend the whole knock
dwwacked@   puls      d                  ; discard the saved retry budget -
                                          ; stack balanced the same as the
                                          ; exhausted path above
dwwgiveup@  ldu       #$C100             ; back to the signalling page for
                                          ; the count/data bytes below
            cmpy      #256
            bne       dwwcnt1@
            clrb                         ; wire encoding: 256 -> byte 0
            bra       dwwcnt2@
dwwcnt1@    tfr       y,d
dwwcnt2@    lda       b,u                ; send count byte

dwwloop@    lda       ,x+                ; fetch next data byte
            tfr       a,b
            lda       b,u                ; "send" it - address encodes the
                                          ; value, the byte actually read
                                          ; back is unused
            leay      -1,y
            bne       dwwloop@

            rts
            ENDC
DWWCODEEND  equ       *
DWWCODELEN  equ       DWWCODEEND-DWWCODE
