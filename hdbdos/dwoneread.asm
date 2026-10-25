*******************************************************
*
* DWRead - One ROM DriveWire bridge Edition
*    Receive a response from the DriveWire server via One ROM's drivewire
*    plugin.  See dwonewrite.asm for why this runs from the stack rather
*    than from ROM, and plugins/user/drivewire for the device side of this
*    protocol.
*
*    Unlike the bit-banger DWRead, there are no leading sync bytes to hunt
*    for: the plugin's status/data handshake (below) is already an explicit,
*    unambiguous "byte ready" signal, so the sync scan that transport needed
*    to find the start of a response does not apply here.
*
* Entry:
*    X  = storage address for incoming data
*    Y  = number of bytes requested (0-65535, matching dw.h's dwread() -
*         see dwonewrite.asm's identical note on its own Y)
*
* Exit:
*    CC = Z set on success, cleared on timeout (this transport has no
*         framing-error concept of its own, so C is always cleared)
*    Y  = checksum - a running 16-bit add-with-carry over the bytes
*         received, computed the same way the bit-banger DWRead computes
*         it, because hdbdos's own DW3 packet code (HREAD in hdbdos.asm)
*         sends this value back to the server to verify and does not know
*         which transport produced it
*    U is preserved
*    All others clobbered
*
* Wire protocol (must exactly match plugins/user/drivewire):
*    "!DWRECV!" knock, then two count bytes (16-bit, high byte first - see
*    dwonewrite.asm's identical encoding).  Each byte then carries a toggle,
*    T0 = $02 for the first, alternating with T1 = $03.  For byte N with
*    toggle T: wait until the status address ($C100) reads T, read the data
*    address ($C101), then acknowledge by reading $C100+T ($C102 or $C103).
*    The plugin publishes byte N+1 only once it sees that acknowledgement.
*
*    Why a toggle rather than a ready flag: with a flag, this loop could get
*    back to polling the status before the plugin had cleared it, and take
*    the same byte twice - which a double-speed CoCo 3 did, desyncing every
*    session.  Waiting for the value to change cannot double-collect, however
*    fast this loop runs.  Why the separate acknowledgement address, re-read
*    on every pass of the next byte's poll: the plugin can miss a single bus
*    read, and a missed one-shot acknowledgement would leave both sides
*    waiting forever.  Re-reading the previous byte's address while polling
*    for the next makes a missed acknowledgement cost one more pass instead.
*
*    The toggles being the acknowledgement addresses' own low bytes is what
*    lets one register do both jobs below (EORB #1 flips it, LDA B,U reads
*    its address), and the plugin depends on those exact values.  Until the
*    plugin publishes the first byte, $C100 still holds this ROM's own byte
*    there, which must not be T0.
*
* CoCo 3: switches the cartridge in and out around its signalling reads,
*    and patches its stack copy's leave address, exactly as DWWrite does -
*    see dwonewrite.asm's "CoCo 3" note for why and how.

* How long DWRead waits for any one byte before giving up, in units of 256
* passes of the 23-cycle poll loop below: 212 x 256 x 23 = ~1.25M cycles,
* 1.4s at the CoCo's normal 0.89MHz and 0.7s at double speed - the same idle
* timeout as the bit-banger DWRead (dwread.asm), which HDB-DOS's callers are
* written around.  Without one, a request the server never answered - an
* ESP32 still booting at power-on lost it - left the CoCo polling forever,
* and HDB-DOS's own boot-probe retry never got a failure to retry on.
DWRTIMEOUT  equ       212

DWRead      pshs      cc                 ; saved for its IRQ mask bits only -
            pshs      u                  ; the returned CC comes from tsta
                                          ; below, not from this save
            orcc      #$50               ; mask interrupts - see
                                          ; dwonewrite.asm
            ldu       #DWRCODEEND
cpydwr@     lda       ,-u
            pshs      a
            cmpu      #DWRCODE
            bne       cpydwr@
            ldb       >$FFFC             ; CoCo 3? - see dwonewrite.asm
            cmpb      #$FE
            bne       dwrnoc3b@
            lda       #$DF               ; leave = $FFDF (RAM mode); +2 is
            sta       DWRLEAVE+2-DWRCODE,s ; the low byte of each operand
            sta       DWRTOLEAVE+2-DWRCODE,s
dwrnoc3b@   jsr       ,s
            leas      DWRCODELEN,s
            puls      u
            puls      cc                 ; restores the original IRQ mask
                                          ; bits; A (the blob's status: 0,
                                          ; or 1 on a DWRTIMEOUT timeout)
                                          ; is untouched by this
            tsta                         ; set Z from the status byte
            andcc     #$FE               ; clear C - no framing error here
            rts

* Position-independent body - copied onto the stack and run from there.
* Entered with X/Y as DWRead's own entry parameters, U free to use.
DWRCODE     sta       >$FFDE             ; cartridge in - see dwonewrite.asm
            ldu       #$C100             ; signalling page - see
                                          ; dwonewrite.asm and
                                          ; plugins/user/drivewire

            ldb       #'!
            lda       b,u
            ldb       #'D
            lda       b,u
            ldb       #'W
            lda       b,u
            ldb       #'R
            lda       b,u
            ldb       #'E
            lda       b,u
            ldb       #'C
            lda       b,u
            ldb       #'V
            lda       b,u
            ldb       #'!
            lda       b,u

            tfr       y,d                ; D = count (A=high byte, B=low)
            pshs      b                  ; save low byte across the high
                                          ; byte's send below
            tfr       a,b
            lda       b,u                ; send high byte of count
            puls      b                  ; restore low byte
            lda       b,u                ; send low byte of count

            pshs      x,y                ; buffer start and count, for the
                                          ; checksum pass below
            ldb       #$03               ; B = previous byte's toggle (T1),
                                          ; so the first byte's is T0

* B carries the previous byte's toggle into each pass - see "Wire protocol"
* above for the handshake.  A and the byte at 0,s count down DWRTIMEOUT
* while polling.  No blank lines from here to the BNE DWRLOOP below - lwasm
* starts a new local-label scope at each one.
DWRLOOP     sta       >$FFDE             ; cartridge in (already in on the
                                          ; first pass)
            lda       #DWRTIMEOUT
            pshs      a                  ; outer timeout count, at 0,s
            clra                         ; inner count - 256 passes
dwrpoll@    tst       b,u                ; re-acknowledge the previous byte
                                          ; ($C100+B) - recovers an
                                          ; acknowledgement the plugin missed
            eorb      #1                 ; B = this byte's toggle
            cmpb      ,u                 ; status ($C100) showing it yet?
            beq       dwrgot@
            eorb      #1                 ; no - back to the previous toggle
            deca
            bne       dwrpoll@
            dec       ,s
            bne       dwrpoll@
            leas      1,s                ; timed out - drop the count,
DWRTOLEAVE  sta       >$FFDE             ; leave - patched like DWRLEAVE
            puls      x,y                ; drop start and count,
            lda       #1                 ; and fail: status nonzero is Z
            rts                          ; clear in DWRead's TSTA - timeout
dwrgot@     leas      1,s                ; drop the timeout count
            lda       $01,u              ; the data byte ($C101)
            tst       b,u                ; acknowledge it ($C100+B) - a read
                                          ; that leaves A alone
DWRLEAVE    sta       >$FFDE             ; leave - patched to $FFDF on a
                                          ; CoCo 3, so the buffer access
                                          ; below sees RAM; A is unchanged
            sta       ,x+                ; store it, advance X
            leay      -1,y
            bne       DWRLOOP

* Checksum pass, over the buffer just filled - kept out of the loop above
* so D is free for the toggle there.  Already in RAM mode (DWRLEAVE), so
* the buffer can be anywhere.  Ends with X past the last byte, as the loop
* above always left it.
            puls      x,y                ; X = buffer start, Y = count
            ldd       #0                 ; running checksum
dwrsum@     addb      ,x+
            adca      #0
            leay      -1,y
            bne       dwrsum@

            tfr       d,y                ; Y = checksum
            clra                         ; status = success (also sets Z,
                                          ; which the trampoline re-asserts
                                          ; via tsta after restoring CC)
            rts
DWRCODEEND  equ       *
DWRCODELEN  equ       DWRCODEEND-DWRCODE
