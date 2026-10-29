*******************************************************
*
* DWRead - One ROM DriveWire bridge Edition
*    Receive a response from the DriveWire server via One ROM's drivewire
*    plugin.  See dwonewrite.asm for why this runs from the stack rather
*    than from ROM, and plugins/user/drivewire for the device side of this
*    protocol.
*
*    Unlike the bit-banger DWRead, there are no leading sync bytes to hunt
*    for: the plugin's ready signal (below) is already an explicit,
*    unambiguous "response here" marker, so the sync scan that transport
*    needed to find the start of a response does not apply here.
*
* Entry:
*    X  = storage address for incoming data
*    Y  = number of bytes requested, 1-256 - the plugin's whole-response
*         buffer is 256 bytes, and it refuses a larger count
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
*    dwonewrite.asm's identical encoding).  The plugin collects the whole
*    response from the server, lays it out in a 256-byte window at $E000,
*    and only then sets the status address ($C100) to DWRREADY.  The CoCo
*    polls $C100 for that, copies the window into the buffer at full bus
*    speed, and acknowledges by reading DWRACK ($C100+DWRREADY), after
*    which the plugin restores $C100.  One wait per response instead of a
*    handshake per byte, which cost ~100 cycles a byte.
*
*    Until the response is ready, the plugin flips $C100 between $03 and
*    $01 as a heartbeat - see DWRTIMEOUT.  Before the plugin's first write,
*    $C100 still holds this ROM's own byte there, which must not be
*    DWRREADY; the plugin checks that at startup.
*
*    The window sits in the 27128's upper 8K, which serves a duplicate of
*    (or padding after) this 8K image and holds no live code, so the plugin
*    leaves the response there afterwards rather than spend time restoring
*    it.
*
* Stack blasting:
*    PULU A,B,DP,X,Y / PSHS A,B,DP,X,Y moves 7 bytes in 24 cycles, with S
*    as the destination pointer - interrupts are masked throughout, so
*    nothing else uses S meanwhile.  PSHS fills downwards while PULU reads
*    upwards, so the plugin stores the 7-byte chunks in reverse order:
*    with R = count mod 7, the window holds the first R bytes, then the
*    buffer's last 7, then the 7 before those, and so on back to byte R.
*    The R head bytes are copied singly, upwards from the buffer start,
*    and the chunks blasted downwards from the buffer end, so nothing is
*    ever written outside the buffer.  CC stays out of the register list,
*    so the interrupt mask is never loaded from data.
*
* CoCo 3: switches the cartridge in and out around its signalling reads,
*    and patches its stack copy's leave address, exactly as DWWrite does -
*    see dwonewrite.asm's "CoCo 3" note for why and how.  A blast reads the
*    window in ROM mode and writes the buffer in the same mode, which is
*    only known to be safe below $8000, so a buffer reaching $8000 or above
*    on a CoCo 3 takes the slow path instead: the same layout, copied a
*    byte at a time with a switch to RAM mode around each store.

* How long DWRead waits with the status byte unchanged before giving up, in
* passes of the 22-cycle poll loop below: 56820 x 22 = ~1.25M cycles, 1.4s
* at the CoCo's normal 0.89MHz and 0.7s at double speed - the same idle
* timeout as the bit-banger DWRead (dwread.asm), which HDB-DOS's callers
* are written around.  Without one, a request the server never answered -
* an ESP32 still booting at power-on lost it - left the CoCo polling
* forever, and HDB-DOS's own boot-probe retry never got a failure to retry
* on.
*
* "Unchanged" is the point: while the plugin is waiting on a slow reply, or
* receiving a cartridge push, it flips the status byte between $03 and $01
* as a heartbeat, and every change restarts the count.  So the CoCo waits as
* long as the plugin is visibly working and gives up ~1.4s after it isn't.
* A fixed timeout couldn't do both: at double speed a CoCo 3 gave up at 0.7s
* on a MOUNT IMAGE whose reply came ~1.0s later, behind the push it triggers.
DWRTIMEOUT  equ       56820
DWRREADY    equ       $02                ; status value: response laid out
DWRACK      equ       $C100+DWRREADY     ; read to acknowledge the response
DWRWINDOW   equ       $E000              ; the response, laid out as above

DWRead      pshs      cc                 ; saved for its IRQ mask bits only -
            pshs      u                  ; the returned CC comes from tsta
                                          ; below, not from this save
            orcc      #$50               ; mask interrupts - see
                                          ; dwonewrite.asm
            pshs      x,y                ; buffer start and count, for the
                                          ; patching and checksum below
* Copy the body onto the stack by the same stack blast as the response,
* 7 bytes at a time, last chunk first so PSHS lays it down in order
* (DWRCODELEN is a multiple of 7 - see the pad at DWRCODEEND).
            pshs      dp                 ; PULU loads DP from code bytes
            ldu       #DWRCODEEND-7
cpydwr@     pulu      a,b,dp,x,y
            pshs      a,b,dp,x,y
            leau      -14,u
            cmpu      #DWRCODE-7
            bne       cpydwr@
            lda       DWRCODELEN,s       ; DP back, from under the copy
            tfr       a,dp
* Patch the stack copy - see the labels named.  DP, X and Y were stacked
* above the copy, at DWRCODELEN,s, DWRCODELEN+1,s and DWRCODELEN+3,s.
            ldd       DWRCODELEN+3,s     ; B = count mod 7 - at most 37
dwrmod@     subd      #7                 ; passes for a 256-byte count
            bcc       dwrmod@
            addb      #7
            stb       DWRHEAD+1-DWRCODE,s
            ldd       DWRCODELEN+3,s
            addd      #DWRWINDOW
            std       DWRWEND+2-DWRCODE,s
            ldd       DWRCODELEN+1,s
            addd      DWRCODELEN+3,s
            std       DWRBEND+2-DWRCODE,s
            ldb       >$FFFC             ; CoCo 3? - see dwonewrite.asm
            cmpb      #$FE
            bne       dwrgo@
            lda       #$DF               ; leave = $FFDF (RAM mode); +2 is
            sta       DWRLEAVE+2-DWRCODE,s ; the low byte of each operand
            sta       DWRTOLEAVE+2-DWRCODE,s
            ldd       DWRBEND+2-DWRCODE,s
            cmpd      #$8000             ; buffer ends at or below $8000?
            bls       dwrgo@
            lda       #$20               ; no - BRN becomes BRA, the slow
            sta       DWRPATH-DWRCODE,s  ; path
dwrgo@      ldx       DWRCODELEN+1,s     ; the body's entry X and Y - the
            ldy       DWRCODELEN+3,s     ; copy above clobbered both
            jsr       ,s
            leas      DWRCODELEN+1,s     ; the copy, and the DP saved above
            puls      x,y                ; buffer start and count
            tsta                         ; timed out?
            bne       dwrout@            ; yes - no checksum to make
* Checksum pass, over the buffer just filled, run in place: the session is
* over, so instruction fetches can no longer confuse the plugin, and on a
* CoCo 3 the body has left the SAM in RAM mode, so the buffer can be
* anywhere.
            ldd       #0                 ; running checksum
dwrsum@     addb      ,x+
            adca      #0
            leay      -1,y
            bne       dwrsum@
            tfr       d,y                ; Y = checksum
            clra                         ; status = success
dwrout@     puls      u
            puls      cc                 ; restores the original IRQ mask
                                          ; bits; A (the status: 0, or 1 on
                                          ; a DWRTIMEOUT timeout) is
                                          ; untouched by this
            tsta                         ; set Z from the status byte
            andcc     #$FE               ; clear C - no framing error here
            rts

* Position-independent body - copied onto the stack and run from there.
* Entered with X/Y as DWRead's own entry parameters, U free to use, and
* DWRHEAD, DWRBEND, DWRWEND, DWRPATH, DWRLEAVE and DWRTOLEAVE patched.
* Returns A = 0 on success, 1 on timeout.  No blank lines from here to
* DWRCODEEND - lwasm starts a new local-label scope at each one.
DWRCODE     sta       >$FFDE             ; cartridge in - see dwonewrite.asm
            ldu       #$C100             ; signalling page - see
                                          ; dwonewrite.asm and
                                          ; plugins/user/drivewire
* The knock, each byte as a read of $C100 plus the byte - the same address
* LDA B,U gives, as a constant offset (every knock byte is below $80).
            lda       '!,u
            lda       'D,u
            lda       'W,u
            lda       'R,u
            lda       'E,u
            lda       'C,u
            lda       'V,u
            lda       '!,u
            tfr       y,d                ; D = count (A=high byte, B=low)
            pshs      b                  ; save low byte across the high
                                          ; byte's send below
            tfr       a,b
            lda       b,u                ; send high byte of count
            puls      b                  ; restore low byte
            lda       b,u                ; send low byte of count
* Wait for DWRREADY.  Y is the DWRTIMEOUT count, A the last status seen.
            ldb       #DWRREADY
            ldy       #DWRTIMEOUT
            lda       ,u                 ; status as first seen
dwrpoll@    cmpb      ,u                 ; ready?
            beq       dwrgot@
            cmpa      ,u                 ; changed since last seen? (the
            beq       dwrsame@           ; plugin's heartbeat)
            lda       ,u                 ; yes - note it, and restart the
            ldy       #DWRTIMEOUT        ; timeout
dwrsame@    leay      -1,y
            bne       dwrpoll@
DWRTOLEAVE  sta       >$FFDE             ; timed out - leave, patched like
            lda       #1                 ; DWRLEAVE, and fail: status
            rts                          ; nonzero is Z clear in DWRead's
                                          ; TSTA
* Copy the window into the buffer - see "Stack blasting" above.  X is the
* buffer start, U walks the window.
dwrgot@     ldu       #DWRWINDOW
DWRHEAD     ldb       #0                 ; patched: count mod 7
DWRPATH     brn       dwrslow@           ; patched to BRA for the slow path
            beq       dwrblast@          ; (BRN leaves LDB's Z alone)
dwrhead@    lda       ,u+                ; the head bytes, upwards
            sta       ,x+
            decb
            bne       dwrhead@
dwrblast@   pshs      dp                 ; PULU loads DP from data
            sts       dwrrest@+2,pcr     ; S is the destination from here
DWRBEND     lds       #0                 ; patched: buffer end
DWRWEND     cmpu      #0                 ; patched: window end
            beq       dwrrest@
            pulu      a,b,dp,x,y
            pshs      a,b,dp,x,y
            bra       DWRWEND
dwrrest@    lds       #0                 ; the STS above fills this in
            puls      dp
dwrack@     tst       >DWRACK            ; acknowledge - the plugin
                                          ; restores $C100
DWRLEAVE    sta       >$FFDE             ; leave - patched to $FFDF on a
                                          ; CoCo 3, so DWRead's checksum
                                          ; pass sees RAM
            clra                         ; status = success
            rts
* The slow path: the same layout, a byte at a time with RAM mode around
* each store.  B is still the head count, X the buffer start.
dwrslow@    bsr       dwrcopy@           ; the head bytes, upwards
            ldx       DWRBEND+2,pcr      ; then the chunks, downwards from
dwrsnext@   cmpu      DWRWEND+2,pcr      ; the buffer end
            beq       dwrack@
            leax      -7,x               ; X = this chunk's start
            ldb       #7
            bsr       dwrcopy@
            leax      -7,x               ; back to it, for the next LEAX
            bra       dwrsnext@
dwrcopy@    tstb                         ; copies B bytes from U to X
            beq       dwrcret@
dwrcbyte@   lda       ,u+
            sta       >$FFDF             ; RAM mode for the store
            sta       ,x+
            sta       >$FFDE             ; cartridge back in
            decb
            bne       dwrcbyte@
dwrcret@    rts
            fill      $12,(7-(*-DWRCODE)%7)%7 ; NOP pad to a multiple of 7,
DWRCODEEND  equ       *                  ; for DWRead's 7-byte copy
DWRCODELEN  equ       DWRCODEEND-DWRCODE
