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
*    Y  = number of bytes requested (1-256; 256 is a normal 16-bit 256 - see
*         dwonewrite.asm)
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
*    "!DWRECV!" knock, then one count byte (0 means 256).  For each
*    requested byte: poll the status address ($C100) until it reads $FF,
*    then read the data address ($C101) to collect the byte.  The plugin
*    does not prepare the next byte until it has seen this code read the
*    data address, so there is no need to acknowledge that separately.

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
            jsr       ,s
            leas      DWRCODELEN,s
            puls      u
            puls      cc                 ; restores the original IRQ mask
                                          ; bits; A (the blob's status,
                                          ; always 0 - see below) is
                                          ; untouched by this
            tsta                         ; set Z from the status byte
            andcc     #$FE               ; clear C - no framing error here
            rts

* Position-independent body - copied onto the stack and run from there.
* Entered with X/Y as DWRead's own entry parameters, U free to use.
DWRCODE     ldu       #$C100             ; signalling page - see
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

            cmpy      #256
            bne       dwrcnt1@
            clrb                         ; wire encoding: 256 -> byte 0
            bra       dwrcnt2@
dwrcnt1@    tfr       y,d
dwrcnt2@    lda       b,u                ; send count byte

            ldd       #0                 ; running checksum
dwrloop@    pshs      d                  ; the poll below clobbers A, so the
                                          ; checksum can't stay in D across it
dwrpoll@    lda       $00,u              ; poll status ($C100)
            cmpa      #$FF
            bne       dwrpoll@
            lda       $01,u              ; single read of the data byte
                                          ; ($C101) - the plugin treats this
                                          ; read as "byte collected" and
                                          ; will not serve the next one
                                          ; until it sees it, so this must
                                          ; not be read a second time
            sta       ,x+                ; store it, advance X
            puls      d                  ; restore the checksum
            addb      -1,x               ; checksum += the byte just stored
                                          ; (a RAM read, not a second ROM
                                          ; read of the data address)
            adca      #0
            leay      -1,y
            bne       dwrloop@

            tfr       d,y                ; Y = checksum
            clra                         ; status = success (also sets Z,
                                          ; which the trampoline re-asserts
                                          ; via tsta after restoring CC)
            rts
DWRCODEEND  equ       *
DWRCODELEN  equ       DWRCODEEND-DWRCODE
