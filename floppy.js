function Fdd(num) {
    // Emulated drive state
    this.disk_present = false;
    this.write_protected = true;
    this.phases = [false, false, false, false];
    this.head_half_step = 0; // 0 to 159
    this.selected = false;
    this.drvsel_callback = null;
    this.track_callback = null;

    this._wipeDisk();
}

Fdd.REVOLUTIONS_PER_SEC = 300 / 60;
Fdd.USECS_PER_REVOLUTION = 1000000 / Fdd.REVOLUTIONS_PER_SEC;
Fdd.USECS_PER_BIT_CELL = 4;
Fdd.BIT_CELLS_PER_TRACK = Fdd.USECS_PER_REVOLUTION / Fdd.USECS_PER_BIT_CELL;
Fdd.BYTES_PER_TRACK = Fdd.BIT_CELLS_PER_TRACK / 8;

// 160KB DSK format info
Fdd.DSK_NUM_TRACKS = 40;
Fdd.DSK_SECTORS_PER_TRACK = 16;
Fdd.DSK_BYTES_PER_SECTOR = 256;
// Info to convert DSK to bit cells
Fdd.DSK_SYNCS_PER_GAP2 = 8;
Fdd.DSK_SYNCS_PER_GAP3 = 14;
Fdd.DSK_SECTOR_INTERLEAVE = 3;

Fdd.byteTo4and4 = function(b) {
    return [(b >> 1) | 0xaa, b | 0xaa];
}

Fdd.nibbleToGcr = function(nibble) {
    const n2g = [
        0x96, 0x97, 0x9a, 0x9b, 0x9d, 0x9e, 0x9f, 0xa6, /* 0x00 */
        0xa7, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb2, 0xb3, /* 0x08 */
        0xb4, 0xb5, 0xb6, 0xb7, 0xb9, 0xba, 0xbb, 0xbc, /* 0x10 */
        0xbd, 0xbe, 0xbf, 0xcb, 0xcd, 0xce, 0xcf, 0xd3, /* 0x18 */
        0xd6, 0xd7, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, /* 0x20 */
        0xdf, 0xe5, 0xe6, 0xe7, 0xe9, 0xea, 0xeb, 0xec, /* 0x28 */
        0xed, 0xee, 0xef, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, /* 0x30 */
        0xf7, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff  /* 0x38 */
    ];
    return n2g[nibble];
}

Fdd.gcrToNibble = function(gcr) {
    const g2n = [
        0x00 , 0x01 , false, false, 0x02 , 0x03 , false, 0x04 ,
        0x05 , 0x06 , false, false, false, false, false, false,
        0x07 , 0x08 , false, false, false, 0x09 , 0x0a , 0x0b ,
        0x0c , 0x0d , false, false, 0x0e , 0x0f , 0x10 , 0x11 ,
        0x12 , 0x13 , false, 0x14 , 0x15 , 0x16 , 0x17 , 0x18 ,
        0x19 , 0x1a , false, false, false, false, false, false,
        false, false, false, false, false, 0x1b , false, 0x1c ,
        0x1d , 0x1e , false, false, false, 0x1f , false, false,
        0x20 , 0x21 , false, 0x22 , 0x23 , 0x24 , 0x25 , 0x26 ,
        0x27 , 0x28 , false, false, false, false, false, 0x29 ,
        0x2a , 0x2b , false, 0x2c , 0x2d , 0x2e , 0x2f , 0x30 ,
        0x31 , 0x32 , false, false, 0x33 , 0x34 , 0x35 , 0x36 ,
        0x37 , 0x38 , false, 0x39 , 0x3a , 0x3b , 0x3c , 0x3d ,
        0x3e , 0x3f
    ];
    if (gcr < 0x96 || gcr > 0xff)
        return false;
    return g2n[gcr - 0x96];
}

Fdd.prototype._wipeDisk = function() {
    this.tracks = new Array(40);
    // Fill the 40 tracks with zeros
    for (var t = 0; t < this.tracks.length; t++)
        this.tracks[t] = new Uint8Array(Fdd.BYTES_PER_TRACK);
}

// Read a byte from the bit cell stream.
// Only read the num first bits (stored starting from MSB)
// Returns a byte on success, false on failure.
Fdd.prototype._readByte = function(trk, bitpos, num) {
    if (num > 8 || num < 0 || trk > 40 || trk < 0) {
        console.error("Invalid argument");
        return false;
    }
    bitpos = bitpos % Fdd.BIT_CELLS_PER_TRACK;
    let retval = 0x00;
    const final_shift = 8 - num;
    if ((bitpos & 0x7) == 0) {
        // Byte-aligned read
        const idx = bitpos >> 3;
        retval = this.tracks[trk][idx];
    } else {
        // Unaligned read
        const idx0 = bitpos >> 3;
        const idx1 = (idx0 + 1) > Fdd.BYTES_PER_TRACK ? 0 : (idx0 + 1);
        const shift0 = bitpos & 0x7;
        const shift1 = 8 - shift0;
        const data0 = this.tracks[trk][idx0];
        const data1 = this.tracks[trk][idx1];
        retval = (data0 << shift0) | (data1 >> shift1);
    }
    retval = (retval << final_shift) & 0xff;
    return retval;
}

// Write a byte inside the bit cell stream.
// Only num bits are written (starting from MSB).
Fdd.prototype._writeByte = function(trk, bitpos, data, num) {
    if (num > 8 || num < 0 || trk > 40 || trk < 0) {
        console.error("Invalid argument");
        return false;
    }
    const mask = 0xff & ~((1 << (8 - num)) - 1);
    bitpos = bitpos % Fdd.BIT_CELLS_PER_TRACK;
    if ((bitpos & 0x7) == 0) {
        // Byte-aligned write
        const idx = bitpos >> 3;
        let v = this.tracks[trk][idx];
        v &= ~mask;
        v |= data & mask;
        this.tracks[trk][idx] = v;
    } else {
        // Unaligned write
        const idx0 = bitpos >> 3;
        const idx1 = (idx0 + 1) > Fdd.BYTES_PER_TRACK ? 0 : (idx0 + 1);
        const shift0 = bitpos & 0x7;
        const shift1 = 8 - shift0;
        const mask0 = mask >> shift0;
        const mask1 = mask << shift1;
        const data0 = data >> shift0;
        const data1 = data << shift1;
        let v0 = this.tracks[trk][idx0];
        let v1 = this.tracks[trk][idx1];
        v0 &= ~mask0;
        v1 &= ~mask1;
        v0 |= data0 & mask0;
        v1 |= data1 & mask1;
        this.tracks[trk][idx0] = v0;
        this.tracks[trk][idx1] = v1;
    }
    return true;
}

// Create a byte array containing sync "bytes".
// Returned value is an array:
// - index 0: byte array containing sync "bytes"
// - index 1: number of significant _bits_ in the array (for use with _writeBits)
Fdd.prototype._makeSyncBytes = function(num) {
    const pattern = [0xff, 0x3f, 0xcf, 0xf3, 0xfc];
    const terminator = [0x00, 0x00, 0xc0, 0xf0, 0xfc];
    let needed_bytes = (num * 10) & 0x7 ? ((num + 1) * 10) >> 3 : (num * 10) >> 3;
    let bytes = new Uint8Array(needed_bytes);
    for (let i = 0; i < bytes.length; i++) {
        if (i == (bytes.length - 1)) {
            bytes[i] = terminator[i % 5];
        } else {
            bytes[i] = pattern[i % 5];
        }
    }
    return [bytes, num * 10];
}

// Create an address field.
// Return an array:
// index 0: Array with data
// index 1: Length in bits
Fdd.prototype._makeAddressField = function(vol, trk, sect) {
    const vol_44 = Fdd.byteTo4and4(vol);
    const trk_44 = Fdd.byteTo4and4(trk);
    const sect_44 = Fdd.byteTo4and4(sect);
    const cksum_44 = Fdd.byteTo4and4(vol ^ trk ^ sect);
    let buffer = new Uint8Array(14);
    buffer[0] = 0xd5;
    buffer[1] = 0xaa;
    buffer[2] = 0x96;
    buffer[3] = vol_44[0];
    buffer[4] = vol_44[1];
    buffer[5] = trk_44[0];
    buffer[6] = trk_44[1];
    buffer[7] = sect_44[0];
    buffer[8] = sect_44[1];
    buffer[9] = cksum_44[0];
    buffer[10] = cksum_44[1];
    buffer[11] = 0xde;
    buffer[12] = 0xaa;
    buffer[13] = 0xeb;
    return [buffer, buffer.length << 3];
}

// Create a data field
// Return an array:
// index 0: Array with data
// index 1: Length in bits
Fdd.prototype._makeDataField = function(data) {
    const pattern = [
        // byte0 mask, byte1 mask, shift
        [0xfc, 0x00, 10],
        [0x03, 0xf0, 4],
        [0x0f, 0xc0, 6],
        [0x3f, 0x00, 8],
    ];
    let buf = new Uint8Array(349);
    buf[0] = 0xd5;
    buf[1] = 0xaa;
    buf[2] = 0xad;
    const base = 3;
    for (let srcidx = 0; srcidx < 256; srcidx++) {
        const idx2 = base + (srcidx % 0x56);
        const idx6 = base + 0x56 + srcidx;
        const d8 = data[srcidx];
        const d2 = d8 & 0x3;
        const d6 = d8 >> 2;
        const shift = Math.trunc(srcidx / 0x56) << 1;
        // Store 2 lower bits (bit order is inverted)
        if (d2 == 1) {
            buf[idx2] |= 2 << shift;
        } else if (d2 == 2) {
            buf[idx2] |= 1 << shift;
        } else {
            buf[idx2] |= d2 << shift;
        }
        // Store upper 6 bits
        buf[idx6] = d6;
    }
    let last = 0;
    for (let dstidx = 0; dstidx < 342; dstidx++) {
        const nib = buf[base + dstidx];
        const xor = nib ^ last;
        last = nib;
        buf[base + dstidx] = Fdd.nibbleToGcr(xor);
    }
    buf[345] = Fdd.nibbleToGcr(last);
    buf[346] = 0xde;
    buf[347] = 0xaa;
    buf[348] = 0xeb;
    return [buf, buf.length << 3];
}

// Write sector, returning number of bit cells written
Fdd.prototype._writeSector = function(trk, bitpos, sect, data) {
    const initial_bitpos = bitpos;
    const gap2 = this._makeSyncBytes(Fdd.DSK_SYNCS_PER_GAP2);
    const gap3 = this._makeSyncBytes(Fdd.DSK_SYNCS_PER_GAP3);
    const addr = this._makeAddressField(0, trk, sect);
    const dat = this._makeDataField(data);
    // Write GAP3
    if (!this.writeBits(trk, bitpos, gap3[0], gap3[1]))
        return 0;
    bitpos += gap3[1];
    // Write address field
    if (!this.writeBits(trk, bitpos, addr[0], addr[1]))
        return 0;
    bitpos += addr[1];
    // Write GAP2
    if (!this.writeBits(trk, bitpos, gap2[0], gap2[1]))
        return 0;
    bitpos += gap2[1];
    // Write data field
    if (!this.writeBits(trk, bitpos, dat[0], dat[1]))
        return 0;
    bitpos += dat[1];

    return bitpos - initial_bitpos;
}

Fdd.prototype._writeTrack = function(trk, bitpos, data) {
    let order = new Array(Fdd.DSK_SECTORS_PER_TRACK);
    // Determine sector order on the track
    for (let offset = 0, sect = 0; offset < Fdd.DSK_SECTOR_INTERLEAVE; offset++) {
        for (let idx = offset; idx < Fdd.DSK_SECTORS_PER_TRACK; idx += Fdd.DSK_SECTOR_INTERLEAVE) {
            order[idx] = sect;
            sect++;
        }
    }
    for (let sect of order) {
        const start = sect * Fdd.DSK_BYTES_PER_SECTOR;
        const sectdata = data.slice(start, start + Fdd.DSK_BYTES_PER_SECTOR);
        let written = this._writeSector(trk, bitpos, sect, sectdata);
        if (!written) {
            console.error("Failed to write sector " + sect + " to track " + trk);
            return false;
        }
        bitpos += written;
    }
    return true;
}

Fdd.prototype._convertDskToBitCells = function(dsk) {
    // First erase current disk
    this._wipeDisk();

    for (var t = 0; t < Fdd.DSK_NUM_TRACKS; t++) {
        const start = t * Fdd.DSK_SECTORS_PER_TRACK * Fdd.DSK_BYTES_PER_SECTOR;
        const end = start + (Fdd.DSK_SECTORS_PER_TRACK * Fdd.DSK_BYTES_PER_SECTOR);
        if (!this._writeTrack(t, 0, dsk.slice(start, end))) {
            console.error("Failed to write track " + t + ".");
            return false;
        }
    }
    return true;
}

Fdd.prototype.isDiskPresent = function() {
    return this.disk_present;
}

Fdd.prototype.loadDisk = function(image) {
    if (image.byteLength != 163840) {
        console.error("Not a valid Apple II 160KB image. Expected size: 163840, image file size: " + image.byteLength);
        return false;
    }
    var bytes = new Uint8Array(image);
    this.disk_present = this._convertDskToBitCells(bytes);
    return this.disk_present;
}

Fdd.prototype.ejectDisk = function() {
    this.disk_present = false;
    this._wipeDisk();
}

// Read [num] bits starting at bit position [bitpos].
// Returns an array of 8-bit integers on success, false on failure.
Fdd.prototype.readBits = function(trk, bitpos, num) {
    if (trk < 0 || trk > 40)
        return false;
    const arraylen = (num + 7) >> 3;
    let retval = new Uint8Array(arraylen);
    for (let i = 0; i < arraylen; i++) {
        const n = (num > 8) ? 8 : num;
        const b = this._readByte(trk, bitpos, n);
        if (b === false)
            return false;
        retval[i] = b;
        num -= n;
        bitpos += n;
    }
    if (num != 0) {
        console.error("Internal inconsistency. num should be 0 but is " + num + ".");
        return false;
    }
    return retval;
}

// Write [num] bits from [data] at bit position [bitpos].
// data can be a 8-bit integer or an array of 8-bit integers
Fdd.prototype.writeBits = function(trk, bitpos, data, num) {
    if (trk < 0 || trk > 40)
        return false;
    if (data instanceof Array || data instanceof Uint8Array) {
        if ((data.length * 8) < num)
            return false;
        for (const b of data) {
            if (num > 8) {
                this._writeByte(trk, bitpos, b, 8);
                bitpos += 8;
                num -= 8;
            } else if (num > 0) {
                this._writeByte(trk, bitpos, b, num);
                bitpos += num;
                num = 0;
            } else {
                console.error("Internal error. num shouldn't be 0 yet.");
                return false;
            }
        }
        return true;
    } else if (data instanceof Number) {
        return this._writeByte(trk, bitpos, data, num);
    } else {
        console.error("Invalid data type.");
        return false;
    }
    return true;
}

// Get the current track number
Fdd.prototype.getCurrentTrack = function() {
    let track = (this.head_half_step + 2) >> 2;
    if (track > 39)
        track = 39;
    return track;
}

Fdd.prototype.setPhases = function(phy0, phy1, phy2, phy3) {
    let p = [phy0, phy1, phy2, phy3];
    let ohs = this.head_half_step;
    let nhs = ohs;
    let num_active = 0;
    if (!this.selected)
        return; // Drive not selected
    if (phy0) num_active++;
    if (phy1) num_active++;
    if (phy2) num_active++;
    if (phy3) num_active++;

    if (num_active == 0 || num_active > 2)
        return; // Not enough torque

    let now = (ohs >> 1) & 0x3;
    let cw = ((ohs >> 1) + 1) & 0x3; // Index of next phase in clockwise direction
    let ccw = ((ohs >> 1) + 3) & 0x3; // Index of next phase on counter-clockwise direction
    if (ohs & 1 == 1) {
        // Aligned between two phases
        if (p[cw] && !p[ccw]) {
            // Moving clockwise
            if ((p[cw] + 1) & 0x3) {
                nhs += 2;
            } else {
                nhs += 1;
            }
        } else if (!p[cw] && p[ccw]) {
            // Moving counter-clockwise
            if ((p[ccw] - 1) & 0x3) {
                nhs -= 2;
            } else {
                nhs -= 1;
            }
        }
    } else {
        // Aligned on phase
        if (p[cw] && !p[ccw]) {
            // Moving clockwise
            if (now) {
                nhs += 1;
            } else {
                nhs += 2;
            }
        } else if (!p[cw] && p[ccw]) {
            // Moving counter-clockwise
            if (now) {
                nhs -= 1;
            } else {
                nhs -= 2;
            }
        }
    }

    // Do not go further than the stops
    if (nhs > 159) {
        nhs = 159;
    } else if (nhs < 0) {
        nhs = 0;
    }
    this.head_half_step = nhs;
    if (ohs != nhs && this.track_callback) {
        this.track_callback(this.getCurrentTrack());
    }
}

Fdd.prototype.setDrvselCallback = function(fn) {
    this.drvsel_callback = fn;
    if (this.drvsel_callback) {
        this.drvsel_callback(this.selected);
    }
}

Fdd.prototype.setTrackCallback = function(fn) {
    this.track_callback = fn;
    if (this.track_callback) {
        this.track_callback(this.getCurrentTrack());
    }
}

Fdd.prototype.getSelected = function() {
    return this.selected;
}

Fdd.prototype.setSelected = function(enable) {
    if (this.selected == enable)
        return;
    this.selected = enable;
    if (!enable) {
        this.phases.forEach((e,i) => this.phases[i] = false);
    }
    if (this.drvsel_callback)
        this.drvsel_callback(enable);
}

function Fdc() {
    // State set by CPU
    this.enable = false;
    this.unit2_select = false;
    this.side2_select = false;
    this.writereq = false;
    this.phy0 = false;
    this.phy1 = false;
    this.phy2 = false;
    this.phy3 = false;

    // Controller FIFO state
    this.fiforeg = 0;
    this.fifo_ready = false;

    // Internal shift register
    this.intreg = 0;

    // Internal state
    this.state = Fdc.STATE_IDLE;
    this.last_update_us = 0;
    this.subcell_accumulator = 0; // Accumulator for the remainder in usec to bitcells division
    this.bitpos = 0;
    this.drives = [null, null];

    // Read state
    this.read_counter = 0;
}

Fdc.STATE_IDLE = 0;
Fdc.STATE_READ = 1;
Fdc.STATE_WRITE = 2;

// Sync the read counter to the first 1 bit cell. Consume at most [max_bits].
// Return the number of bits consumed.
// Updates: this.bitpos, this.read_counter, this.intreg
Fdc.prototype._syncRead = function(drv, max_bits) {
    if (this.read_counter > 0)
        return 0;
    const trk = drv.getCurrentTrack();
    let count = 0;
    let b = 0; // 8 bit cells from disk, updated every 8 iterations
    while(count < max_bits) {
        if ((count & 0x7) == 0) {
            let a = drv.readBits(trk, this.bitpos, 8);
            b = a[0];
        }
        this.bitpos++;
        count++;
        if (b & 0x80) {
            this.read_counter = 7;
            this.intreg = ((this.intreg << 1) | 1) & 0xff;
            break;
        }
        b <<= 1;
    }
    return count;
}

// Try to read one byte from the disk into the internal shift register.
// Read no more than [max_bits], or the remaining read bit counter.
// Return the number of bits consumed.
// Updates: this.bitpos, this.read_counter, this.intreg, this.fifo_ready
Fdc.prototype._readByte = function(drv, max_bits) {
    if (!this.read_counter)
        return 0; // Not synced
    const trk = drv.getCurrentTrack();
    let count = 0;
    let b = drv.readBits(trk, this.bitpos, 8);
    while(this.read_counter > 0 && count < max_bits) {
        this.read_counter--;
        count++;
        this.bitpos++;
        let bit = (b & 0x80) ? 1 : 0;
        this.intreg = ((this.intreg << 1) | bit) & 0xff;
        b <<= 1;
    }
    // Latch fifo register when byte is complete
    if (this.read_counter == 0) {
        this.fiforeg = this.intreg;
        this.fifo_ready = true;
    }
    return count;
}

Fdc.prototype._updateIdle = function(delta_us) {
    // Well, do nothing, the FDC is idle.
}

Fdc.prototype._updateRead = function(delta_us) {
    const initial_delta_us = delta_us;
    const drv = this.drives[this.unit2_select ? 1 : 0];
    if (drv === null || !drv.isDiskPresent())
        return; // No drive/disk. Nothing happens.

    // Don't waste time computing the new state for more than one revolution.
    if (delta_us > Fdd.USECS_PER_REVOLUTION) {
        delta_us = Fdd.USECS_PER_REVOLUTION;
        // Update initial bitpos
        this.bitpos += initial_delta_us % (Fdd.BIT_CELLS_PER_TRACK * Fdd.USECS_PER_BIT_CELL);
        this.bitpos %= Fdd.BIT_CELLS_PER_TRACK;
    }
    let bits_left = Math.trunc(delta_us / Fdd.USECS_PER_BIT_CELL);
    // Accumulate sub-bitcell time
    const subcell_acc = this.subcell_accumulator + (delta_us % Fdd.USECS_PER_BIT_CELL);
    if (subcell_acc >= Fdd.USECS_PER_BIT_CELL) {
        bits_left++;
        this.subcell_accumulator = subcell_acc - Fdd.USECS_PER_BIT_CELL;
    } else {
        this.subcell_accumulator = subcell_acc;
    }

    while(bits_left) {
        // Sync if needed
        bits_left -= this._syncRead(drv, bits_left);
        // Read to shift buffer
        bits_left -= this._readByte(drv, bits_left);
    }
}

Fdc.prototype._updateWrite = function(delta_us) {
    // TODO
}

Fdc.prototype._update = function(time) {
    const delta_us = time - this.last_update_us;
    if (this.state == Fdc.STATE_IDLE) {
        this._updateIdle(delta_us);
    } else if (this.state == Fdc.STATE_READ) {
        this._updateRead(delta_us);
    } else if (this.state == Fdc.STATE_WRITE) {
        this._updateWrite(delta_us);
    } else {
        console.error("Invalid FDC state: " + this.state);
        return false;
    }
    this.last_update_us = time;
}

Fdc.prototype._transition = function() {
    if (this.state == Fdc.STATE_IDLE) {
        if (this.enable && !this.writereq) { // IDLE -> READ
            this.read_counter = 0;
            this.fifo_ready = false;
            this.subcell_counter = 0;
            this.state = Fdc.STATE_READ;
            if (this.drives[0])
                this.drives[0].setSelected(!this.unit2_select);
            if (this.drives[1])
                this.drives[1].setSelected(this.unit2_select);
            const drv = this.unit2_select ? this.drives[1] : this.drives[0];
            if (drv)
                drv.setPhases(this.phy0, this.phy1, this.phy2, this.phy3);
        } else if (this.enable && this.writereq) { // IDLE -> WRITE
            console.warn("Write support is not implemented!");
            this.state = Fdc.STATE_WRITE;
        }
    } else if (this.state == Fdc.STATE_READ) {
        if (!this.enable) { // READ -> IDLE
            this.read_counter = 0;
            this.subcell_counter = 0;
            this.state = Fdc.STATE_IDLE;
            if (this.drives[0])
                this.drives[0].setSelected(false);
            if (this.drives[1])
                this.drives[1].setSelected(false);
        } else if (this.writereq) { // READ -> WRITE

        } else { // No change

        }
    } else if (this.state == Fdc.STATE_WRITE) {
    }
}

Fdc.prototype.ioRead = function(port, time) {
    //console.log(`FDC read port ${hex(port)}h, pc=${hex(cpu.getState().pc,4)}h`);
    let val = 0x00;
    this._update(time);
    switch(port) {
        case 0x10:
        case 0x11:
            console.log(`Attempting to read write-only port ${hex(port)}h`);
            break;
        case 0x12:
            if (fdc.fifo_ready)
                val |= 0x80;
            val |= 0x01; // Write-protect by default
            if (this.enable) {
                if (this.unit2_select && this.drives[1] && !this.drives[1].write_protected) {
                    val &= ~0x01;
                } else if (this.drives[0] && !this.drives[0].write_protected) {
                    val &= ~0x01;
                }
            }
            break;
        case 0x13:
            val = fdc.fiforeg;
            if (this.state == Fdc.STATE_READ) {
                this.fifo_ready = false;
            }
            break;
    }
    this._transition();
    return val;
}

Fdc.prototype.ioWrite = function(port, value, time) {
    //console.log(`FDC write port ${hex(port)}h value ${hex(value)}h, pc=${hex(cpu.getState().pc,4)}h`);
    this._update(time);
    switch(port) {
        case 0x10:
            const old_drvsel = this.unit2_select;
            this.phy0 = (value & 0x1) != 0;
            this.phy1 = (value & 0x2) != 0;
            this.phy2 = (value & 0x4) != 0;
            this.phy3 = (value & 0x8) != 0;
            this.enable = (value & 0x10) != 0;
            this.unit2_select = (value & 0x20) != 0;
            this.writereq = (value & 0x40) == 0;
            this.side2_select = (value & 0x80) != 0;
            // Handle drvsel change while not IDLE
            if (this.state != Fdc.STATE_IDLE && (this.unit2_select != old_drvsel)) {
                if (this.drives[0])
                    drives[0].setSelected(!this.unit2_select);
                if (this.drives[1])
                    drives[1].setSelected(this.unit2_select);
            }
            // Set stepper phases on drive
            if (this.state != Fdc.STATE_IDLE) {
                if (!this.unit2_select && this.drives[0]) {
                    this.drives[0].setPhases(this.phy0, this.phy1, this.phy2, this.phy3);
                } else if (this.unit2_select && this.drives[1]) {
                    this.drives[1].setPhases(this.phy0, this.phy1, this.phy2, this.phy3);
                }
            }
            break;
        case 0x11:
            const mode16 = (value & 0x1) != 0;
            if (mode16) {
                console.warn("16 bit mode is not implemented!");
            }
            break;
        case 0x12:
            break;
        case 0x13:
            fdc.fiforeg = value;
            if (this.state == Fdc.STATE_WRITE) {
                this.fifo_ready = false;
            }
            break;
    }
    this._transition();
}

Fdc.prototype.setDrive = function(idx, drive) {
    if (idx !== 0 && idx !== 1) {
        console.error("Invalid drive index: " + idx.toString());
        return;
    }
    if (!drive instanceof Fdd) {
        console.error("drive is not a Fdd instance");
        return;
    }
    this.drives[idx] = drive;
}

Fdc.prototype.getDrive = function(idx) {
    if (idx !== 0 && idx !== 1) {
        console.error("Invalid drive index: " + idx.toString());
        return;
    }
    return this.drives[idx];
}
