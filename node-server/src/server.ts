import * as net from 'net'

const PORT = 10001
const IP = '127.0.0.1'
const BACKLOG = 100

interface Goto {
    length: number,
    type: number,
    timestamp: Date,
    ra: number,
    dec: number
}

interface Equatorial {
    ra: number;
    dec: number;
}

interface Horizontal {
    alt: number;
    az: number;
}

/**
 * Implements message serialisation for MessageCurrentPosition of Stellarium Telescope Protocol version 1.0
 * see http://svn.code.sf.net/p/stellarium/code/trunk/telescope_server/stellarium_telescope_protocol.txt
 * @param position in Equatorial coordinates (RA/DEC)
 * @returns the buffer with the serialized data (length 24)
 */
function packCurrentPosition(position: Equatorial): Buffer {
    const length = 24;
    const buf = Buffer.alloc(length);

    // serialize in little endian byte order
    buf.writeInt16LE(length, 0);    // LENGTH
    buf.writeInt16LE(0, 2);     // TYPE
    buf.writeBigInt64LE(BigInt(Date.now() * 1000), 4);  // TIME
    buf.writeUInt32LE(position.ra * (0x80000000 / 12.), 12);     // RA; right ascension of the telescope (J2000)
    buf.writeInt32LE(position.dec * (0x40000000 / 90.), 16);     // DEC; declination of the telescope (J2000)
    buf.writeInt32LE(0, 20);    // STATUS (0=OK)

    return buf;
}

/**
 * Serializes 'MessageGoto' of Stellarium Telescope Protocol version 1.0
 * byte order is little endian
 * see http://svn.code.sf.net/p/stellarium/code/trunk/telescope_server/stellarium_telescope_protocol.txt
 * @param input the raw buffer as received via the socket connection
 * @returns an object with all properties
 */
function unpackGoto(input: Buffer): Goto {
    return {
        length: input.readInt16LE(0),
        type: input.readInt16LE(2),
        timestamp: new Date(Number(input.readBigUInt64LE(4) / BigInt(1000))),
        ra: input.readUInt32LE(12) / (0x80000000 / 12.),
        dec: input.readInt32LE(16) / (0x40000000 / 90.)
    };
}


class Telescope {
    position: Equatorial = { 
        ra: 0,  // hours
        dec: 0  // degrees
    };

    offset: Equatorial = { 
        ra: 0,  // hours
        dec: 0  // degrees
    };

    getCorrected(): Equatorial {
        return {
            ra: this.position.ra + this.offset.ra,
            dec: this.position.dec + this.offset.dec,
        }
    }

    setPosition(newPos: Equatorial) {
        this.position = newPos;
    }

    calibrate(reference: Equatorial) {
        this.offset.ra = reference.ra - this.position.ra;
        this.offset.dec = reference.dec - this.position.dec;
        console.log(`raOffset=${this.offset.ra}, decOffset=${this.offset.dec}`)
    }

    toHorizontalPosition(): Horizontal {
        let altAz: Horizontal = { alt: 0, az: 0 }

        return altAz;
    }

    fromHorizontalPosition(azimuth: number, altitude: number, latitude: number, localSiderealTimeDegrees: number) {
        const az = this.rad(azimuth);
        const el = this.rad(altitude);
        const phi = this.rad(latitude);

        const sa = Math.sin(az);
        const ca = Math.cos(az);
        const se = Math.sin(el);
        const ce = Math.cos(el);
        const sp = Math.sin(phi);
        const cp = Math.cos(phi);

        const x = - ca * ce * sp + se * cp;
        const y = - sa * ce;
        const z = ca * ce * cp + se * sp;

        const r = Math.sqrt(x * x + y * y);
        const ha = (r != 0.0) ? Math.atan2(y, x) : 0.0;
        this.position.dec = this.deg(Math.atan2(z, r));
        this.position.ra = this.degToHours(localSiderealTimeDegrees - this.deg(ha));
        if(this.position.ra < 0) {
            this.position.ra += 24;
        }
        // console.log(`setting to ra=${this.position.ra}, dec=${this.position.dec}`);
    }

    rad(degrees: number): number {
        return (degrees * Math.PI / 180);
    }

    deg(radians: number): number {
        return (radians / Math.PI * 180);
    }

    degToHours(degrees: number): number {
        return(degrees/360*24);
    }
};


function onConnect(socket: net.Socket) {

    console.log(`Connection from '${socket.remoteAddress}'`);
    const telescope = new Telescope();
    telescope.fromHorizontalPosition(135.54, 52.71, 48, 32.166);

    telescope.offset = {
        ra: -0.018416198265134298,
        dec: -0.02824170708658258
    }

    const timer = setInterval(() => {
        socket.write(packCurrentPosition(telescope.getCorrected()), (err) => {
            if (err) console.log(err);
        });
    }, 1000);

    socket.on('end', () => {
        clearInterval(timer);
        console.log(`Connection to '${socket.remoteAddress}' closed.`);
    });

    socket.on('data', (data: Buffer) => {
        if (data.length >= 20) {
            const message = unpackGoto(data);

            console.log(`Received ${data.length} Bytes: {`
                + `length: ${message.length.toString()}, `
                + `type: ${message.type.toString()}, `
                + `time: ${message.timestamp.toLocaleTimeString()}, `
                + `ra: ${message.ra.toFixed(4).toString()}h, `
                + `dec: ${message.dec.toFixed(4).toString()}°, `
                + `}`);

            // telescope.setPosition({ dec: message.dec, ra: message.ra });
            // telescope.calibrate({ dec: message.dec, ra: message.ra });

            console.log(telescope.toHorizontalPosition());
        }
        else {
            console.log(`Received ${data.length} Bytes: ${data.toString('hex')}`)
        }
    });
}

const server = net.createServer();
server.addListener('connection', onConnect);
server.listen(PORT, IP, BACKLOG, () => {
    console.log(`awaiting connection on port ${PORT.toString()}...`);
});
