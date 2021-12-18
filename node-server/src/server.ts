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
    position: Equatorial = { ra: 0, dec: 0 };

    setPosition(newPos: Equatorial) {
        this.position = newPos;
    }
};


function onConnect(socket: net.Socket) {

    console.log(`Connection from '${socket.remoteAddress}'`);
    const telescope = new Telescope();
    let counter = 0;

    // insert fake movement
    const timer = setInterval(() => {
        counter++;
        telescope.position.ra = Math.sin(counter / 10) + 1;
        socket.write(packCurrentPosition(telescope.position), (err) => {
            if (err) console.log(err);
        });
    }, 200);

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
            telescope.setPosition({ dec: message.dec, ra: message.ra });
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
