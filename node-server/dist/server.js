"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    Object.defineProperty(o, k2, { enumerable: true, get: function() { return m[k]; } });
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null) for (var k in mod) if (k !== "default" && Object.prototype.hasOwnProperty.call(mod, k)) __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
};
Object.defineProperty(exports, "__esModule", { value: true });
const net = __importStar(require("net"));
const PORT = 10001;
const IP = '127.0.0.1';
const BACKLOG = 100;
/**
 * Implements message serialisation for MessageCurrentPosition of Stellarium Telescope Protocol version 1.0
 * see http://svn.code.sf.net/p/stellarium/code/trunk/telescope_server/stellarium_telescope_protocol.txt
 * @param position in Equatorial coordinates (RA/DEC)
 * @returns the buffer with the serialized data (length 24)
 */
function packCurrentPosition(position) {
    const length = 24;
    const buf = Buffer.alloc(length);
    // serialize in little endian byte order
    buf.writeInt16LE(length, 0); // LENGTH
    buf.writeInt16LE(0, 2); // TYPE
    buf.writeBigInt64LE(BigInt(Date.now() * 1000), 4); // TIME
    buf.writeUInt32LE(position.ra * (0x80000000 / 12.), 12); // RA; right ascension of the telescope (J2000)
    buf.writeInt32LE(position.dec * (0x40000000 / 90.), 16); // DEC; declination of the telescope (J2000)
    buf.writeInt32LE(0, 20); // STATUS (0=OK)
    return buf;
}
/**
 * Serializes 'MessageGoto' of Stellarium Telescope Protocol version 1.0
 * byte order is little endian
 * see http://svn.code.sf.net/p/stellarium/code/trunk/telescope_server/stellarium_telescope_protocol.txt
 * @param input the raw buffer as received via the socket connection
 * @returns an object with all properties
 */
function unpackGoto(input) {
    return {
        length: input.readInt16LE(0),
        type: input.readInt16LE(2),
        timestamp: new Date(Number(input.readBigUInt64LE(4) / BigInt(1000))),
        ra: input.readUInt32LE(12) / (0x80000000 / 12.),
        dec: input.readInt32LE(16) / (0x40000000 / 90.)
    };
}
class Telescope {
    constructor() {
        this.position = { ra: 0, dec: 0 };
    }
    setPosition(newPos) {
        this.position = newPos;
    }
    getHorizontalPosition() {
        let altAz = { alt: 0, az: 0 };
        return altAz;
    }
}
;
function onConnect(socket) {
    console.log(`Connection from '${socket.remoteAddress}'`);
    const telescope = new Telescope();
    let counter = 0;
    const timer = setInterval(() => {
        // insert fake movement
        // counter++;
        // telescope.position.ra = Math.sin(counter / 10) + 1;
        const fakePosition = new Telescope();
        fakePosition.setPosition({ dec: 16.5527, ra: 69.298 / 360 * 24 });
        socket.write(packCurrentPosition(fakePosition.position), (err) => {
            if (err)
                console.log(err);
        });
    }, 200);
    socket.on('end', () => {
        clearInterval(timer);
        console.log(`Connection to '${socket.remoteAddress}' closed.`);
    });
    socket.on('data', (data) => {
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
            telescope.setPosition({ dec: message.dec, ra: message.ra });
            console.log(telescope.getHorizontalPosition());
        }
        else {
            console.log(`Received ${data.length} Bytes: ${data.toString('hex')}`);
        }
    });
}
const server = net.createServer();
server.addListener('connection', onConnect);
server.listen(PORT, IP, BACKLOG, () => {
    console.log(`awaiting connection on port ${PORT.toString()}...`);
});
