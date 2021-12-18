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
function packCurrentPosition(position) {
    const length = 24;
    const buf = Buffer.alloc(length);
    buf.writeInt16LE(length, 0); // LENGTH
    buf.writeInt16LE(0, 2); // TYPE
    buf.writeBigInt64LE(BigInt(0), 4); // TIME
    buf.writeUInt32LE(position.ra); // RA; right ascension of the telescope (J2000)
    buf.writeInt32LE(position.dec); // DEC; declination of the telescope (J2000)
    buf.writeInt32LE(0); // STATUS
    return buf;
}
function unpackGoto(input) {
    return {
        length: input.readInt16LE(0),
        type: input.readInt16LE(2),
        timestamp: new Date(Number(input.readBigUInt64LE(4) / BigInt(1000))),
        ra: input.readUInt32LE(12) / (0x80000000 / 12.),
        dec: input.readInt32LE(16) / (0x40000000 / 90.)
    };
}
net.createServer()
    .listen(PORT, IP, BACKLOG)
    .on('connection', socket => socket
    .on('data', buffer => {
    if (buffer.length >= 20) {
        const message = unpackGoto(buffer);
        console.log(`Received ${buffer.length} Bytes: {`
            + `length: ${message.length.toString()}, `
            + `type: ${message.type.toString()}, `
            + `time: ${message.timestamp.toLocaleTimeString()}, `
            + `ra: ${message.ra.toFixed(4).toString()}h, `
            + `dec: ${message.dec.toFixed(4).toString()}°, `
            + `}`);
        socket.write(packCurrentPosition({ ra: message.ra, dec: message.dec }));
    }
    else {
        console.log(`Received ${buffer.length} Bytes: ${buffer.toString('hex')}`);
    }
    socket.end();
}));
