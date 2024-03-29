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
const IP = '0.0.0.0';
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
        this.position = {
            ra: 0,
            dec: 0 // degrees
        };
        this.offset = {
            ra: 0,
            dec: 0 // degrees
        };
    }
    getCorrected() {
        return {
            ra: this.position.ra + this.offset.ra,
            dec: this.position.dec + this.offset.dec,
        };
    }
    setPosition(newPos) {
        this.position = newPos;
    }
    calibrate(reference) {
        this.offset.ra = reference.ra - this.position.ra;
        this.offset.dec = reference.dec - this.position.dec;
        console.log(`raOffset=${this.offset.ra}, decOffset=${this.offset.dec}`);
    }
    toHorizontalPosition() {
        let altAz = { alt: 0, az: 0 };
        return altAz;
    }
    fromHorizontalPosition(azimuth, altitude, latitude, localSiderealTimeDegrees) {
        const az = this.rad(azimuth);
        const el = this.rad(altitude);
        const phi = this.rad(latitude);
        const sa = Math.sin(az);
        const ca = Math.cos(az);
        const se = Math.sin(el);
        const ce = Math.cos(el);
        const sp = Math.sin(phi);
        const cp = Math.cos(phi);
        const x = -ca * ce * sp + se * cp;
        const y = -sa * ce;
        const z = ca * ce * cp + se * sp;
        const r = Math.sqrt(x * x + y * y);
        const ha = (r != 0.0) ? Math.atan2(y, x) : 0.0;
        this.position.dec = this.deg(Math.atan2(z, r));
        this.position.ra = this.degToHours(localSiderealTimeDegrees - this.deg(ha));
        if (this.position.ra < 0) {
            this.position.ra += 24;
        }
        // console.log(`setting to ra=${this.position.ra}, dec=${this.position.dec}`);
    }
    rad(degrees) {
        return (degrees * Math.PI / 180);
    }
    deg(radians) {
        return (radians / Math.PI * 180);
    }
    degToHours(degrees) {
        return (degrees / 360 * 24);
    }
}
;
class NexStar {
    constructor() {
        this.versionMajor = 4;
        this.versionMinor = 10;
    }
    handleRequest(request) {
        let response = "";
        // Echo
        if (request.match(/^K[A-Za-z0-9]$/)) {
            // console.log(`Received 'Echo': '${request}'`);
            response = `${request.charAt(1)}#`;
        }
        else if (request.match(/^V$/)) {
            // console.log(`Received 'Get Version': '${request}'`);
            response = `${String.fromCharCode(this.versionMajor)}${String.fromCharCode(this.versionMinor)}#`;
        }
        else if (request.match(/^P\x01(\x10|\x11|\xb0|\xb2)\xfe\x00\x00\x00\x02$/)) {
            // console.log(`Received 'Get Device Version': '${request}'`);
            response = `\x01\x00#`;
        }
        else if (request.match(/^m$/)) {
            // console.log(`Received 'Get Model': '${request}'`);
            response = `\x0c#`; // fake 'GPS Series'
        }
        else if (request.match(/^w$/)) {
            // console.log(`Received 'Get Location': '${request}'`);
            response = `\x30\x0f\x29\x00\x0a\x3b\x01\x00#`; // Fake... 
        }
        else if (request.match(/^e$/)) {
            // console.log(`Received 'Get precise RA/DEC': '${request}'`);
            response = `34AB0500,12CE0500#`; // Fake...
        }
        else if (request.match(/^h$/)) {
            // console.log(`Received 'Get Time': '${request}'`);
            const today = new Date();
            response = `${String.fromCharCode(today.getHours())}` +
                `${String.fromCharCode(today.getMinutes())}` +
                `${String.fromCharCode(today.getSeconds())}` +
                `${String.fromCharCode(today.getMonth() + 1)}` +
                `${String.fromCharCode(today.getDate())}` +
                `${String.fromCharCode(today.getFullYear() - 2000)}` +
                `\x01\x00#`;
            // response = `\x10\x0d\x10\x01\x0d\x16\x01\x00#`;    // Fake... 48100c07010d160100
        }
        else if (request.match(/^L$/)) {
            // console.log(`Received 'Is GOTO in Progress': '${request}'`);
            response = `\x00#`; // Fake...
        }
        else if (request.match(/^J$/)) {
            // console.log(`Received 'Is Alignment Complete?': '${request}'`);
            response = `\x00#`; // Fake...
        }
        else if (request.match(/^t$/)) {
            // console.log(`Received 'Get Tracking Mode': '${request}'`);
            response = `\x01#`; // Fake Alt/Az ...
        }
        else if (request.match(/^s/)) {
            const today = new Date();
            console.log(`Received 'Sync precise RA/DEC' on ${today.toLocaleString("de-DE")}: '${request}'`);
            response = `#`; // Fake Alt/Az ...
        }
        else {
            console.log(`[NexStar] not implemented: '${request}'`);
        }
        return response;
    }
}
function onConnect(socket) {
    console.log(`Connection from '${socket.remoteAddress}'`);
    const telescope = new Telescope();
    const nexStar = new NexStar();
    telescope.fromHorizontalPosition(135.54, 52.71, 48, 32.166);
    telescope.offset = {
        ra: -0.018416198265134298,
        dec: -0.02824170708658258
    };
    // const timer = setInterval(() => {
    //     socket.write(packCurrentPosition(telescope.getCorrected()), (err) => {
    //         if (err) console.log(err);
    //     });
    // }, 1000);
    socket.on('end', () => {
        // clearInterval(timer);
        console.log(`Connection to '${socket.remoteAddress}' closed.`);
    });
    socket.on('data', (data) => {
        // console.log(`Received ${data.length} Bytes: ${data.toString('hex')}`)
        const response = nexStar.handleRequest(data.toString('latin1'));
        if (response.length > 0) {
            socket.write(response, (err) => {
                if (err)
                    console.log(err);
            });
        }
        else if (data.length >= 20) {
            const message = unpackGoto(data);
            console.log(`Received ${data.length} Bytes: {`
                + `length: ${message.length.toString()}, `
                + `type: ${message.type.toString()}, `
                + `time: ${message.timestamp.toLocaleTimeString()}, `
                + `ra: ${message.ra.toFixed(4).toString()}h, `
                + `dec: ${message.dec.toFixed(4).toString()}°, `
                + `}`);
            // telescope.setPosition({ dec: message.dec, ra: message.ra });
            telescope.calibrate({ dec: message.dec, ra: message.ra });
            // console.log(telescope.toHorizontalPosition());
            socket.write(packCurrentPosition(telescope.getCorrected()), (err) => {
                if (err)
                    console.log(err);
            });
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
