#!/usr/bin/env python3
"""Build an unsigned driver catalog (.cat) for Windows 8.1.

  mkcat.py OUT.cat FILE...

The catalog lists every FILE by its SHA1 hash: the Authenticode image hash
for PE files (.sys/.dll, so an embedded signature does not change it), the
hash of the whole file otherwise (.inf).  The catalog-level OSAttr is
"2:6.3" (NT 6.3 = Windows 8.1).  The output is a PKCS#7 SignedData with
no signers yet; sign it afterwards, e.g.

  osslsigncode sign -certs C.crt -key C.key -h sha256 -in OUT.cat -out SIGNED.cat

Layout follows what makecat.exe produces (szOID_CTL content with
szOID_CATALOG_LIST subjects).
"""

import datetime
import hashlib
import os
import struct
import sys

# --- minimal DER -----------------------------------------------------------

def _len(n):
    if n < 0x80:
        return bytes([n])
    b = n.to_bytes((n.bit_length() + 7) // 8, "big")
    return bytes([0x80 | len(b)]) + b

def tlv(tag, body):
    return bytes([tag]) + _len(len(body)) + body

def seq(*items):
    return tlv(0x30, b"".join(items))

def set_(*items):
    return tlv(0x31, b"".join(sorted(items)))

def integer(v):
    b = v.to_bytes((v.bit_length() + 8) // 8 or 1, "big", signed=False)
    return tlv(0x02, b)

def octets(b):
    return tlv(0x04, b)

def null():
    return b"\x05\x00"

def bmp(s):
    return tlv(0x1E, s.encode("utf-16-be"))

def oid(dotted):
    parts = [int(p) for p in dotted.split(".")]
    body = bytes([parts[0] * 40 + parts[1]])
    for p in parts[2:]:
        chunk = [p & 0x7F]
        p >>= 7
        while p:
            chunk.insert(0, 0x80 | (p & 0x7F))
            p >>= 7
        body += bytes(chunk)
    return tlv(0x06, body)

def ctx(n, body, constructed=True):
    return tlv((0xA0 if constructed else 0x80) | n, body)

def utctime(t):
    return tlv(0x17, t.strftime("%y%m%d%H%M%SZ").encode())

def utf16z(s):
    return (s + "\0").encode("utf-16-le")

# --- OIDs ------------------------------------------------------------------

OID_SIGNED_DATA = "1.2.840.113549.1.7.2"
OID_CTL = "1.3.6.1.4.1.311.10.1"
OID_CATALOG_LIST = "1.3.6.1.4.1.311.12.1.1"
OID_CATALOG_LIST_MEMBER = "1.3.6.1.4.1.311.12.1.2"
OID_CAT_NAMEVALUE = "1.3.6.1.4.1.311.12.2.1"
OID_CAT_MEMBERINFO = "1.3.6.1.4.1.311.12.2.2"
OID_SPC_INDIRECT_DATA = "1.3.6.1.4.1.311.2.1.4"
OID_SPC_PE_IMAGE_DATA = "1.3.6.1.4.1.311.2.1.15"
OID_SPC_CAB_DATA = "1.3.6.1.4.1.311.2.1.25"
OID_SHA1 = "1.3.14.3.2.26"

GUID_PE = "{C689AAB8-8E78-11D0-8C47-00C04FC295EE}"
GUID_FLAT = "{DE351A42-8E59-11D0-8C47-00C04FC295EE}"

NAMEVALUE_FLAGS = 0x10010001  # CRYPTCAT_ATTR_NAMEASCII|DATAASCII|AUTHENTICATED

# --- hashes ----------------------------------------------------------------

def pe_hash(data):
    """Authenticode SHA1: skip CheckSum, the certificate-table directory
    entry and the certificate table itself."""
    pe = struct.unpack_from("<I", data, 0x3C)[0]
    if data[pe:pe + 4] != b"PE\0\0":
        return None
    opt = pe + 24
    magic = struct.unpack_from("<H", data, opt)[0]
    checksum = opt + 64
    certdir = opt + (128 if magic == 0x10B else 144)
    cert_off, cert_len = struct.unpack_from("<II", data, certdir)
    end = cert_off if (cert_off and cert_len) else len(data)
    h = hashlib.sha1()
    h.update(data[:checksum])
    h.update(data[checksum + 4:certdir])
    h.update(data[certdir + 8:end])
    return h.digest()

def member(path):
    data = open(path, "rb").read()
    name = os.path.basename(path).lower()
    digest = pe_hash(data) if data[:2] == b"MZ" else None
    if digest is not None:
        guid = GUID_PE
        # SpcPeImageData: flags (empty bit string), file = SpcLink [2] { [0] "" }
        spc = seq(oid(OID_SPC_PE_IMAGE_DATA),
                  seq(b"\x03\x01\x00", ctx(0, ctx(2, ctx(0, b"", constructed=False)))))
    else:
        digest = hashlib.sha1(data).digest()
        guid = GUID_FLAT
        spc = seq(oid(OID_SPC_CAB_DATA), ctx(2, ctx(0, b"", constructed=False)))
    indirect = seq(spc, seq(seq(oid(OID_SHA1), null()), octets(digest)))
    attrs = set_(
        seq(oid(OID_CAT_NAMEVALUE), set_(seq(bmp("File"), integer(NAMEVALUE_FLAGS), octets(utf16z(name))))),
        seq(oid(OID_CAT_MEMBERINFO), set_(seq(bmp(guid), integer(512)))),
        seq(oid(OID_SPC_INDIRECT_DATA), set_(indirect)),
    )
    tag = utf16z(digest.hex().upper())
    return seq(octets(tag), attrs), name, digest

def catalog(files):
    members = [member(f) for f in files]
    os_attr = seq(oid(OID_CAT_NAMEVALUE),
                  octets(seq(bmp("OSAttr"), integer(NAMEVALUE_FLAGS), octets(utf16z("2:6.3")))))
    ctl = seq(
        seq(oid(OID_CATALOG_LIST)),
        octets(os.urandom(16)),
        utctime(datetime.datetime.now(datetime.timezone.utc)),
        seq(oid(OID_CATALOG_LIST_MEMBER), null()),
        seq(*[m[0] for m in members]),
        ctx(0, seq(os_attr)),
    )
    signed = seq(integer(1), set_(), seq(oid(OID_CTL), ctx(0, ctl)), set_())
    return seq(oid(OID_SIGNED_DATA), ctx(0, signed)), members

def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    der, members = catalog(sys.argv[2:])
    open(sys.argv[1], "wb").write(der)
    for _, name, digest in members:
        print("  %s %s" % (digest.hex().upper(), name))

if __name__ == "__main__":
    main()
