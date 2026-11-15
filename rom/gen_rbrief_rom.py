#!/usr/bin/env python3
"""
Sinh ROM pattern rBRIEF da xoay san cho orb-accelerator-de2.

  - Lay bit_pattern_31_ (256 cap diem) tu orb.cpp cua OpenCV
  - Xoay cho 30 bin (bin k = 12*k do) theo dung cong thuc va cach lam tron cua OpenCV
  - Moi cap -> 1 tu ROM = {addr_p, addr_q}, addr la dia chi trong smoothed buffer W x W
        addr = (R + y) * W + (R + x),  W = 2R+1,  R = ban kinh lon nhat sau khi xoay
  - Dia chi ROM = {angle_bin[4:0], pair_idx[7:0]} = bin*256 + pair
  - Bit descriptor: bit = I(p) < I(q); byte i, bit k (LSB truoc) = cap 8*i + k

Chay:
    python3 gen_rbrief_rom.py --orb-cpp orb.cpp --out rbrief_pattern.mif --selftest
"""
import argparse
import re
import sys

import numpy as np

NUM_BINS = 30
BIN_DEG = 12
KERNEL_HALF = 3          # Gaussian 7x7


def load_pattern(path):
    txt = open(path, encoding="utf-8", errors="ignore").read()
    i = txt.index("bit_pattern_31_[256*4]")
    j = txt.index("{", i)
    k = txt.index("};", j)
    body = re.sub(r"/\*.*?\*/", "", txt[j + 1:k], flags=re.S)
    nums = [int(t) for t in re.findall(r"-?\d+", body)]
    if len(nums) != 256 * 4:
        sys.exit("bit_pattern_31_ khong du 1024 so: %d" % len(nums))
    return np.array(nums, dtype=np.int32).reshape(256, 4)      # x1, y1, x2, y2


def rotate_bin(pat, b):
    """Giong GET_VALUE trong computeOrbDescriptors cua OpenCV (float32, cvRound = lam tron chan)."""
    angle = np.float32(b * BIN_DEG)
    angle = np.float32(angle * np.float32(np.pi / 180.0))
    a = np.float32(np.cos(np.float64(angle)))
    s = np.float32(np.sin(np.float64(angle)))
    out = np.zeros((256, 4), dtype=np.int32)
    for c in (0, 2):
        px = pat[:, c].astype(np.float32)
        py = pat[:, c + 1].astype(np.float32)
        x = px * a - py * s
        y = px * s + py * a
        out[:, c] = np.rint(x).astype(np.int32)
        out[:, c + 1] = np.rint(y).astype(np.int32)
    return out


def build_tables(pat):
    return [rotate_bin(pat, b) for b in range(NUM_BINS)]


def addr_bits(w):
    n = w * w - 1
    return max(1, int(n).bit_length())


def make_rom(tables, r):
    w = 2 * r + 1
    ab = addr_bits(w)
    words = []
    for b in range(NUM_BINS):
        t = tables[b]
        for p in range(256):
            ap = (r + t[p, 1]) * w + (r + t[p, 0])
            aq = (r + t[p, 3]) * w + (r + t[p, 2])
            words.append((int(ap) << ab) | int(aq))
    return words, w, ab


def write_mif(path, words, width):
    hexw = (width + 3) // 4
    with open(path, "w") as f:
        f.write("-- rBRIEF pattern ROM: address = {angle_bin[4:0], pair_idx[7:0]}\n")
        f.write("-- word = {addr_p, addr_q}; bit = I(p) < I(q)\n")
        f.write("DEPTH = %d;\nWIDTH = %d;\n" % (len(words), width))
        f.write("ADDRESS_RADIX = HEX;\nDATA_RADIX = HEX;\nCONTENT BEGIN\n")
        for i, wd in enumerate(words):
            f.write("    %X : %0*X;\n" % (i, hexw, wd))
        f.write("END;\n")


def selftest(pat, tables, r):
    """So khop tung bit voi cv2.ORB: lam mo bang GaussianBlur 7x7 sigma 2, tra ROM, so sanh."""
    import cv2
    rng = np.random.RandomState(1)
    img = rng.randint(0, 256, (240, 240), dtype=np.uint8)
    blur = cv2.GaussianBlur(img, (7, 7), sigmaX=2, sigmaY=2, borderType=cv2.BORDER_REFLECT_101)

    kps, ref = [], []
    cx, cy = 120, 120
    for b in range(NUM_BINS):
        kps.append(cv2.KeyPoint(float(cx + (b % 5) - 2), float(cy + (b // 5) - 3), 31, float(b * BIN_DEG), 0, 0, -1))
    orb = cv2.ORB_create(nfeatures=500, scaleFactor=1.2, nlevels=1,
                         edgeThreshold=31, patchSize=31, WTA_K=2)
    kp_out, desc = orb.compute(img, kps)
    if desc is None or len(kp_out) != len(kps):
        print("selftest: cv2 tra ve %d/%d keypoint" % (0 if desc is None else len(kp_out), len(kps)))
        return False

    exact = 0
    near = 0        # bit lech nhung |I(p)-I(q)| <= 1: do sai so lam tron cua bo loc Gaussian
    unexplained = 0
    for kp, d in zip(kp_out, desc):
        b = int(round(kp.angle / BIN_DEG))
        if abs(kp.angle - b * BIN_DEG) > 1e-3:
            print("selftest: cv2 doi goc keypoint (%.3f)" % kp.angle)
            return False
        x0, y0 = int(round(kp.pt[0])), int(round(kp.pt[1]))
        t = tables[b]
        ref_bits = np.unpackbits(d, bitorder="little")
        nd = 0
        for p in range(256):
            ip = int(blur[y0 + t[p, 1], x0 + t[p, 0]])
            iq = int(blur[y0 + t[p, 3], x0 + t[p, 2]])
            mine = 1 if ip < iq else 0
            if mine != ref_bits[p]:
                nd += 1
                if abs(ip - iq) <= 1:
                    near += 1
                else:
                    unexplained += 1
                    print("  bin %2d cap %3d: I(p)=%d I(q)=%d (cv2 bit=%d)" % (b, p, ip, iq, ref_bits[p]))
        if nd == 0:
            exact += 1
    total = 256 * len(kp_out)
    print("selftest: %d/%d bin khop tuyet doi; %d/%d bit lech, trong do %d bit chi do |I(p)-I(q)|<=1 "
          "(sai so lam tron Gaussian), %d bit khong giai thich duoc"
          % (exact, len(kp_out), near + unexplained, total, near, unexplained))
    return unexplained == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--orb-cpp", required=True, help="duong dan orb.cpp cua OpenCV")
    ap.add_argument("--out", default="rbrief_pattern.mif")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    pat = load_pattern(args.orb_cpp)
    print("pattern goc: |x|max=%d |y|max=%d" % (np.abs(pat[:, [0, 2]]).max(), np.abs(pat[:, [1, 3]]).max()))

    tables = build_tables(pat)
    allc = np.concatenate([t.reshape(-1) for t in tables])
    r = int(np.abs(allc).max())
    per_bin = [int(np.abs(t).max()) for t in tables]
    print("ban kinh lon nhat sau khi xoay R = %d  (theo bin: %s)" % (r, per_bin))
    print("smoothed buffer: %dx%d,  patch tho can: %dx%d,  vien keypoint can >= %d"
          % (2 * r + 1, 2 * r + 1, 2 * (r + KERNEL_HALF) + 1, 2 * (r + KERNEL_HALF) + 1, r + KERNEL_HALF))

    words, w, ab = make_rom(tables, r)
    width = 2 * ab
    write_mif(args.out, words, width)
    print("ROM: depth=%d, width=%d bit (addr %d bit x2), %d kbit" % (len(words), width, ab, len(words) * width // 1000))
    print("ghi %s" % args.out)

    if args.selftest:
        ok = selftest(pat, tables, r)
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
