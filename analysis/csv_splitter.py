import os, sys

src = sys.argv[1]
limit = int(3.5 * 1024**3)  # bytes per part
base, ext = os.path.splitext(src)

with open(src, "rb") as f:
    header = f.readline()
    part, out, size = 0, None, limit
    for line in f:
        if size + len(line) > limit:
            if out: out.close()
            part += 1
            out = open(f"{base}_part{part:03d}{ext}", "wb")
            out.write(header); size = len(header)
        out.write(line); size += len(line)
    if out: out.close()