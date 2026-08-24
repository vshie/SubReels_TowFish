import sys

from pymavlink import mavutil


def peek(path, types, n=5):
    want = set(types)
    seen = {}
    mlog = mavutil.mavlink_connection(path)
    while True:
        m = mlog.recv_match(type=list(want))
        if m is None:
            break
        t = m.get_type()
        lst = seen.setdefault(t, [])
        if len(lst) < n:
            lst.append(m)
        if len(seen) == len(want) and all(len(v) >= n for v in seen.values()):
            break
    print("=" * 70)
    print(path)
    for t in types:
        print(f"--- {t} ---")
        for m in seen.get(t, []):
            print("   ", m)


if __name__ == "__main__":
    path = sys.argv[1]
    peek(path, sys.argv[2:])
