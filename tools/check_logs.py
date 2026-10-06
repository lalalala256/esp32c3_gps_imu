#!/usr/bin/env python3
"""
Host-side sanity checker for the CSV logs produced by esp32c3_gps_imu.

Checks, per file:
  * every data row has the same number of columns as the header,
  * `t_us` is strictly increasing,
  * the achieved sample rate and the largest gap between samples
    (a gap much larger than 1/rate means the producer stalled or the card
    could not keep up),
  * `valid` flags and, for the GNSS log, the fix/PPS state.

It deliberately uses only the standard library so it runs anywhere.

Usage:
    python3 tools/check_logs.py /path/to/logs            # all *.csv in a dir
    python3 tools/check_logs.py imu_0001.csv gnss_0001.csv
    python3 tools/check_logs.py --plot /path/to/logs     # needs matplotlib
"""

import argparse
import glob
import os
import statistics
import sys

RATE_HINT = {
    "imu": 100.0,
    "mag": 100.0,
    "gnss": 1.0,
    "events": None,
}


def parse_file(path):
    """Return (columns, rows, meta) with comment lines separated out."""
    columns, rows, meta = None, [], []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                meta.append(line[1:].strip())
                continue
            if columns is None:
                columns = line.split(",")
                continue
            rows.append((lineno, line.split(",")))
    return columns, rows, meta


def check_file(path, gap_tolerance=5.0):
    name = os.path.basename(path)
    kind = name.split("_")[0]
    problems = []

    columns, rows, meta = parse_file(path)
    if columns is None:
        problems.append("no header row found")
        return name, problems, {}
    if not rows:
        problems.append("no data rows")
        return name, problems, {}

    # Column count consistency.
    bad = [ln for ln, f in rows if len(f) != len(columns)]
    if bad:
        problems.append(
            f"{len(bad)} row(s) have a different column count (e.g. line {bad[0]})"
        )

    if "t_us" not in columns:
        problems.append("no t_us column")
        return name, problems, {}

    ti = columns.index("t_us")
    ts = []
    for ln, f in rows:
        try:
            ts.append(int(f[ti]))
        except (ValueError, IndexError):
            problems.append(f"unparseable t_us at line {ln}")
            break

    stats = {"rows": len(rows)}
    if len(ts) < 2:
        return name, problems, stats

    if any(b <= a for a, b in zip(ts, ts[1:])):
        i = next(i for i, (a, b) in enumerate(zip(ts, ts[1:])) if b <= a)
        problems.append(f"t_us not strictly increasing around row {i + 2}")

    span_us = ts[-1] - ts[0]
    if span_us <= 0:
        problems.append("zero time span")
        return name, problems, stats

    gaps = [b - a for a, b in zip(ts, ts[1:])]
    stats["span_s"] = span_us / 1e6
    stats["rate_hz"] = (len(ts) - 1) / (span_us / 1e6)
    stats["gap_median_us"] = statistics.median(gaps)
    stats["gap_max_us"] = max(gaps)

    hint = RATE_HINT.get(kind)
    if hint:
        expect = 1e6 / hint
        if stats["gap_max_us"] > expect * gap_tolerance:
            problems.append(
                f"largest gap {stats['gap_max_us']} us is "
                f">{gap_tolerance:g}x the expected {expect:.0f} us "
                "(stall or dropped samples)"
            )
        if stats["rate_hz"] < hint * 0.9:
            problems.append(
                f"achieved {stats['rate_hz']:.1f} Hz is <90% of the {hint:g} Hz target"
            )

    # valid flags
    if "valid" in columns:
        vi = columns.index("valid")
        invalid = sum(1 for _, f in rows if len(f) > vi and f[vi].strip() == "0")
        stats["invalid"] = invalid
        if invalid:
            problems.append(f"{invalid} row(s) marked valid=0")

    # GNSS specific
    if kind == "gnss":
        extra = {}
        if "fix_type" in columns and "sats" in columns:
            fi = columns.index("fix_type")
            si = columns.index("sats")
            extra["fix"] = sum(1 for _, f in rows if len(f) > fi and f[fi] not in ("0", ""))
            extra["max_sats"] = max(
                (int(f[si]) for _, f in rows if len(f) > si and f[si].isdigit()),
                default=0,
            )
        if "pps_t_us" in columns:
            pi = columns.index("pps_t_us")
            extra["paired"] = sum(1 for _, f in rows if len(f) > pi and f[pi] not in ("0", ""))
        stats.update(extra)

    if kind == "events":
        ei = columns.index("event") if "event" in columns else None
        if ei is not None:
            stats["events"] = [f[ei] for _, f in rows]

    return name, problems, stats


def report(name, problems, stats, meta):
    status = "OK  " if not problems else "FAIL"
    print(f"[{status}] {name}")
    for m in meta:
        print(f"        meta: {m}")

    if "rows" in stats:
        bits = [f"rows={stats['rows']}"]
        if "rate_hz" in stats:
            bits.append(f"rate={stats['rate_hz']:.2f} Hz")
        if "span_s" in stats:
            bits.append(f"span={stats['span_s']:.1f} s")
        if "gap_max_us" in stats:
            bits.append(f"max_gap={stats['gap_max_us'] / 1000:.1f} ms")
        if "invalid" in stats:
            bits.append(f"invalid={stats['invalid']}")
        if "fix" in stats:
            bits.append(f"rows_with_fix={stats['fix']}")
        if "max_sats" in stats:
            bits.append(f"max_sats={stats['max_sats']}")
        if "paired" in stats:
            bits.append(f"pps_paired={stats['paired']}")
        print("        " + "  ".join(bits))

    if stats.get("events"):
        print(f"        events: {', '.join(stats['events'])}")

    for p in problems:
        print(f"        ! {p}")


def maybe_plot(paths):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not available, skipping plots", file=sys.stderr)
        return

    for path in paths:
        kind = os.path.basename(path).split("_")[0]
        if kind not in ("imu", "mag"):
            continue
        columns, rows, _ = parse_file(path)
        if not columns or not rows:
            continue
        try:
            t0 = int(rows[0][0][columns.index("t_us")])
            t = [(int(f[columns.index("t_us")]) - t0) / 1e6 for _, f in rows]
            ax_labels = columns[1:-1]
            fig, ax = plt.subplots(figsize=(11, 4))
            for i, label in enumerate(ax_labels, start=1):
                ax.plot(t, [float(f[i]) for _, f in rows], label=label, linewidth=0.8)
            ax.set_xlabel("t (s)")
            ax.set_title(os.path.basename(path))
            ax.legend(fontsize=7, ncol=2)
            ax.grid(alpha=0.3)
            out = os.path.splitext(path)[0] + ".png"
            fig.tight_layout()
            fig.savefig(out, dpi=110)
            print(f"wrote {out}")
        except (ValueError, IndexError) as exc:
            print(f"plot skipped for {path}: {exc}", file=sys.stderr)


def collect(targets):
    files = []
    for t in targets:
        if os.path.isdir(t):
            files.extend(sorted(glob.glob(os.path.join(t, "*.csv"))))
        else:
            files.extend(sorted(glob.glob(t)))
    return files


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("targets", nargs="+", help="CSV files or directories")
    ap.add_argument("--plot", action="store_true", help="write PNG plots for imu/mag")
    ap.add_argument("--gap-tolerance", type=float, default=5.0,
                    help="max allowed gap as a multiple of the nominal period (default 5)")
    args = ap.parse_args()

    files = collect(args.targets)
    if not files:
        print("no CSV files found", file=sys.stderr)
        return 2

    failures = 0
    for path in files:
        name, problems, stats = check_file(path, args.gap_tolerance)
        columns, _, meta = parse_file(path)
        report(name, problems, stats, meta)
        if problems:
            failures += 1
        print()

    if args.plot:
        maybe_plot(files)

    print(f"{len(files) - failures}/{len(files)} file(s) passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
