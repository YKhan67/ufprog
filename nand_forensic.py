#!/usr/bin/env python3

import argparse
import csv
import math
import os
import statistics
import sys
from collections import Counter


# ============================================================
# ESMT F50L1G41LB
#
# 128 MiB MAIN AREA ONLY
#
# Geometry:
#   Page main area       = 2048 bytes
#   OOB                  = 64 bytes (NOT PRESENT in BIN)
#   Pages                = 65536
#   Pages / block        = 64
#   Blocks               = 1024
#   Block size           = 128 KiB
# ============================================================

PAGE_SIZE = 2048
OOB_SIZE = 64
PAGES_PER_BLOCK = 64
BLOCKS = 1024
TOTAL_PAGES = BLOCKS * PAGES_PER_BLOCK
BLOCK_SIZE = PAGE_SIZE * PAGES_PER_BLOCK
TOTAL_SIZE = PAGE_SIZE * TOTAL_PAGES


# ------------------------------------------------------------
# Thresholds
# ------------------------------------------------------------

# Page is considered "erased-like" if >= this percentage FF.
ERASED_FF_THRESHOLD = 99.5

# Page is considered "zero-heavy" if >= this percentage 00.
ZERO_HEAVY_THRESHOLD = 99.5

# Mixed FF/00 pattern.
MIXED_FF00_THRESHOLD = 90.0

# Very low entropy.
LOW_ENTROPY_THRESHOLD = 0.20

# High entropy.
HIGH_ENTROPY_THRESHOLD = 7.95

# Page-to-page difference.
NEIGHBOR_DIFF_THRESHOLD = 0.35

# Block-level abnormality thresholds.
BLOCK_FF_THRESHOLD = 99.5
BLOCK_ZERO_THRESHOLD = 99.5

# A page containing this many distinct byte values is suspicious
# only when combined with other abnormal characteristics.
LOW_UNIQUE_THRESHOLD = 4


# ------------------------------------------------------------
# Utility functions
# ------------------------------------------------------------

def entropy(data):
    if not data:
        return 0.0

    counts = Counter(data)
    n = len(data)

    h = 0.0

    for count in counts.values():
        p = count / n
        h -= p * math.log2(p)

    return h


def percentage(value, total):
    if total == 0:
        return 0.0
    return (value * 100.0) / total


def byte_counts(data):
    c = Counter(data)

    ff = c.get(0xFF, 0)
    zero = c.get(0x00, 0)

    return c, ff, zero


def hamming_fraction(a, b):
    """
    Fraction of byte positions that differ.
    """
    if len(a) != len(b):
        return 1.0

    different = 0

    for x, y in zip(a, b):
        if x != y:
            different += 1

    return different / len(a)


def dominant_byte(data):
    if not data:
        return 0, 0

    value, count = Counter(data).most_common(1)[0]
    return value, count


def is_all(data, value):
    return all(x == value for x in data)


def count_transitions(data):
    if len(data) < 2:
        return 0

    return sum(1 for i in range(1, len(data)) if data[i] != data[i - 1])


def repeated_chunk_score(data, chunk_size=16):
    """
    Measures how repetitive a page is.

    Returns fraction of chunks that are identical to the
    immediately preceding chunk.
    """
    if len(data) < chunk_size * 2:
        return 0.0

    chunks = [
        data[i:i + chunk_size]
        for i in range(0, len(data), chunk_size)
    ]

    if len(chunks) < 2:
        return 0.0

    same = 0

    for i in range(1, len(chunks)):
        if chunks[i] == chunks[i - 1]:
            same += 1

    return same / (len(chunks) - 1)


def classify_page(stats):
    reasons = []

    ff_pct = stats["ff_pct"]
    zero_pct = stats["zero_pct"]
    entropy_value = stats["entropy"]
    unique = stats["unique"]
    dominant_pct = stats["dominant_pct"]
    dominant = stats["dominant"]
    mixed_pct = stats["mixed_ff00_pct"]
    transitions = stats["transitions"]
    repeated = stats["repeated_chunk_score"]

    if ff_pct >= ERASED_FF_THRESHOLD:
        reasons.append("ERASED_LIKE")

    if zero_pct >= ZERO_HEAVY_THRESHOLD:
        reasons.append("ZERO_HEAVY")

    if mixed_pct >= MIXED_FF00_THRESHOLD:
        reasons.append("FF00_MIXED")

    if entropy_value <= LOW_ENTROPY_THRESHOLD:
        reasons.append("VERY_LOW_ENTROPY")

    if entropy_value >= HIGH_ENTROPY_THRESHOLD:
        reasons.append("VERY_HIGH_ENTROPY")

    if unique <= LOW_UNIQUE_THRESHOLD:
        reasons.append("FEW_UNIQUE_BYTES")

    if dominant_pct >= 99.0 and dominant not in (0x00, 0xFF):
        reasons.append("DOMINANT_BYTE")

    if transitions < 10 and not (ff_pct >= ERASED_FF_THRESHOLD):
        reasons.append("VERY_FEW_TRANSITIONS")

    if repeated >= 0.90:
        reasons.append("HIGHLY_REPETITIVE")

    return reasons


# ------------------------------------------------------------
# Page analysis
# ------------------------------------------------------------

def analyze_page(data, page_number, previous_page=None):
    c, ff, zero = byte_counts(data)

    ent = entropy(data)

    dominant, dominant_count = dominant_byte(data)

    transitions = count_transitions(data)

    repeated = repeated_chunk_score(data)

    ff_pct = percentage(ff, len(data))
    zero_pct = percentage(zero, len(data))
    dominant_pct = percentage(dominant_count, len(data))

    mixed_ff00 = percentage(ff + zero, len(data))

    unique = len(c)

    neighbor_diff = None

    if previous_page is not None:
        neighbor_diff = hamming_fraction(previous_page, data)

    stats = {
        "page": page_number,
        "block": page_number // PAGES_PER_BLOCK,
        "page_in_block": page_number % PAGES_PER_BLOCK,
        "offset": page_number * PAGE_SIZE,
        "ff": ff,
        "ff_pct": ff_pct,
        "zero": zero,
        "zero_pct": zero_pct,
        "unique": unique,
        "entropy": ent,
        "dominant": dominant,
        "dominant_count": dominant_count,
        "dominant_pct": dominant_pct,
        "mixed_ff00_pct": mixed_ff00,
        "transitions": transitions,
        "repeated_chunk_score": repeated,
        "neighbor_diff": neighbor_diff,
        "all_ff": is_all(data, 0xFF),
        "all_zero": is_all(data, 0x00),
    }

    reasons = classify_page(stats)

    if neighbor_diff is not None:
        if neighbor_diff >= NEIGHBOR_DIFF_THRESHOLD:
            reasons.append("LARGE_NEIGHBOR_CHANGE")

    stats["reasons"] = reasons
    stats["abnormal"] = bool(reasons)

    return stats


# ------------------------------------------------------------
# Block analysis
# ------------------------------------------------------------

def analyze_block(pages, block_number, page_stats):
    data = b"".join(pages)

    c, ff, zero = byte_counts(data)

    ent = entropy(data)

    dominant, dominant_count = dominant_byte(data)

    ff_pct = percentage(ff, len(data))
    zero_pct = percentage(zero, len(data))

    unique = len(c)

    dominant_pct = percentage(dominant_count, len(data))

    all_ff = is_all(data, 0xFF)
    all_zero = is_all(data, 0x00)

    block_page_stats = [
        x for x in page_stats
        if x["block"] == block_number
    ]

    abnormal_pages = [
        x for x in block_page_stats
        if x["abnormal"]
    ]

    erased_pages = [
        x for x in block_page_stats
        if x["all_ff"]
    ]

    zero_pages = [
        x for x in block_page_stats
        if x["all_zero"]
    ]

    reasons = []

    if all_ff:
        reasons.append("ALL_FF")

    elif all_zero:
        reasons.append("ALL_ZERO")

    else:

        if ff_pct >= BLOCK_FF_THRESHOLD:
            reasons.append("FF_HEAVY")

        if zero_pct >= BLOCK_ZERO_THRESHOLD:
            reasons.append("ZERO_HEAVY")

        if len(abnormal_pages) >= 8:
            reasons.append("MANY_ABNORMAL_PAGES")

        if len(erased_pages) >= 56:
            reasons.append("MOSTLY_ERASED")

        if len(zero_pages) >= 56:
            reasons.append("MOSTLY_ZERO")

    # Strong mixed 00/FF signature.
    mixed_pct = percentage(ff + zero, len(data))

    if mixed_pct >= 95.0 and not all_ff:
        reasons.append("STRONG_FF00_PATTERN")

    return {
        "block": block_number,
        "first_page": block_number * PAGES_PER_BLOCK,
        "last_page": block_number * PAGES_PER_BLOCK + PAGES_PER_BLOCK - 1,
        "offset": block_number * BLOCK_SIZE,
        "ff": ff,
        "ff_pct": ff_pct,
        "zero": zero,
        "zero_pct": zero_pct,
        "unique": unique,
        "entropy": ent,
        "dominant": dominant,
        "dominant_count": dominant_count,
        "dominant_pct": dominant_pct,
        "mixed_ff00_pct": mixed_pct,
        "all_ff": all_ff,
        "all_zero": all_zero,
        "abnormal_pages": len(abnormal_pages),
        "erased_pages": len(erased_pages),
        "zero_pages": len(zero_pages),
        "reasons": reasons,
        "abnormal": bool(reasons),
    }


# ------------------------------------------------------------
# Human-readable formatting
# ------------------------------------------------------------

def hex_offset(value):
    return f"0x{value:08X}"


def print_page_summary(p):
    reason = ",".join(p["reasons"])

    print(
        f"Page {p['page']:5d} "
        f"(block {p['block']:4d} +{p['page_in_block']:02d}) "
        f"offset={hex_offset(p['offset'])} "
        f"FF={p['ff_pct']:6.2f}% "
        f"00={p['zero_pct']:6.2f}% "
        f"uniq={p['unique']:3d} "
        f"H={p['entropy']:5.2f} "
        f"reason={reason}"
    )


def print_block_summary(b):
    reason = ",".join(b["reasons"])

    print(
        f"Block {b['block']:4d} "
        f"pages {b['first_page']:5d}-{b['last_page']:5d} "
        f"offset={hex_offset(b['offset'])} "
        f"FF={b['ff_pct']:6.2f}% "
        f"00={b['zero_pct']:6.2f}% "
        f"uniq={b['unique']:3d} "
        f"H={b['entropy']:5.2f} "
        f"abnormal_pages={b['abnormal_pages']:2d} "
        f"reason={reason}"
    )


# ------------------------------------------------------------
# CSV writers
# ------------------------------------------------------------

def write_page_csv(path, pages):
    fields = [
        "page",
        "block",
        "page_in_block",
        "offset",
        "ff",
        "ff_pct",
        "zero",
        "zero_pct",
        "unique",
        "entropy",
        "dominant",
        "dominant_count",
        "dominant_pct",
        "mixed_ff00_pct",
        "transitions",
        "repeated_chunk_score",
        "neighbor_diff",
        "all_ff",
        "all_zero",
        "abnormal",
        "reasons",
    ]

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()

        for p in pages:
            row = dict(p)
            row["reasons"] = ";".join(p["reasons"])
            writer.writerow(row)


def write_block_csv(path, blocks):
    fields = [
        "block",
        "first_page",
        "last_page",
        "offset",
        "ff",
        "ff_pct",
        "zero",
        "zero_pct",
        "unique",
        "entropy",
        "dominant",
        "dominant_count",
        "dominant_pct",
        "mixed_ff00_pct",
        "all_ff",
        "all_zero",
        "abnormal_pages",
        "erased_pages",
        "zero_pages",
        "abnormal",
        "reasons",
    ]

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()

        for b in blocks:
            row = dict(b)
            row["reasons"] = ";".join(b["reasons"])
            writer.writerow(row)


def write_suspicious_pages(path, pages):
    suspicious = [p for p in pages if p["abnormal"]]

    fields = [
        "page",
        "block",
        "page_in_block",
        "offset",
        "ff_pct",
        "zero_pct",
        "unique",
        "entropy",
        "dominant",
        "dominant_pct",
        "mixed_ff00_pct",
        "transitions",
        "repeated_chunk_score",
        "neighbor_diff",
        "reasons",
    ]

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()

        for p in suspicious:
            row = {k: p[k] for k in fields if k != "reasons"}
            row["reasons"] = ";".join(p["reasons"])
            writer.writerow(row)


def write_suspicious_blocks(path, blocks):
    suspicious = [b for b in blocks if b["abnormal"]]

    fields = [
        "block",
        "first_page",
        "last_page",
        "offset",
        "ff_pct",
        "zero_pct",
        "unique",
        "entropy",
        "dominant",
        "dominant_pct",
        "mixed_ff00_pct",
        "abnormal_pages",
        "erased_pages",
        "zero_pages",
        "reasons",
    ]

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()

        for b in suspicious:
            row = {k: b[k] for k in fields if k != "reasons"}
            row["reasons"] = ";".join(b["reasons"])
            writer.writerow(row)


# ------------------------------------------------------------
# Full report
# ------------------------------------------------------------

def generate_report(
    report_path,
    filename,
    file_size,
    page_stats,
    block_stats,
):
    total_ff = sum(p["ff"] for p in page_stats)
    total_zero = sum(p["zero"] for p in page_stats)

    all_ff_pages = [p for p in page_stats if p["all_ff"]]
    all_zero_pages = [p for p in page_stats if p["all_zero"]]

    suspicious_pages = [
        p for p in page_stats
        if p["abnormal"]
    ]

    suspicious_blocks = [
        b for b in block_stats
        if b["abnormal"]
    ]

    entropies = [p["entropy"] for p in page_stats]

    with open(report_path, "w", encoding="utf-8") as f:

        def out(s=""):
            f.write(s + "\n")

        out("=" * 78)
        out("ESMT F50L1G41LB 128-MiB NAND FORENSIC ANALYSIS")
        out("=" * 78)
        out()

        out(f"File              : {filename}")
        out(f"File size         : {file_size:,} bytes")
        out(f"File size         : {file_size / 1024 / 1024:.3f} MiB")
        out()

        out("GEOMETRY")
        out("-" * 78)
        out(f"Main/page         : {PAGE_SIZE:,} bytes")
        out(f"OOB/page          : {OOB_SIZE} bytes")
        out(f"Pages             : {TOTAL_PAGES:,}")
        out(f"Pages/block       : {PAGES_PER_BLOCK}")
        out(f"Blocks            : {BLOCKS:,}")
        out(f"Main/block        : {BLOCK_SIZE:,} bytes")
        out()

        out("GLOBAL STATISTICS")
        out("-" * 78)
        out(f"FF bytes          : {total_ff:,} ({percentage(total_ff, file_size):.2f}%)")
        out(f"00 bytes          : {total_zero:,} ({percentage(total_zero, file_size):.2f}%)")
        out(f"All-FF pages      : {len(all_ff_pages):,}")
        out(f"All-00 pages      : {len(all_zero_pages):,}")
        out(f"Suspicious pages  : {len(suspicious_pages):,}")
        out(f"Suspicious blocks : {len(suspicious_blocks):,}")
        out(f"Min page entropy  : {min(entropies):.3f}")
        out(f"Max page entropy  : {max(entropies):.3f}")
        out(f"Mean page entropy : {statistics.mean(entropies):.3f}")
        out()

        out("SUSPICIOUS BLOCKS")
        out("-" * 78)

        if not suspicious_blocks:
            out("None detected.")
        else:
            for b in suspicious_blocks:
                out(
                    f"Block {b['block']} "
                    f"pages {b['first_page']}-{b['last_page']} "
                    f"offset {hex_offset(b['offset'])}"
                )

                out(
                    f"  FF={b['ff_pct']:.2f}% "
                    f"00={b['zero_pct']:.2f}% "
                    f"entropy={b['entropy']:.3f} "
                    f"unique={b['unique']}"
                )

                out(
                    f"  abnormal pages={b['abnormal_pages']} "
                    f"erased pages={b['erased_pages']} "
                    f"zero pages={b['zero_pages']}"
                )

                out(
                    f"  reasons: {', '.join(b['reasons'])}"
                )

                out()

        out("SUSPICIOUS PAGES")
        out("-" * 78)

        if not suspicious_pages:
            out("None detected.")
        else:
            for p in suspicious_pages:
                out(
                    f"Page {p['page']} "
                    f"(block {p['block']} +{p['page_in_block']:02d}) "
                    f"offset {hex_offset(p['offset'])}"
                )

                out(
                    f"  FF={p['ff_pct']:.2f}% "
                    f"00={p['zero_pct']:.2f}% "
                    f"unique={p['unique']} "
                    f"entropy={p['entropy']:.3f}"
                )

                if p["neighbor_diff"] is not None:
                    out(
                        f"  neighbor difference={p['neighbor_diff']:.3f}"
                    )

                out(
                    f"  reasons: {', '.join(p['reasons'])}"
                )

                out()

        out("ALL-FF BLOCKS")
        out("-" * 78)

        erased_blocks = [
            b for b in block_stats
            if b["all_ff"]
        ]

        if erased_blocks:
            for b in erased_blocks:
                out(
                    f"Block {b['block']:4d} "
                    f"pages {b['first_page']}-{b['last_page']}"
                )
        else:
            out("None.")

        out()

        out("ALL-00 BLOCKS")
        out("-" * 78)

        zero_blocks = [
            b for b in block_stats
            if b["all_zero"]
        ]

        if zero_blocks:
            for b in zero_blocks:
                out(
                    f"Block {b['block']:4d} "
                    f"pages {b['first_page']}-{b['last_page']}"
                )
        else:
            out("None.")

        out()

        out("=" * 78)
        out("END OF FORENSIC REPORT")
        out("=" * 78)


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="128-MiB SPI NAND forensic analyzer"
    )

    parser.add_argument(
        "binfile",
        help="128-MiB main-area-only NAND dump"
    )

    parser.add_argument(
        "--outdir",
        default="nand_forensic_output",
        help="Directory for reports"
    )

    args = parser.parse_args()

    filename = os.path.abspath(args.binfile)

    if not os.path.isfile(filename):
        print(f"ERROR: file does not exist:")
        print(filename)
        return 1

    file_size = os.path.getsize(filename)

    print()
    print("=" * 78)
    print("ESMT F50L1G41LB 128-MiB NAND FORENSIC ANALYSIS")
    print("=" * 78)
    print()
    print(f"File: {filename}")
    print(f"Size: {file_size:,} bytes")
    print()

    if file_size != TOTAL_SIZE:
        print("WARNING: unexpected file size")
        print()
        print(f"Expected main-only size : {TOTAL_SIZE:,}")
        print(f"Actual size             : {file_size:,}")
        print()

        if file_size == TOTAL_SIZE + TOTAL_PAGES * OOB_SIZE:
            print("This appears to contain 64-byte OOB per page.")
            print("This script expects MAIN AREA ONLY.")
            return 1

        print("Continuing anyway is unsafe.")
        return 1

    os.makedirs(args.outdir, exist_ok=True)

    page_csv = os.path.join(
        args.outdir,
        "page_report.csv"
    )

    block_csv = os.path.join(
        args.outdir,
        "block_report.csv"
    )

    suspicious_page_csv = os.path.join(
        args.outdir,
        "suspicious_pages.csv"
    )

    suspicious_block_csv = os.path.join(
        args.outdir,
        "suspicious_blocks.csv"
    )

    report_txt = os.path.join(
        args.outdir,
        "forensic_report.txt"
    )

    page_stats = []
    block_stats = []

    print("Analyzing pages...")
    print()

    with open(filename, "rb") as f:

        previous_page = None

        for page in range(TOTAL_PAGES):

            data = f.read(PAGE_SIZE)

            if len(data) != PAGE_SIZE:
                print(
                    f"ERROR: short read at page {page}"
                )
                return 1

            stats = analyze_page(
                data,
                page,
                previous_page
            )

            page_stats.append(stats)

            previous_page = data

            if page % 4096 == 0:
                print(
                    f"  page {page:5d} / {TOTAL_PAGES}"
                )

    print()
    print("Analyzing blocks...")
    print()

    with open(filename, "rb") as f:

        for block in range(BLOCKS):

            pages = []

            for _ in range(PAGES_PER_BLOCK):

                data = f.read(PAGE_SIZE)

                if len(data) != PAGE_SIZE:
                    print(
                        f"ERROR: short read in block {block}"
                    )
                    return 1

                pages.append(data)

            stats = analyze_block(
                pages,
                block,
                page_stats
            )

            block_stats.append(stats)

            if block % 64 == 0:
                print(
                    f"  block {block:4d} / {BLOCKS}"
                )

    print()
    print("Writing reports...")
    print()

    write_page_csv(
        page_csv,
        page_stats
    )

    write_block_csv(
        block_csv,
        block_stats
    )

    write_suspicious_pages(
        suspicious_page_csv,
        page_stats
    )

    write_suspicious_blocks(
        suspicious_block_csv,
        block_stats
    )

    generate_report(
        report_txt,
        filename,
        file_size,
        page_stats,
        block_stats
    )

    # --------------------------------------------------------
    # Compact console summary
    # --------------------------------------------------------

    suspicious_pages = [
        p for p in page_stats
        if p["abnormal"]
    ]

    suspicious_blocks = [
        b for b in block_stats
        if b["abnormal"]
    ]

    erased_blocks = [
        b for b in block_stats
        if b["all_ff"]
    ]

    zero_blocks = [
        b for b in block_stats
        if b["all_zero"]
    ]

    print()
    print("=" * 78)
    print("FORENSIC SUMMARY")
    print("=" * 78)
    print()

    print(f"Total bytes       : {file_size:,}")
    print(f"Total pages       : {TOTAL_PAGES:,}")
    print(f"Total blocks      : {BLOCKS:,}")
    print()

    print(
        f"All-FF pages      : "
        f"{sum(1 for p in page_stats if p['all_ff']):,}"
    )

    print(
        f"All-00 pages      : "
        f"{sum(1 for p in page_stats if p['all_zero']):,}"
    )

    print(
        f"Suspicious pages  : "
        f"{len(suspicious_pages):,}"
    )

    print(
        f"Suspicious blocks : "
        f"{len(suspicious_blocks):,}"
    )

    print()

    if suspicious_blocks:

        print("*** SUSPICIOUS BLOCK LIST ***")

        for b in suspicious_blocks:

            print(
                f"  Block {b['block']:4d} "
                f"(pages {b['first_page']} - "
                f"{b['last_page']})"
            )

            print(
                f"       FF={b['ff_pct']:.2f}% "
                f"00={b['zero_pct']:.2f}% "
                f"entropy={b['entropy']:.3f} "
                f"abnormal-pages={b['abnormal_pages']}"
            )

            print(
                f"       reasons: "
                f"{', '.join(b['reasons'])}"
            )

    else:
        print("*** NO SUSPICIOUS BLOCKS DETECTED ***")

    print()

    if suspicious_pages:

        print("*** FIRST 100 SUSPICIOUS PAGES ***")

        for p in suspicious_pages[:100]:
            print_page_summary(p)

        if len(suspicious_pages) > 100:
            print()
            print(
                f"... {len(suspicious_pages) - 100} "
                f"additional suspicious pages in CSV."
            )

    else:
        print("*** NO SUSPICIOUS PAGES DETECTED ***")

    print()

    print("ALL-FF BLOCKS:", len(erased_blocks))
    print("ALL-00 BLOCKS:", len(zero_blocks))

    print()

    print("OUTPUT FILES")
    print("-" * 78)
    print(f"Full report       : {report_txt}")
    print(f"Page report       : {page_csv}")
    print(f"Block report      : {block_csv}")
    print(f"Suspicious pages  : {suspicious_page_csv}")
    print(f"Suspicious blocks : {suspicious_block_csv}")

    print()
    print("=" * 78)
    print("ANALYSIS COMPLETE")
    print("=" * 78)

    return 0


if __name__ == "__main__":
    sys.exit(main())