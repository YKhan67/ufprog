from pathlib import Path
import sys

# Usage:
#   python check_bin.py "D:\path\to\your\dump.bin"

PAGE_SIZE = 2048
OOB_SIZE = 64
PAGE_WITH_OOB = PAGE_SIZE + OOB_SIZE
PAGES_PER_BLOCK = 64
BLOCK_SIZE = PAGE_SIZE * PAGES_PER_BLOCK
BLOCK_WITH_OOB = PAGE_WITH_OOB * PAGES_PER_BLOCK

EXPECTED_MAIN = 128 * 1024 * 1024
EXPECTED_OOB = 131 * 1024 * 1024

if len(sys.argv) != 2:
    print(f"Usage: python {Path(sys.argv[0]).name} <file.bin>")
    sys.exit(1)

path = Path(sys.argv[1])

if not path.is_file():
    print(f"ERROR: file not found: {path}")
    sys.exit(1)

size = path.stat().st_size

print("=" * 70)
print("SPI NAND BIN FILE ANALYSIS")
print("=" * 70)
print(f"File       : {path}")
print(f"Size       : {size:,} bytes")
print(f"Size       : {size / 1024 / 1024:.3f} MiB")
print(f"Size       : {size / 1_000_000:.3f} MB")
print()

print("EXPECTED SIZES")
print(f"128 MiB main area       : {EXPECTED_MAIN:,} bytes")
print(f"128 MiB + 64B OOB/page  : {EXPECTED_OOB:,} bytes")
print()

if size == EXPECTED_MAIN:
    print("FORMAT: 2048-byte MAIN AREA ONLY")
    has_oob = False
elif size == EXPECTED_OOB:
    print("FORMAT: 2048-byte MAIN + 64-byte OOB")
    has_oob = True
else:
    print("FORMAT: NOT AN EXACT STANDARD 128-MiB MAIN/OOB DUMP")
    has_oob = None

print()

if has_oob is False:
    total_pages = size // PAGE_SIZE
    total_blocks = total_pages // PAGES_PER_BLOCK
elif has_oob is True:
    total_pages = size // PAGE_WITH_OOB
    total_blocks = total_pages // PAGES_PER_BLOCK
else:
    print("Cannot determine exact page layout from size alone.")
    total_pages = None
    total_blocks = None

if total_pages is not None:
    print("GEOMETRY")
    print(f"Page size             : {PAGE_SIZE} bytes")
    print(f"OOB size              : {OOB_SIZE} bytes")
    print(f"Pages                 : {total_pages:,}")
    print(f"Pages/block           : {PAGES_PER_BLOCK}")
    print(f"Blocks                : {total_blocks:,}")
    print(f"Block main size       : {BLOCK_SIZE:,} bytes")
    print(f"Block main size       : {BLOCK_SIZE / 1024:.0f} KiB")
    print()

# ------------------------------------------------------------
# Block 535 analysis
# ------------------------------------------------------------

TARGET_BLOCK = 535

if total_blocks is not None and TARGET_BLOCK < total_blocks:

    print("=" * 70)
    print(f"BLOCK {TARGET_BLOCK} ANALYSIS")
    print("=" * 70)

    first_page = TARGET_BLOCK * PAGES_PER_BLOCK
    last_page = first_page + PAGES_PER_BLOCK - 1

    print(f"First page            : {first_page}")
    print(f"Last page             : {last_page}")
    print()

    with path.open("rb") as f:

        if has_oob is False:
            block_offset = TARGET_BLOCK * BLOCK_SIZE
            block_length = BLOCK_SIZE

        else:
            block_offset = TARGET_BLOCK * BLOCK_WITH_OOB
            block_length = BLOCK_WITH_OOB

        f.seek(block_offset)
        block = f.read(block_length)

    print(f"File offset           : 0x{block_offset:X}")
    print(f"Block length          : {len(block):,}")
    print()

    non_ff = sum(b != 0xFF for b in block)
    non_00 = sum(b != 0x00 for b in block)

    print(f"Non-FF bytes          : {non_ff:,}")
    print(f"Non-00 bytes          : {non_00:,}")

    if non_ff == 0:
        print("BLOCK 535 IS COMPLETELY FF IN THE BIN")
    else:
        print("BLOCK 535 CONTAINS DATA IN THE BIN")

    print()

    # First 256 bytes of each page
    print("FIRST 256 BYTES OF EACH PAGE")
    print("-" * 70)

    with path.open("rb") as f:

        for page_in_block in range(PAGES_PER_BLOCK):

            page = first_page + page_in_block

            if has_oob is False:
                offset = page * PAGE_SIZE
            else:
                offset = page * PAGE_WITH_OOB

            f.seek(offset)
            data = f.read(256)

            non_ff_page = sum(b != 0xFF for b in data)

            print(
                f"Page {page:5d} "
                f"(block+{page_in_block:02d}) "
                f"non-FF={non_ff_page:3d} : "
                + " ".join(f"{b:02X}" for b in data[:32])
            )

    print()

    # OOB analysis
    if has_oob:

        print("=" * 70)
        print("OOB ANALYSIS - BLOCK 535")
        print("=" * 70)

        with path.open("rb") as f:

            for page_in_block in range(PAGES_PER_BLOCK):

                page = first_page + page_in_block
                offset = page * PAGE_WITH_OOB + PAGE_SIZE

                f.seek(offset)
                oob = f.read(OOB_SIZE)

                non_ff = sum(b != 0xFF for b in oob)

                print(
                    f"Page {page:5d} "
                    f"OOB non-FF={non_ff:2d} : "
                    + " ".join(f"{b:02X}" for b in oob)
                )

    print()

# ------------------------------------------------------------
# Search entire dump for obvious bad-block markers
# ------------------------------------------------------------

if has_oob:

    print("=" * 70)
    print("BAD-BLOCK MARKER SCAN")
    print("=" * 70)

    print("Scanning first/second page OOB of every block...")
    print()

    bad_blocks = []

    with path.open("rb") as f:

        for block in range(total_blocks):

            first_page = block * PAGES_PER_BLOCK

            found_marker = False

            for page_in_block in (0, 1):

                page = first_page + page_in_block
                offset = page * PAGE_WITH_OOB + PAGE_SIZE

                f.seek(offset)
                oob = f.read(OOB_SIZE)

                # Common NAND bad-block marker locations:
                # byte 0 and byte 1 of OOB.
                if len(oob) >= 2:
                    if oob[0] != 0xFF or oob[1] != 0xFF:
                        found_marker = True

            if found_marker:
                bad_blocks.append(block)

    print(f"Blocks with non-FF first OOB bytes: {len(bad_blocks)}")

    if bad_blocks:
        print()
        print("Blocks:")
        for b in bad_blocks:
            print(f"  {b}")

        if TARGET_BLOCK in bad_blocks:
            print()
            print(f"BLOCK {TARGET_BLOCK} HAS A BAD-BLOCK MARKER IN THE BIN")
        else:
            print()
            print(f"BLOCK {TARGET_BLOCK} HAS NO COMMON BAD-BLOCK MARKER")

else:
    print("=" * 70)
    print("BAD-BLOCK MARKER SCAN")
    print("=" * 70)
    print("Skipped: BIN contains no OOB area.")

print()

# ------------------------------------------------------------
# Overall entropy-ish byte statistics
# ------------------------------------------------------------

print("=" * 70)
print("OVERALL FILE CHECK")
print("=" * 70)

ff_count = 0
zero_count = 0
total = 0

with path.open("rb") as f:
    while True:
        chunk = f.read(1024 * 1024)
        if not chunk:
            break

        total += len(chunk)
        ff_count += chunk.count(0xFF)
        zero_count += chunk.count(0x00)

print(f"Total bytes           : {total:,}")
print(f"FF bytes              : {ff_count:,} ({ff_count / total * 100:.2f}%)")
print(f"00 bytes              : {zero_count:,} ({zero_count / total * 100:.2f}%)")

print()
print("=" * 70)
print("DONE")
print("=" * 70)