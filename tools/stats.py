import argparse
import csv
import statistics
import sys
from rich.console import Console
from rich.table import Table


def parse_args():
    parser = argparse.ArgumentParser(
        description="Summary statistics for columns of a CSV saved by logger.py.")
    parser.add_argument("file", help="CSV file, e.g. logs/run_20261007_171309.csv")
    parser.add_argument("columns", nargs="+", metavar="COLUMN",
                        help="columns to summarise, e.g. dn1 dn2 x_mm")
    return parser.parse_args()


def read_columns(path, columns):
    """Returns {column: [values]} for the named columns, and the run length in seconds."""
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        missing = [c for c in columns if c not in reader.fieldnames]
        if missing:
            sys.exit(f"no column {', '.join(missing)} in {path}. "
                     f"Columns: {', '.join(reader.fieldnames)}")

        values = {c: [] for c in columns}
        times = []
        for row in reader:
            for c in columns:
                values[c].append(float(row[c]))
            if "t_ms" in row:
                times.append(float(row["t_ms"]))

    duration = (times[-1] - times[0]) / 1000 if len(times) > 1 else None
    return values, duration


def main():
    args = parse_args()
    values, duration = read_columns(args.file, args.columns)

    count = len(next(iter(values.values())))
    if count == 0:
        sys.exit(f"no records in {args.file}")

    title = f"{args.file}: {count} records"
    if duration is not None:
        title += f" over {duration:.2f} s"
    table = Table(title=title)
    for heading in ("Column", "Mean", "Min", "Max", "Std dev", "Variance"):
        table.add_column(heading, justify="left" if heading == "Column" else "right")

    # Population statistics: the file holds every record of the run.
    for column, data in values.items():
        table.add_row(
            column,
            f"{statistics.fmean(data):.4f}",
            f"{min(data):g}",
            f"{max(data):g}",
            f"{statistics.pstdev(data):.4f}",
            f"{statistics.pvariance(data):.4f}",
        )

    Console().print(table)


if __name__ == "__main__":
    main()
