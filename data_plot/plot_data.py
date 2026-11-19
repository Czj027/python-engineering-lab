"""Clean tabular measurements and export one publication figure per signal."""

import argparse
import csv
import io
import json
from numbers import Real
from pathlib import Path
import re
import shutil
import sys
from tempfile import TemporaryDirectory
from xml.etree.ElementTree import ParseError
from zipfile import BadZipFile

import matplotlib
matplotlib.use("Agg")
from matplotlib import dates, font_manager, ft2font
import matplotlib.pyplot as plt
import numpy as np
from openpyxl.utils.exceptions import InvalidFileException
import pandas as pd
from xlrd import XLRDError


SUPPORTED = {".csv", ".xlsx", ".xlsm", ".xls"}
MISSING = {"", "na", "n/a", "null", "nan", "none", "-"}
DELIMITERS = {"comma": ",", "semicolon": ";", "tab": "\t", "pipe": "|"}
UNIT_PATTERN = re.compile(r"^(.+?)\s*(?:\[([^\[\]]+)\]|\(([^()]+)\)|（([^（）]+)）)$")
GARBLED_HEADER = re.compile(r"[\u0000-\u001f\u007f-\u009f\ufffd]|锟斤拷|烫烫烫|Ã[\u00a0-\u00bf]|Â°|â€|æµ‹")


class DataError(ValueError):
    """A user-correctable input error."""


def text_cell(value):
    return "" if pd.isna(value) else str(value).strip()


def csv_rows(path, encoding, delimiter):
    raw = path.read_bytes()
    if not raw:
        raise DataError("空文件：没有可读数据。")
    if encoding:
        candidates = [encoding]
    elif raw.startswith((b"\xff\xfe", b"\xfe\xff")):
        candidates = ["utf-16"]
    else:
        candidates = ["utf-8-sig", "gb18030"]
    for candidate in candidates:
        try:
            content = raw.decode(candidate).removeprefix("\ufeff")
        except (UnicodeError, LookupError):
            continue
        break
    else:
        raise DataError("无法解码 CSV，可能存在乱码。请检查原始编码或指定 --encoding。")
    if not content.strip():
        raise DataError("空文件：没有可读数据。")
    if not delimiter:
        first = next(line for line in content.splitlines() if line.strip())
        try:
            delimiter = csv.Sniffer().sniff(first, delimiters=",;\t|").delimiter
        except csv.Error:
            delimiter = ","
    reader = csv.reader(io.StringIO(content, newline=""), delimiter=delimiter, strict=True)
    rows = []
    for row in reader:
        if not row or (len(row) == 1 and not row[0].strip()):
            continue
        if rows and len(row) != len(rows[0]):
            raise DataError(f"CSV 第 {reader.line_num} 行有 {len(row)} 列，表头有 {len(rows[0])} 列。"
                            "请检查引号、分隔符或补齐空单元格。")
        rows.append(row)
    return rows, {"encoding": candidate, "delimiter": delimiter}


def read_table(path, args):
    if path.suffix.lower() == ".csv":
        rows, details = csv_rows(path, args.encoding, DELIMITERS.get(args.delimiter))
    else:
        sheet = int(args.sheet[1:]) if re.fullmatch(r"#\d+", args.sheet) else args.sheet
        engine = "xlrd" if path.suffix.lower() == ".xls" else "openpyxl"
        if path.stat().st_size == 0:
            raise DataError("空文件：工作簿没有内容。")
        try:
            frame = pd.read_excel(path, sheet_name=sheet, header=None, dtype=object, engine=engine,
                                  keep_default_na=False)
        except (KeyError, ParseError, BadZipFile, InvalidFileException, XLRDError) as exc:
            raise DataError(f"Excel 工作簿格式不完整或已损坏：{exc}") from exc
        rows, details = frame.values.tolist(), {"sheet": sheet, "engine": engine}

    if not rows:
        raise DataError("空文件或空工作表：没有表头和数据。")
    header = [text_cell(value) for value in rows[0]]
    for ordinal, column in enumerate(header, start=1):
        if GARBLED_HEADER.search(column):
            raise DataError(f"第 {ordinal} 列表头疑似乱码：{column!r}。请检查编码或修正表头。")
        if not column:
            raise DataError(f"第 {ordinal} 列表头为空，请提供列名。")
    if len(header) != len(set(header)):
        raise DataError("存在重复列名，请为每列指定唯一名称。")
    if len(rows) == 1:
        raise DataError("只有表头，文件没有数据行。")
    def normalize(value):
        content = text_cell(value)
        if isinstance(value, Real) and not isinstance(value, bool) and args.decimal == ",":
            return content.replace(".", ",")
        return content

    return pd.DataFrame(rows[1:], columns=header, dtype=object).map(normalize), details


def is_missing(series):
    return series.str.casefold().isin(MISSING)


def numeric(series, decimal):
    values = series.mask(is_missing(series))
    if decimal == ",":
        # With comma-decimal input, a period is not a decimal or grouping mark.
        values = values.mask(values.str.contains(".", regex=False, na=False)).str.replace(",", ".", regex=False)
    numbers = pd.to_numeric(values, errors="coerce")
    return numbers.where(np.isfinite(numbers))


def axis_parts(column):
    match = UNIT_PATTERN.fullmatch(column)
    if match:
        label, *units = match.groups()
        unit = next(unit.strip() for unit in units if unit is not None)
        return label.strip(), unit
    return column, ""


def axis_description(column, overrides, default_unit=""):
    label, unit = axis_parts(column)
    unit = overrides.get(column, default_unit or unit)
    if not unit:
        raise DataError(f"列 {column!r} 缺少单位，请填写 [单位] 或用 --unit '列名=单位'。"
                        "无量纲可用 1。")
    return {"column": column, "label": label, "unit": unit}


def parse_columns(raw, args, units):
    x = args.x or raw.columns[0]
    required = [x, *args.y, *units]
    absent = [column for column in required if column not in raw.columns]
    if absent:
        raise DataError(f"表中找不到列：{', '.join(repr(column) for column in absent)}。")
    if x in args.y or len(set(args.y)) != len(args.y):
        raise DataError("纵轴列必须互不重复，且不能与横轴列相同。")

    parsed = {column: numeric(raw[column], args.decimal) for column in raw.columns if column != x}
    signals = args.y or [column for column, values in parsed.items()
                         if column in units or axis_parts(column)[1] or values.notna().any()]
    if not signals:
        raise DataError("没有可绘制的数值列，请检查表格或显式指定 --y。")
    if args.x_mode == "datetime":
        parsed[x] = pd.to_datetime(raw[x].mask(is_missing(raw[x])), format="ISO8601", errors="coerce", utc=True)
    else:
        parsed[x] = numeric(raw[x], args.decimal)

    selected = [x, *signals]
    for column in selected:
        missing = is_missing(raw[column])
        invalid = ~missing & parsed[column].isna()
        if invalid.any():
            sample = ", ".join(repr(value) for value in raw.loc[invalid, column].unique()[:3])
            problem = "非 ISO 日期时间" if column == x and args.x_mode == "datetime" else "非数值或非有限值"
            raise DataError(f"列 {column!r} 有 {invalid.sum()} 个{problem}：{sample}。")
        if missing.any() and args.strict_missing:
            raise DataError(f"列 {column!r} 有 {missing.sum()} 个缺失值（--strict-missing）。")

    cleaned = pd.DataFrame({column: parsed[column] for column in selected})
    no_x = int(cleaned[x].isna().sum())
    missing_before = {column: int(cleaned[column].isna().sum()) for column in signals}
    cleaned = cleaned.dropna(subset=[x])
    no_signal = int(cleaned[signals].isna().all(axis=1).sum())
    cleaned = cleaned.dropna(subset=signals, how="all")
    duplicates = int(cleaned.duplicated().sum())
    cleaned = cleaned.drop_duplicates().sort_values(x, kind="stable")
    x_info = axis_description(x, units, "UTC" if args.x_mode == "datetime" else "")
    if args.x_mode == "datetime":
        x_info["unit"] = "UTC"
    series = []
    for column in signals:
        points = cleaned.loc[cleaned[column].notna(), x]
        if points.nunique() < 2:
            raise DataError(f"列 {column!r} 有效数据点不足，需要至少两个不同横坐标的有效值。")
        series.append({**axis_description(column, units), "missing": missing_before[column],
                       "plotted_points": len(points)})
    report = {"x": x_info, "x_mode": args.x_mode, "rows_read": len(raw), "rows_written": len(cleaned),
              "rows_without_x": no_x, "rows_without_signals": no_signal,
              "duplicate_rows": duplicates, "series": series,
              "ignored_columns": [column for column in raw.columns if column not in selected]}
    return cleaned, report


def output_font(report, requested):
    labels = [entry[key] for entry in [report["x"], *report["series"]] for key in ["label", "unit"]]
    characters = {ord(character) for label in labels for character in label if not character.isspace()}
    choices = [requested] if requested else ["DejaVu Sans", "Microsoft YaHei", "Noto Sans CJK SC",
                                             "SimHei", "PingFang SC", "WenQuanYi Zen Hei"]
    for name in choices:
        try:
            font = font_manager.findfont(font_manager.FontProperties(family=[name]), fallback_to_default=False)
        except ValueError:
            continue
        if characters.issubset(ft2font.FT2Font(font).get_charmap()):
            return name
    raise DataError("找不到覆盖所有图中文字的字体，请安装相应字体并用 --font 指定名称。")


def save_plots(staging, data, report):
    style = {"font.family": report["font"], "font.size": 10, "pdf.fonttype": 42,
             "axes.unicode_minus": False, "text.parse_math": False}
    with plt.rc_context(style):
        x_info = report["x"]
        for ordinal, series in enumerate(report["series"], start=1):
            fig, axis = plt.subplots(figsize=(6.3, 3.8), layout="constrained")
            try:
                x = data[x_info["column"]]
                y = data[series["column"]]
                axis.plot(x, y, label=series["label"], color="#146685", linewidth=1.65,
                          marker="o" if len(y) <= 30 else None, markersize=4)
                axis.set(xlabel=f"{x_info['label']} ({x_info['unit']})",
                         ylabel=f"{series['label']} ({series['unit']})")
                if report["x_mode"] == "datetime":
                    locator = dates.AutoDateLocator()
                    axis.xaxis.set_major_locator(locator)
                    axis.xaxis.set_major_formatter(dates.ConciseDateFormatter(locator))
                axis.grid(alpha=0.30, color="#95aabb", linewidth=0.7)
                axis.set_axisbelow(True)
                axis.spines[["top", "right"]].set_visible(False)
                axis.legend(loc="best", frameon=False)
                slug = re.sub(r"[^a-z0-9]+", "_", series["label"].lower()).strip("_")[:45] or "signal"
                name = f"{ordinal:02d}_{slug}"
                for extension in ("png", "pdf"):
                    fig.savefig(staging / f"{name}.{extension}", dpi=300)
                series["figure"] = name
            finally:
                plt.close(fig)


def publish(source, output, args, units):
    table, details = read_table(source, args)
    data, report = parse_columns(table, args, units)
    report.update(source=source.name, reader=details, font=output_font(report, args.font), png_dpi=300)
    output.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix=".plot-", dir=output) as tmp:
        staging = Path(tmp)
        save_plots(staging, data, report)
        data.to_csv(staging / "cleaned.csv", index=False, encoding="utf-8-sig")
        (staging / "cleaning.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n",
                                               encoding="utf-8")
        prefix = f"{source.stem[:65]}_{source.suffix[1:].lower()}"
        dest = output / prefix
        suffix = 2
        while True:
            try:
                dest.mkdir()
                break
            except FileExistsError:
                dest = output / f"{prefix}_{suffix}"
                suffix += 1
        try:
            for item in staging.iterdir():
                item.replace(dest / item.name)
        except OSError:
            shutil.rmtree(dest)
            raise
    return dest, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, nargs="+", help="CSV/Excel files or directories (non-recursive)")
    parser.add_argument("-o", "--output", type=Path, default=Path("plots"))
    parser.add_argument("--x", help="Exact x column header; first column by default")
    parser.add_argument("--y", action="append", default=[], help="Exact y column header; repeat to select several")
    parser.add_argument("--unit", action="append", default=[], metavar="COL=UNIT")
    parser.add_argument("--x-mode", choices=["numeric", "datetime"], default="numeric")
    parser.add_argument("--sheet", default="#0", help="Excel sheet name or zero-based index as #0, #1, ...")
    parser.add_argument("--encoding", help="CSV text encoding override, e.g. gb18030")
    parser.add_argument("--delimiter", choices=DELIMITERS)
    parser.add_argument("--decimal", choices=[".", ","], default=".", help="Decimal separator in text cells")
    parser.add_argument("--font", help="Installed font family")
    parser.add_argument("--strict-missing", action="store_true", help="Reject missing values in selected columns")
    args = parser.parse_args()
    units = {}
    for rule in args.unit:
        column, separator, unit = (part.strip() for part in rule.partition("="))
        if not separator or not column or not unit or column in units:
            parser.error("--unit 要求唯一的非空 '列名=单位' 规则。")
        units[column] = unit

    args.output = args.output.resolve()
    sources = []
    for path in args.input:
        if path.is_dir():
            files = sorted(child for child in path.iterdir() if child.is_file()
                           and child.suffix.lower() in SUPPORTED and not child.name.startswith("~$"))
            if not files:
                parser.error(f"目录 {path} 没有支持的 CSV 或 Excel 文件。")
            sources.extend(files)
        else:
            sources.append(path)

    failed = False
    for source in dict.fromkeys(sources):
        try:
            if source.suffix.lower() not in SUPPORTED:
                raise DataError("扩展名不支持，接受 .csv、.xlsx、.xlsm、.xls。")
            dest, report = publish(source, args.output, args, units)
            missing = sum(item["missing"] for item in report["series"])
            if missing or report["rows_without_x"]:
                print(f"[WARN] {source.name}: 选中列含缺失值。缺少横坐标 {report['rows_without_x']} 行，"
                      f"纵轴缺失 {missing} 个单元格，请查看 cleaning.json。", file=sys.stderr)
            print(f"[OK] {source.name}: {report['rows_read']} → {report['rows_written']} 行，"
                  f"{len(report['series'])} 组图 → {dest}")
        except (OSError, ValueError, csv.Error) as exc:
            print(f"[ERROR] {source}: {exc}", file=sys.stderr)
            failed = True
    return int(failed)


if __name__ == "__main__":
    sys.stdout.reconfigure(errors="backslashreplace")
    raise SystemExit(main())
