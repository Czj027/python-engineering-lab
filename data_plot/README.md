# CSV/Excel 数据清洗与批量出图

一条命令读入原始表格、清洗数据并为每个选中的纵轴变量生成论文插图。可传入多个文件或目录，每个文件输出到独立子目录。原始文件只读，重复运行会为新结果分配编号目录。

## 安装与运行

在本目录使用 Python 3.13 安装一次依赖。

```bash
python -m pip install -r requirements.txt
python plot_data.py examples/sensors.csv examples/calibration.xlsx --output plots
python plot_data.py examples/drive.csv --decimal "," --output plots
```

支持 `.csv`、`.xlsx`、`.xlsm` 和 `.xls`。目录按当前层批量读取。CSV 自动尝试 UTF-8、带 BOM 的 UTF-16 或 GB18030，自动检测逗号、分号、制表符、竖线分隔。默认小数点是 `.`，以 `,` 作小数点的文本请加 `--decimal ","`。数值文本请去掉千位分隔符。

## 输入与清洗

从第一行表头开始组织数据。默认以第一列为横轴，将有单位标记或有可解析数值的其余列作为纵轴。用 `--x`、重复的 `--y` 可明确指定完整表头。推荐在表头写 `Time [s]`、`Temperature [°C]`，也可用括号写单位。缺少单位时用 `--unit '列名=单位'`，无量纲变量可写 `1`。

```csv
Time [s],Temperature [°C],Pressure [kPa],Note
2,24.6,103.1,warm-up
0,23.1,100.8,start
1,23.5,101.9,
3,25.3,,pressure dropout
```

- 去除单元格首尾空格，将空单元格、`NA`、`N/A`、`null`、`NaN`、`None`、`-` 识别为缺失值。默认给出警告，删除缺少横坐标或所有纵轴变量均缺失的行。单个变量缺值时保留同行的其他变量，该变量的曲线在对应位置断开。`--strict-missing` 将任一选中列缺值视为错误。
- 对选中列去除重复的数据行，按横轴稳定排序。相同横坐标但测量值不同的行会保留。每个变量至少需要两个不同横坐标处的有效值。
- Excel 默认读第一个工作表。用 `--sheet Calibration` 选表名，或用 `--sheet "#1"` 选第二张表。`--x-mode datetime` 解析 ISO 日期时间并按 UTC 绘图，无时区值按 UTC 解释。
- 非法数值、疑似乱码表头、列数不符、缺列和空文件会显示错误并指出文件。同批其他文件仍可处理，发生任何文件错误时退出码为 `1`。

指定列与补单位的例子，假定 `readings.csv` 表头为 `Time,Current`。

```bash
python plot_data.py readings.csv --x Time --y Current --unit "Time=s" --unit "Current=A" -o plots
```

用 `python plot_data.py --help` 查看强制编码、分隔符和字体等参数。中文表头会选用覆盖全部图中文字的已安装字体。需要指定时可用 `--font "Microsoft YaHei"` 或 `--font "Noto Sans CJK SC"`。

## 示例输入与输出

| 示例输入 | 格式 | 清洗前后 | 输出图 |
|---|---|---|---|
| `examples/sensors.csv` | UTF-8、逗号，含缺失值和重复行 | 9 → 7 行 | 温度、压力 |
| `examples/drive.csv` | GB18030、分号、逗号小数 | 7 → 7 行 | 转速、转矩 |
| `examples/calibration.xlsx` | Excel 原生数值单元格 | 7 → 7 行 | 载荷、电压 |

每个结果目录包含 `cleaned.csv`、`cleaning.json` 及每个变量的 `PNG + PDF`。PNG 为 300 dpi，PDF 为矢量图。图例使用变量名，横纵轴都标明单位，同一图使用统一字体。 [查看温度 PNG](../results/data_plot/sensors_csv/01_temperature.png) · [查看温度 PDF](../results/data_plot/sensors_csv/01_temperature.pdf) · [查看清洗日志](../results/data_plot/sensors_csv/cleaning.json)

`examples/invalid/` 包含空文件、乱码表头、混入非法数值、整列缺失四种失败样例。运行 `python plot_data.py examples/invalid --output plots` 可检查所有异常提示。也可加 `--strict-missing` 对 `sensors.csv` 检查普通缺失值的报错。
