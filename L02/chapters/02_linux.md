# 第 02 章 Linux 第一次操作

终端是人与程序交流的入口，Bash 是解释输入的程序，文件系统是组织工程的树状结构。刚开始不需要背很多命令，先掌握“我在哪里、操作哪个文件、命令怎样拆成参数、结果怎样判断”。本章的练习只操作临时学习目录，机器人原工程保持不变。

## 目录和路径

Linux 的根目录是 `/`，用户主目录通常是 `/home/用户名`。`/home/ceihao/robot_project` 是当前作者机的机器人目录，`/home/ceihao/self_learning` 是学习材料目录。两者用途不同。Windows 下载教材后可放在任意位置，但书中的 Linux 原工程绝对路径不能照搬为你的 Windows 路径。

绝对路径从 `/` 开始，不依赖当前目录。相对路径例如 `examples/l02/labs.py` 从当前目录开始寻找。`.` 表示当前目录，`..` 表示上一级。`~` 通常由 shell 展开为当前用户主目录；它不是 Python 路径字符串自动理解的特殊字符，Python 中需要 `Path.home()` 或 `expanduser()`。

```bash
cd /home/ceihao/self_learning
pwd
ls
ls -ld . L02 examples
```

`cd` 改变当前 shell 的目录；`pwd` 打印目录；`ls` 列出目录内容；`-l` 请求详细信息；`-d` 查看目录本身而不展开内容。`ls -ld` 中的两个选项可以合并，因为它们都是无额外参数的短选项；不能据此推断所有程序都支持任意合并。

## 一条命令怎样分词

下面的路径中有空格。双引号使它作为一个参数传给 `mkdir`，而不是分成三个名称。

```bash
mkdir -p "$HOME/robot learning notes"
printf '%s\n' '今天学习导航与作业的区别' > "$HOME/robot learning notes/day01.txt"
cat "$HOME/robot learning notes/day01.txt"
```

`mkdir -p` 创建缺失的父目录，已有目录不会因此报错。`$HOME` 是系统提供的用户主目录变量；使用它可以，不能把它改成项目工作变量。`printf` 按格式输出，`%s` 放入一个字符串，`\n` 表示换行。单引号中的内容一般保持字面意义，双引号允许变量展开。`>` 把标准输出重定向到文件，已有同名文件会被覆盖；`>>` 是追加。先确认目标路径，再使用重定向。

程序通常接收一个参数数组。例如 `python3 examples/l02/labs.py 02` 被拆成解释器、脚本、章节编号。`02` 在 Bash 中只是字符串，Python 的 `argparse` 再决定它是否为合法章节。不要把文档中的提示符 `$` 一起粘贴到命令行。

## 文件权限和执行

详细列表中的 `r`、`w`、`x` 分别表示读、写、执行/进入目录的权限。目录的 `x` 表示能通过它访问内部路径，与普通文件执行含义不同。`bash script.sh` 由 Bash 读取脚本；`./script.sh` 则要求脚本可执行且有合适的解释器声明。遇到 Permission denied 时先检查目标与权限，不要立即对整个工程递归 `chmod 777`。

```bash
test -f README.md
echo $?
test -r README.md
echo $?
command -v python3
python3 --version
```

`test` 通常不打印文字，使用退出码表达判断。`echo $?` 必须紧跟被检查的命令，否则拿到的是别的命令的状态。`command -v` 显示 shell 实际找到的程序，这比凭印象认为“已经安装 Python”更有用。

## 安全地找资料

`rg` 搜索文本，`rg --files` 列出文件。先在小范围找入口，再打开附近代码。不要从主目录递归输出所有日志，也不要用文件日期推断可删除性；旧地图和动态库可能仍被当前任务绑定。

```bash
rg --files L02
rg -n 'POINT_SERVICE' baseline/source/optimization_v2/ws/src/mission_interfaces/action/ToolTask.action
sed -n '1,80p' examples/inspect_spray.py
```

`-n` 显示行号；引号中的搜索串避免被 shell 特殊解释。`sed -n '1,80p'` 只打印 1 到 80 行，适合阅读短片段。这里的行号只对固定文件版本有效。`rg` 没有匹配时通常返回 1，这不是程序崩溃；命令不存在则通常由 shell 返回 127。

## Python 和 Bash 怎样处理同一个路径

Python 的 `Path` 对象可以连接路径、检查存在、读取 UTF-8 文本。`Path(__file__).resolve()` 从脚本自身位置出发，因此离线例子不依赖你把仓库下载在哪。Bash 的相对路径则主要取决于当前工作目录，所以教材在运行前明确写 `cd`。

实际项目的 `advance.sh` 使用 `${BASH_SOURCE[0]}` 找到脚本目录，再定位 `env.sh`。`$(...)` 是命令替换：先执行括号内命令，再把输出作为文本。双引号保留路径中的空格和中文。`"$@"` 保留每个参数边界，不能随意改成不加引号的 `$*`。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### advance.sh 第 1 至 6 行

[离线带行号源码](../../baseline/l02_views/142670af88c5-advance.sh.html#L1) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/three_tasks/advance.sh#L1)

```bash
#!/usr/bin/env bash
set -Eeuo pipefail
r_base="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$r_base/env.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
exec "$r_base/venvs/scene/bin/python" "$r_base/ws/src/task_platform/scripts/dispatch.py" "$@"
```

set 的选项影响 shell 错误行为；r_base 通过脚本自身路径计算。source 在当前 shell 加载环境，exec 替换 shell，最后的 "$@" 保留参数边界。把这些词按顺序解释，比死记启动命令更容易迁移目录。

<!-- generated-production-end -->

## 本章练习及预期

```bash
cd /home/ceihao/self_learning
python3 examples/l02/labs.py 02
```

例子用 `TemporaryDirectory` 创建带空格的目录，在其中写入中文文件，检查文件名、相对路径和内容，退出上下文时自动清理它自己创建的目录。`with` 管理资源生命周期；你无需手工找到临时目录再删除。这是学习临时文件管理，比在原工程随便创建和删除文件更容易复现。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_02.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab02():
    """Paths and quoting, with a disposable directory containing a space."""
    with tempfile.TemporaryDirectory(prefix='robot learning ') as tmp:
        p = Path(tmp) / '笔记.txt'
        p.write_text('navigation != work\n', encoding='utf-8')
        assert p.is_file() and p.read_text(encoding='utf-8').endswith('\n')
        return dict(absolute=p.is_absolute(), file_name=p.name,
                    relative=str(p.relative_to(tmp)), suffix=p.suffix,
                    exists_during_lab=p.exists(), temporary_cleanup=True)
```

```text
{
  "chapter": "02",
  "passed": true,
  "result": {
    "absolute": true,
    "file_name": "笔记.txt",
    "relative": "笔记.txt",
    "suffix": ".txt",
    "exists_during_lab": true,
    "temporary_cleanup": true
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 常见故障和排查顺序

`No such file or directory`：先执行 `pwd`，再 `ls` 检查上级目录；确认大小写、中文目录、空格是否正确。Linux 通常区分大小写，`L02` 与 `l02` 不同。若脚本文件明明存在，仍提示找不到解释器，检查首行解释器路径及 Windows CRLF 换行。

中文乱码：先看编辑器是否按 UTF-8 保存，再用 `locale` 查看环境。不要通过更改源码字符串来掩盖编码问题。教材 Python 示例显式指定 `encoding='utf-8'`，减少不同系统默认编码差异。

输出太多：缩小检索范围，使用 `sed` 阅读指定行，不要删除日志解决显示问题。复制路径时也不要把文件内容当命令执行。

## 自测与答案

问：为什么 `cd` 后的相对路径会改变，而绝对路径不变？答：相对路径以当前工作目录为起点；绝对路径以根目录为起点。

问：`echo $?` 输出 0 是否代表机器人作业完成？答：只代表上一条程序按自己的约定返回 0。`cat` 成功读完一个 PARTIAL 报告也会返回 0。

练习：在临时笔记目录保存“原任务分母”和“本次选择范围”两行文字，再用 `cat` 查看。能解释每个参数和重定向符号后进入第 03 章。
