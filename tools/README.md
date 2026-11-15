# 教材构建与检查工具

普通读者直接打开根目录 `index.html`，不需要运行这里的脚本。下面说明作者如何维护 L01。

## 从已冻结资料重建预览

需要 Python 3、`requirements-build.txt` 中的 Markdown 3.7，以及命令行可用的 Graphviz `dot`。在教材根目录执行：

```bash
python3 -m venv .build_venv
.build_venv/bin/python -m pip install -r requirements-build.txt
.build_venv/bin/python tools/build_l01.py
```

这组是 Linux Bash 命令。它读取已经提交的 baseline，生成固定版本引用、源码阅读页、四张 SVG 图和根目录预览；不启动机器人。构建会重新生成源码导航和文档尾部引用定义，因此这些生成部分应在生成器中维护。

## 冻结来源

`freeze_baseline.py` 只供原作者机使用，读取 `/home/ceihao/robot_project` 和已核实的 Git 树缓存，核对 821 个源码文件，复制精选快照，收集只读依赖信息。源码不一致会失败；历史文档字节不一致时读取固定 Git 提交并核对 blob。它不修改原机器人文件。

普通读者无需重新冻结，否则没有原地图与完整工程的电脑自然无法完成此步骤。资料来源与哈希见 baseline/manifest.json。

## 检查

`validate_l01.py` 检查冻结快照、生成网页的本地链接和锚点，并执行两个离线例子。先按样章编译 C++ 示例再运行检查；这需要本机已有 nlohmann/json 开发头文件。

`browser_check.py` 是作者机的 Linux Chrome 检查脚本，使用临时独立浏览器配置与 websocket-client。它只读取本地教材页面，退出时关闭自己创建的浏览器。当前实际运行时使用原工程 scene 环境中的 websocket-client；该脚本及其验证范围不等于 Windows 原生浏览器已经实测。

所有结果写入 validation。通过教材检查不升级机器人任务的验收状态。`.build_venv`、编译输出和发布中间数据不纳入学习仓库。
