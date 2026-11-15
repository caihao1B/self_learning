# 教材构建与检查工具

普通读者直接打开根目录 `index.html`，不需要运行这里的脚本。下面说明作者如何维护 L01 和 L02。

## L02 构建与检查

在已安装 Markdown 3.7 的 `.build_venv` 中执行：

```bash
python3 tools/build_l02_figures.py
.build_venv/bin/python tools/build_l02.py
.build_venv/bin/python tools/validate_l02.py
```

图表需要 Graphviz `dot`。检查 C++ 需要 g++、Eigen3 与 nlohmann/json 开发头文件。`build_l02.py` 根据 `l02_catalog.py` 从固定源码提取原样片段，保留手写正文，运行无 ROS 的教学例子并构建分章阅读页。生成片段由明确注释分隔，不手改这些片段。

`freeze_l02.py` 仅供原作者机从同一 Git 树追加核对来源，普通读者无需执行。`browser_l02.py` 使用 Linux Chrome 与 websocket-client，核查每章本地阅读；Windows 原生体验仍需人工确认。教材代码和静态检查不意味着重新运行了机器人任务。

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

## L03 学习站

普通 Windows 读者直接打开根目录 `index.html`。站点运行不需要 Node.js、Python 或 npm；构建器需要已安装 Markdown 3.7 的 Python 环境：

```bash
.build_venv/bin/python tools/build_l03.py
.build_venv/bin/python tools/validate_l03.py
```

`build_l03.py` 会先核对 L03/input_manifest.json 中的 274 个不变输入，再构建跨章索引、源码数据和根首页。不要重新运行 build_l01.py 覆盖当前首页；如需维护历史阅读版，先阅读构建脚本的写入范围。

`browser_l03.py` 使用 Chrome 和 websocket-client，在独立配置与临时目录中检查真实界面：HTTP 中文子目录、断网 file URL、搜索、复制文本、笔记下载/导入、窄屏、键盘与打印。测试结束关闭自己创建的浏览器和服务器；不使用用户的浏览器配置。

`package_l03.py` 将静态阅读资料复制到已注册的独立 Sites 源码工作目录，不包含凭证和本机运行时。正式站点由 Sites 源码/打包/发布辅助流程发布；GitHub 保存同一套教材与站点文件。见 L03/TECHNICAL.md 和 L03/RESULTS.md。
