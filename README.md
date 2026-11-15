# 农田机器人 · 自学工坊

从第一条 Linux 命令，到定位、规划、行为树与有效作业反馈。面向刚接触 Linux、ROS 2、C++ 和 Python 的学习者，以真实项目和固定证据为依据。

**L01、L02 教材与 L03 学习站已实现。** 在线发布状态见 [L03 检查报告](L03/RESULTS.md)。机器人基线保持 `9aff8337`，R10–R12 与原 S17/S18 仍为 PARTIAL。

## 开始学习

- **在线站点**：[农田机器人 · 自学工坊](https://ceihao-robot-learning.ceihao.chatgpt.site)。Sites 默认仅所有者访问，使用创建站点的同一 ChatGPT 账号登录。
- **Windows 离线**：点击 **Code → Download ZIP**，全部解压后双击根目录 `index.html`，用 Edge 或 Chrome 打开；阅读不需要 ROS、Python、Node.js 或服务器。
- **直接在 GitHub 阅读**：[三十章目录](L02/README.md) · [学习路线](L02/learning_routes.md) · [使用说明](L03/USAGE.md)。
- **备用阅读版**：下载后打开 `L02/index.html`。正文不依赖学习站脚本。

## 学习站提供什么

- 三十章完整教材、章节导航、字号调整、明暗主题和当前章打印。
- 跨章中文/英文搜索，支持函数、错误码与多词查询，定位到小节或代码行。
- 76 个固定源码文件：行号、函数导航、文件内搜索、文本复制与关联章节。
- 架构图解、GUI 历史截图、真实验收文档、三十个离线练习输出。
- 逐章完成标记与笔记，JSON 导出/导入，方便在 Linux 和 Windows 之间迁移。

搜索与笔记在浏览器本地处理，不调用模型、不连接机器人。在线和离线的笔记不会自动同步，切换系统前请导出记录。历史附件未随教材提供时会明确说明，不用空文件或新结论替代。

## 建议顺序

先读 01–07 章建立基础，再读第 08 章串起一次真实点喷闭环；09–25 章解释各技术层；26–30 章集中讲环境、操作、GUI、故障和研究。重点入口：[三点闭环](L02/chapters/08_spray_walkthrough.md)、[环境搭建](L02/chapters/26_environment.md)、[人工操作](L02/chapters/27_operations.md)、[故障分析](L02/chapters/29_failures.md)。

Windows 可以阅读全部正文与代码。真实机器人运行仍需要原 Linux 工程、ROS/Gazebo、大型地图和相应本地资源；学习仓库不是完整仿真环境镜像。相关边界见 [第 26 章](L02/chapters/26_environment.md)。

## 基线与检查

机器人源码固定于 [ROS2_BT- 的 9aff8337 提交](https://github.com/caihao1B/ROS2_BT-/commit/9aff8337b7794f704c5556a973a5220e73f17808)。L02 的三十章、56 段源码解读、Python 练习和 C++ 判据检查保持原记录。L03 在这些固定输入之上增加阅读站，没有改变机器人验收。

[固定来源清单](baseline/manifest.json) · [L03 不变输入清单](L03/input_manifest.json) · [L02 检查](L02/RESULTS.md) · [L03 检查](L03/RESULTS.md) · [站点技术设计](L03/TECHNICAL.md) · [维护工具](tools/README.md)

Windows 原生人工体验保持 **PENDING_USER**；Linux Chrome 的离线路径、HTTP 子目录和窄屏测试单独记录。原完整 A1/B1 及 S17/S18 的 PARTIAL 不因网站交付而升级。

L01 需求演变、架构、目录设计与原样章保留在 [L01](L01/00_baseline.md)，原聚合首页保留在 `L03/legacy-l01.html`。历史文档中的“下一步”按其记录时间理解。

原代码版权和第三方归属保持不变；本仓库没有推定或补授机器人项目的统一开源许可。
