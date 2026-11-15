# L03 技术设计与维护

## 同一份资料，两种访问方式

根目录 index.html 加载本地 CSS、book-data.js 与 site.js。没有打包器、CDN、运行时 fetch 或必须存在的服务端。Python 构建器将固定 Markdown 转成 HTML，把章节、小节索引、源码、素材描述打包成普通 JavaScript 数据文件；因此 file URL 不会遇到 fetch 本地 JSON 的跨域限制。

页面采用 hash 路由：chapter、source、materials、search、notebook 等路径都在 `#` 后。在线从子目录打开与 Windows 直接打开的资源路径一致，刷新不会要求服务器提供虚构的 /chapter/14 路径。无法执行脚本时仍有 L02 静态阅读入口。

## 固定输入与证据

input_manifest.json 记录 L03 开始时 274 个教材、源码和证据文件的 SHA-256。build_l03.py 每次生成前检查这些输入，任何改变都会中断。机器人基线保持 9aff8337；L02 学习提交记录为 bd1f9c6。L03 新增站点和使用指南，不重写原章节或历史验收。

原 L01 首页移至 legacy-l01.html，旧 # 锚点由新入口转回此页。L02 阅读页与全部固定源码快照保留。源码工作台显示的文本与快照按字节内容核查，复制按钮不会包含行号。

## 搜索实现

构建器按标题划分章节小节，保留标题、正文、代码、路由与定位锚点；源码以文件索引并在匹配后计算首次命中行。运行时执行不区分大小写的包含搜索，多词使用 AND，标题与小节命中优先。它是确定性的全文子串检索，不使用向量库或模型，不声称具备语义问答能力。

中英文无需额外分词库；代价是同义词、拼写纠错和很长自然语言提问的召回有限。界面建议改用短关键词，并提供资料类型过滤。结果摘要用文本转义后标记命中，查询内容不会作为 HTML 执行。

## 浏览器数据

学习记录使用基线限定的 localStorage key，包含 schema、baseline、完成章节、每章笔记、最后阅读位置和主题/字号。读写都处理存储异常。导出使用 Blob 下载，导入先检查格式、大小、章节和基线，再合并；迁移不依赖两个系统的文件路径一致。

Windows file URL 的持久化规则可能随浏览器与策略不同，所以导出是正式迁移方式。剪贴板优先使用 Clipboard API；file URL 无权限时提供兼容复制及手工选择提示。复制功能不会启动 shell。

## 构建与发布

只读用户无需构建。作者在已安装 requirements-build.txt 的 Python 环境运行：

```bash
.build_venv/bin/python tools/build_l03.py
.build_venv/bin/python tools/validate_l03.py
```

构建器生成 web/book-data.js 与根首页。web/site.js 与 site.css 是维护源文件。浏览器自动检查由 tools/browser_l03.py 执行，需要 Chrome 与 websocket-client；它使用独立临时浏览器配置和自己拥有的本地服务器。

站点静态包由发布准备脚本按白名单复制到独立 Sites 工作目录的 dist，排除 .authoring、虚拟环境、编译物和秘密。Sites 原生发布回执保存非敏感标识；短期 Git 凭证只经辅助工具的隐藏 stdin 传入，不写入仓库。GitHub 同步使用预期旧 SHA 保护，并比较发布树中每个文件的 Git blob。

该仓库不包含机器人控制后端。后续修正文稿应建立明确的新学习版本，而不是绕过固定输入检查或悄悄更新历史状态。
