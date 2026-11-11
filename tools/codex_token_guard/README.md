# Codex 全局 token 工作流 hook v1

根据 M2b_token_audit_001 的证据，将重点放在减少正常监看中的模型往返、短交接和摘要读取。适用于本机当前用户的 Codex CLI / IDE；不是跨主机、跨产品的组织级策略。

## 行为与开销

|事件|行为|上限|
|---|---|---|
|SessionStart|注入简短工作约定；普通 resume 不重复，compact/clear/fork 后恢复|每次上下文重建一份，约150个英文词|
|PreToolUse|本地计数；第3次可见等待类操作、大显式输出预算、累计50次可见工具时分别提醒|每类每会话最多1次，共3次；正常情况无输出|

阈值只是初版启发式，不是已验证最优参数。大显式输出预算指 `max_output_tokens` / `max_tokens` 大于4000或为0；它不是已产生的输出量，也不自动截断工具结果。等待类识别只看可见工具名/参数中的特征，不解析或执行代码；混合代码中可能误报。命令和所需验证继续正常执行，提醒不导致自动重试或新的模型调用。

两个 handler 都设2秒超时、`additionalContextLimit=400`。这只限制各 hook 的附加上下文，不限制整个会话或工具输出。hook 每次启动本地 Python，不调用模型、网络或子进程；不读取会话全文、日志正文或认证文件。只保存匿名会话键、近期工具ID的哈希、计数、提醒标记与时间；没有原始命令、提示或响应正文。状态位于 `~/.codex/hooks/token_guard/state/`，可在会话结束后按需删除；删状态会使提醒重新计数。

**这是工作方式提醒，不能保证省 token。** 现有 hook 无法自动消除轮询、压缩历史、清空上下文或让长实验自动推进。尤其原生 `write_stdin` 不再次触发 PreToolUse，托管工具和部分专用路径也可能不可见；代码模式的外层是否触发取决于客户端。计数不是完整调用计数、HTTP数、token数或纯等待关联用量。子代理可能共享父会话计数；已明确跳过可识别的 codex-auto-review/guardian。

## 安装与启用

```sh
python3 tools/codex_token_guard/test_guard.py
python3 tools/codex_token_guard/install.py
```

安装器合并当前 `CODEX_HOME`（默认 `~/.codex`）的 hooks.json，保留其他 hook，并在改动已有文件前保存 `hooks.json.before-token-guard`。独立安装脚本使用绝对路径，可从任何项目启动。相同版本重装幂等；发现同名脚本或 handler 内容不同会拒绝覆盖，升级需先审查。它不修改 config.toml、模型、推理等级、权限、信任哈希或 guardian。

安装后在 CLI 输入 `/hooks`，或 IDE 的 Settings → Hooks 中检查并信任本包新增的 SessionStart 和 PreToolUse 定义，再在新会话确认。不要使用 bypass-hook-trust，也不要批量信任不相关 hook。安装本身不等于已信任或已触发；变更定义后可能需要重新审核。

本次安装验证：本地 app-server 的 `initialize` + `hooks/list` 已识别本包两条 user hook，`warnings=[]`、`errors=[]`，均为 `enabled=true` / `trustStatus=untrusted`；安装脚本与工作区源文件 SHA256 一致。完整小型回执见 [verification.json](verification.json)。未创建真实模型会话，也未修改信任状态，尚待用户审核与下一轮工作流实测。

临时停用：在 `/hooks` 中仅关闭本包两条 handler。卸载：从 hooks.json 仅移除 command 指向 `hooks/token_guard/guard.py` 的两个 handler；保留同组其他 handler，再删除专用 token_guard 目录。不直接恢复旧备份覆盖后来加入的其他 hook。

## 真正减少等待往返的执行约定

1. 开始前将阶段、超时、停止条件、失败摘要和准确恢复入口固定；复用已有实验驱动，不为本包另建实验执行器。
2. 正常阶段由驱动自动推进，保持失败非零退出、完整磁盘日志、必要独立验收；优先使用平台确实支持的完成/失败通知。否则低频、有界、可中断观察，遵守用户进度更新要求，不用超长阻塞等待代替。
3. 状态只展示阶段/进度/退出码/失败层/证据路径。模型在失败、阶段结束或需要判断时展开证据；不得吞掉异常、删除必需重复或通过标签伪造成功。
4. 自然阶段边界保存短交接：目标、固定约束、源版本、证据路径、问题、未完成项、准确恢复命令。新会话由用户/实际客户端工作流开启，hook不自行创建；不为省 token 中断当前授权任务。
5. 复用已验证验收/归档工具；写入和恢复按内容校验。hook不替代工具持久化修复，也不再次审计历史全量记录。

## 下一轮怎么验证收益

沿用原审计的逐响应增量口径，分主模型/guardian，分输入/其中缓存/输出/其中推理；另统计完整工具日志中的监看次数、纯等待关联记录和上下文规模。hook状态只能作为覆盖有限的辅助计数，真实usage仍来自任务会话记录。比较前注明任务内容/难度/场景差异，保留失败发现、验收与有效产出。无实测前不报节省比例，不换算费用/额度。此版本只做合成协议测试，未启动真实模型会话或新实验。

依据：[官方 Hooks 文档](https://learn.chatgpt.com/docs/hooks)。本机 Codex CLI 0.164.0-alpha.11.1，`features list` 显示 hooks stable=true；已安装 IDE 提供 hook 信任界面。文档明确要求非托管 hook 先审查信任，并说明 write_stdin 的覆盖限制。不同客户端/版本或云编排需重新确认支持范围。
