# FreeCAD MCP 安装与验证

本文件记录本仓库使用的 FreeCAD MCP 连接方式，供后续 AI 或开发者直接执行。
目标是让 Codex 通过 MCP 控制当前打开的 FreeCAD 文档，读取模型、执行
FreeCAD Python、修改对象并获取视图截图。

## 连接结构

```text
Codex
  | MCP stdio
  v
uvx freecad-mcp
  | XML-RPC HTTP, localhost:9875
  v
FreeCADMCP addon -> 当前运行中的 FreeCAD GUI 和活动文档
```

需要同时满足两端条件：

1. Codex 端能启动 `freecad-mcp` MCP 进程。
2. FreeCAD 端已安装 `FreeCADMCP` 插件，并启动 RPC Server。

仅打开 FreeCAD 不代表 RPC Server 已启动；仅配置 Codex 也不能代替
FreeCAD 插件。

## 当前机器的已验证配置

以下是本机当前使用的配置（路径是 macOS Apple Silicon/Homebrew 路径，换机器时
应先用 `command -v uvx` 查找实际路径）：

| 项目 | 当前值 |
| --- | --- |
| FreeCAD 应用 | `/Applications/FreeCAD.app` |
| FreeCAD 版本目录 | `~/Library/Application Support/FreeCAD/v1-1/` |
| 插件目录 | `~/Library/Application Support/FreeCAD/v1-1/Mod/FreeCADMCP/` |
| Codex 配置 | `~/.codex/config.toml` |
| MCP 启动器 | `/opt/homebrew/bin/uvx` |
| MCP 包 | `freecad-mcp` |
| FreeCAD RPC 地址 | `http://127.0.0.1:9875` |

当前安装的 `freecad-mcp` 包版本为 `0.1.22`，要求 Python `>=3.12`。使用
`uvx` 时通常不需要手动创建 Python 环境。

## 1. 安装前检查

```bash
test -d /Applications/FreeCAD.app && echo "FreeCAD found"
command -v uvx
uvx --version
```

如果没有 `uvx`，macOS 可使用 Homebrew 安装：

```bash
brew install uv
command -v uvx
```

如果机器没有 Homebrew，按 uv 官方安装方式安装后，重新打开终端并确认
`command -v uvx` 有输出。

## 2. 安装 FreeCAD 插件

FreeCAD 1.1 的 macOS 用户插件目录是：

```text
~/Library/Application Support/FreeCAD/v1-1/Mod/
```

FreeCAD 1.0 使用 `v1-0/Mod/`。必须根据实际版本选择目录。

下面的命令会把插件复制到 FreeCAD 1.1 目录。如果目标目录已有旧插件，先将它
移动为带时间戳的备份，再复制新版本；不要把插件复制成嵌套的
`FreeCADMCP/FreeCADMCP` 目录。

```bash
MCP_SRC_DIR="$(mktemp -d)/freecad-mcp"
git clone https://github.com/neka-nat/freecad-mcp.git "$MCP_SRC_DIR"

FREECAD_MOD_DIR="$HOME/Library/Application Support/FreeCAD/v1-1/Mod"
MCP_ADDON_DIR="$FREECAD_MOD_DIR/FreeCADMCP"
mkdir -p "$FREECAD_MOD_DIR"

if [ -d "$MCP_ADDON_DIR" ]; then
  mv "$MCP_ADDON_DIR" "$FREECAD_MOD_DIR/FreeCADMCP.backup.$(date +%Y%m%d-%H%M%S)"
fi
cp -R "$MCP_SRC_DIR/addon/FreeCADMCP" "$FREECAD_MOD_DIR/"

test -f "$MCP_ADDON_DIR/InitGui.py" && echo "FreeCADMCP addon installed"
```

安装或更新插件后必须完全退出并重新打开 FreeCAD。当前机器的插件文件应能在
以下路径看到：

```text
~/Library/Application Support/FreeCAD/v1-1/Mod/FreeCADMCP/InitGui.py
```

## 3. 在 FreeCAD 中启动 RPC

1. 打开 FreeCAD，并打开需要操作的 `.FCStd` 文档。
2. 工作台下拉菜单选择 `MCP Addon`。
3. 在 `FreeCAD MCP` 工具栏或菜单中点击 `Start RPC Server`。
4. 看到 FreeCAD 控制台输出 `RPC Server started at localhost:9875` 后，桥接端才可连接。
5. 可选：勾选 `Auto-Start Server`，以后每次 FreeCAD 启动后自动开启 RPC。

默认只监听本机 `localhost`，这是本仓库的推荐设置。除非确实要从另一台机器
访问，否则不要开启 `Remote Connections`。

## 4. 配置 Codex

编辑：

```text
~/.codex/config.toml
```

添加或保留以下配置。`command` 必须使用本机 `command -v uvx` 找到的绝对路径；
当前机器是 `/opt/homebrew/bin/uvx`。

```toml
[mcp_servers.freecad]
command = "/opt/homebrew/bin/uvx"
args = ["freecad-mcp"]
```

如果只需要文字结果、希望减少截图传输，可改为：

```toml
[mcp_servers.freecad]
command = "/opt/homebrew/bin/uvx"
args = ["freecad-mcp", "--only-text-feedback"]
```

修改配置后完全退出并重新打开 Codex，使 MCP 配置重新加载。不要把 API key、
代理密码或其他秘密写入仓库文档或 TOML 片段。

### 不同机器的配置示例

如果 `uvx` 在 `/usr/local/bin/uvx` 或其他位置，只替换 `command`：

```toml
[mcp_servers.freecad]
command = "/absolute/path/to/uvx"
args = ["freecad-mcp"]
```

不需要把 `freecad-mcp` 仓库路径填到 Codex 配置中；`uvx freecad-mcp` 会从包索引
解析并缓存 MCP 服务。

## 5. 验证连接

### 从终端验证 FreeCAD RPC

FreeCAD 必须保持打开，并且已经点击 `Start RPC Server`：

```bash
python3 - <<'PY'
import xmlrpc.client

rpc = xmlrpc.client.ServerProxy("http://127.0.0.1:9875", allow_none=True)
print("ping:", rpc.ping())
print("status:", rpc.get_rpc_status())
PY
```

预期 `ping` 为 `True`，并返回 RPC/GUI dispatch 状态。

也可以检查 9875 端口：

```bash
lsof -nP -iTCP:9875 -sTCP:LISTEN
```

### 从 Codex 验证

让 Codex 调用 FreeCAD MCP 的以下只读能力：

- `get_rpc_status`
- `get_objects`
- `get_object`
- `get_view`

先读取对象和状态，再执行创建、编辑或删除操作。FreeCAD 中应保持目标文档为
活动文档，否则 AI 可能操作到错误的文档。

## 6. 常见故障

### FreeCAD 已打开，但 Codex 报 `Connection refused`

这表示 `localhost:9875` 没有服务监听。按顺序检查：

1. FreeCAD 是否已重启过（插件安装后必须重启）。
2. 是否切换到 `MCP Addon` 工作台。
3. 是否点击了 `Start RPC Server`。
4. `lsof` 是否显示 9875 监听。
5. 是否有另一个 FreeCAD 实例占用了当前文档或端口。

### 看不到 `MCP Addon` 工作台

检查插件目录是否正好包含 `Init.py` 和 `InitGui.py`：

```bash
find "$HOME/Library/Application Support/FreeCAD/v1-1/Mod/FreeCADMCP" \
  -maxdepth 1 -type f -print
```

如果路径中多了一层 `FreeCADMCP/FreeCADMCP`，重新按第 2 节复制。FreeCAD 1.0
则检查 `v1-0/Mod/`。

### Codex 中没有 `freecad` MCP

检查 `~/.codex/config.toml`：

```bash
rg -n -A3 '^\[mcp_servers\.freecad\]' ~/.codex/config.toml
```

确认 `command` 是存在的绝对路径，`args` 包含 `freecad-mcp`，然后完全重启
Codex。不要只重启 FreeCAD；两端配置是独立的。

### `uvx` 下载或缓存失败

先确认网络和 uv：

```bash
uvx --version
uvx freecad-mcp --help
```

若 uv 缓存目录权限异常，可指定一个当前用户可写的缓存目录后重试：

```bash
UV_CACHE_DIR="$(mktemp -d)" uvx freecad-mcp --help
```

如果该命令成功，应将同样的环境变量配置到 MCP 启动环境，或修复用户级 uv
缓存目录权限；不要使用 `sudo uvx`。

## 7. 远程 FreeCAD（仅在明确需要时）

默认配置只允许本机访问。若 Codex 和 FreeCAD 在不同机器：

1. 在 FreeCAD MCP 工具栏开启 `Remote Connections`。
2. 在 `Configure Allowed IPs` 中只填入 Codex 所在机器的 IP 或 CIDR，例如
   `192.168.1.100` 或 `192.168.1.0/24`。
3. 重启 FreeCAD RPC Server。
4. 在 Codex 的 MCP 参数中添加 FreeCAD 主机地址：

```toml
[mcp_servers.freecad]
command = "/absolute/path/to/uvx"
args = ["freecad-mcp", "--host", "192.168.1.100"]
```

不要把 RPC 端口直接暴露到公网，也不要为了省事将允许网段设置为 `0.0.0.0/0`。

## 8. 本仓库的使用约定

- FreeCAD MCP 只控制当前运行中的 FreeCAD GUI 和活动 `.FCStd` 文档。
- MuJoCo 场景和脚本在 `dexh13_mjcf/`，不依赖 FreeCAD MCP；两者不要混淆。
- 需要读取或修改几何时，AI 先调用 `get_rpc_status`、`get_objects`，再执行写操作。
- 保存 FreeCAD 文档时使用明确的绝对路径，并在操作后重新读取对象确认结果。
- MCP 配置属于用户环境；仓库只记录模板和路径规则，不提交 `~/.codex/config.toml`。

## 给下一次 AI 的最短执行清单

```text
1. 读取 docs/FREECAD_MCP_SETUP.md。
2. 检查 FreeCAD 版本和 Mod 目录是否存在 FreeCADMCP/InitGui.py。
3. 检查 ~/.codex/config.toml 是否有 [mcp_servers.freecad]。
4. 检查 uvx 绝对路径和 freecad-mcp 是否可启动。
5. 要求用户在 FreeCAD 的 MCP Addon 工作台点击 Start RPC Server，除非已启用 Auto-Start。
6. 先用 get_rpc_status 和 get_objects 验证，再进行模型操作。
7. 若出现 Connection refused，优先检查 FreeCAD 端 RPC，而不是重复编辑 Codex 配置。
```
