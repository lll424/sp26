#!/usr/bin/env bash
# 上车计划 T1/T3/T4 变更的 VM 端落地脚本
# 用途：拉代码 → 校验拉到 → 编译 → 提示复验命令
# 用法：在 VM 的项目根目录（有 .git 的那层）执行
#   bash tools/vm_pull_and_build.sh
#
# 设计原则：每一步都先检查前置条件，不满足就停下并说明原因，不硬着头皮往下走。

set -u

# ---------- 小工具 ----------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

ok()   { echo -e "${GREEN}[OK]${NC}   $*"; }
warn() { echo -e "${YELLOW}[注意]${NC} $*"; }
die()  { echo -e "${RED}[错误]${NC} $*"; exit 1; }
step() { echo ""; echo "=================================================="; echo " $*"; echo "=================================================="; }

TARGET_BRANCH="换摄像头测ekf"
EXPECT_COMMIT="aef9fab"   # 至少要有这个提交

# ---------- 步骤 0：确认在项目根目录 ----------
step "步骤 0 / 检查项目根目录"
if [ ! -d ".git" ]; then
  die "当前目录不是 git 仓库根目录。请先 cd 到项目根目录（ls -a 能看到 .git 那一层）再执行。"
fi
if [ ! -f "tasks/auto_aim/target.cpp" ]; then
  die "找不到 tasks/auto_aim/target.cpp，确认这是 sp26 项目根目录。"
fi
if [ ! -d "build" ]; then
  die "找不到 build 目录。请先按 README 完成 cmake 配置（cmake -B build）。"
fi
ok "项目根目录：$(pwd)"

# ---------- 步骤 1：确认工作区干净 ----------
step "步骤 1 / 检查 VM 本地有没有未提交的改动"
DIRTY=$(git status --porcelain)
if [ -n "$DIRTY" ]; then
  echo "以下文件有未提交的改动："
  echo "$DIRTY"
  echo ""
  die "先不要 pull。请把上面的情况告诉我，判断这些改动要不要保留。
如果确定都是垃圾（比如 build 产物、临时文件），可以执行：
    git stash push -u -m 'vm-local-before-pull'
再重新运行本脚本。"
fi
ok "工作区干净，可以安全 pull"

# ---------- 步骤 2：切到目标分支 ----------
step "步骤 2 / 切换到 $TARGET_BRANCH 分支"
CURRENT=$(git branch --show-current)
if [ "$CURRENT" != "$TARGET_BRANCH" ]; then
  warn "当前在 $CURRENT 分支，正在切换"
  git checkout "$TARGET_BRANCH" || die "切换分支失败，可能本地有冲突的改动。"
fi
ok "当前分支：$(git branch --show-current)"

# ---------- 步骤 3：拉代码 ----------
step "步骤 3 / 从 GitHub 拉取最新提交"
git fetch origin || die "fetch 失败，检查网络（能否访问 github.com）。"
BEFORE=$(git rev-parse --short HEAD)
git pull --ff-only origin "$TARGET_BRANCH" || die "pull 失败。
如果是 'divergent branches' 或 'not possible to fast-forward'，
说明 VM 本地有额外提交，告诉我，我帮你判断怎么处理。"
AFTER=$(git rev-parse --short HEAD)
echo ""
echo "拉取前提交：$BEFORE"
echo "拉取后提交：$AFTER"
if [ "$BEFORE" = "$AFTER" ]; then
  warn "提交没变化，可能已经是最新的"
else
  ok "已更新"
fi

# ---------- 步骤 4：校验关键改动真的到位了 ----------
step "步骤 4 / 校验变更是否落地（这一步是小白最容易漏的）"
FAIL=0

# T1: R_dig[0] 应为 3e-6
if grep -q 'R_dig{{3e-6, 1e-6, 1e-3, 6e-3}}' tasks/auto_aim/target.cpp; then
  ok "T1  R_dig[0] = 3e-6"
else
  echo -e "${RED}[错误]${NC} T1  R_dig 不是预期值，实际是："
  grep -n 'R_dig{{' tasks/auto_aim/target.cpp
  FAIL=1
fi

# T3: solver 里应有迟滞相关代码
if grep -q 'has_last_yaw_' tasks/auto_aim/solver.cpp && \
   grep -q 'HYSTERESIS_PX' tasks/auto_aim/solver.cpp; then
  ok "T3  optimize_yaw 迟滞代码存在"
else
  echo -e "${RED}[错误]${NC} T3  solver.cpp 里找不到迟滞代码，检查是否拉到了 aef9fab"
  FAIL=1
fi

# T3: 头文件应有 mutable 成员
if grep -q 'mutable double last_yaw_' tasks/auto_aim/solver.hpp; then
  ok "T3  solver.hpp 的 last_yaw_ 已声明为 mutable"
else
  echo -e "${RED}[错误]${NC} T3  solver.hpp 缺 mutable 声明，编译会报错"
  FAIL=1
fi

# T4: sentry.yaml 应为 GPU
if grep -q '^device: GPU' configs/sentry.yaml; then
  ok "T4  sentry.yaml device = GPU"
else
  echo -e "${RED}[错误]${NC} T4  sentry.yaml 的 device 不是 GPU，实际是："
  grep -n '^device:' configs/sentry.yaml
  FAIL=1
fi

# 提醒项：single_armor_test 应仍为 true（单板环境）
if grep -q '^single_armor_test: true' configs/mindvision_vm.yaml; then
  ok "提醒 single_armor_test = true（单板联调，符合预期）"
else
  warn "single_armor_test 不是 true。当前是单板环境吗？若是，应改回 true。"
fi

if [ "$FAIL" -ne 0 ]; then
  die "有变更没落地，先别编译。把上面的输出发给我。"
fi

# ---------- 步骤 5：编译 ----------
step "步骤 5 / 编译（改了 .hpp，需要重编依赖它的所有文件）"
JOBS=$(nproc 2>/dev/null || echo 4)
echo "使用 $JOBS 线程并行编译"
echo ""
if ! cmake --build build -j"$JOBS"; then
  die "编译失败。把报错的前 30 行发给我。"
fi
ok "编译成功"

# ---------- 收尾：给出复验命令 ----------
step "完成 / 下一步做什么"
cat <<'EOF'
代码已就位并编译通过。现在需要你手动跑起来验证：

  终端 A：启动自瞄
    ./build/mt_auto_aim_debug configs/mindvision_vm.yaml

  终端 B：采集统计（目标要动起来，比如手动摆头或者转动装甲板）
    python3 tools/ekf_monitor.py --stats 10 --keys residual_angle,residual_yaw,nis,w

验收标准：
  residual_angle 峰峰值  改前 2.469 rad（141°）  →  目标 < 0.35 rad（20°）
  nis 均值               改前 7.8                →  目标 ~6 或更低

两项都达标  → 告诉我，然后进入 T2（min_confidence 改 0.8）
residual_angle 仍超标 → 上兜底方案 R_dig[3] 6e-3 → 2e-2
EOF
