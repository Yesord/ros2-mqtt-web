#!/bin/bash

# test虚拟环境激活脚本
# 使用conda激活test虚拟环境

echo "正在激活test虚拟环境..."

# 检查conda是否已安装
if ! command -v conda &> /dev/null; then
    echo "错误: conda未安装或不在PATH中"
    echo "请先安装conda或miniconda"
    exit 1
fi

# 初始化conda (必须在激活前执行)
echo "初始化conda环境..."
eval "$(conda shell.bash hook)"

# 检查test环境是否存在
if conda env list | grep -q "test_env"; then
    echo "找到test环境，正在激活..."
    conda activate test
    echo "test虚拟环境已激活！"
    echo "当前环境: $CONDA_DEFAULT_ENV"
else
    echo "警告: test环境不存在"
    echo "可用的conda环境:"
    conda env list
    echo ""
    echo "要创建test环境，请运行:"
    echo "conda create -n test python=3.8"
    exit 1
fi

# 显示Python版本和路径
echo "Python版本: $(python --version)"
echo "Python路径: $(which python)"

# 获取脚本的真实路径
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    # 脚本被source执行
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
    # 脚本被直接执行
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi

echo "脚本目录: $SCRIPT_DIR"

cd "$SCRIPT_DIR/test_ws" || {
    echo "错误: 无法进入test_ws目录 ($SCRIPT_DIR/test_ws)"
    echo "请检查test_ws目录是否存在"
    return 1
}