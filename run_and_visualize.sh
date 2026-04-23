#!/bin/bash

# ============================================
# MAPF Research 项目运行和可视化脚本
# ============================================
# 使用方法: 在 WSL 终端中运行
#   ./run_and_visualize.sh
# ============================================

set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  MAPF Research 运行和可视化脚本${NC}"
echo -e "${GREEN}========================================${NC}"

# ============================================
# 交互式提问
# ============================================

echo ""
echo -e "${CYAN}----- 输入文件 -----${NC}"
echo "可选输入文件:"
echo "  1) example_problems/random.domain/random_32_32_20_100.json"
echo "  2) example_problems/city.domain/paris_1_256_250.json"
echo "  3) example_problems/game.domain/brc202d_500.json"
echo "  4) example_problems/warehouse.domain/warehouse_small_200.json"
echo "  5) example_problems/warehouse.domain/warehouse_large_5000.json"
echo "  6) example_problems/warehouse.domain/sortation_large_2000.json"
echo "  7) example_problems/random.domain/delays/delay.json"
echo ""
read -p "请选择输入文件 [1]: " input_choice
case "${input_choice:-1}" in
    1) INPUT_FILE="./example_problems/random.domain/random_32_32_20_100.json" ;;
    2) INPUT_FILE="./example_problems/city.domain/paris_1_256_250.json" ;;
    3) INPUT_FILE="./example_problems/game.domain/brc202d_500.json" ;;
    4) INPUT_FILE="./example_problems/warehouse.domain/warehouse_small_200.json" ;;
    5) INPUT_FILE="./example_problems/warehouse.domain/warehouse_large_5000.json" ;;
    6) INPUT_FILE="./example_problems/warehouse.domain/sortation_large_2000.json" ;;
    7) INPUT_FILE="./example_problems/random.domain/delays/delay.json" ;;
    *) INPUT_FILE="./example_problems/random.domain/random_32_32_20_100.json" ;;
esac

echo ""
echo -e "${CYAN}----- 输出文件 -----${NC}"
read -p "输出文件名 [output.json]: " OUTPUT_FILE
OUTPUT_FILE="${OUTPUT_FILE:-output.json}"

echo ""
echo -e "${CYAN}----- 日志设置 -----${NC}"
read -p "日志文件路径 (留空不写日志) []: " log_file

echo ""
echo -e "${CYAN}----- 模拟最大 tick 数 -----${NC}"
read -p "模拟最大 tick 数 (留空使用输入文件默认值) []: " simulation_time

echo ""
echo -e "${CYAN}----- PlanViz 可视化 -----${NC}"
read -p "是否启动 PlanViz? [Y/n]: " launch_planviz
launch_planviz="${launch_planviz:-Y}"

# ============================================
# 1. 编译项目
# ============================================
echo ""
echo -e "${YELLOW}[1/4] 编译项目...${NC}"

if [ ! -d "build" ]; then
    echo "创建 build 目录..."
    mkdir build
fi

echo "运行 CMake 配置..."
cmake -B build ./ -DPYTHON=false -DCMAKE_BUILD_TYPE=Release 2>/dev/null

echo "编译..."
make -C build -j 2>/dev/null

if [ -f "build/lifelong" ]; then
    echo -e "${GREEN}编译成功!${NC}"
else
    echo -e "${RED}编译失败!${NC}"
    exit 1
fi

# ============================================
# 2. 构建命令并运行
# ============================================
echo -e "\n${YELLOW}[2/4] 运行程序...${NC}"

echo "输入文件: $INPUT_FILE"
echo "输出文件: $OUTPUT_FILE"

# 构建命令
LIFELONG_CMD="./build/lifelong --inputFile \"$INPUT_FILE\" -o \"$OUTPUT_FILE\""

# 添加可选参数
[ -n "$log_file" ] && LIFELONG_CMD="$LIFELONG_CMD --logFile \"$log_file\""
[ -n "$simulation_time" ] && LIFELONG_CMD="$LIFELONG_CMD --simulationTime $simulation_time"

echo "命令: $LIFELONG_CMD"
eval $LIFELONG_CMD

if [ -f "$OUTPUT_FILE" ]; then
    echo -e "${GREEN}运行成功!${NC}"
else
    echo -e "${RED}运行失败!${NC}"
    exit 1
fi

# ============================================
# 3. 获取地图信息
# ============================================
echo -e "\n${YELLOW}[3/4] 提取地图文件信息...${NC}"

INPUT_DIR="$(dirname "$INPUT_FILE")"
MAP_FILE=""

if [ -f "$INPUT_FILE" ]; then
    MAP_RELATIVE=$(python3 -c "import json; data=json.load(open('$INPUT_FILE')); print(data.get('mapFile', ''))" 2>/dev/null || echo "")

    if [ -n "$MAP_RELATIVE" ]; then
        MAP_FILE="$INPUT_DIR/$MAP_RELATIVE"
        MAP_FILE=$(echo "$MAP_FILE" | sed 's/\\/\//g')
        echo "地图文件: $MAP_FILE"

        if [ -f "$MAP_FILE" ]; then
            echo -e "${GREEN}地图文件存在${NC}"
        else
            echo -e "${YELLOW}地图文件不存在${NC}"
        fi
    else
        echo "输入文件中未指定地图文件"
    fi
else
    echo "输入文件不存在"
fi

# ============================================
# 4. 启动 PlanViz
# ============================================
echo -e "\n${YELLOW}[4/4] 启动 PlanViz...${NC}"

if [[ "$launch_planviz" =~ ^[Yy]$ ]]; then
    # PlanViz 安装路径 (WSL Unix 路径)
    PLANVIZ_DIR="/mnt/f/PlanViz"

    if [ -d "$PLANVIZ_DIR" ]; then
        echo -e "${GREEN}启动 PlanViz...${NC}"
        echo "PlanViz 路径: $PLANVIZ_DIR"
        echo ""

        # 构建 PlanViz 命令 (使用 Unix 绝对路径)
        if [ -n "$MAP_FILE" ] && [ -f "$MAP_FILE" ]; then
            echo "python3 $PLANVIZ_DIR/script/run.py --plan $PROJECT_DIR/$OUTPUT_FILE --map $MAP_FILE"
            python3 "$PLANVIZ_DIR/script/run.py" --plan "$PROJECT_DIR/$OUTPUT_FILE" --map "$MAP_FILE"
        else
            echo "python3 $PLANVIZ_DIR/script/run.py --plan $PROJECT_DIR/$OUTPUT_FILE"
            python3 "$PLANVIZ_DIR/script/run.py" --plan "$PROJECT_DIR/$OUTPUT_FILE"
        fi
    else
        echo -e "${RED}PlanViz 未找到: $PLANVIZ_DIR${NC}"
    fi
else
    echo "跳过 PlanViz"
fi

echo -e "\n${GREEN}完成!${NC}"
