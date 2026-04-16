#!/bin/bash

# YOLOv8检测器编译验证脚本

echo "=========================================="
echo "YOLOv8 Tracking Compilation Verification"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 工作目录
WORKSPACE="/home/liuyao/px4_ros2_sdk_sitl"
BUILD_DIR="$WORKSPACE/build/yolov8_detector"

echo "[1/4] Checking modified files..."
echo ""

# 检查文件是否存在
files_to_check=(
    "src/px4_target_track/src/yolov8_detector/src/yolov8_detector_node.cpp"
    "src/px4_target_track/src/yolov8_detector/src/YOLOv8Detector.cpp"
    "src/px4_target_track/src/yolov8_detector/include/yolov8_detector/YOLOv8Detector.hpp"
)

all_exist=true
for file in "${files_to_check[@]}"; do
    full_path="$WORKSPACE/$file"
    if [ -f "$full_path" ]; then
        echo -e "${GREEN}✓${NC} $file"
    else
        echo -e "${RED}✗${NC} $file - NOT FOUND"
        all_exist=false
    fi
done

echo ""
if [ "$all_exist" = false ]; then
    echo -e "${RED}ERROR: Some files are missing!${NC}"
    exit 1
fi

echo "[2/4] Syntax validation..."
echo ""

# 检查关键代码片段
echo "Checking for required modifications:"

# 检查yolov8_detector_node.cpp
echo -n "  - handle_mouse_event() method... "
if grep -q "void handle_mouse_event" "$WORKSPACE/src/px4_target_track/src/yolov8_detector/src/yolov8_detector_node.cpp"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

echo -n "  - select_target_by_region() method... "
if grep -q "void select_target_by_region" "$WORKSPACE/src/px4_target_track/src/yolov8_detector/src/yolov8_detector_node.cpp"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

echo -n "  - draw_detections_and_tracks() method... "
if grep -q "void draw_detections_and_tracks" "$WORKSPACE/src/px4_target_track/src/yolov8_detector/src/yolov8_detector_node.cpp"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

# 检查YOLOv8Detector.hpp
echo -n "  - get_tracked_targets() declaration... "
if grep -q "get_tracked_targets" "$WORKSPACE/src/px4_target_track/src/yolov8_detector/include/yolov8_detector/YOLOv8Detector.hpp"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

echo -n "  - get_track_trail() declaration... "
if grep -q "get_track_trail" "$WORKSPACE/src/px4_target_track/src/yolov8_detector/include/yolov8_detector/YOLOv8Detector.hpp"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

echo -n "  - track_trails_ member variable... "
if grep -q "track_trails_" "$WORKSPACE/src/px4_target_track/src/yolov8_detector/include/yolov8_detector/YOLOv8Detector.hpp"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

# 检查YOLOv8Detector.cpp
echo -n "  - get_track_trail() implementation... "
if grep -q "std::deque<cv::Point2f> YOLOv8Detector::get_track_trail" "$WORKSPACE/src/px4_target_track/src/yolov8_detector/src/YOLOv8Detector.cpp"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

echo ""
echo "[3/4] Required headers check..."
echo ""

echo -n "  - #include <deque> in node file... "
if grep -q "#include <deque>" "$WORKSPACE/src/px4_target_track/src/yolov8_detector/src/yolov8_detector_node.cpp"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

echo -n "  - #include <map> in header file... "
if grep -q "#include <map>" "$WORKSPACE/src/px4_target_track/src/yolov8_detector/include/yolov8_detector/YOLOv8Detector.hpp"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

echo ""
echo "[4/4] Documentation check..."
echo ""

doc_files=(
    "README_TRACKING.md"
    "IMPLEMENTATION_SUMMARY.md"
    "CHECKLIST.md"
)

for doc in "${doc_files[@]}"; do
    doc_path="$WORKSPACE/src/px4_target_track/src/yolov8_detector/$doc"
    if [ -f "$doc_path" ]; then
        lines=$(wc -l < "$doc_path")
        echo -e "${GREEN}✓${NC} $doc ($lines lines)"
    else
        echo -e "${RED}✗${NC} $doc - NOT FOUND"
    fi
done

echo ""
echo "=========================================="
echo "Verification Summary"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Navigate to workspace: cd $WORKSPACE"
echo "2. Build package: colcon build --packages-select yolov8_detector"
echo "3. Run node: ros2 run yolov8_detector yolov8_detector_node"
echo ""
echo "For usage instructions, see: README_TRACKING.md"
echo "For implementation details, see: IMPLEMENTATION_SUMMARY.md"
echo ""
