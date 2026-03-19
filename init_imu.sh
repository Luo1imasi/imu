#!/bin/bash

# HiPNUC IMU (J1939) 波特率修改脚本
# 默认将设备从 500k 修改为 1M

CAN_IF="can4"

echo "================================================="
echo "  HiPNUC IMU 波特率配置脚本 (Target: 1Mbps)"
echo "  当前使用接口: ${CAN_IF}"
echo "================================================="

# 检查系统是否安装了 can-utils
if ! command -v cansend &> /dev/null; then
    echo "[错误] 未找到 'cansend' 命令。请先安装 can-utils。"
    echo "Ubuntu/Debian 运行: sudo apt-get install can-utils"
    exit 1
fi

echo "[1/3] 发送修改波特率为 1M 指令..."
# 地址 009A, 写命令 06, 数据 0 = 1000K
cansend ${CAN_IF} 0CEF0800#9A00060000000000
if [ $? -ne 0 ]; then
    echo "[错误] CAN 报文发送失败，请检查 ${CAN_IF} 接口是否处于 UP 状态且波特率匹配当前 IMU。"
    exit 1
fi
# 给微控制器一点时间处理寄存器变更
sleep 0.5 

echo "[2/3] 发送保存配置指令..."
# 地址 0000, 写命令 06, 数据 0 = 保存
cansend ${CAN_IF} 0CEF0800#0000060000000000
# 写入 Flash 通常比较慢，给足 1 秒钟防止中断
sleep 1.0 

echo "[3/3] 发送设备复位指令..."
# 地址 0000, 写命令 06, 数据 FF = 复位
cansend ${CAN_IF} 0CEF0800#00000600FF000000
# 等待设备完成重启并重新初始化总线
sleep 1.5 

echo "================================================="
echo "[成功] IMU 指令下发完毕！"
echo "IMU 现在应该已经以 1Mbps 的波特率运行。"
echo ""
echo "!!! 请注意 !!!"
echo "你的主机 ${CAN_IF} 接口仍停留在旧的波特率。"
echo "请手动运行以下命令，将主机的 CAN 接口切换到 1M："
echo "  sudo ip link set ${CAN_IF} down"
echo "  sudo ip link set ${CAN_IF} up type can bitrate 1000000"
echo "================================================="