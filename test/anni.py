import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def analyze_vertical_flight(log_path):
    # 1. 首先尝试自动推断数据类型
    try:
        df = pd.read_csv(log_path, comment='#')
    except Exception as e:
        print(f"标准读取失败: {e}")
        # 2. 如果失败，尝试逐行读取并清理数据
        with open(log_path, 'r') as f:
            lines = [line for line in f if not line.startswith('#')]

        # 移除特殊字符
        cleaned_lines = []
        for line in lines:
            # 替换所有非打印字符为空格
            cleaned_line = ''.join(char if char.isprintable() else ' ' for char in line)
            cleaned_lines.append(cleaned_line)

        # 从清理后的文本创建DataFrame
        from io import StringIO
        df = pd.read_csv(StringIO(''.join(cleaned_lines)))

    # 3. 强制转换数值列
    numeric_cols = ['timestamp', 'baro_pressure', 'gps_alt', 'gps_vel_z',
                   'gps_fix_type', 'gps_num_sats', 'gps_pdop', 'imu_accel_z',
                   'ekf_pos_z', 'ekf_vel_z']

    for col in numeric_cols:
        if col in df:
            # 先转换为字符串，再清理特殊字符，最后转换为数值
            df[col] = df[col].astype(str).str.replace(r'[^\d.+-]', '', regex=True)
            df[col] = pd.to_numeric(df[col], errors='coerce')

    # 4. 检查数据质量
    print("\n数据摘要:")
    print(df.info())
    print("\n前5行数据:")
    print(df.head())

    # 5. 检查是否有NaN值
    print("\n缺失值统计:")
    print(df.isna().sum())

    # 6. 填充或删除缺失值
    df = df.ffill().bfill()  # 前向后向填充

    # 从这里开始继续你的分析代码...
    # 转换坐标系
    df['true_altitude'] = df['ekf_pos_z']
    df['ekf_velocity_up'] = df['ekf_vel_z']

    # 气压高度转换（简化模型）
    sea_level_pressure = 101.325
    df['baro_altitude'] = (sea_level_pressure - df['baro_pressure']) * (8000.0 / sea_level_pressure)

    # 创建基本图表
    plt.figure(figsize=(12, 8))

    # 高度比较
    plt.subplot(2, 1, 1)
    plt.plot(df['timestamp'], df['true_altitude'], 'b-', label='EKF Altitude')
    if 'gps_alt' in df:
        plt.plot(df['timestamp'], df['gps_alt'], 'r--', label='GPS Altitude')
    # plt.plot(df['timestamp'], df['baro_altitude'], 'g-.', label='Baro Altitude')
    plt.ylabel('Altitude (m)')
    plt.legend()
    plt.grid(True)

    # 速度与加速度
    plt.subplot(2, 1, 2)
    if 'ekf_velocity_up' in df:
        plt.plot(df['timestamp'], df['ekf_velocity_up'], 'b-', label='EKF Velocity')
    if 'imu_accel_z' in df:
        plt.plot(df['timestamp'], df['imu_accel_z'], 'r--', label='IMU Accel Z')
    plt.ylabel('Velocity (m/s) / Accel (m/s²)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig('basic_analysis.png')
    plt.show()

if __name__ == "__main__":
    analyze_vertical_flight("gps_test_vertical.csv")
