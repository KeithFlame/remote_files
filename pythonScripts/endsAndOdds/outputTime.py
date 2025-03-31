import tkinter as tk
from datetime import datetime, timedelta
import pytz

# 更新时间的函数
def update_time():
    # 获取当前时间
    beijing_tz = pytz.timezone('Asia/Shanghai')
    uk_tz = pytz.timezone('Europe/London')
    us_tz = pytz.timezone('America/New_York')
    pst_tz = pytz.timezone('America/Los_Angeles')
    pacific_tz = pytz.timezone('America/Los_Angeles')

    beijing_time = datetime.now(beijing_tz)
    uk_time = datetime.now(uk_tz)
    us_time = datetime.now(us_tz)
    pacific_time = datetime.now(pacific_tz)

    # 指定太平洋时间点
    target_time = pst_tz.localize(datetime(2025, 2, 16, 23, 59, 59))

    # 计算时间差
    beijing_diff = beijing_time - target_time.astimezone(beijing_tz)
    uk_diff = uk_time - target_time.astimezone(uk_tz)
    us_diff = us_time - target_time.astimezone(us_tz)
    pacific_diff = pacific_time - target_time.astimezone(pacific_tz)

    # 格式化时间和时间差
    beijing_label.config(text=f"北京时间: {beijing_time.strftime('%Y-%m-%d %H:%M:%S')} (差: {beijing_diff})")
    uk_label.config(text=f"英国时间: {uk_time.strftime('%Y-%m-%d %H:%M:%S')} (差: {uk_diff})")
    us_label.config(text=f"美国时间: {us_time.strftime('%Y-%m-%d %H:%M:%S')} (差: {us_diff})")
    pacific_label.config(text=f"太平洋时间: {pacific_time.strftime('%Y-%m-%d %H:%M:%S')} (已过: {pacific_diff})")

    # 每秒更新一次时间
    root.after(1000, update_time)

# 创建主窗口
root = tk.Tk()
root.title("世界时间显示")

# 创建标签
beijing_label = tk.Label(root, font=('宋体', 16))
beijing_label = tk.Label(root, font=('times new roman', 16))
beijing_label.pack(pady=10)

uk_label = tk.Label(root, font=('宋体', 16))
uk_label = tk.Label(root, font=('times new roman', 16))
uk_label.pack(pady=10)

us_label = tk.Label(root, font=('宋体', 16))
us_label = tk.Label(root, font=('times new roman', 16))
us_label.pack(pady=10)

# pacific_label = tk.Label(root, font=('宋体', 16))
pacific_label = tk.Label(root, font=('times new roman', 16))
pacific_label.pack(pady=10)

# 启动时间更新
update_time()

# 运行主循环
root.mainloop()