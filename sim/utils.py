from sim.config import *
def clamp(value, lower, upper):
    """将数值限制在 [lower, upper] 区间内，避免速度等控制量越界。"""
    return max(lower, min(upper, value))

