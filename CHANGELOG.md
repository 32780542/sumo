# 更新日志

本文件记录每次上传到 GitHub 的工程更新内容。

## 2026-05-09 third upload - 2026.5.9first

- 更新 `main.py`，将运行模式调整为连续运行关闭/开启两组实验并保存对比结果。
- 扩展仿生编队层：加入 ADEPT 局部间距触发思想与 Boids 对齐、内聚、分离、速度匹配规则。
- 新增基于转向灯的信息素传播机制，用于队列重构、拥堵意图传递和换道协同。
- 新增目标车道前方净空估计、阻塞识别、安全重构方向选择等辅助逻辑。
- 更新并上传当前实验输出文件：`test_swarm_off.xml`、`test_swarm_on.xml`。
- 新增并上传对比实验输出文件：`test_pheromone_off.xml`、`test_pheromone_on.xml`、`test_reconfig_off.xml`、`test_reconfig_on.xml`。

## 2026-05-09 second upload - 2026.5.9first

- 创建并上传 `2026.5.9first` 分支。
- 更新 `main.py`。
- 新增并上传 `test_swarm_off.xml`、`test_swarm_on.xml` 两个实验输出文件。

## 2026-05-09 first upload - main

- 初始化 Git 工程并创建首个提交。
- 新增根目录 `.gitignore`，排除 Python 缓存、虚拟环境、本地配置、日志临时文件和常见 SUMO 仿真输出。
- 上传工程源码、SUMO 输入配置、依赖清单、文档和示例资源。
- 将初始工程推送到 GitHub `main` 分支。
