# Way
机器人相关。

## Issues
### 不同规划器的使用
如果主程序包含了多个行为，且每个行为包含了不同规划器，那么注意在行为执行之前注意调用对应的规划器，例如：
```cpp
// 设定navfn/NavfnROS规划器
system("rosrun dynamic_reconfigure dynparam set /move_base base_global_planner navfn/NavfnROS");
// 设定自定义规划器
system("rosrun dynamic_reconfigure dynparam set /move_base base_global_planner simple_global_planner/SimpleGlobalPlanner");

```