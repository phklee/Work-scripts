# uwb_follow
uwb跟随节点，具体参数详见 `naive_follow.launch`

- 编译，编译前确保已编译 `uwb` 和`usradar`包
```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="uwb"
catkin_make -DCATKIN_WHITELIST_PACKAGES="usradar"
catkin_make -DCATKIN_WHITELIST_PACKAGES="uwb_follow"
```
- 运行
```bash
source devel/setup.bash
方式一：
    roslaunch uwb uwb_pdoa.launch
    roslaunch usradar usradar_a02.launch
    roslaunch uwb_follow naive_follow.launch
方式二：
    roslaunch uwb_follow uwb_follow.launch
```
- 方式一：运行`roslaunch uwb_follow naive_follow.launch`前需要运行uwb和usradar节点。
- 方式二：也可以通过运行`roslaunch uwb_follow uwb_follow.launch`将uwb、usradar节点和naive_follw跟随节点全部运行起来。