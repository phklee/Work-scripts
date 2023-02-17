# bag包数据解析
此脚本主要用于bag包感知模块的数据解析，读取bag包，将消息生成对应的csv文件。
- [bag包数据解析](#bag包数据解析)
  - [1、创建环境](#1创建环境)
  - [2、运行步骤](#2运行步骤)
  - [3、Q&A](#3qa)
## 1、创建环境
两种方式创建环境，二选一即可。
- 1）python
```
  pip install -r requirements.txt -i https://pypi.mirrors.ustc.edu.cn/simple/
```

- 2）conda

  添加下载源
```
  conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/
  conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/
  conda config --set show_channel_urls yes
```
```
  conda env create -f environment.yaml
```

## 2、运行步骤
同时适用于RoboTaxi和RoboBus。

bag_file为bag包所在绝对路径，也是存放csv文件的路径。
- 解析business工程下记录的bag包
```
  conda activate decode_rosbag  # 若使用自带python，则不需要运行
```

```python
  ./decode_rosbag_0411_for_business.py ${bag_file}
```

- 解析KTD_update工程下记录的bag包
```python
  ./decode_rosbag_0826_for_KTD.py ${bag_file}
```

## 3、Q&A
- 1、出现`raise ROSBagException('unsupported compression type: %s' % chunk_header.compression)`错误。

  原因: python3不支持采用LZ4 压缩的bag包

  解决: python2 decode_rosbag_0826_for_KTD.py ${bag_file}