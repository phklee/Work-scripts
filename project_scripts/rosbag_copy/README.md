######功能说明############
1.硬件连接
    笔记本电脑配到192.168.1.*
2.脚本执行
	运行 ./copyBags.sh -h 可以看帮助文件可以快速运行
	示例: ./copyBags.sh 20220707174544 2
	脚本会从/media/nvidia/user/record_bags/bags中找到对应的时间戳文件夹，
因此需要20220707174544时间落在bags中的文件中，否则不会拷贝数据
	
    例如下图:bags中最小时间20220707173344
    		 bags中最大时间20220707175044
    		 20220707174544 +- 2落在这个区域之间，会把符合条件的文件拷贝到
    destination文件夹中.
    
.
├── bags
│   └── 20220707
│       ├── IDPXJT100010BA0002_112ex_20220707173344_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707173444_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707173544_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707173644_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707173744_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707173844_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707173944_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707174044_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707174144_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707174244_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707174344_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707174444_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707174544_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707174644_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707174744_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707174844_pedal.bag
│       ├── IDPXJT100010BA0002_112ex_20220707174944_pedal.bag
│       └── IDPXJT100010BA0002_112ex_20220707175044_pedal.bag
├── copyBags.sh
├── destination
│   ├── 112
│   │   ├── IDPXJT100010BA0002_112ex_20220707174344_pedal.bag
│   │   ├── IDPXJT100010BA0002_112ex_20220707174444_pedal.bag
│   │   ├── IDPXJT100010BA0002_112ex_20220707174544_pedal.bag
│   │   ├── IDPXJT100010BA0002_112ex_20220707174644_pedal.bag
│   │   └── IDPXJT100010BA0002_112ex_20220707174744_pedal.bag
│   ├── 113
│   ├── 114
│   └── 115
└── readme

    由于车端有四个xavier，所以在destination文件夹中会自动创建112-115四个文件夹
分别存放。


3.依赖
    如果没有sshpass，需要先安装sshpass:
    	sudo apt-get install sshpass
    	
4.运行截图见附件
    	

	
	
	
	

	
	
	
	
	
	
