<!-- TABLE OF CONTENTS -->

## Table of Contents

* [Getting Started](#getting-started)
* [Contact](#contact)

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites
```sh
Ubuntu 16.04
OpenCV
```

### 项目概述
```sh
为方便阅读，本部分使用中文：
１．目的：本项目主要是为解决自动驾驶中的uturn问题，在某些情况下，实际场景中并没有足够的空间，车辆需要来回前进后退运动来实现uturn；在test_results文件夹下面有几种测试样例，包含宽道和窄道场景，调整config.h中的lane_gap进行实现；
２．MAP说明：请根据test_results文件夹下面的图片阅读如下内容，图中浅蓝色为当前车辆所在车道，浅蓝色的点代表中心线；绯红色代表目标车道（车道宽度为４ｍ，可配置config.h文件;图中黑色的点代表障碍物，因为通常道路中间是障碍物，当然，你也根据你的需求来进行配置；两条车道中间的距离可以在config.h中进行配置（lane_gap），用以实现狭窄路段的uturn和宽路段的uturn；
３．配置说明：config.h文件中有关于Map和算法等的一些配置，你可根据你的需求进行调试；
４．输入输出：输入主要是当前和目标车道中心线，当然还包括一些简单的地图配置和障碍物配置；输出主要是计算出的x, y, phi, kappa, v, a, steer;你可根据你的需求进行获取如前四项分别为位置　角度　曲率；
５．碰撞：车辆不能和任何障碍物相互碰撞，你可添加障碍物；车辆不能超出地图边界；
６．其它：本程序中没有使用gtest, 主要是因为gtest属于第三方库；当然可以简单配置实现；
```

### Compile and Run Source code
```sh
cd u_turn
./run.sh
```

<!-- CONTACT -->

## Contact
Liuzhu Tong - tongliuzhu@126.com
