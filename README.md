# M05Proj
M05Class 4-Axis-Flight
Code for 2023-93M05 course
此仓库用于无人机项目的版本管理
## 编写代码约定
- 尽量减少内存占用，用整型变量的时候标清是哪一种
- 每一个外设都要新建一个.cpp文件存放函数定义和对应的.h文件存放函数声明和宏定义
- 每添加一个文件记得修改makefile文件，否则无法过编译
- 提交前请保证代码可以过编译且尽量做到没有任何warning（vscode有些很严重的问题也会作为warning，可以过编译但是在单片机上绝对无法运行），如果有无法避免的warning，请在pull request时指明。
- ./Core/Src用来存放我们使用的源代码，添加新的.cpp文件也是加入这里
- ./Core/Inc用来存放我们使用的头文件，添加新的.h文件也是加入这里
- 一定要保证写入的代码在USER CODE BEGIN和USER CODE END之间。
- 为代码添加必要的注释
