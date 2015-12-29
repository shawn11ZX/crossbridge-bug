## 背景 ##

我们是一个游戏公司，基于Adobe Flash Player开发3D游戏。 在一个项目中我们需要使用Adobe的Crossbridge编译器编译[physx](https://developer.nvidia.com/physx-sdk)代码（C++
），使之能运行在Flash上。

Flash Player里内嵌了一个虚拟机，称之为[AVM(ActionScript Virtual Machine)](https://www.adobe.com/content/dam/Adobe/en/devnet/actionscript/articles/avm2overview.pdf)。 Crossbridge能把C++代码成编译成avm机器码ABC Code，从而能在Flash上执行。

Crossbridge的编译器是在llvm-2.9,llvm-gcc-4.2基础上修改得到的

## 问题现象 ##
我们碰到的问题是，同样的输入，CrossBridge编译出来的程序会死循环，而原生的使用原生llvm-2.9、llvm-gcc-4.2在Windows上编译得到的程序不会。

死循环发生在一个名为_gjkRelativeRayCast的函数。经过一些测试，我们发现两个现象：


### 现象一　###
CrossBridge编译后的程序，变量sDist在循环的迭代N中的值，在进入下一个迭代N+1后，会更改。

	while(BAllEq(bNotTerminated, bTrue))
		{
			// 在迭代N+1开始时，sDist 突然变为 0.0506684184074401855468750
			printf("Start Loop: sDist.x is %.60f\n", sDist.x); 
			minDist = sDist;
			
			... // 省略的代码
			
			
			sDist = V3Dot(closest, closest);

			... // 省略的代码

			// 在迭代Ｎ结束时， sDist值为 0.050668418176030627364525571465492248535156250
			printf("End Loop: sDist.x is %.60f\n", sDist.x); 
			
		}

### 现象二 ###

两段有相同语义的代码会有不同的执行结果。　


- 在代码Ａ中，minDist在两个地方被赋值(赋值1,赋值2）。
- 在代码Ｂ中，我们先确定minDist会不会发生赋值2。如果不发生赋值2，再执行赋值1。否则赋值1最终反正会被赋值2覆盖。

注：minDist和sDist没有在省略的代码中被引用。

代码Ａ

	... // 省略的代码
	if(FAllGrtr(vw, zero))
	{
		minDist = sDist;　// 赋值1
		if(FAllGrtrOrEq(vr, zero))
		{
			return false;
		}
		else
		{
			const FloatV _oldLambda = _lambda;
			_lambda = FSub(_lambda, FDiv(vw, vr));
			if(FAllGrtr(_lambda, _oldLambda))
			{
				... //　省略的代码
				minDist = maxDist; // 赋值2
				nor = closest;
			}
		}
	}
	... // 省略的代码

代码Ｂ
    
	... // 省略的代码
	bool modified = false;
	if(FAllGrtr(vw, zero))
	{
		if(FAllGrtrOrEq(vr, zero))
		{
			return false;
		}
		else
		{
			const FloatV _oldLambda = _lambda;
			_lambda = FSub(_lambda, FDiv(vw, vr));
			if(FAllGrtr(_lambda, _oldLambda))
			{
				... // 省略的代码
				minDist = maxDist;　// 赋值2
				modified = true;
				nor = closest;
				
			}
		}
	}
	if(modified == false) {
		minDist = sDist;　// 赋值1
	}
	... // 省略的代码

