## 背景(Background) ##

我们是一个游戏公司，基于Adobe Flash Player开发3D游戏。 在一个项目中我们需要使用Adobe的Crossbridge编译器编译[physx](https://developer.nvidia.com/physx-sdk)代码（C++
），使之能运行在Flash上。

Flash Player里内嵌了一个虚拟机，称之为[AVM(ActionScript Virtual Machine)](https://www.adobe.com/content/dam/Adobe/en/devnet/actionscript/articles/avm2overview.pdf)。 Crossbridge能把C++代码成编译成avm机器码ABC Code，从而能在Flash上执行。

Crossbridge的编译器是在llvm-2.9,llvm-gcc-4.2基础上修改得到的

We are a game developmenting company, who has developed several 3D games based on Adobe Flash Player. In one of our projects, we are trying to use Adobe Crossbridge to compile the [physx](https://developer.nvidia.com/physx-sdk) library (C++ based) to make it runnable on Flash.

Flash player embedded a virtual machine called [AVM(ActionScript Virtual Machine)](https://www.adobe.com/content/dam/Adobe/en/devnet/actionscript/articles/avm2overview.pdf), and Crossbridge can turn C++ code into AVM machine code(named the ABC code) thus make it runnable on flash.

Crossbridge is based on llvm-2.9 and llvm-gcc-4.2, and have made some changes to them.

## 问题现象 (Problem Summary) ##
我们碰到的问题是，同样的输入，CrossBridge编译出来的程序会死循环，而原生的使用原生llvm-2.9、llvm-gcc-4.2在Windows上编译得到的程序不会。

死循环发生在一个名为_gjkRelativeRayCast的函数。经过一些测试，我们发现两个现象：

The problem we have encounterd is that, fed with same input the program generated by crossbridge will produce different results with the program generated with original llvm-2.9,llvm-gcc-4.2 for windows. The former will run into dead loop sometimes.

The dead loop happens in a function named _gjkRelativeRayCast. After some testing, we have two observations:


### 现象一(Observation 1)　###
CrossBridge编译后的程序，变量**sDist**在循环的迭代N中的值，在进入下一个迭代N+1后，会更改。

If the following source code is compiled by CrossBridge, variable **sDist** will be changed unexpectedly at the end of one loop iteration.

	while(BAllEq(bNotTerminated, bTrue))
		{
			// 在迭代N+1开始时，sDist 突然变为 0.0506684184074401855468750
			// when iternal N+1 begins, sDist becomes 0.0506684184074401855468750
			printf("Start Loop: sDist.x is %.60f\n", sDist.x); 
			minDist = sDist;
			
			... // 省略的代码
			
			
			sDist = V3Dot(closest, closest);

			... // 省略的代码

			// 在迭代N结束时， sDist值为 0.050668418176030627364525571465492248535156250
			// when iteraion N ends, sDist equals 0.050668418176030627364525571465492248535156250
			printf("End Loop: sDist.x is %.60f\n", sDist.x); 
			
		}

### 现象二 (Observation 2)　###

两段有相同语义的代码会有不同的执行结果。　

- 在代码Ａ中，minDist在两个地方被赋值(赋值1,赋值2）。
- 在代码Ｂ中，我们确定minDist有没有发生赋值2。如果没有发生赋值2，再执行赋值1，因为赋值1最终反正会被赋值2覆盖。

注：minDist和sDist没有在省略的代码中被引用。

Two sematicly equal blocks of source code, will have different result:


- In Code A, minDist will be assined at two locations(assignment 1, assignment 2)
- In code B, we determine if assignment 2 to minDist happens. If not we then run assignment 1, else do nothing. That's becuase assignment 1 will be override by assignment 2 anyway. 

Note: minDist And sDist is not referenced in the omitted code.

#### 代码A (code A) ####

	... // 省略的代码(Intentionally omitted)
	if(FAllGrtr(vw, zero))
	{
		minDist = sDist;　// 赋值1(assignment 1)
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
				... //　省略的代码(Intentionally omitted)
				minDist = maxDist; // 赋值2(assignment 2)
				nor = closest;
			}
		}
	}
	... // 省略的代码(Intentionally omitted)

#### 代码B (code B) ####
    
	... // 省略的代码(Intentionally omitted)
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
				... // 省略的代码(Intentionally omitted)
				minDist = maxDist;　// 赋值2(assignment 2)
				modified = true;
				nor = closest;
				
			}
		}
	}
	if(modified == false) {
		minDist = sDist;　// 赋值1(assignment 1)
	}
	... // 省略的代码(Intentionally omitted)

### Result
The problem is that Crossbridge does not implement FP_ROUND (Rounding from a larger floating point type down to the precision of the destination), what it does is instead simply an assignment.
