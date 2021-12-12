#include "function.h"
#include <iostream>
using namespace std;
int main()
{
	scan();
	StructureMatrix();
	DOF = node_num * node_DOF;
	colve_equation(DOF, global_stiffness_matrix, P);
	OutputInternalForce();
	//test();
	
	//测试colve_equation函数的代码如下
	//double a[100], x[100];
	//colve_equation(6, a, x);
	//for (int i = 1; i <= 6; ++i)
	//	cout << x[i] << '\t';
}

