
#ifndef INCFILE_H_
#define INCFILE_H_


#define _CRT_SECURE_NO_WARNINGS
#pragma warning(disable:4996) 

#include "stdio.h"


//存储结构节点位移
extern FILE* save_node_dis;
//存储单元内力
extern FILE* save_unit_force;
//存储节点力
extern FILE* save_node_force;
//显示输入数据
extern FILE* show_input_data;
//原始数据
extern FILE* input_data;
//单元的节点总数
extern int node_num_each_unit;
//单个节点的自由度数
extern int node_DOF;
//节点总数
extern int node_num;
//单元总数
extern int unit_num;
//受约束的自由度总数
extern int constrained_DOF_num;
//单元类别总数，单元的类别数
extern int unit_kind_num;
//N=node_num*node_DOF
extern int DOF;
//一维存储global_stiffness_matrix的总容量
extern int stiffness_matrix_capacity;
//各个节点的三维坐标
extern double X[50], Y[50], Z[50];
extern double X2, X1, Y2, Y1, Z2, Z1, b;
//每个单元的节点号，node_id_of_unit[1][i]存放第i个单元第一个节点坐标号，node_id_of_unit[2][i]存放第i个单元第二个节点坐标号
extern int node_id_of_unit[4][50];
//约束的位移号
extern int constrained_dx_id[30];
//PDE_id[i]记录第i行主对角元global_stiffness_matrix[i][i]在一维存储中的编号
extern int PDE_id[50];
//每个单元的类别
extern int unit_kind[100];
extern int NMN;
//unit_attribute[1][i]存放第i种类型的单元的杨氏模量，unit_attribute[2][i]存放第i种类型的单元的横截面积
extern double unit_attribute[4][100];
//节点载荷
extern double P[100], P1[100];
//整体结构节点力
extern double PP[100];
//单元横截面
extern double A;
//单元杨氏模量
extern double E;
//单元惯性矩
extern double I;
//算刚度矩阵的中间变量
extern double C;
//单元刚度矩阵
extern double TK[7][7];
//坐标转换矩阵
extern double T[7][7];
//坐标转换矩阵的转置
extern double TT[7][7];
//整体刚度矩阵
extern double global_stiffness_matrix[100];
//整体坐标系下的单元刚度阵
extern double partial_stiffness_matrix[7][7];
//作矩阵乘法时的中间矩阵
extern double s[7][7];
extern int partial_id_to_global_id[7];
//杆单元的长度
extern double L;
//单元应力
extern double SG;
//结构位移矩阵
extern double dr[100];
//单元位移矩阵
extern double dr_result[100];
//局部坐标下的单元位移矩阵
extern double dee[50][7];
//单元内力（局部坐标系下的单元节点力）
extern double Fee[50][7];
//单元整体坐标系中的节点力
extern double F[50][100];
//解方程时用到的L,Y矩阵
extern double l[100][100], y[100];


//输入输出仿真所需要的参数
void scan();

//求每i根杆的长度(i表示单元号)
void Length(int i);

//求每i根杆单元的单元刚度矩阵(单位坐标系下)
void StiffnessMatrix_unit(int i);

//形成空间桁架单元的坐标转换矩阵
void TransformMatrix(int i);

//计算杆元总体坐标系中的刚度矩阵
void MultiplyMatrix(int i);

//计算partial_id_to_global_id数组，partial_id_to_global_id(i)表示在单元中的第i个位移对应总体桁架结构中的位移编号
void creating_partial_id_to_global_id(int i);

//计算PDE_id数组，有node_DOF*unit_num即一个节点对应的自由度数乘以单元的节点数，PDE_id[i]记录第i行主对角元global_stiffness_matrix[i][i]在一维存储中的编号
void creating_PDE_id(int i);

//将每个单元刚度矩阵组集为结构刚度矩阵，该结构刚度矩阵一维存储，并且只存储下三角的半带宽
void StructureMatrix();

//解大型线性方程组AX=B,其中A是n阶大型稀疏对称正定矩阵，X和B是n*1阶矩阵

void colve_equation(int n, double a[100], double x[100]);

//求解单元内力以及约束反力并检查平衡
void OutputInternalForce();


//测试用函数
void test();


#endif  /* INCFILE_H_ */

