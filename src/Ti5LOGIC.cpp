#include "Ti5LOGIC.h"
#include <iostream>
#include <time.h>
#include <mathfunc.h>
#include <tool.h>
// float l[5]={189.5,300,115.5,184,95.5},width=131.5,rl1=45.5;//3kg
// float l[5]={249,450,165.5,284,96},width=105,rl1=60;//5kg
using namespace std;

extern "C"{//添加extern "C"

/*检测点范围是否合法
参数:
    P1:点是否与P0,d线段碰撞
	线段方程两端：P0,d
	P1点
	线段长度 l
	x12：x轴向上12点缩放参数
	r1：线段P0，d机械臂半径
    r2：点P1机械臂半径
检查两个坐标系之间的转换是否相交。它接受以下参数：
P0：第一个坐标系的三个坐标值。
d：从第一个坐标系到第二个坐标系的转换向量。
P1：第二个坐标系的三个坐标值。
x12：两个坐标系之间的距离。
l：第一个坐标系内的圆心到圆周的距离。
r1：第一个坐标系内的半径。
r2：第二个坐标系内的半径。
*/

// 检测两连杆是否碰撞，P1、P2构成一根连杆，半径为r1；P3、P4构成另一根连杆，半径为r2

bool pos_trans::LLcolsp(float P1[3], float P2[3], float P3[3], float P4[3], float r1, float r2) {
			//return false;
			if(r1==0 or r2==0) return false;
			float d12[3],d34[3],d13[3];vec_subtraction(P2,P1,d12);vec_subtraction(P4,P3,d34);vec_subtraction(P3,P1,d13);
			float n[3];float distance;
			vec_cross(d12,d34,n);
			if(sqrt(vec_dot(n,n))<0.1){//平行或重合
				vec_rescale(d34,1,d34);
				float h[3];
				vec_cross(d13,d34,h);
				distance=sqrt(vec_dot(h,h));
				if(distance>=r1+r2) return false;
				else{
					//cout<<"distance "<<distance<<endl;
					float d[3];
					vec_cross(d12,h,d);
					if(distance) vec_rescale(d,distance,d);
					float Pa[3],Pb[3];vec_addition(P3,d,Pa);vec_subtraction(P3,d,Pb);
					float Pc[3],Pd[3];vec_addition(P4,d,Pc);vec_subtraction(P4,d,Pd);
					
					float t1=0,t2=1,t3,t4;
					for(int i=0;i<3;i++){
						if(d12[i]){
							if(pointsdistance(Pa,P1)<pointsdistance(Pb,P1)){
								t3=(Pa[i]-P1[i])/d12[i];
								t4=(Pc[i]-P1[i])/d12[i];
							}
							else{
								
								t3=(Pb[i]-P1[i])/d12[i];
								t4=(Pd[i]-P1[i])/d12[i];
								//cout<<"t3: "<<t3<<" t4: "<<t4<<endl;
							}
							break;	
						}
					}
					if(min(t3,t4)>t2 or max(t3,t4)<t1){
						//cout<<"t3: "<<t3;
						return false;
					}
					else{
						//cout<<"重合"<<endl; 
						return true;
					}	
				}
			}else{//异面或相交
				//cout<<"here"<<endl;
				vec_rescale(n,1,n);
				distance=abs(vec_dot(n,d13));
				if(distance>=r1+r2){
					//cout<<"异面"<<endl;
					return false;
				}else{
					float n1[3];vec_cross(d12,n,n1);
					float t=-(n1[0]*d13[0]+n1[1]*d13[1]+n1[2]*d13[2])/(n1[0]*d34[0]+n1[1]*d34[1]+n1[2]*d34[2]);
					if(t<0 or t>1) return false;
					//cout<<"相交"<<endl;
					for(int i=0;i<3;i++){
						if(d12[i]){
							t=(P3[i]+d34[i]*t-P1[i])/d12[i];
							break;	
						}
					}
					if(t<0 or t>1){
						//cout<<t<<endl;
						return false;
					}
					else return true;
				}
			}
}


/*函数的作用是初始化rod数组中每个元素的i1和i2成员变量的值。

具体来说，代码通过一个for循环遍历rod数组的每个索引i，从0到rodnum-1。在循环中，将当前索引i赋值给rod[i]的i1成员变量，并将i+1赋值给rod[i]的i2成员变量。

这段代码的目的是为了将rod数组中每个元素的i1和i2成员变量设置为相应的索引值。通过这种方式，可以方便地在后续的操作中使用这些索引值来访问和处理数组中的元素。*/
void pos_trans::init_rodindex()
{
	for (int i = 0; i < rodnum; i++)
		rod[i].i1 = i, rod[i].i2 = i + 1;
}

// 若点距和连杆长度一致则认为点坐标值正确
bool pos_trans::check_Points()
{
	float v[3];
	for (int i = 0; i < rodnum; i++)
	{
		vec_subtraction(P_0[rod[i].i1], P_0[rod[i].i2], v);
		if (abs(vec_length(v) - rod[i].l) > 0.5)
		{
			//cout << "i: " << i << " i1 " << rod[i].i1 << " i2 " << rod[i].i2 << " len " << vec_length(v) << " l " << rod[i].l << endl;
			return false;
		}
	}
	return true;
}
int pos_trans::gotostep(int now, bool recall, bool *dft)
{
	int step;
	if (recall)
	{
		for (step = now; step >= 0; step--)
		{
			if (dft[step])
				break;
			else
				dft[step] = true;
		}
		if (step > -1)
			dft[step] = false;
	}
	else
		step = now + 1;
	return step;
};
// 根据变换矩阵计算位姿
bool pos_trans::MatrixT2Pos(float T[4][4], bool mend)
{
	int modea, modeb;
	if (mend)
	{
		if (pos[5] + pos[3] > pi)
			modea = 2;
		else if (pos[5] + pos[3] < -pi)
			modea = -2;
		else
			modea = 0;
		if (pos[5] - pos[3] > pi)
			modeb = 2;
		else if (pos[5] - pos[3] < -pi)
			modeb = -2;
		else
			modeb = 0;
	}
	else
		modea = modeb = 0;
	bool solvable;
	float k1 = T[0][0], k4 = T[1][0], k7 = T[2][0], k8 = T[2][1], k9 = T[2][2];
	pos[0] = T[0][3];
	pos[1] = T[1][3];
	pos[2] = T[2][3];
	float r, p, y, sp, cp2, cp, cy, sy;
	float a, b;
	sp = -k7;
	cp2 = 1 - sp * sp;
	if (cp2 >= scd)
	{
		solvable = true;
		a = calcu_angle(k1 * k8 + k4 * k9, k1 * k9 - k4 * k8, cp2) + modea * pi; // r+y
		b = calcu_angle(k1 * k8 - k4 * k9, k1 * k9 + k4 * k8, cp2) + modeb * pi; // r-y
		r = (a + b) / 2, y = (a - b) / 2;
		sy = sin(y), cy = cos(y);
		if (abs(cy) >= scd)
			cp = k1 / cy;
		else
			cp = k4 / sy;
		p = calcu_angle(sp, cp, 1);
		pos[5] = r;
		pos[4] = p;
		pos[3] = y;
	}
	else
	{
		solvable = false;
		pos[4] = calcu_angle(-k7, 0, 1);
		pos[3] = pos[5] = 0;
	}
	return true;
	//return solvable;
};
// 根据位姿计算变换矩阵
void pos_trans::Pos2MatrixT(float T[4][4])
{
	float
	sy = sin(pos[3]),
	cy = cos(pos[3]),
	sp = sin(pos[4]), cp = cos(pos[4]),
	sr = sin(pos[5]), cr = cos(pos[5]);
	T[0][0] = cp * cy;
	T[0][1] = sp * sr * cy - sy * cr;
	T[0][2] = sp * cr * cy + sr * sy;
	T[0][3] = pos[0];
	T[1][0] = sy * cp;
	T[1][1] = sp * sr * sy + cr * cy;
	T[1][2] = sp * sy * cr - sr * cy;
	T[1][3] = pos[1];
	T[2][0] = -sp;
	T[2][1] = sr * cp;
	T[2][2] = cp * cr;
	T[2][3] = pos[2];
	T[3][0] = T[3][1] = T[3][2] = 0;
	T[3][3] = 1;
};
bool pos_trans::mendjoints(float j0[6]){
	for(int i=0;i<6;i++){
		float j1=max(jr1[i],j0[i]-gap),j2=min(jr2[i],j0[i]+gap);
		if(j[i]<j1){
			while(j[i]<j1){
				j[i]+=2*pi;
			}if(j[i]>j2) return false;
		}else{
			while(j[i]>j2){
				j[i]-=2*pi;
			}
			if(j[i]<j1) return false;
		}
	}
	return true;
}

// 检测所有连杆间的碰撞
bool pos_trans::check_colsp()
{
	// return false;
	// showpointsinfo();
	for (int i = 2; i <= rodnum; i++)
		if (P_0[i][2] < 10)
			return true; // 与地面碰撞
	for (int i = 0; i < rodnum; i++)
	{
		for (int t = i + 2; t < rodnum; t++)
			if (LLcolsp(P_0[rod[i].i1], P_0[rod[i].i2], P_0[rod[t].i1], P_0[rod[t].i2], rod[i].r, rod[t].r))
			{
				// show_value("colsp j",j,6);
				// cout<<"clps "<<i<<" "<<t<<endl;
				return true;
			}
	}
	return false;
}

// 检测逆运动解出的角是否满足原始位姿
bool pos_trans::checkacc(float T[4][4]){
	float oriT[4][4];
	Pos2MatrixT(oriT);
	if( abs(oriT[0][0]-T[0][0])<prc2 and abs(oriT[0][1]-T[0][1])<prc3 and abs(oriT[0][2]-T[0][2])<prc3 and abs(oriT[0][3]-T[0][3])<1 
	and abs(oriT[1][0]-T[1][0])<prc2 and abs(oriT[1][1]-T[1][1])<prc3 and abs(oriT[1][2]-T[1][2])<prc3 and abs(oriT[1][3]-T[1][3])<1
	and abs(oriT[2][0]-T[2][0])<prc1 and abs(oriT[2][1]-T[2][1])<prc2 and abs(oriT[2][2]-T[2][2])<prc2 and abs(oriT[2][3]-T[2][3])<1)
	return true;else return false;	
}

// 求解除六个坐标系原点的其他点
void robotArm1::fromSn2S0(float *P0,int n){
	float P[3];
	for(int t=n-1;t>-1;t--){
		(this->*Si2Sj[t])(P0,P,true);
		copy_value(P,P0,3);
	}
}
void robotArm1::getOtherPoints(){//2,3,6
	P_0[2][0]=-len[5],P_0[2][1]=0,P_0[2][2]=0;
	P_0[3][0]=len[1],P_0[3][1]=0,P_0[3][2]=len[5];
	P_0[6][0]=-len[7],P_0[6][1]=0,P_0[6][2]=len[3];
	fromSn2S0(P_0[2],1);
	fromSn2S0(P_0[3],2);
	fromSn2S0(P_0[6],4);
}
// 点或向量在不同坐标系转换（p=true表示点，p=false表示向量）
void robotArm1::fromS0toS1(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[0]) * P0[0] + sin(j[0]) * P0[1];
	P[1] = cos(j[0]) * P0[1] - sin(j[0]) * P0[0];
	P[2] = P0[2];
	if (p)
		P[2] -= len[0];
};
void robotArm1::fromS1toS2(float P0[3], float P[3], bool p)
{
	P[0] = sin(j[1]) * P0[1] + cos(j[1]) * P0[2];
	P[1] = cos(j[1]) * P0[1] - sin(j[1]) * P0[2];
	P[2] = -P0[0];
}
void robotArm1::fromS2toS3(float P0[3], float P[3], bool p)
{
	P[0] = sin(j[2]) * P0[1] - cos(j[2]) * P0[0];
	P[1] = cos(j[2]) * P0[1] + sin(j[2]) * P0[0];
	P[2] = -P0[2];
	if (p)
	{
		P[0] += len[1] * cos(j[2]);
		P[1] -= len[1] * sin(j[2]);
	}
}
void robotArm1::fromS3toS4(float P0[3], float P[3], bool p)
{
	P[0] = sin(j[3]) * P0[1] + cos(j[3]) * P0[2];
	P[1] = cos(j[3]) * P0[1] - sin(j[3]) * P0[2];
	P[2] = -P0[0];
	if (p)
		P[2] -= len[2];
}
void robotArm1::fromS4toS5(float P0[3], float P[3], bool p)
{
	P[0] = sin(j[4]) * P0[1] - cos(j[4]) * P0[2];
	P[1] = cos(j[4]) * P0[1] + sin(j[4]) * P0[2];
	P[2] = P0[0];
	if (p)
	{
		P[0] += len[3] * cos(j[4]);
		P[1] -= len[3] * sin(j[4]);
	}
}
void robotArm1::fromS1toS0(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[0]) * P0[0] - sin(j[0]) * P0[1];
	P[1] = sin(j[0]) * P0[0] + cos(j[0]) * P0[1];
	P[2] = P0[2];
	if (p)
		P[2] += len[0];
};
void robotArm1::fromS2toS1(float P0[3], float P[3], bool p)
{
	P[0] = -P0[2];
	P[1] = sin(j[1]) * P0[0] + cos(j[1]) * P0[1];
	P[2] = cos(j[1]) * P0[0] - sin(j[1]) * P0[1];
}
void robotArm1::fromS3toS2(float P0[3], float P[3], bool p)
{
	P[0] = sin(j[2]) * P0[1] - cos(j[2]) * P0[0];
	P[1] = sin(j[2]) * P0[0] + cos(j[2]) * P0[1];
	P[2] = -P0[2];
	if (p)
		P[0] += len[1];
}
void robotArm1::fromS4toS3(float P0[3], float P[3], bool p)
{
	P[0] = -P0[2];
	P[1] = sin(j[3]) * P0[0] + cos(j[3]) * P0[1];
	P[2] = cos(j[3]) * P0[0] - sin(j[3]) * P0[1];
	if (p)
		P[0] -= len[2];
}
void robotArm1::fromS5toS4(float P0[3], float P[3], bool p)
{
	P[0] = P0[2];
	P[1] = sin(j[4]) * P0[0] + cos(j[4]) * P0[1];
	P[2] = sin(j[4]) * P0[1] - cos(j[4]) * P0[0];
	if (p)
		P[2] += len[3];
}
void robotArm1::fromS6toS5(float P0[3], float P[3], bool p)
{
	P[0]=-P0[2];
	P[1]=sin(j[5])*P0[0]+cos(j[5])*P0[1];
	P[2]=cos(j[5])*P0[0]-sin(j[5])*P0[1];
	if(p) P[0]-=len[4];
}
// 逆运动解算器
bool robotArm1::Pos2J(){
	float j0[6];copy_value(j,j0,6);
	float
	sy=sin(pos[3]),cy=cos(pos[3]),
	sp=sin(pos[4]),cp=cos(pos[4]),
	sr=sin(pos[5]),cr=cos(pos[5]);
	float d56_0[3]={sp*cr*cy + sr*sy,sp*sy*cr-sr*cy,cp*cr},P5_0[3];
	float X6_0[3]={cp*cy,sy*cp,-sp};
	vec_rescale(d56_0,len[4],d56_0);
	vec_subtraction(pos,d56_0,P5_0);
	
	float d15_0[3];
	vec_subtraction(P5_0,P_0[1],d15_0);
	float l15=vec_length(d15_0);
	if(l15>len[1]+len[2]+len[3] or l15<abs(len[1]-len[2]-len[3])) return false;
    float a135=solve_trangle(len[1],len[2]+len[3],l15);
    float a513=solve_trangle(l15,len[1],len[2]+len[3]);
    
    float n_0[3],n0_0[3]={1,0,0};
	int step=0;
	bool dft[3]={true,true,true};
	while(-1<step and step<3){
		if(step==0){
			if(dft[step])
				if(d15_0[0]==0 and d15_0[1]==0) copy_value(n0_0,n_0,3);
				else n_0[0]=-d15_0[1],n_0[1]=d15_0[0],n_0[2]=0;
			else n_0[0]=-n_0[0],n_0[1]=-n_0[1];
			j[0]=vec_angle(n0_0,n_0,n_0[1]);
			step++;
		}else if(step==1){
			float d15_1[3],d150_1[3]={0,0,1};
			fromS0toS1(d15_0,d15_1,false);
			float mid=vec_angle(d150_1,d15_1,d15_1[1]);
			if(l15==len[1]+len[2]+len[3]) {a513=0;a135=pi;dft[step]=false;}
			else if(l15==len[1]-len[2]-len[3]) {mid=0;a513=0;a135=0;dft[step]=false;}//mid 无穷多解
			
			
			if(dft[step]){
				j[1]=mid+a513;
            	j[2]=pi-a135;
            	if(j[1]>=pi) j[1]-=2*pi;
			}
			else{
            	j[1]=mid-a513;
    			j[2]=a135-pi;
    			if(j[1]<-pi) j[1]+=2*pi;
			}	
			step++;
		}else if(step==2){
			float d56_3[3],d56_4[3];
			fromS0toS1(d56_0,d56_3,false);
			fromS1toS2(d56_3,d56_4,false);
			fromS2toS3(d56_4,d56_3,false);
			if(d56_3[1]==0 and d56_3[2]==0) {j[3]=0;dft[step]=false;}
			else{
				float n_3[3]={0,-d56_3[2],d56_3[1]},n0_3[3]={0,0,1};
				if(!dft[step]) n_3[1]=-n_3[1],n_3[2]=-n_3[2];
				j[3]=vec_angle(n0_3,n_3,n_3[1]);
			}
			fromS3toS4(d56_3,d56_4,false);
			float d560_4[3]={0,0,1};
			j[4]=vec_angle(d560_4,d56_4,-d56_4[1]);
			float X6_5[3],X6_t[3],X60_5[3]={0,0,1};
			fromS0toS1(X6_0,X6_5,false);
			fromS1toS2(X6_5,X6_t,false);
			fromS2toS3(X6_t,X6_5,false);
			fromS3toS4(X6_5,X6_t,false);
			fromS4toS5(X6_t,X6_5,false);
		    j[5]=vec_angle(X60_5,X6_5,X6_5[1]);
			step=gotostep(step,!checkcalj(j0),dft);
		}
	}
	if(step==3){
		//if(j[4]>=pi/2) j[4]-=2*pi; 
		return true;
	}else{
		copy_value(j0,j,6);
		return false;
	} 
};
// 根据角度计算点坐标
void robotArm1::J2Point(){
	for(int i=2;i<7;i++){//计算位于坐标系原点的点坐标
		P_0[O[i]][0]=0,P_0[O[i]][1]=0,P_0[O[i]][2]=0;
		fromSn2S0(P_0[O[i]],i);
	}
	getOtherPoints();
	//for(int i=1;i<9;i++) cout<<"P"<<i<<" "<<P_0[i][0]<<" "<<P_0[i][1]<<" "<<P_0[i][2]<<endl;
}
bool robotArm1::checkcalj(float j0[6]){
	J2Point();
	//if(check_Points()) cout<<"points correct"<<endl;else cout<<"points error"<<endl;
	if(mendjoints(j0) and !check_colsp()) return true;
	else return false;
}
robotArm1::robotArm1()
{
	init_arm_structure();
	init_model_structure();
};
// 根据角度计算点变换矩阵
void robotArm1::J2MatrixT(float T[4][4])
{
	float
	s1 = sin(j[0]), c1 = cos(j[0]),
	s2 = sin(j[1]), c2 = cos(j[1]),
	s4 = sin(j[3]), c4 = cos(j[3]),
	s5 = sin(j[4]), c5 = cos(j[4]),
	s6 = sin(j[5]), c6 = cos(j[5]),
	s23 = sin(j[1] - j[2]), c23 = cos(j[1] - j[2]);
	T[0][0]=-((s1*c4*c23 + s4*c1)*c5 + s1*s5*s23)*s6 - (s1*s4*c23 - c1*c4)*c6;
	T[0][1]=-((s1*c4*c23 + s4*c1)*c5 + s1*s5*s23)*c6 + (s1*s4*c23 - c1*c4)*s6;
	T[0][2]=(s1*c4*c23 + s4*c1)*s5 - s1*s23*c5;
	T[0][3]=len[4]*((s1*c4*c23 + s4*c1)*s5 - s1*s23*c5) - (len[1]*s2 + len[2]*s23 + len[3]*s23)*s1;
	T[1][0]=-((s1*s4 - c1*c4*c23)*c5 - s5*s23*c1)*s6 + (s1*c4 + s4*c1*c23)*c6;
	T[1][1]=-((s1*s4 - c1*c4*c23)*c5 - s5*s23*c1)*c6 - (s1*c4 + s4*c1*c23)*s6;	
	T[1][2]=(s1*s4 - c1*c4*c23)*s5 + s23*c1*c5;
	T[1][3]=len[4]*((s1*s4 - c1*c4*c23)*s5 + s23*c1*c5) + (len[1]*s2 + len[2]*s23 + len[3]*s23)*c1;
	T[2][0]=(s5*c23 - s23*c4*c5)*s6 - s4*s23*c6;
	T[2][1]=(s5*c23 - s23*c4*c5)*c6 + s4*s6*s23;
	T[2][2]=s5*s23*c4 + c5*c23;
	T[2][3]=len[0] + len[1]*c2 + len[2]*c23 + len[3]*c23 + len[4]*(s5*s23*c4 + c5*c23);
	T[3][0] = T[3][1] = T[3][2] = 0;
	T[3][3] = 1;
}
// 正运动
bool robotArm1::forward_move()
{
	float T[4][4];
	J2MatrixT(T);
	J2Point();
	//if(check_Points()) cout<<"points correct"<<endl;else cout<<"points error"<<endl;
	return (!check_colsp() and MatrixT2Pos(T, false));
};
// 逆运动
bool robotArm1::backward_move()
{
	return Pos2J();
};
// 测试fromSi2Sj函数是否正确（是否抄错）
void robotArm1::testj2p(){
	if(forward_move()){
		float P6[3]={0,0,0};
		fromSn2S0(P6,6);
		if(abs(P6[0]-pos[0])<1 and abs(P6[1]-pos[1])<1 and abs(P6[2]-pos[2])<1) cout<<"fromSi2Sj correct"<<endl;
		else cout<<"fromSi2Sj wrong"<<endl;
	}
}
/*
void (robotArm2::*Si2Sj[6])(float *, float *, bool) = {fromS1toS0, fromS2toS1, fromS3toS2, fromS4toS3, fromS5toS4, fromS6toS5};
void robotArm2::init_arm_structure()
{
	rodnum = 7;
	rod[0].r = 80, rod[1].r = 80, rod[2].r = 50, rod[3].r = 50, rod[4].r = 35, rod[5].r = 35, rod[6].r = 35;
	rod[0].l = 160.5, rod[1].l = 190.5, rod[2].l = 714.5, rod[3].l = 147.5, rod[4].l = 636.33, rod[5].l = 125, rod[6].l = 125.5;
	init_rodindex();
}
void robotArm2::init_model_structure()
{
	O[1] = 1, O[2] = 2, O[3] = 3, O[4] = 5, O[5] = 6, O[6] = 7;
	jr[0] = pi, jr[1] = pi, jr[2] = pi, jr[3] = pi, jr[4] = pi, jr[5] = pi;
	len[0] = 160.5, len[1] = 190.5, len[2] = 714.5, len[3] = 147.5, len[4] = 636.33, len[5] = 125, len[6] = 125.5;
	dst = len[1] - len[3] + len[5];
	forward_move();
}
*/
bool robotArm2::checkcalj(float j0[6]){
	J2Point();//if(check_Points()) cout<<"points correct"<<endl;
	if(mendjoints(j0) and !check_colsp()) return true;
	else return false;
}
void robotArm2::fromSn2S0(float *P0,int n){
	float P[3];
	for(int t=n-1;t>-1;t--){
		(this->*Si2Sj[t])(P0,P,true);
		copy_value(P,P0,3);
	}
}
void robotArm2::getOtherPoints(){//4
	P_0[4][0]=0,P_0[4][1]=0,P_0[4][2]=-len[3];
	fromSn2S0(P_0[4],3);
}
void robotArm2::fromS0toS1(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[0]) * P0[0] + sin(j[0]) * P0[1];
	P[1] = cos(j[0]) * P0[1] - sin(j[0]) * P0[0];
	P[2] = P0[2];
	if (p)
		P[2] -= len[0];
};
void robotArm2::fromS1toS2(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[1]) * P0[0] - sin(j[1]) * P0[2];
	P[1] = -sin(j[1]) * P0[0] - cos(j[1]) * P0[2];
	P[2] = P0[1];
	if (p)
		P[2] -= len[1];
}
void robotArm2::fromS2toS3(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[2]) * P0[0] + sin(j[2]) * P0[1];
	P[1] = cos(j[2]) * P0[1] - sin(j[2]) * P0[0];
	P[2] = P0[2];
	if (p)
	{
		P[0] += len[2] * sin(j[2]);
		P[1] += len[2] * cos(j[2]);
	}
}
void robotArm2::fromS3toS4(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[3]) * P0[0] - sin(j[3]) * P0[1];
	P[1] = -sin(j[3]) * P0[0] - cos(j[3]) * P0[1];
	P[2] = -P0[2];
	if (p)
	{
		P[0] -= len[4] * sin(j[3]);
		P[1] -= len[4] * cos(j[3]);
		P[2] -= len[3];
	}
}
void robotArm2::fromS4toS5(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[4]) * P0[0] + sin(j[4]) * P0[2];
	P[1] = cos(j[4]) * P0[2] - sin(j[4]) * P0[0];
	P[2] = -P0[1];
	if (p)
	{
		P[0] += len[5] * sin(j[4]);
		P[1] += len[5] * cos(j[4]);
	}
}
void robotArm2::fromS1toS0(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[0]) * P0[0] - sin(j[0]) * P0[1];
	P[1] = sin(j[0]) * P0[0] + cos(j[0]) * P0[1];
	P[2] = P0[2];
	if (p)
		P[2] += len[0];
};
void robotArm2::fromS2toS1(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[1]) * P0[0] - sin(j[1]) * P0[1];
	P[1] = P0[2];
	P[2] = -sin(j[1]) * P0[0] - cos(j[1]) * P0[1];
	if (p)
		P[1] += len[1];
}
void robotArm2::fromS3toS2(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[2]) * P0[0] - sin(j[2]) * P0[1];
	P[1] = sin(j[2]) * P0[0] + cos(j[2]) * P0[1];
	P[2] = P0[2];
	if (p)
		P[1] -= len[2];
}
void robotArm2::fromS4toS3(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[3]) * P0[0] - sin(j[3]) * P0[1];
	P[1] = -sin(j[3]) * P0[0] - cos(j[3]) * P0[1];
	P[2] = -P0[2];
	if (p)
	{
		P[1] -= len[4];
		P[2] -= len[3];
	}
}
void robotArm2::fromS5toS4(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[4]) * P0[0] - sin(j[4]) * P0[1];
	P[1] = -P0[2];
	P[2] = sin(j[4]) * P0[0] + cos(j[4]) * P0[1];
	if (p)
		P[2] -= len[5];
}
void robotArm2::fromS6toS5(float P0[3], float P[3], bool p)
{
	P[0] = cos(j[5]) * P0[0] - sin(j[5]) * P0[1];
	P[1] = -P0[2];
	P[2] = sin(j[5]) * P0[0] + cos(j[5]) * P0[1];
	if (p)
		P[2] -= len[6];
}
bool robotArm2::Pos2J(){
	float j0[6];copy_value(j,j0,6);
	float d16_0[3];
	vec_subtraction(pos,P_0[1],d16_0);
	float d16proj0_0[3]={0,1,0},d16proj_0[3]={d16_0[0],d16_0[1],0};
	float l16proj_0=vec_length(d16proj_0);
	if(l16proj_0==0) return false; 
	float fi=vec_angle(d16proj0_0,d16proj_0,-d16proj_0[0]);
	float as;
	if(abs(l16proj_0)<dst){
		if(abs(l16proj_0)>dst-1) l16proj_0=l16proj_0>0?dst:-dst;
		else return false;
	}
	as=acos(dst/l16proj_0);
	//子树求解
	float
	sy=sin(pos[3]),cy=cos(pos[3]),
	sp=sin(pos[4]),cp=cos(pos[4]),
	sr=sin(pos[5]),cr=cos(pos[5]);
	float X6_0[3]={cp*cy,sy*cp,-sp},Z6_0[3]={sp*cr*cy+sr*sy,sp*sy*cr-sr*cy,cp*cr},Z6_1[3];
	int step=0;bool dft[3]={true,true,true};
	float d56_1[3];
	float d15proj_1[3],l15_prj;
	float P6_1[3];
	while(-1<step and step<3){
		if(step==0){
			if(dft[step]){
				j[0]=fi-as;
				if(j[0]<-pi) j[0]+=2*pi;
			}else{
				j[0]=fi+as;
				if(j[0]>=pi) j[0]-=2*pi;
			}
			step++;
		}else if(step==1){
			float P5_1[3];
			fromS0toS1(pos,P6_1,true);
			if(dft[step]){
				fromS0toS1(Z6_0,Z6_1,false);
				if(abs(Z6_1[1])>0.999999){
					d56_1[0]=P6_1[0],d56_1[1]=0,d56_1[2]=P6_1[2];
				}else{
					d56_1[0]=Z6_1[2],d56_1[1]=0,d56_1[2]=-Z6_1[0];
				}
				vec_rescale(d56_1,len[6],d56_1);
			}else{
				d56_1[0]=-d56_1[0],d56_1[1]=0,d56_1[2]=-d56_1[2];
			}
			vec_subtraction(P6_1,d56_1,P5_1);
			d15proj_1[0]=P5_1[0],d15proj_1[1]=0,d15proj_1[2]=P5_1[2];
			l15_prj=vec_length(d15proj_1);
			step=gotostep(step,l15_prj>len[2]+len[4] or l15_prj<len[2]-len[4],dft);
		}else if(step==2){
			float d15proj0_1[3]={0,0,1};
			float mid=vec_angle(d15proj0_1,d15proj_1,d15proj_1[0]);
			float a315,a531;
			if(l15_prj==len[2]+len[4]){
				a315=0,a531=pi;dft[step]=false;
			}else if(l15_prj==len[2]-len[4]){
				a315=0,a531=0;dft[step]=false;
				mid=0;
			}else{
				a315=solve_trangle(len[2],l15_prj,len[4]);
				a531=solve_trangle(len[2],len[4],l15_prj);
			}
			
			if(dft[step]){
				j[1]=mid-a315;
				j[2]=pi-a531;
				if(j[1]<-pi) j[1]+=2*pi;
			}
			else{
				j[1]=mid+a315;
				j[2]=a531-pi;//a531=0时确保为-pi
				if(j[1]>=pi) j[1]-=2*pi;
			}
			float d56_2[3],d56_3[3];
			fromS1toS2(d56_1,d56_2,false);
			fromS2toS3(d56_2,d56_3,false);
			float v[3]={0,-1,0};
			j[3]=vec_angle(v,d56_3,-d56_3[0]);
			
			float Z6_3[3],Z6_4[3],Z60_4[3]={0,0,-1};
			fromS1toS2(Z6_1,Z6_4,false);
			fromS2toS3(Z6_4,Z6_3,false);
			fromS3toS4(Z6_3,Z6_4,false);
			j[4]=vec_angle(Z60_4,Z6_4,Z6_4[0]);
	
			float X60_5[3],X6_5[3];
			fromS0toS1(X6_0,X6_5,false);
			fromS1toS2(X6_5,X60_5,false);
			fromS2toS3(X60_5,X6_5,false);
			fromS3toS4(X6_5,X60_5,false);
			fromS4toS5(X60_5,X6_5,false);
			X60_5[0]=1,X60_5[1]=0,X60_5[2]=0;
			j[5]=vec_angle(X60_5,X6_5,X6_5[2]);
			step=gotostep(step,!checkcalj(j0) or rc-->0,dft);
		}
	}
	
	if(step==3) return true;
	else{
		copy_value(j0,j,6);
		return false;
	} 
};
void robotArm2::J2Point(){
	for(int i=2;i<7;i++){
		P_0[O[i]][0]=0,P_0[O[i]][1]=0,P_0[O[i]][2]=0;
		fromSn2S0(P_0[O[i]],i);
	}
	getOtherPoints();
}

robotArm2::robotArm2()
{
	init_arm_structure();
	init_model_structure();
};
void robotArm2::J2MatrixT(float T[4][4])
{
	float
		s1 = sin(j[0]),
		c1 = cos(j[0]),
		s2 = sin(j[1]), c2 = cos(j[1]),
		s23 = sin(j[1] + j[2]), c23 = cos(j[1] + j[2]),
		s234 = sin(j[1] + j[2] - j[3]), c234 = cos(j[1] + j[2] - j[3]),
		s5 = sin(j[4]), c5 = cos(j[4]),
		s6 = sin(j[5]), c6 = cos(j[5]);
	T[0][0] = (s1 * s5 + c1 * c5 * c234) * c6 - s6 * s234 * c1;
	T[0][1] = -(s1 * s5 + c1 * c5 * c234) * s6 - s234 * c1 * c6;
	T[0][2] = -s1 * c5 + s5 * c1 * c234;
	T[0][3] = -len[1] * s1 + len[2] * s2 * c1 + len[3] * s1 + len[4] * s23 * c1 - len[5] * s1 + len[6] * s234 * c1;
	T[1][0] = (s1 * c5 * c234 - s5 * c1) * c6 - s1 * s6 * s234;
	T[1][1] = -(s1 * c5 * c234 - s5 * c1) * s6 - s1 * s234 * c6;
	T[1][2] = s1 * s5 * c234 + c1 * c5;
	T[1][3] = len[1] * c1 + len[2] * s1 * s2 - len[3] * c1 + len[4] * s1 * s23 + len[5] * c1 + len[6] * s1 * s234;
	T[2][0] = -s6 * c234 - s234 * c5 * c6;
	T[2][1] = s6 * s234 * c5 - c6 * c234;
	T[2][2] = -s5 * s234;
	T[2][3] = len[0] + len[2] * c2 + len[4] * c23 + len[6] * c234;
	T[3][0] = T[3][1] = T[3][2] = 0;
	T[3][3] = 1;
}
bool robotArm2::forward_move()
{
	float T[4][4];
	J2MatrixT(T);
	J2Point();
	// if(check_Points()) cout<<"points correct"<<endl;
	return (!check_colsp() and MatrixT2Pos(T, false));
};
bool robotArm2::backward_move()
{
	return Pos2J();
};
void robotArm2::testj2p(){
	if(forward_move()){
			float P6[3]={0,0,0};
			fromSn2S0(P6,6);
			if(abs(P6[0]-pos[0])<1 and abs(P6[1]-pos[1])<1 and abs(P6[2]-pos[2])<1) cout<<"fromSi2Sj correct"<<endl;
			else cout<<"fromSi2Sj wrong"<<endl;
	}
}
}//添加extern "C"