#include "tool.h"


using namespace std;


// extern "C"{//添加extern "C"
void Mcopy(float (*C)[4], float (*P)[4])
{
	P[0][0] = C[0][0];
	P[0][1] = C[0][1];
	P[0][2] = C[0][2];
	P[0][3] = C[0][3];
	P[1][0] = C[1][0];
	P[1][1] = C[1][1];
	P[1][2] = C[1][2];
	P[1][3] = C[1][3];
	P[2][0] = C[2][0];
	P[2][1] = C[2][1];
	P[2][2] = C[2][2];
	P[2][3] = C[2][3];
}

void show_point(string name, float *value)
{
	cout << name << endl;
	for (int i = 0; i < 3; i++)
		cout << value[i] << " ";
	cout << endl;
};

void show_value(string name, float (*T)[4])
{
	cout << name << endl;
	for (int i = 0; i < 3; i++)
	{
		for (int k = 0; k < 4; k++)
			cout << T[i][k] << " ";
		cout << endl;
	}
	cout << endl;
}

void show_value(string name, float *value)
{
	cout << name << endl;
	for (int i = 0; i < IDNUM; i++)
		cout << value[i] << " ";
	cout << endl;
}
void show_value(string name, uint32_t *value)
{
	cout << name << endl;
	for (int i = 0; i < IDNUM; i++)
		cout << int(value[i]) << " ";
	cout << endl;
}

void copy_value(float *copy, float *paste, int n)
{
	for (int i = 0; i < n; i++)
		paste[i] = copy[i];
};

void get_cmdlist(uint8_t *L, uint8_t c)
{
	for (int i = 0; i < IDNUM; i++)
		L[i] = c;
}
void get_canidlist(uint8_t *cL)
{
	for (int i = 0; i < IDNUM; i++)
		cL[i] = i + 1;
}
void get_paralist(uint32_t *L, uint32_t c)
{
	for (int i = 0; i < IDNUM; i++)
		L[i] = c;
}

int getch() // nonblock
{
	static struct termios oldt, newt;
	tcgetattr(STDIN_FILENO, &oldt); // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON); // disable buffering
	newt.c_cc[VMIN] = 0;
	newt.c_cc[VTIME] = 0;

	tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings
	int c = getchar();						 // read character (non-blocking)
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
	return c;
}

char scanKeyboard() // block
{
	struct termios stored_settings, new_settings;
	tcgetattr(0, &stored_settings);
	new_settings = stored_settings;
	new_settings.c_lflag &= (~ICANON);
	new_settings.c_cc[VMIN] = 1;
	new_settings.c_cc[VTIME] = 0;

	tcgetattr(0, &stored_settings);

	tcsetattr(0, TCSANOW, &new_settings);
	char in = getchar();
	tcsetattr(0, TCSANOW, &stored_settings);
	return in;
};

// 用案例测试解算器的正确性
// void test_demo()
// {
// 	class robotArm2 TH;
// 	TH.j[0] = 0.78, TH.j[1] = -0.21, TH.j[2] = -0.04, TH.j[3] = 1.2, TH.j[4] = -2.43, TH.j[5] = -0.45;
// 	float T[4][4];
// 	// TH.rc=2;
// 	if (TH.forward_move())
// 	{
// 		// TH.pos[0]=123.425,TH.pos[1]=0,TH.pos[2]=250,TH.pos[3]=0.049768,TH.pos[4]=-3.13849,TH.pos[5]=0;
// 		show_value("orij", TH.j);
// 		if (TH.backward_move())
// 		{
// 			show_value("nowj", TH.j);
// 			TH.J2MatrixT(T);
// 			if (TH.checkcalj(T))
// 				cout << "solver correct." << endl;
// 		}
// 	}
// 	TH.testj2p();
// }
// // 用案例测试解算器的正确性
// void test()
// {
// 	class robotArm2 TH;
// 	float j1, j2, j3, j4, j5, j6, jstp = 3;
// 	int total = 0, svb = 0, crt = 0;
// 	float oripos[6], orij[6];
// 	int t;
// 	float T[4][4];
// 	float st = -pi;
// 	for (j1 = st; j1 < pi; j1 += jstp)
// 		for (j2 = st; j2 < pi; j2 += jstp)
// 			for (j3 = st; j3 < pi; j3 += jstp)
// 				for (j4 = st; j4 < pi; j4 += jstp)
// 					for (j5 = st; j5 < pi; j5 += jstp)
// 						for (j6 = st; j6 < pi; j6 += jstp)
// 						{
// 							TH.j[0] = j1, TH.j[1] = j2, TH.j[2] = j3, TH.j[3] = j4, TH.j[4] = j5, TH.j[5] = j6;
// 							copy_value(TH.j, orij, 6);
// 							if (TH.forward_move())
// 							{
// 								copy_value(TH.pos, oripos, 6);
// 								for (t = 0; t < 8; t++)
// 								{
// 									TH.rc = t;
// 									// TH.rc=8;
// 									total++;
// 									if (TH.backward_move())
// 									{
// 										// show_value("j",j);
// 										svb++;
// 										///*
// 										// 利用矩阵检验
// 										TH.J2MatrixT(T);
// 										if (TH.checkcalj(T))
// 											crt++;
// 										else
// 										{
// 											show_value("wrong j", TH.j);
// 											show_value("for ori j", orij);
// 											show_value("oripos", oripos);
// 											cout << endl;
// 										}
// 									} // else cout<<"ori j1 "<<j1<<endl<<"--------------------------------------"<<endl;//show_value("ori j",orij);
// 								}
// 							}
// 						}
// 	cout << "total: " << total << " svb: " << svb << " crt: " << crt << " svb_rate: " << float(svb) / total << " crt_rate: " << float(crt) / svb << endl;
// };

// }//添加extern "C"