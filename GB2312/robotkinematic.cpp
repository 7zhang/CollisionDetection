#define __ROBOTDATA_EXP_
#include "robotdata.h"
#include "robotkinematic.h"

////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// ģ��������KunshanRKA������Ԫ������ʵ�֡�
// ���ߣ�����
// ���ڣ�2011-3-29
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////
bool KunshanRKA::PositiveRobot(const JAngle& JA,TRANS& t6)
{
	return KS_PositiveRobot(JA, t6);
}
illegal_joint KunshanRKA::IsLegal(const JAngle& JA)
{
	return NOJOINT;
}
bool KunshanRKA::InverseRobotEx(vector<JAngle>& vecJointangle, const TRANS& t6)
{
	JAngle Jointangle, lastJointangle(0,0,0,0,0,0);
	if ( !InverseRobot(Jointangle, lastJointangle, t6) )
	{
		return false;
	}
	vecJointangle.push_back(Jointangle);
	return true;
}

bool KunshanRKA::InverseRobot(JAngle& Jointangle,const JAngle& lastJointangle,const TRANS& t6)
{
	JResolution reso;
	double diff = 0, temp=0;
	int i, j, min=0;
	bool flag=0;		//��¼�Ƿ��һ����ȡ�ؽڽǲ�ֵdiff
	if( KSNew_InverseRobot(reso, t6) )  //��8���������򷵻�1����ʾ����
	{
		return 1;
	}

	///////////��ؽڽǲ�ֵ��С��һ���//////////////////
	for(i=0; i<8; i++)          //�������Ϊԭ��ȡ���Ž�
	{
		if( reso.err[i] == 1)
		{
			continue;			//���������ֱ������
		}
		diff = 0;
		for(j=0; j<6; j++) 
		{
			diff += fabs(lastJointangle.angle[j]-reso.m_Resolution[i].angle[j]);  //��������һ���ؽڽǵĲ�ֵ
		}
		if(flag == 0 || (flag == 1 && diff < temp))		//flag == 0,������һ����ȡ�ؽڽǲ�ֵdiff
		{
			min = i;
			temp = diff;	//����ؽڽǲ�ֵ����Сֵ
		}
		flag = 1;
	}

	if( reso.err[min] == 2 )    //�����������ʱ  ��������������ǣ����ж�����⣬���������ܻ������⣩
	{
		reso.m_Resolution[min].angle[3] = lastJointangle.angle[3];  //�ؽڽ�4����һ����ͬ
	//	reso.m_Resolution[min].angle[4] = 0;     //�ؽڽ�5 = 0

		double nx, ny, nz, ox, oy, oz, ax, ay, az;
		nx = t6.rot.mem[0][0]; ny = t6.rot.mem[1][0]; nz = t6.rot.mem[2][0];
		ox = t6.rot.mem[0][1]; oy = t6.rot.mem[1][1]; oz = t6.rot.mem[2][1];
		ax = t6.rot.mem[0][2]; ay = t6.rot.mem[1][2]; az = t6.rot.mem[2][2];
		double temp1 = reso.m_Resolution[min].angle[0] * PI/180;
		double temp23 = (reso.m_Resolution[min].angle[1] + reso.m_Resolution[min].angle[2]) * PI/180;
		double temp4 = reso.m_Resolution[min].angle[3] * PI/180;
		double c5 =  cos(temp23)*(ax*cos(temp1)+ay*sin(temp1)) - sin(temp23)*az;
		reso.m_Resolution[min].angle[4] = atan2(0,c5) * 180/PI;   //�ؽڽ�5
		double s6 = -c5*( cos(temp4) * ( sin(temp23)*(cos(temp1)*ox+sin(temp1)*oy) + cos(temp23)*oz )
								 +sin(temp4) * ( sin(temp1)*ox-cos(temp1)*oy ) );
		double c6 = c5*( cos(temp4) * ( sin(temp23)*(cos(temp1)*nx+sin(temp1)*ny) + cos(temp23)*nz )
								+sin(temp4) * ( sin(temp1)*nx-cos(temp1)*ny ) );
		if( fabs(s6) < 1e-4 )
		{
			s6 = 0;			//Ϊ��ֹ����180��-180���ؽ�����
		}
		reso.m_Resolution[min].angle[5] = atan2(s6,c6) * 180/PI;   //�ؽڽ�6
	}

	for(i=0; i<6; i++)
		Jointangle.angle[i] = reso.m_Resolution[min].angle[i];
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////
// ==============================================================================
// ģ��������JResolution������Ԫ������ʵ�֡�
// ���ߣ�����
// ���ڣ�2011-3-29
// ==============================================================================
////////////////////////////////////////////////////////////////////////////////////

JResolution::JResolution()
{
	for(int i=0; i<8; i++)
		err[i] = 0;
}

JResolution::~JResolution()
{

}
