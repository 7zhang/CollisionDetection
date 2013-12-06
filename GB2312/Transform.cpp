// Transform.cpp: implementation of the Transform class.
//
//////////////////////////////////////////////////////////////////////


#include "Transform.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Transform::Transform()
{

}

Transform::~Transform()
{

}

void Transform::getTransBaseToJoints(const JAngle& jAngle, 
		TRANS& T01, TRANS& T02, TRANS& T03, TRANS& T04, TRANS& T05, TRANS& T06)
{
// 	double angle1 = jAngle.angle[0]*PI/180.0;
// 	double angle2 = jAngle.angle[1]*PI/180.0;
// 	double angle3 = jAngle.angle[2]*PI/180.0;
// 	double angle4 = jAngle.angle[3]*PI/180.0;
// 	double angle5 = jAngle.angle[4]*PI/180.0;
// 	double angle6 = jAngle.angle[5]*PI/180.0;

	double angle1 = jAngle.angle[0];
	double angle2 = jAngle.angle[1];
	double angle3 = jAngle.angle[2];
	double angle4 = jAngle.angle[3];
	double angle5 = jAngle.angle[4];
	double angle6 = jAngle.angle[5];

	T01 = trans(0.0, 0.0, KUNSHANROBOT_D1) * rotateZ(angle1);
	T02 = T01 * trans(KUNSHANROBOT_A2,0.0,0.0) * rotateX(-90.0) * rotateZ(angle2);
	T03 = T02 * trans(KUNSHANROBOT_A3,0.0,0.0) * rotateZ(-90.0) * rotateZ(angle3);
	T04 = T03 * rotateX(-90.0) * rotateZ(angle4);
	T05 = T04 * trans(0.0,0.0,KUNSHANROBOT_D4) * rotateX(90.0) * rotateZ(angle5);
	T06 = T05 * trans(0.0,KUNSHANROBOT_D5,0.0) * rotateX(-90.0) * rotateZ(angle6);

}

//�����������ϵ�������������α任����
TRANS Transform::getTransWorldToRail(const JAngle& extJAngle)
{
	return Transform::trans(121.0, extJAngle.angle[2], 728.0);//��λ�Ǻ���
}
//�����������ϵ�������˻�������α任����,�����������Ϊ�ⲿ���3������
TRANS Transform::getTransWorldToBase(const JAngle& extJAngle)
{
	//�������������ϵ������˻�����ϵ�غ�
	return Transform::getTransWorldToRail(extJAngle);
}
//�����������ϵ����λ������α任����
void Transform::getTransWorldToPos(const JAngle& extJAngle, TRANS& TP1, TRANS& TP2)
{
	double ext1 = extJAngle.angle[0];//��λ��1��Ƕ�
	double ext2 = extJAngle.angle[1];//��λ��2��Ƕ�

	TP1 = trans(1416.15, 13.67, 775.00)*rotateZ(-90)*rotateX(ext1);
	TP2 = TP1*trans(0.0,0.0,154.5)*rotateZ(ext2);
}
//�����������ϵ����������ϵ����α任����
TRANS Transform::getTransWorldToWorkpiece(const JAngle& extJAngle)
{
	//��������ϵ���λ��2������ϵ�غ�
	double ext1 = extJAngle.angle[0];//��λ��1��Ƕ�
	double ext2 = extJAngle.angle[1];//��λ��2��Ƕ�

	TRANS TP1 = trans(1416.15, 13.67, 775.00)*rotateZ(-90)*rotateX(ext1);
	TRANS TP2 = TP1*trans(0.0,0.0,154.5)*rotateZ(ext2);
	return TP2;
}
//�����ؽڵ������̵�T
TRANS Transform::getTrans6ToFlange()
{
	TRANS T6ToFlange = Transform::trans(0.0, 0.0, 14.0);
	return T6ToFlange;	
}

//�����̵���ǹĩβ��T
TRANS Transform::getTransFlangeToTorch()
{
	RPY flangleToEndRPY(-40.23, 0.16, 439.2, 0.0, -65.98, 180.0);//��λ�Ǻ���,�޸ĺ�ĺ�ǹʵ�ʽǶ�Ϊ24��
	TRANS flangleToEnd;
	flangleToEndRPY.RPY2Trans(flangleToEnd);
	return flangleToEnd;
}
//�����ؽڵ���ǹĩβ��T
TRANS Transform::getTrans6ToTorch()
{
	TRANS T6ToEnd;
	RPY T6ToEndRPY(-40.23, 0.16, 453.2, 0.0, -65.98, 180.0);//��λ�Ǻ���
	T6ToEndRPY.RPY2Trans(T6ToEnd);
	return T6ToEnd;
}

//�����ؽڵ���ǹ����ϵ
TRANS Transform::getTrans6ToGun()
{ 
	TRANS T6ToGun;
//	RPY T6ToGunRPY(0.0, 0.0, 14.0, 90.0,0.0,-90.0);//��λ�Ǻ��ף����Ժ�ǹ
	RPY T6ToGunRPY(0.0, 0.0, 17.0, 0.0, -90.0, 180.0);//��ȷ��ǹ��
	T6ToGunRPY.RPY2Trans(T6ToGun);
	return T6ToGun;
}
//�����������ϵ����ǹĩ������ϵ����α任����
TRANS Transform::getTransWorldToTorch(const JAngle& extJAngle, const TRANS& TWorkpieceToSeam)
{
	TRANS TWorldToTorch = getTransWorldToWorkpiece(extJAngle) * TWorkpieceToSeam* rotateZ(-90.0);
	return TWorldToTorch;
}
//��û����˻�������ϵ����ǹĩ������ϵ����α任����(�ɺ�����������ϵΪ����Ϊ�����)
TRANS Transform::getTransBaseToTorch(const JAngle& extJAngle, const TRANS& TWorkpieceToSeam)
{
	TRANS TWorldToTorch = getTransWorldToTorch(extJAngle,TWorkpieceToSeam);
	
	TRANS TWorldToBase = getTransWorldToBase(extJAngle);
	TWorldToBase.inverse();
	return TWorldToBase * TWorldToTorch;
}
//���T6
TRANS Transform::getT6ByTransBaseToTorch(const TRANS& TBaseToTorch)
{
	TRANS trans6ToTorch = getTrans6ToTorch();
	trans6ToTorch.inverse();
	return TBaseToTorch*trans6ToTorch;
}


TRANS Transform::rotateX(double angle)//�Ƕ�ֵ
{
	TRANS res;
	res.rotx(angle);
	return res;
}

TRANS Transform::rotateY(double angle)//�Ƕ�ֵ
{
	TRANS res;
	res.roty(angle);
	return res;
}

TRANS Transform::rotateZ(double angle)//�Ƕ�ֵ
{
	TRANS res;
	res.rotz(angle);
	return res;
}

TRANS Transform::trans(double x, double y, double z)
{
	TRANS res;
	res.trans(x, y, z);
	return res;
}


//-----------------------------------------------------------
//��ɽһ�Ż���������
TRANS positiveKSRobot(const JAngle& JA)
{
	TRANS t6;
	TRANS temp;
	double s1=sin(JA.angle[0]*PI/180.0);
	double c1=cos(JA.angle[0]*PI/180.0);
	double s2=sin(JA.angle[1]*PI/180.0);
	double c2=cos(JA.angle[1]*PI/180.0);
	double s3=sin(JA.angle[2]*PI/180.0);
	double c3=cos(JA.angle[2]*PI/180.0);
	double s4=sin(JA.angle[3]*PI/180.0);
	double c4=cos(JA.angle[3]*PI/180.0);
	double s5=sin(JA.angle[4]*PI/180.0);
	double c5=cos(JA.angle[4]*PI/180.0);
	double s6=sin(JA.angle[5]*PI/180.0);
	double c6=cos(JA.angle[5]*PI/180.0);     
    double s23=s2*c3+c2*s3;
	double c23=c2*c3-s2*s3;
	
	temp.rot.mem[0][0] = s1*(c5*c6*s4+c4*s6)+c1*(c4*c5*c6*s23+c23*c6*s5-s23*s4*s6);
	temp.rot.mem[1][0] = c6*(c4*c5*s1*s23-c1*c5*s4+c23*s1*s5)-(c1*c4+s1*s23*s4)*s6;
	temp.rot.mem[2][0] = -c6*s23*s5+c23*(c4*c5*c6-s4*s6);
	temp.rot.mem[0][1] = c6*(c4*s1-c1*s23*s4)-s6*(c5*s1*s4+c1*(c4*c5*s23+c23*s5));
	temp.rot.mem[1][1] = c1*(-c4*c6+c5*s4*s6)-s1*(c6*s23*s4+(c4*c5*s23+c23*s5)*s6);
	temp.rot.mem[2][1] = s23*s5*s6-c23*(c6*s4+c4*c5*s6);
	temp.rot.mem[0][2] = temp.rot.mem[1][0]*temp.rot.mem[2][1]-temp.rot.mem[2][0]*temp.rot.mem[1][1];    
	temp.rot.mem[1][2] = temp.rot.mem[2][0]*temp.rot.mem[0][1]-temp.rot.mem[0][0]*temp.rot.mem[2][1];
	temp.rot.mem[2][2] = temp.rot.mem[0][0]*temp.rot.mem[1][1]-temp.rot.mem[1][0]*temp.rot.mem[0][1];
	temp.pos.dx = c1*(KUNSHANROBOT_A2+c2*KUNSHANROBOT_A3+c23*(KUNSHANROBOT_D4+c5*KUNSHANROBOT_D5))
		-KUNSHANROBOT_D5*(c1*c4*s23+s1*s4)*s5;
	temp.pos.dy = (KUNSHANROBOT_A2+c2*KUNSHANROBOT_A3+c23*(KUNSHANROBOT_D4+c5*KUNSHANROBOT_D5))*s1
		-KUNSHANROBOT_D5*(c4*s1*s23-c1*s4)*s5;
	temp.pos.dz = KUNSHANROBOT_D1-KUNSHANROBOT_A3*s2-(KUNSHANROBOT_D4+c5*KUNSHANROBOT_D5)*s23
		-c23*c4*KUNSHANROBOT_D5*s5;
	
	t6 = temp;    
	return t6;
}

//��ɽһ�Ż��������    
JAngle inverseKSRobot(const TRANS& robotposition, const JAngle& lastrobotangle, bool& succeeded)
{
	KunshanRKA kunshanRKA;
	JAngle res;
	if (1 == kunshanRKA.InverseRobot(res, lastrobotangle, robotposition))
	{
		succeeded = false;
	}
	else
	{
		if (isJAngleInside(res))//�Ƿ��ڿɴ�������
		{
			succeeded = true;
		}
		else
			succeeded = false;
	}
	return res;
}

//-----------------------------------------------------------
double BETALIMIT = 30.0;
