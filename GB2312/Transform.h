// Transform.h: interface for the Transform class.
// Author: ������
// ���ܣ������õ�������ģ������ϵ����������ϵ�ı任��ϵ���ڶ�����ǰ����߱��ƽ̨����غ�����������
//////////////////////////////////////////////////////////////////////
#ifndef _TRANSFORM_H_
#define _TRANSFORM_H_


#include "robotdata.h"
#include "robotkinematic.h"
//#include "robotmotion.h"
#include <stdlib.h>

extern double BETALIMIT;
inline const JAngle operator+(const JAngle& lhs, const JAngle& rhs)
{
	JAngle res;
	for (int i = 0; i < 6; ++i)
	{
		res.angle[i] = lhs.angle[i] + rhs.angle[i];
	}
	return res;
}
inline const JAngle operator-(const JAngle& lhs, const JAngle& rhs)
{
	JAngle res;
	for (int i = 0; i < 6; ++i)
	{
		res.angle[i] = lhs.angle[i] - rhs.angle[i];
	}
	return res;
}
inline const JAngle operator/(const JAngle& lhs, double ratio)
{
	JAngle res;
	for (int i = 0; i < 6; ++i)
	{
		res.angle[i] = lhs.angle[i] / ratio;
	}
	return res;
}
inline const JAngle operator*(const JAngle& lhs, double ratio)
{
	JAngle res;
	for (int i = 0; i < 6; ++i)
	{
		res.angle[i] = lhs.angle[i] * ratio;
	}
	return res;
}
//�������ٱȽ�����JAngle��û�������,����Դ�ؽڽ��м�Ȩ���ȽϷ��ϴ�ؽڲ����˶�̫���ʵ�����
inline bool operator<(const JAngle& lhs, const JAngle& rhs)
{
	double totalAngle1 = 0.0;
	double totalAngle2 = 0.0;
	double weights[6] = {1.8, 1.7, 1.7, 1.4, 1.0, 1.0};//������Խ�������
	for (int i = 0; i < 6; ++i)
	{
		totalAngle1 += fabs(lhs.angle[i]) * weights[i];
		totalAngle2 += fabs(rhs.angle[i]) * weights[i];
	}
	return totalAngle1 < totalAngle2;
}

//�ؽڽǷ�Χ��û������ɽһ�ŵ���Ч��Χ��
inline bool isJAngleInside(const JAngle& jAngle)
{
	return ((-150.0 < jAngle.angle[0]) && (jAngle.angle[0] < 180.0))
		&& ((-125.0 < jAngle.angle[1]) && (jAngle.angle[1] < 30.0))
		&& ((-120.0 < jAngle.angle[2]) && (jAngle.angle[2] < 150.0))
		&& ((-180.0 < jAngle.angle[3]) && (jAngle.angle[3] < 180.0))
		&& ((-120.0 < jAngle.angle[4]) && (jAngle.angle[4] < 120.0))
		&& ((-180.0 < jAngle.angle[5]) && (jAngle.angle[5] < 180.0));
}
//�Ƚ������ؽڽ��ĸ���ο��ؽڽǽϽ��ĺ�������
class JAngleComparer
{
public:
JAngleComparer(const JAngle & reference): m_referenceJAngle(reference)
	{
		m_bIsThereRefrence = true;
	}
	JAngleComparer()//���û��ָ���ο��ؽڽǣ��������ж������ؽڽǱ����Ƿ��㹻�ӽ�
	{
		m_bIsThereRefrence = false;
	}

	bool operator()(const JAngle& lhs, const JAngle& rhs)const
	{
		if (!m_bIsThereRefrence)
		{
//			throw std::runtime_error("û�бȽ����壡��ȷ���ο���");
		}
		JAngle diff1 = lhs - m_referenceJAngle;
		JAngle diff2 = rhs - m_referenceJAngle;
		return diff1 < diff2;
	}
	bool isNearby(const JAngle& lhs, const JAngle& rhs, double Mneighbor = 80.0)const
	{//��������������λ���Ƿ��㹻�ӽ�
		JAngle diff = lhs - rhs;
		double weights[6] = {1.3, 1.3, 1.3, 1.1, 1.0, 1.0};
		double total_diff = 0.0;
		for (int i = 0; i < 6; ++i)
		{
			total_diff += pow(diff.angle[i] * weights[i], 2);
		}
		total_diff = sqrt(total_diff);
		if (total_diff < Mneighbor)
		{
			return true;
		}
		return false;
	}

private:
	JAngle m_referenceJAngle;//�ο��ؽڽ�
	bool m_bIsThereRefrence;
};

//ע��������漰���Ƕȶ����ǻ��ȣ��ǽǶȣ�ֻ������math������Ҫת��������
//���ȵ�λ�Ǻ���
//��˹������ר��
class Transform  
{
public:
	Transform();
	virtual ~Transform();

public:
	static void getTransBaseToJoints(const JAngle& jAngle, 
					 TRANS& T01, TRANS& T02, TRANS& T03, TRANS& T04, TRANS& T05, TRANS& T06); //��û����˻�����ϵ�������ؽ�����ϵ����α任����

	static void getTransWorldToPos(const JAngle& extJAngle, TRANS& TP1, TRANS& TP2);//�����������ϵ����λ������α任����

	static TRANS getTransWorldToRail(const JAngle& extJAngle);//�����������ϵ�������������α任����
	static TRANS getTransWorldToBase(const JAngle& extJAngle);//�����������ϵ�������˻�������α任����
	//static TRANS getTransWorldToPositioner();//�����������ϵ����λ������α任����
	static TRANS getTransWorldToWorkpiece(const JAngle& extJAngle);//�����������ϵ����������ϵ����α任����
	static TRANS getTrans6ToFlange();//�����ؽڵ������̵�T
	static TRANS getTransFlangeToTorch();//�����̵���ǹĩβ��T
	static TRANS getTrans6ToTorch();//�����ؽڵ���ǹĩ������ϵ
	static TRANS getTrans6ToGun();//�����ؽڵ���ǹ����ϵ

	static TRANS getTransWorldToTorch(const JAngle& extJAngle, const TRANS& TWorkpieceToSeam);//�����������ϵ����ǹĩ������ϵ����α任����
	static TRANS getTransBaseToTorch(const JAngle& extJAngle, const TRANS& TWorkpieceToSeam);//��û����˻�������ϵ����ǹĩ������ϵ����α任����

	static TRANS getT6ByTransBaseToTorch(const TRANS& TBaseToTorch);//���BaseTo6

	static TRANS rotateX(double angle);
	static TRANS rotateY(double angle);
	static TRANS rotateZ(double angle);
	static TRANS trans(double x, double y, double z);
};
//�������¶�������⺯��
TRANS positiveKSRobot(const JAngle& jAngle);                                       //��ɽһ�Ż���������
JAngle inverseKSRobot(const TRANS& t6, const JAngle& lastJointangle, bool& succeeded);//��ɽһ�Ż�������⣺


#endif /* _TRANSFORM_H_ */
