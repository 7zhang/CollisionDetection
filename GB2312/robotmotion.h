// !@File
// ***************************************************************
// *<Pre>
// *�ļ�����robotmotion.h
// *����������
// *���ߣ�����
// *���ڣ�2011-3-7
// *</Pre>
// ***************************************************************
#ifndef __ROBOMOTION_H_
#define __ROBOMOTION_H_

#define IN
#define OUT
#define INOUT

//#include <assert.h>
#include "geometric.h"
#include "robotdata.h"

#define EXPENSIN 0.00001
const double Ts=0.010;  //����



// !@class
// ******************************************************************
// *@<pre>
// *@������InterpolatePara
// *@���ߣ�����
// *@���ڣ�2011-3-7
// *@</pre>
// ******************************************************************
class  InterpolatePara  
{
public:
	InterpolatePara();
	virtual ~InterpolatePara();

	double aimLen;
	double aimVel;
	double aimAcc;
	double aimJerk;			//�����ļӼ��ٶ�
	double beginVel;
	double endVel;
	double err[2];
	double timepoint[7];      //�岹�˶��еĹؼ�ʱ��ڵ�
	double lenpoint[5];       //�岹�˶��еĹؼ�·�̽ڵ�

	bool cal_InterpolatePara();       //����岹�˶�����
	bool next_StepInterpolate(double &nextLen, double timecounter);  //��ʱ��������岹����

protected:
	double m_vel[2], m_acc[2], m_jerk[2];	//�ӡ����ٶε��ٶȡ����ٶȡ����ٶȵ���
};

// !@class
// ******************************************************************
// *@<pre>
// *@������IMotionTask	�����࣬������������
// *@���ߣ�����
// *@���ڣ�2011-3-7
// *@</pre>
// ******************************************************************
class  IMotionTask
{
public:
	RouteSeg route;	
	InterpolatePara interPara;
	RoutePoint currPos;
	RoutePoint nextPos;
	
public:
//	bool init_route(double acc,double vel,RoutePoint BPos,RoutePoint EPos,RouteType type,double length=0);
	//���麯��
	virtual bool init_para() = 0;					//��ʼ��·������
	virtual bool init_task(double velRate) = 0;		//��ʼ������		
	virtual bool get_nextpoint(double velRate) = 0;	//��ȡ��һ�岹��
	virtual bool Is_EndInter() = 0;					//�жϲ岹�Ƿ����

	virtual double flyby_checkVar(double radius) = 0;	//flyby�м��·������
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang) = 0;	//flyby�л�ȡ���ɶ���ʼ�㼰�䵥λ������
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang) = 0;	//flyby�л�ȡ���ɶ���ֹ�㼰�䵥λ������

	bool get_MotionTime(double& time){				//��ȡ�岹ʱ��
			if(interPara.timepoint[6]==0)
			{
				time = 0;
				return false;
			}
			time = interPara.timepoint[6];
			return true;
	}
	virtual bool get_MotionRatio(double& ratio) = 0;//��ȡ�Ѳ岹�ı���
};

// !@class
// ******************************************************************************
// *@<pre>
// *@������
// *@������Flyby����ӿ�
// *@���ߣ�����
// *@���ڣ�2011-4-28
// *@</pre>
// ******************************************************************************
class  IFlybyAlg
{
	virtual double flyby_checkVar(double radius) = 0;	//flyby�м��·������
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang) = 0;	//flyby�л�ȡ���ɶ���ʼ�㼰�䵥λ������
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang) = 0;	//flyby�л�ȡ���ɶ���ֹ�㼰�䵥λ������	
};


// !@class
// ******************************************************************************
// *@<pre>
// *@������JointMTask
// *@������
// *@���ߣ�����
// *@���ڣ�2011-4-28
// *@</pre>
// ******************************************************************************
class  JointMTask :public IMotionTask
{
public:
	JointMTask();
	bool init_para();					//��ʼ��·������
	bool init_task(double velRate);		//��ʼ������	
	bool get_nextpoint(double velRate);	//��ȡ��һ�岹��
	bool Is_EndInter();					//�жϲ岹�Ƿ����

	double flyby_checkVar(double radius);	//flyby�м��·������
	bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang);	//flyby�л�ȡ���ɶ���ʼ�㼰�䵥λ������
	bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang);	//flyby�л�ȡ���ɶ���ֹ�㼰�䵥λ������

	void setScale(int * velScale, int * accScale, int *jerkScale);   //�趨���ؽڵ��ٶȡ����ٶȡ��Ӽ��ٶȱ���
	bool get_MotionPara(double& len, double& vel, double& acc, int& number, double& jerk);  //��ȡ�˶�����
	bool get_MotionPara(double& len, double& vel, double& acc, int& number);
	bool get_MotionPara(double& len);

	virtual bool get_MotionRatio(double& ratio){	//��ȡ�Ѳ岹�ı���
		if( fabs(m_selLen)<EXPENSIN )
		{
			return false;
		}
		ratio = currLen/m_selLen;
		if( ratio>1 || ratio<0 )
		{
			return false;
		}
		return true;
	}

protected:
	JAngle m_angBegin2End;	//��㵽�յ�Ĺؽڽ�	
	double m_timecounter;	//��¼�岹ʱ��
	int m_selNumber;		//ѡ���Ĺؽڱ�ţ�0-5��
	double m_selLen;		//ѡ����·�̲�ֵ
	double m_selVel;		//ѡ�����˶��ٶ�
	double m_selAcc;		//ѡ�����˶����ٶ�
	double m_selJerk;		//ѡ�����˶��Ӽ��ٶ�
	double m_velRate;       //����˶��ٶȵİٷֱ�
	double m_saveLen;
	int Velo_Max[6];		//��λ:��/��   {70,70,70,70,70,150}
	int Acce_Max[6];		//��λ:��/�뷽 {200,200,200,600,600,900} 
	int Jerk_Max[6];		//��λ:��/������ {200,200,200,600,600,900} 
	
public:
//	RoutePoint currPos;
//	RoutePoint nextPos;
	double currLen;
	double nextLen;
};

class  LineMTask :public IMotionTask
{
public:
	LineMTask();
	virtual bool init_para();					//��ʼ��·������
	virtual bool init_task(double velRate);		//��ʼ������	
	virtual bool get_nextpoint(double velRate);	//��ȡ��һ�岹��
	virtual bool Is_EndInter();					//�жϲ岹�Ƿ����

	virtual double flyby_checkVar(double radius);	//flyby�м��·������
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang);	//flyby�л�ȡ���ɶ���ʼ�㼰�䵥λ������
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang);	//flyby�л�ȡ���ɶ���ֹ�㼰�䵥λ������

	bool get_MotionPara(double& len);
	bool get_MotionPara(double& len,double& vel, double& acc);

	virtual bool get_MotionRatio(double& ratio){	//��ȡ�Ѳ岹�ı���
		if( fabs(m_len)<EXPENSIN )
		{
			return false;
		}
		ratio = currLen/m_len;
		if( ratio>1 || ratio<0 )
		{
			return false;
		}
		return true;
	}

protected:
	RPY m_rpyBegin2End;		//���ָ���յ��RPY����
	double m_timecounter;	//��¼�岹ʱ��
	double m_len;			//·�̲�ֵ
	double m_vel;			//�˶��ٶ�
	double m_velRate;       //����˶��ٶȵİٷֱ�
	double m_saveLen;

public:
//	RoutePoint currPos;
//	RoutePoint nextPos;
	double currLen;
	double nextLen;
};

class  ArcMTask :public IMotionTask
{
public:
	ArcMTask();
	virtual bool init_para();					//��ʼ��·������
	virtual bool init_task(double velRate);		//��ʼ������	
	virtual bool get_nextpoint(double velRate);	//��ȡ��һ�岹��
	virtual bool Is_EndInter();					//�жϲ岹�Ƿ����

	virtual double flyby_checkVar(double radius);	//flyby�м��·������
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang);	//flyby�л�ȡ���ɶ���ʼ�㼰�䵥λ������
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang);	//flyby�л�ȡ���ɶ���ֹ�㼰�䵥λ������
	
	bool get_MotionPara(double& arc);
	bool get_MotionPara(double& arc,double& r,double& vel, double& acc);

	virtual bool get_MotionRatio(double& ratio){	//��ȡ�Ѳ岹�ı���
		if( fabs(m_arc)<EXPENSIN )
		{
			return false;
		}
		ratio = currArc/m_arc;
		if( ratio>1 || ratio<0 )
		{
			return false;
		}
		return true;
	}

protected:
	RPY m_rpyBegin2Mid;		// ���ָ���е��RPY����
	RPY m_rpyMid2End;		// �е�ָ��ĩ���RPY����
	double m_radBegin2Mid;	// ��㵽�е�Ļ���
	double m_radMid2End;	// �е㵽ĩ��Ļ���
	double m_radBegin2End;	// ��㵽ĩ��Ļ���
	double m_timecounter;	// ��¼�岹ʱ��
	double m_r;				// �뾶
	TRANS m_T;				// Բ��������α任����,posΪԲ������
	double m_arc;			// ��㵽ĩ��Ļ���
	double m_arc1;			// ��㵽�е�Ļ���
	double m_arc2;			// �е㵽ĩ��Ļ���
	double m_vel;			// ѡ�����˶��ٶ�
	double m_velRate;       // ����˶��ٶȵİٷֱ�
	double m_saveArc;

public:
	RoutePoint midPos;		// �м�ʾ�̵㣨����Բ���岹�У�
//	RoutePoint currPos;
//	RoutePoint nextPos;
	double currArc;
	double nextArc;
};

// !@class
// ******************************************************************************
// *@<pre>
// *@������CirMTask
// *@�����������Բ�岹�Ĺ��ܣ����������㣬���岹����Բ��������ArcMTask��
// *@���ߣ�����
// *@���ڣ�2012-02-03
// *@</pre>
// ******************************************************************************
class  CirMTask :public IMotionTask
{
public:
	CirMTask();
	virtual bool init_para();					//��ʼ��·������
	virtual bool init_task(double velRate);		//��ʼ������	
	virtual bool get_nextpoint(double velRate);	//��ȡ��һ�岹��
	virtual bool Is_EndInter();					//�жϲ岹�Ƿ����
	
	virtual double flyby_checkVar(double radius);	//flyby�м��·������
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang);	//flyby�л�ȡ���ɶ���ʼ�㼰�䵥λ������
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang);	//flyby�л�ȡ���ɶ���ֹ�㼰�䵥λ������
	
	bool get_MotionPara(double& arc);
	bool get_MotionPara(double& arc,double& r,double& vel, double& acc);
	
	virtual bool get_MotionRatio(double& ratio){	//��ȡ�Ѳ岹�ı���
		if( fabs(m_arc)<EXPENSIN )
		{
			return false;
		}
		ratio = currArc/(m_arc);
		if( ratio>1 || ratio<0 )
		{
			return false;
		}
		return true;
	}
	
protected:
	RPY m_rpyBegin2Mid;		// ���ָ���е��RPY����
	RPY m_rpyMid2End;		// �е�ָ��ĩ���RPY����
	double m_radBegin2Mid;	// ��㵽�е�Ļ���
	double m_radMid2End;	// �е㵽ĩ��Ļ���
	double m_radBegin2End;	// ��㵽ĩ��Ļ���
	double m_timecounter;	// ��¼�岹ʱ��
	double m_r;				// �뾶
	TRANS m_T;				// Բ��������α任����,posΪԲ������
	double m_arc;			// ��㵽ĩ��Ļ���
	double m_arc1;			// ��㵽�е�Ļ���
	double m_arc2;			// �е㵽ĩ��Ļ���
	double m_vel;			// ѡ�����˶��ٶ�
	double m_velRate;       // ����˶��ٶȵİٷֱ�
	double m_saveArc;
	
public:
	RoutePoint midPos;		// �м�ʾ�̵㣨����Բ���岹�У�
	double currArc;
	double nextArc;
};

class  FlybyMTask :public IMotionTask
{
public:
	FlybyMTask();
	virtual bool init_para();						//��ʼ��·������
	virtual bool init_task(double velRate);			//��ʼ������	
	virtual bool get_nextpoint(double velRate);		//��ȡ��һ�岹��
	virtual bool Is_EndInter();						//�жϲ岹�Ƿ����

	virtual double flyby_checkVar(double radius);	//flyby�м��·������
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang);	//flyby�л�ȡ���ɶ���ʼ�㼰�䵥λ������
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang);	//flyby�л�ȡ���ɶ���ֹ�㼰�䵥λ������

	bool get_MotionPara(double& len);
	Vector3D get_curvepoint(Matrix4D MCurve,double t);		//��ȡ����ɭ�����ϵĵ�
	double get_curvelength(const Matrix4D& MCurve);			//��ȡ����ɭ���ߵĳ���
	Matrix4D get_curveMatrix(const Vector3D& beginPos,const Vector3D& endPos,
							const Vector3D& beginTang,const Vector3D& endTang);		//��ȡflyby�岹���õĸ���ɭ���߾���
	double get_minDist(Matrix4D MCurve,Vector3D point);		//��ȡ�㵽����ɭ���ߵ���̾���

	virtual bool get_MotionRatio(double& ratio){	//��ȡ�Ѳ岹�ı���
		if( fabs(m_len)<EXPENSIN )
		{
			return false;
		}
		ratio = currLen/m_len;
		if( ratio>1 || ratio<0 )
		{
			return false;
		}
		return true;
	}

protected:
	RPY m_rpyBegin2End;		// ���ָ���յ��RPY����
	double m_timecounter;	// ��¼�岹ʱ��
	double m_len;			// ���߳���
	double m_vel;			// ѡ�����˶��ٶ�
	double m_velRate;       // ����˶��ٶȵİٷֱ�
	double m_saveLen;

public:
	Matrix4D m_MCurve;		// ����ɭ���߾��󣬰�����ʼ����ֹ����Ϣp0,p1����ʼ����ֹ����T0,T1�����������ѹ滮�ã�
//	RoutePoint currPos;
//	RoutePoint nextPos;
	double currLen;
	double nextLen;
	double path_radius1;	//����·������ʼ�㵽���ƽ�ʾ�̵�ľ���
	double path_radius2;	//����·������ֹ�㵽���ƽ�ʾ�̵�ľ���
	double path_dist;		//����ɭ���ߵ����ƽ�ʾ�̵�����������̾���
};

class  SplineMTask :public IMotionTask
{
public:
	SplineMTask();
	virtual bool init_para();					//��ʼ��·������
	virtual bool init_task(double velRate);		//��ʼ������	
	virtual bool get_nextpoint(double velRate);	//��ȡ��һ�岹��
	virtual bool Is_EndInter();					//�жϲ岹�Ƿ����
	
	virtual double flyby_checkVar(double radius);	//flyby�м��·������
	virtual bool flyby_beginPoint(double radius, RPY &beginPoint, Vector3D &beginTang);	//flyby�л�ȡ���ɶ���ʼ�㼰�䵥λ������
	virtual bool flyby_endPoint(double radius, RPY &endPoint, Vector3D &endTang);	//flyby�л�ȡ���ɶ���ֹ�㼰�䵥λ������
	
	virtual bool get_MotionRatio(double& ratio){	//��ȡ�Ѳ岹�ı���
		if( fabs(m_len)<EXPENSIN )
		{
			return false;
		}
		ratio = currLen/(m_len);
		if( ratio>1 || ratio<0 )
		{
			return false;
		}
		return true;
	}

protected:
	RPY m_rpyBegin2End;		// ���ָ���յ��RPY����
	double m_timecounter;	// ��¼�岹ʱ��
	double m_vel;			// ѡ�����˶��ٶ�
	double m_velRate;       // ����˶��ٶȵİٷֱ�
	double m_saveLen;
	
public:
	//	RoutePoint currPos;
	//	RoutePoint nextPos;
	double m_len;			// ·�̲�ֵ
	double m_tense;			// ����ϵ��
	double currLen;			// �����
	double nextLen;			// �����
	Vector3D ddBeginPos;	// ������ߵĶ��׵���
	Vector3D ddEndPos;		// �յ����ߵĶ��׵���
	
	// add by geliuhao 20120713
//	vector<Quaternion> m_QuatList;		// �洢���������ϵ�������̬��
	unsigned int m_num;			// ��¼�ö����������������е����,��1��ʼ
	Quaternion m_AuxQuat;		// �������ɱ��������ߵĸ�����
};

typedef CTypedPtrList	<CPtrList,SplineMTask*>	SplineMTaskList;	//����������������
typedef CTypedPtrList	<CPtrList,Vector3D*>	Vector3DList;		//��ά��������

// [[ <FUNCTION GROUP>
	//����岹ʱ�ٶȱ仯
	  BOOL VeloStep(double Vcurrent, double Vaim, double accel, double& Vnext);  
	 
	 //�����ռ�������Բ�ġ��뾶��ת������
	  BOOL GetCenRT(Point3D& p1, Point3D& p2, Point3D& p3, Point3D& cen, double& r, TRANS& T);	
	
	 //��Բ��˳������֮���Բ�Ľ�
	  void GetCenAngle(const Point3D& q1,const Point3D& q2,const Point3D& q3,double& rad_12,double& rad_23,double& rad_13);	
	 
	  bool add_flyby(INOUT IMotionTask *seg1, INOUT IMotionTask *seg2, OUT FlybyMTask *seg3);
	  bool _IsEndInter(double selLen,double nextLen );	//�жϲ岹·���Ƿ������

	  bool get_SplinePara(SplineMTaskList* splineList);		//��������������߲���	 
	  bool solve_SplinePara(Vector3DList* splinePointList, double tense, Vector3DList* ddPointList);	//���������߲���
	  bool get_SplinePointSlope(Vector3D p1, Vector3D p2, Vector3D p3, double x1, double x2, double x3, Vector3D& slope);	//����������߶˵��б��(�����������)
	  void free_Vector3DList(Vector3DList* plist);			//�ͷ������ڴ�ռ�
	  void free_SplineMTaskList(SplineMTaskList* plist);		//�ͷ������ڴ�ռ�

	  bool get_SplineQuatPara(SplineMTaskList* splineList);		// ��ȡ�����岹�е���Ԫ����̬����
// ]] </FUNCTION GROUP>

#endif /*__ROBOMOTION_H__*/
