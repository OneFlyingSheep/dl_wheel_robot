#ifndef DATACONVERT_ALEXWEI_H
#define DATACONVERT_ALEXWEI_H


//������Դ�����ĽǶ�ת��
double rad2angle(double rad)
{
	//��-�У��У�ת������0�� 2�У�
	double angle = 0;
	if (rad > 0) {
		angle = rad * 180 / 3.14159;
	}
	else {
		angle = ((2 * 3.14159 + rad) * 180) / 3.14159;
	}

	//��ʱ��ת����˳ʱ��
	return (360 - angle);
}

#endif
