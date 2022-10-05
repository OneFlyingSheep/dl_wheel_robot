#ifndef DATACONVERT_ALEXWEI_H
#define DATACONVERT_ALEXWEI_H


//仅仅针对此软件的角度转换
double rad2angle(double rad)
{
	//（-π，π）转换到（0， 2π）
	double angle = 0;
	if (rad > 0) {
		angle = rad * 180 / 3.14159;
	}
	else {
		angle = ((2 * 3.14159 + rad) * 180) / 3.14159;
	}

	//逆时针转换到顺时针
	return (360 - angle);
}

#endif
