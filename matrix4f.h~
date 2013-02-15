#ifndef _matrix_H
#define _matrix_H

#include <iostream>
#include <fstream>

class Vector3f {

	float _item[3];

	public:

	float & operator [] (int i) {
		return _item[i];
    	}

	float operator [] (int i) const {
		return _item[i];
    	}


	Vector3f(float x, float y, float z) 
	{  _item[0] = x ; _item[1] = y ; _item[2] = z; };

	Vector3f() { _item[0]=_item[1]=_item[2]=0.f;};


	Vector3f & operator = (const Vector3f & obj) 
	{
		_item[0] = obj[0];
		_item[1] = obj[1];
		_item[2] = obj[2];

		return *this;
	};

	Vector3f & operator += (const Vector3f & obj) 
	{
		_item[0] += obj[0];
		_item[1] += obj[1];
		_item[2] += obj[2];

		return *this;
	};


	Vector3f & operator -= (const Vector3f & obj) 
	{
		_item[0] -= obj[0];
		_item[1] -= obj[1];
		_item[2] -= obj[2];

		return *this;
	};




	Vector3f & operator /= (const float & f) 
	{
		_item[0] /= f;
		_item[1] /= f;
		_item[2] /= f;

		return *this;
	};


	float distance(Vector3f & v) 
	{
		float ret=0.f;

		ret = (v[0]-_item[0])*(v[0]-_item[0]) 
		    + (v[1]-_item[1])*(v[1]-_item[1])
		    + (v[2]-_item[2])*(v[2]-_item[2]);

		ret = sqrt(ret);

		return ret;
	}


	float dot(const Vector3f & input) const
	{
		return input._item[0]*_item[0] + input._item[1]*_item[1] + input._item[2]*_item[2] ; 
	}

	float normalize()
	{
		float ret = sqrt(_item[0]*_item[0] + _item[1]*_item[1] + _item[2]*_item[2]); 
		for (int i = 0; i < 3; i++) _item[i] /= ret;

		return ret;
	}




	friend Vector3f operator % ( const Vector3f &ob1, const Vector3f &ob2);
	friend Vector3f operator - ( const Vector3f &ob1, const Vector3f &ob2);
	friend Vector3f operator + ( const Vector3f &ob1, const Vector3f &ob2);

};




class Matrix4f {
	float _item[4][4];

	public:

	float & operator () (int i, int j) {
		return _item[i][j];
    	}

	float  operator () (int i, int j) const {
		return _item[i][j];
    	}

	
	void setIdentity() 
	{
		for (int i = 0; i < 4; i++) 
			for (int j = 0; j < 4; j++) 
				_item[i][j] = 0;
		_item[0][0] = _item[1][1] = _item[2][2] = _item[3][3] = 1.;
	}



	Matrix4f() 
	{ 
		for (int i = 0; i< 4; i++) 
			for (int j = 0; j< 4; j++) _item[i][j] = 0; 
	}


  	Matrix4f(const float & a00, const float & a01, const float & a02, const float & a03, 
		 const float & a10, const float & a11, const float & a12, const float & a13, 
		 const float & a20, const float & a21, const float & a22, const float & a23, 
		 const float & a30, const float & a31, const float & a32, const float & a33)
	{
		_item[0][0]=a00;_item[0][1]=a01;_item[0][2]=a02;_item[0][3]=a03;
		_item[1][0]=a10;_item[1][1]=a11;_item[1][2]=a12;_item[1][3]=a13;
		_item[2][0]=a20;_item[2][1]=a21;_item[2][2]=a22;_item[2][3]=a23;
		_item[3][0]=a30;_item[3][1]=a31;_item[3][2]=a32;_item[3][3]=a33;
	}


	float lu(Matrix4f & a, int ip[]);

	float inv(Matrix4f & a, Matrix4f & a_inv);

	Matrix4f operator !();

	void setTranslation(const Vector3f  & vec) ;

	Matrix4f & operator *= (const Matrix4f& m2);

	Matrix4f trans();

	Matrix4f & operator = ( const Matrix4f & obj );

	friend Vector3f operator *  (const Matrix4f & obj1, const Vector3f & obj2) ;
	friend Matrix4f operator *  (const Matrix4f & obj1, const Matrix4f & obj2) ;
};


Matrix4f rotX(const float & rad );
Matrix4f rotY(const float & rad);
Matrix4f rotZ(const float & rad);
Matrix4f translation(const Vector3f  & vec);

Vector3f operator - (const Vector3f& v1, const Vector3f& v2);
Vector3f operator + (const Vector3f& v1, const Vector3f& v2);
Vector3f operator % ( const Vector3f & ob1, const Vector3f & ob2);
float distance(Vector3f & v1, Vector3f & v2);

#endif // _matrix_H
