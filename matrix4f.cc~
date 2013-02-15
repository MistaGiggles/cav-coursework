using namespace std;
#include <iostream>
#include <cmath>
#include "matrix4f.h"



ostream & operator << (ostream & stream, Matrix4f & obj) 
{
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			stream << obj(i,j) << ' ';
		}
		stream << '\n';
	}
};


//
// additional functions
//  create a matrix that is same as the one of OOGL 
//  X rot
//  input  : angle -> radian
//  output : matrix 4x4

Matrix4f rotX(const float & rad )
{
  Matrix4f mat(1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.);
  //Matrix4f mat;
  float s;
   
  s = (float)sin(rad);
 
  mat(1,1) = mat(2,2) = (float)cos(rad);
  mat(2,1) = s;
  mat(1,2) = -s;
  
  return mat;
};

//  create a matrix that is same as the one of OOGL 
//  Y rot
//  input  : angle -> radian
//  output : matrix 4x4

Matrix4f rotY(const float & rad)
{ 
  Matrix4f mat(1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.);
  float s;

  s = (float)sin(rad);
  
  mat(0,0) = mat(2,2) = (float)cos(rad);
  mat(0,2) = s;
  mat(2,0) = -s;
  
  return mat;
}

//  create a matrix that is same as the one of OGL 
//  Z rot
//  input  : angle -> radian 
//  output : matrix 4x4

Matrix4f rotZ(const float & rad)
{
  Matrix4f mat(1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.);
  float s;

  s = (float)sin(rad);
    
  mat(0,0) = mat(1,1) = (float)cos(rad);
  mat(1,0) = s;
  mat(0,1) = -s;
  
  return mat;
}


float Matrix4f::lu(Matrix4f & a, int ip[])
{
	int n = 4;

	int i, j, k, ii, ik;
	float t, u, det;
	float *weight = new float[n]; 

	det = 0;             
	for (k = 0; k < n; k++) {  
		ip[k] = k;          
		u = 0;                
		for (j = 0; j < n; j++) {
			t = fabs(a(k,j));  if (t > u) u = t;
		}
		if (u == 0) goto EXIT; 
		weight[k] = 1 / u;     
	}
	det = 1;                   
	for (k = 0; k < n; k++) {  
		u = -1;
		for (i = k; i < n; i++) {  
			ii = ip[i];            
			t = fabs(a(ii, k)) * weight[ii];
			if (t > u) {  u = t;  j = i;  }
		}
		ik = ip[j];
		if (j != k) {
			ip[j] = ip[k];  ip[k] = ik;  
			det = -det;  
		}
		u = a(ik,k);  det *= u; 
		if (u == 0) goto EXIT;    
		for (i = k + 1; i < n; i++) { 
			ii = ip[i];
			t = (a(ii,k) /= u);
			for (j = k + 1; j < n; j++)
				a(ii, j) -= t * a(ik, j);
		}
	}
EXIT:

	delete [] weight;

	return det;         
}


float Matrix4f::inv(Matrix4f & a, Matrix4f & a_inv)
{

	int n = 4;

	int i, j, k, ii;
	float t, det;

	int * ip = new int [n];


	det = lu(a, ip);

	if (det != 0)
		for (k = 0; k < n; k++) {
			for (i = 0; i < n; i++) {
				ii = ip[i];  t = (ii == k);
				for (j = 0; j < i; j++)
					t -= a(ii,j) * a_inv(j,k);
				a_inv(i,k) = t;
			}
			for (i = n - 1; i >= 0; i--) {
				t = a_inv(i,k);  ii = ip[i];
				for (j = i + 1; j < n; j++)
					t -= a(ii,j) * a_inv(j,k);
				a_inv(i,k) = t / a(ii,i);
			}
		}


	delete [] ip ;

	return det;
}


Matrix4f Matrix4f::operator !()
{
	Matrix4f mat(*this);
	Matrix4f ret;

	inv(mat, ret);

	return ret;
}


Vector3f operator * (const Matrix4f & mat, const Vector3f & vec) 
{
	Vector3f ret(0.f,0.f,0.f);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			ret[i] += mat(i,j) * vec[j];
		ret[i] += mat(i,3);
	}
	
	return ret;
}

Matrix4f & Matrix4f::operator = ( const Matrix4f & obj )
{

	for (int i =0; i < 4; i++) {
		for (int j =0; j < 4; j++) {
			(*this)(i,j) = obj(i,j);
		}
	}
    return *this; 
}

Matrix4f Matrix4f::trans()
{
	Matrix4f ret;	

	for (int i =0; i < 4; i++) {
		for (int j =0; j < 4; j++) {

			ret(i,j) = (*this)(j,i);
		}
	}

    	return ret; 
}


Matrix4f operator + (const Matrix4f& m1, const Matrix4f& m2)
{
    Matrix4f m;

    for (int i = 0; i < 4; i++) {
	for (int j = 0; j < 4; j++)
	    m(i, j) = m1(i, j) + m2(i, j);
    }

    return m;
}

Matrix4f operator * (const Matrix4f& m1, const Matrix4f& m2)
{
    Matrix4f m;
    for (int i = 0; i < 4; ++i) {
	for (int j = 0; j < 4; ++j) {
	    m(i, j) = 0.0;
	    for (int k = 0; k < 4; ++k)
		m(i, j) += m1(i, k) * m2(k, j);
	}
    }

    return m;
}






Matrix4f & Matrix4f::operator *= (const Matrix4f& m2)
{
    Matrix4f m;

    for (int i = 0; i < 4; i++) {
	for (int j = 0; j < 4; j++) {
	    m(i, j) = 0.0;
	    for (int k = 0; k < 4; k++)
		m(i, j) += (*this)(i, k) * m2(k, j);
	}
    }

    *this = m;	

    return *this;
}


Matrix4f translation(const Vector3f  & vec)
{
	Matrix4f m;

	m.setIdentity();
	m.setTranslation(vec);

	return m;
};

void Matrix4f::setTranslation(const Vector3f  & vec) {
	for (int i = 0; i < 3; i++) 
		_item[i][3] = vec[i];

	_item[3][3] = 1.0;
}



Vector3f operator % ( const Vector3f & ob1, const Vector3f & ob2)
{
    int i;              // loop counter
    Vector3f ret;            // return value
    int N_DV3 = 3;	
		
    for (i = 0; i < N_DV3; i++)
	ret._item[ i ] =
	    ob1[ ( i + 1 ) % N_DV3 ] * ob2[ ( i + 2 ) % N_DV3 ]
	    - ob2[ ( i + 1 ) % N_DV3 ] * ob1[ ( i + 2 ) % N_DV3 ];

    return ret;
}


Vector3f operator + (const Vector3f& v1, const Vector3f& v2)
{
    Vector3f v(v1);
    for (int i = 0; i < 3; i++)
	v[i] += v2[i];

    return v;
}

Vector3f operator - (const Vector3f& v1, const Vector3f& v2)
{

    Vector3f v(v1);
    for (int i = 0; i < 3 ; i++)
	v[i] -= v2[i];

    return v;
}




float distance(Vector3f & v1, Vector3f & v2)
{

	float ret;

	ret = sqrt((v2[0]-v1[0])*(v2[0]-v1[0]) + (v2[1]-v1[1])*(v2[1]-v1[1]) + (v2[2]-v1[2])*(v2[2]-v1[2]));

	return ret;
}


ostream & operator << (ostream & stream, Vector3f & obj) 
{
	stream << obj[0] << ' ' << obj[1] << ' ' << obj[2] << ' ';
};


/***
 * example of using the functions
 *

int main()
{
	Vector3f tmpf(1.,0.,0.);
	Vector3f zerov(0.,0.,0.);
	Matrix4f r1,r2,r3, t1,t2;

	t1 = !translation(tmpf);
	r1 = rotY(M_PI/4.);

	Vector3f out;

	out = r1 * t1* zerov;

	cout << " rotZ \n"<< r1 << endl;
	cout << " trans \n"<< t1 << endl;
	cout << "out " << out << endl;
}

**/

